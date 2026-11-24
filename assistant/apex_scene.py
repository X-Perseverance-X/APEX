#!/usr/bin/env python3
"""Continuous scene awareness for the APEX rescue assistant.

A small YOLOX detector (ONNX, CPU) runs about once per second on the camera
snapshot, independently of the Hailo VLM.  It gives the robot an always-fresh
answer to "is there a person, where, in which posture, and which objects are
around" - for the operator overlay and as grounding text for the VLM.

Only numpy, Pillow and onnxruntime are needed (onnxruntime is already in the
assistant venv because Piper uses it).  Nothing here touches motors.
"""

from __future__ import annotations

import io
import threading
import time
import urllib.request
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np

# (English, Turkish) for the 80 COCO classes, in model order.
COCO = (
    ("person", "insan"), ("bicycle", "bisiklet"), ("car", "araba"), ("motorcycle", "motosiklet"),
    ("airplane", "uçak"), ("bus", "otobüs"), ("train", "tren"), ("truck", "kamyon"), ("boat", "tekne"),
    ("traffic light", "trafik ışığı"), ("fire hydrant", "yangın musluğu"), ("stop sign", "dur levhası"),
    ("parking meter", "parkmetre"), ("bench", "bank"), ("bird", "kuş"), ("cat", "kedi"), ("dog", "köpek"),
    ("horse", "at"), ("sheep", "koyun"), ("cow", "inek"), ("elephant", "fil"), ("bear", "ayı"),
    ("zebra", "zebra"), ("giraffe", "zürafa"), ("backpack", "sırt çantası"), ("umbrella", "şemsiye"),
    ("handbag", "el çantası"), ("tie", "kravat"), ("suitcase", "bavul"), ("frisbee", "frizbi"),
    ("skis", "kayak"), ("snowboard", "snowboard"), ("sports ball", "top"), ("kite", "uçurtma"),
    ("baseball bat", "beyzbol sopası"), ("baseball glove", "beyzbol eldiveni"), ("skateboard", "kaykay"),
    ("surfboard", "sörf tahtası"), ("tennis racket", "tenis raketi"), ("bottle", "şişe"),
    ("wine glass", "kadeh"), ("cup", "bardak"), ("fork", "çatal"), ("knife", "bıçak"), ("spoon", "kaşık"),
    ("bowl", "kase"), ("banana", "muz"), ("apple", "elma"), ("sandwich", "sandviç"), ("orange", "portakal"),
    ("broccoli", "brokoli"), ("carrot", "havuç"), ("hot dog", "sosisli"), ("pizza", "pizza"),
    ("donut", "donut"), ("cake", "pasta"), ("chair", "sandalye"), ("couch", "kanepe"),
    ("potted plant", "saksı bitkisi"), ("bed", "yatak"), ("dining table", "masa"), ("toilet", "klozet"),
    ("tv", "ekran"), ("laptop", "dizüstü bilgisayar"), ("mouse", "fare"), ("remote", "kumanda"),
    ("keyboard", "klavye"), ("cell phone", "telefon"), ("microwave", "mikrodalga"), ("oven", "fırın"),
    ("toaster", "tost makinesi"), ("sink", "lavabo"), ("refrigerator", "buzdolabı"), ("book", "kitap"),
    ("clock", "saat"), ("vase", "vazo"), ("scissors", "makas"), ("teddy bear", "oyuncak ayı"),
    ("hair drier", "saç kurutma makinesi"), ("toothbrush", "diş fırçası"),
)

_POSITION = {"left": "solda", "center": "ortada", "right": "sağda"}
_DISTANCE = {"very close": "çok yakın", "near": "yakın", "far": "uzak"}
_POSTURE = {"lying down": "yatay, yerde yatıyor olabilir", "upright": "dik"}


def prepare_frame(image: Any) -> tuple[Any, bool]:
    """Return (RGB image, is_infrared).

    The NoIR night-vision sensor delivers a strongly magenta picture under IR
    light.  Vision models read that as "a pink room", so such frames are
    turned into a contrast-stretched grayscale image.
    """
    from PIL import Image, ImageOps

    image = image.convert("RGB")
    small = np.asarray(image.resize((48, 48), Image.Resampling.BILINEAR), dtype=np.float32)
    red, green, blue = (float(small[..., i].mean()) for i in range(3))
    infrared = red > 12.0 and green < 0.82 * red and green < 0.9 * blue
    if infrared:
        gray = ImageOps.autocontrast(image.getchannel("R"), cutoff=1)
        image = Image.merge("RGB", (gray, gray, gray))
    return image, infrared


def describe_box(box: list[float], label: str, frame_aspect: float) -> dict[str, str]:
    """Coarse position / distance / posture words from a normalised box."""
    x1, y1, x2, y2 = box
    center = (x1 + x2) / 2.0
    width, height = x2 - x1, y2 - y1
    position = "left" if center < 0.36 else "right" if center > 0.64 else "center"
    extent = max(height, width)
    distance = "very close" if extent >= 0.75 else "near" if extent >= 0.4 else "far"
    info = {"pos": position, "dist": distance}
    if label == "person":
        # A close-up that fills the frame says nothing about posture.
        pixel_ratio = (width * frame_aspect) / max(height, 1e-6)
        info["posture"] = "lying down" if pixel_ratio > 1.4 and extent < 0.9 else "upright"
    return info


class YoloxDetector:
    """YOLOX (nano/tiny) ONNX inference without OpenCV."""

    def __init__(self, model: Path, threads: int = 2, confidence: float = 0.35, nms: float = 0.45) -> None:
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(model), sess_options=options, providers=["CPUExecutionProvider"])
        node = self.session.get_inputs()[0]
        self.input_name = node.name
        self.size = int(node.shape[2]) if isinstance(node.shape[2], int) else 416
        self.confidence = confidence
        self.nms = nms
        grids, strides = [], []
        for stride in (8, 16, 32):
            cells = self.size // stride
            xv, yv = np.meshgrid(np.arange(cells), np.arange(cells))
            grids.append(np.stack((xv, yv), axis=2).reshape(-1, 2))
            strides.append(np.full((cells * cells, 1), stride))
        self._grids = np.concatenate(grids).astype(np.float32)
        self._strides = np.concatenate(strides).astype(np.float32)

    def detect(self, image: Any) -> list[dict[str, Any]]:
        from PIL import Image

        width, height = image.size
        ratio = min(self.size / width, self.size / height)
        new_w, new_h = max(1, int(width * ratio)), max(1, int(height * ratio))
        resized = np.asarray(image.resize((new_w, new_h), Image.Resampling.BILINEAR), dtype=np.uint8)
        canvas = np.full((self.size, self.size, 3), 114, dtype=np.uint8)
        canvas[:new_h, :new_w] = resized[:, :, ::-1]  # RGB -> BGR, as the model was trained
        blob = np.ascontiguousarray(canvas.transpose(2, 0, 1)[None], dtype=np.float32)

        output = self.session.run(None, {self.input_name: blob})[0][0]
        centers = (output[:, :2] + self._grids) * self._strides
        sizes = np.exp(np.clip(output[:, 2:4], -10, 10)) * self._strides
        scores_all = output[:, 4:5] * output[:, 5:]
        classes = scores_all.argmax(axis=1)
        scores = scores_all[np.arange(len(classes)), classes]
        keep = scores > self.confidence
        if not keep.any():
            return []
        centers, sizes, classes, scores = centers[keep], sizes[keep], classes[keep], scores[keep]
        boxes = np.concatenate((centers - sizes / 2, centers + sizes / 2), axis=1) / ratio
        chosen = _nms(boxes + classes[:, None] * 4096.0, scores, self.nms)  # class-aware
        aspect = width / height
        detections = []
        for index in chosen:
            box = boxes[index]
            norm = [
                float(np.clip(box[0] / width, 0, 1)), float(np.clip(box[1] / height, 0, 1)),
                float(np.clip(box[2] / width, 0, 1)), float(np.clip(box[3] / height, 0, 1)),
            ]
            label, label_tr = COCO[int(classes[index])]
            detections.append({
                "label": label, "label_tr": label_tr, "conf": round(float(scores[index]), 2),
                "box": [round(value, 4) for value in norm], **describe_box(norm, label, aspect),
            })
        detections.sort(key=lambda item: (item["label"] != "person", -item["conf"]))
        return detections


def _nms(boxes: np.ndarray, scores: np.ndarray, threshold: float) -> list[int]:
    x1, y1, x2, y2 = boxes.T
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep: list[int] = []
    while order.size:
        best = int(order[0])
        keep.append(best)
        rest = order[1:]
        inter_w = np.maximum(0.0, np.minimum(x2[best], x2[rest]) - np.maximum(x1[best], x1[rest]))
        inter_h = np.maximum(0.0, np.minimum(y2[best], y2[rest]) - np.maximum(y1[best], y1[rest]))
        inter = inter_w * inter_h
        overlap = inter / (areas[best] + areas[rest] - inter + 1e-9)
        order = rest[overlap <= threshold]
    return keep


def summarize(detections: list[dict[str, Any]], turkish: bool = False) -> str:
    """One line such as "person x1 (center, very close, lying down); chair x2"."""
    if not detections:
        return "tanınan nesne yok" if turkish else "no known object detected"
    parts: list[str] = []
    counts: dict[str, int] = {}
    for item in detections:
        if item["label"] == "person":
            if turkish:
                words = [_POSITION[item["pos"]], _DISTANCE[item["dist"]]]
                if item.get("posture") == "lying down":
                    words.append(_POSTURE["lying down"])
                parts.append(f"insan ({', '.join(words)})")
            else:
                words = [item["pos"], item["dist"]]
                if item.get("posture") == "lying down":
                    words.append("possibly lying down")
                parts.append(f"person ({', '.join(words)})")
        else:
            key = item["label_tr"] if turkish else item["label"]
            counts[key] = counts.get(key, 0) + 1
    parts += [f"{name} x{count}" if count > 1 else name for name, count in counts.items()]
    return "; ".join(parts)


class SceneWatcher:
    """Background loop: snapshot -> detector -> shared scene state."""

    def __init__(
        self, snapshot_url: str, model: Path, interval: float = 1.0, threads: int = 2,
        busy: Any = None,
    ) -> None:
        self.snapshot_url = snapshot_url
        self.model = model
        self.interval = max(0.2, interval)
        self.threads = threads
        self.busy = busy  # callable: True while the VLM is generating
        self.detector: YoloxDetector | None = None
        self.error: str | None = None
        self._lock = threading.Lock()
        self._detections: list[dict[str, Any]] = []
        self._recent_people: deque[int] = deque(maxlen=3)
        self._updated = 0.0
        self._infer_ms: float | None = None
        self._infrared = False
        self._frames = 0

    def start(self) -> None:
        try:
            self.detector = YoloxDetector(self.model, threads=self.threads)
        except Exception as exc:  # model or onnxruntime missing: assistant still works
            self.error = f"{type(exc).__name__}: {exc}"
            return
        threading.Thread(target=self._loop, name="apex-scene", daemon=True).start()

    def fetch(self) -> tuple[Any, bool]:
        from PIL import Image

        request = urllib.request.Request(self.snapshot_url, headers={"Cache-Control": "no-cache"})
        with urllib.request.urlopen(request, timeout=3.0) as response:
            payload = response.read(12 * 1024 * 1024)
        with Image.open(io.BytesIO(payload)) as image:
            image.load()
            return prepare_frame(image)

    def _loop(self) -> None:
        assert self.detector is not None
        while True:
            started = time.monotonic()
            try:
                image, infrared = self.fetch()
                detections = self.detector.detect(image)
                with self._lock:
                    self._detections = detections
                    self._recent_people.append(sum(1 for d in detections if d["label"] == "person"))
                    self._updated = time.monotonic()
                    self._infer_ms = (self._updated - started) * 1000.0
                    self._infrared = infrared
                    self._frames += 1
                self.error = None
            except Exception as exc:
                self.error = f"{type(exc).__name__}: {exc}"
            # While the Hailo VLM is generating, scan three times slower to
            # keep the peak power draw and CPU contention down.
            interval = self.interval * 3 if (self.busy is not None and self.busy()) else self.interval
            time.sleep(max(0.05, interval - (time.monotonic() - started)))

    def detections(self, max_age: float = 5.0) -> list[dict[str, Any]] | None:
        """Latest detections, or None when the detector is off or stale."""
        with self._lock:
            if self.detector is None or not self._updated or time.monotonic() - self._updated > max_age + 5:
                return None
            return list(self._detections)

    def state(self) -> dict[str, Any]:
        with self._lock:
            age = (time.monotonic() - self._updated) if self._updated else None
            fresh = age is not None and age < 10.0
            detections = list(self._detections) if fresh else []
            people_now = sum(1 for d in detections if d["label"] == "person")
            # A person seen in any of the last three frames still counts
            # (a single missed frame must not clear a human-presence alert).
            people = max([people_now, *self._recent_people]) if fresh else 0
            return {
                "ok": self.detector is not None and fresh,
                "enabled": self.detector is not None,
                "error": self.error,
                "age_s": round(age, 2) if age is not None else None,
                "infer_ms": round(self._infer_ms, 1) if self._infer_ms is not None else None,
                "infrared": self._infrared,
                "frames": self._frames,
                "people": people,
                "objects": detections,
                "summary_tr": summarize(detections, turkish=True) if fresh else "algılayıcı verisi yok",
            }
