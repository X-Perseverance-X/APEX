#!/usr/bin/env python3
"""API smoke test for the APEX AI assistant (stdlib only; run from the PC or the Pi).

    python tools/ai_assistant_api_test.py --host apex.local --out output/ai_api_after.json

Sends visual, injury (triage), small-talk and live-news questions, records
which of them used the camera and saves the detector state and the prompt trace.  It never triggers alarms and never touches motors.
Works against the old service too, so it can be run before and after a deploy.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request

QUESTIONS = (
    # (question, camera frame expected?, expected intent)
    ("Ne görüyorsun?", True, "visual"),
    ("ne yapıyorum anlat", True, "visual"),
    ("elimle kaç gösteriyorum", True, "visual"),
    ("Etrafında insan var mı?", True, "visual"),
    ("Bileğimde ağrı hissediyorum", False, "triage"),
    ("Şişlik var ama kanama yok. Ne yapmalıyım?", False, "triage"),
    ("Sen kimsin, burada ne yapıyorsun?", False, "chat"),
    ("Bugünkü haberleri anlat", False, "live"),
)


def call(base: str, path: str, body: dict | None = None, timeout: float = 180.0):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(
        base + path, data=data, method="GET" if body is None else "POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, response.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read().decode("utf-8", "replace")


def wait_ready(base: str, seconds: float) -> dict:
    deadline = time.monotonic() + seconds
    last = "no answer"
    while time.monotonic() < deadline:
        try:
            status, text = call(base, "/api/health", timeout=5)
            health = json.loads(text)
            if status == 200 and health.get("ok") and not health.get("busy"):
                return health
            last = text
        except Exception as exc:  # service restarting
            last = str(exc)
        time.sleep(2)
    raise SystemExit(f"Servis hazır değil: {last}")


def chat(base: str, message: str, speak: bool) -> dict:
    started = time.monotonic()
    # Same payload as a browser with the camera box ticked.
    status, text = call(base, "/api/chat", {"message": message, "use_camera": True, "camera": "auto", "speak": speak})
    events = [json.loads(line) for line in text.splitlines() if line.strip()]
    done = next((e for e in events if e.get("type") == "done"), {})
    error = next((e.get("error") for e in events if e.get("type") == "error"), None)
    return {
        "question": message, "http": status, "answer": done.get("text"), "error": error,
        "phase": next((e.get("text") for e in events if e.get("type") == "phase"), None),
        "camera_used": done.get("camera_used"), "intent": done.get("intent"), "critical": done.get("critical"),
        "seconds": done.get("seconds"), "first_sentence_seconds": done.get("first_sentence_seconds"),
        "sentences": sum(1 for e in events if e.get("type") == "delta"),
        "wall_s": round(time.monotonic() - started, 1),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="apex.local")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--wait", type=float, default=150, help="seconds to wait for the service")
    parser.add_argument("--speak", action="store_true", help="also play the answers on the robot speaker")
    parser.add_argument("--out", help="write the full result as JSON")
    args = parser.parse_args()
    base = f"http://{args.host}:{args.port}"

    report: dict = {"base": base, "health_before": wait_ready(base, args.wait), "chats": [], "checks": []}
    new_api = "scene" in report["health_before"]
    for question, expect_camera, expect_intent in QUESTIONS:
        result = chat(base, question, args.speak)
        report["chats"].append(result)
        print(f"\nSORU : {question}\nCEVAP: {result['answer'] or result['error']}\n"
              f"       kamera={result['camera_used']} niyet={result['intent']} süre={result['seconds']} "
              f"ilk_cümle={result['first_sentence_seconds']}")
        if new_api:
            report["checks"].append((f"{question!r} kamera={expect_camera}", result["camera_used"] is expect_camera))
            report["checks"].append((f"{question!r} niyet={expect_intent}", result["intent"] == expect_intent))
        report["checks"].append((f"{question!r} hatasız", result["http"] == 200 and not result["error"] and bool(result["answer"])))
        wait_ready(base, 60)

    if new_api:
        answers = [c["answer"] for c in report["chats"] if c["question"] == "Ne görüyorsun?"]
        report["checks"].append(("'kamera görüyorum' cevabı yok", not any("kamera" in (a or "").lower() for a in answers)))
        report["scene"] = json.loads(call(base, "/api/scene")[1])
        report["trace"] = json.loads(call(base, "/api/trace")[1]).get("trace")
        scene = report["scene"]
        print("\nSAHNE:", scene.get("summary_tr"), f"| insan={scene.get('people')} gece_görüşü={scene.get('infrared')} "
              f"algılama={scene.get('infer_ms')} ms hata={scene.get('error')}")
        report["checks"].append(("nesne algılayıcı çalışıyor", bool(scene.get("ok"))))
    status, _ = call(base, "/api/sound", {"name": "bogus"})
    report["checks"].append(("/api/sound geçersiz ad -> 400", status == 400))
    status, text = call(base, "/api/reset", {})
    report["checks"].append(("/api/reset", status == 200 and json.loads(text).get("ok") is True))
    health = report["health_after"] = wait_ready(base, 60)
    report["checks"].append(("red_alert_ready", bool(health.get("red_alert_ready"))))
    report["checks"].append(("last_error yok", health.get("last_error") is None))
    report["checks"].append(("undervoltage_now yok", not health.get("undervoltage_now")))

    print("\nSAĞLIK:", json.dumps({k: health.get(k) for k in (
        "version", "default_voice", "tts_backend", "tts_backend_note", "audio_available", "voice_ready", "translation_ready",
        "last_error", "tts_error", "power_voltage_v", "power_flags", "undervoltage_now", "undervoltage_seen",
    )}, ensure_ascii=False))
    failed = [name for name, ok in report["checks"] if not ok]
    for name, ok in report["checks"]:
        print(("  OK   " if ok else "  HATA ") + name)
    if not new_api:
        print("  (eski servis sürümü: kamera yönlendirme kontrolleri atlandı)")
    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
