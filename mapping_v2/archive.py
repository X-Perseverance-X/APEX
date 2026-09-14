"""Atomic, local persistence for completed mapping sessions."""

from __future__ import annotations

import json
import os
import re
import tempfile
import unicodedata
import zipfile
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


MAP_SCHEMA = "apex.mapping.archive.v1"
_SAFE_ID = re.compile(r"^[a-z0-9][a-z0-9_-]{0,79}$")


def _slug(value: str) -> str:
    ascii_value = unicodedata.normalize("NFKD", value).encode("ascii", "ignore").decode("ascii")
    slug = re.sub(r"[^a-z0-9]+", "-", ascii_value.lower()).strip("-")
    return slug[:40] or "harita"


class MapArchive:
    def __init__(self, directory: str | os.PathLike[str]):
        self.directory = Path(directory)

    def _ensure_directory(self) -> None:
        self.directory.mkdir(parents=True, exist_ok=True)

    def _path(self, map_id: str) -> Path:
        if not _SAFE_ID.fullmatch(map_id):
            raise ValueError("geçersiz harita kimliği")
        path = (self.directory / f"{map_id}.npz").resolve()
        directory = self.directory.resolve()
        if path.parent != directory:
            raise ValueError("harita yolu arşiv dışında")
        return path

    def save(self, payload: dict, display_name: str = "Ev Haritası") -> dict:
        self._ensure_directory()
        now = datetime.now(timezone.utc)
        map_id = f"{now.strftime('%Y%m%d-%H%M%S')}-{_slug(display_name)}"
        path = self._path(map_id)
        suffix = 2
        while path.exists():
            map_id = f"{now.strftime('%Y%m%d-%H%M%S')}-{_slug(display_name)}-{suffix}"
            path = self._path(map_id)
            suffix += 1

        log_odds = np.asarray(payload["log_odds"], dtype=np.float32)
        observed = np.asarray(payload["observed"], dtype=np.uint8)
        hit_count = np.asarray(payload["hit_count"], dtype=np.uint16)
        if log_odds.ndim != 2 or observed.shape != log_odds.shape or hit_count.shape != log_odds.shape:
            raise ValueError("harita katman boyutları uyuşmuyor")
        if not np.all(np.isfinite(log_odds)):
            raise ValueError("harita geçersiz sayı içeriyor")

        metadata = dict(payload.get("metadata") or {})
        metadata.update({
            "schema": MAP_SCHEMA,
            "map_id": map_id,
            "display_name": str(display_name).strip()[:80] or "Ev Haritası",
            "saved_at_utc": now.isoformat(),
            "width": int(log_odds.shape[1]),
            "height": int(log_odds.shape[0]),
        })
        fd, temporary_name = tempfile.mkstemp(prefix=f".{map_id}-", suffix=".tmp", dir=self.directory)
        try:
            with os.fdopen(fd, "wb") as handle:
                np.savez_compressed(
                    handle,
                    log_odds=log_odds,
                    observed=observed,
                    hit_count=hit_count,
                    pose=np.asarray(payload["pose"], dtype=np.float64),
                    metadata=np.asarray(json.dumps(metadata, ensure_ascii=False)),
                )
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary_name, path)
        except Exception:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise
        return metadata

    def load(self, map_id: str) -> dict:
        path = self._path(map_id)
        if not path.is_file():
            raise FileNotFoundError("harita bulunamadı")
        if path.stat().st_size > 32 * 1024 * 1024:
            raise ValueError("harita dosyası beklenen sınırdan büyük")
        try:
            with zipfile.ZipFile(path) as bundle:
                if sum(item.file_size for item in bundle.infolist()) > 8 * 1024 * 1024:
                    raise ValueError("harita arşivinin açılmış boyutu beklenen sınırdan büyük")
        except zipfile.BadZipFile as exc:
            raise ValueError("harita arşivi bozuk") from exc
        with np.load(path, allow_pickle=False) as archive:
            required = {"log_odds", "observed", "hit_count", "pose", "metadata"}
            if not required.issubset(archive.files):
                raise ValueError("harita arşivi eksik")
            metadata = json.loads(str(archive["metadata"].item()))
            if metadata.get("schema") != MAP_SCHEMA or metadata.get("map_id") != map_id:
                raise ValueError("harita şeması/kimliği geçersiz")
            result = {
                "metadata": metadata,
                "log_odds": np.asarray(archive["log_odds"], dtype=np.float32).copy(),
                "observed": np.asarray(archive["observed"], dtype=np.bool_).copy(),
                "hit_count": np.asarray(archive["hit_count"], dtype=np.uint16).copy(),
                "pose": np.asarray(archive["pose"], dtype=np.float64).copy(),
            }
        shape = result["log_odds"].shape
        if len(shape) != 2 or result["observed"].shape != shape or result["hit_count"].shape != shape:
            raise ValueError("harita katman boyutları uyuşmuyor")
        if shape != (int(metadata["height"]), int(metadata["width"])):
            raise ValueError("harita metadata boyutu uyuşmuyor")
        if not np.all(np.isfinite(result["log_odds"])) or result["pose"].shape != (3,):
            raise ValueError("harita sayısal verisi geçersiz")
        return result

    def load_metadata(self, map_id: str) -> dict:
        """Read only the tiny metadata member; numpy leaves grid arrays lazy."""
        path = self._path(map_id)
        if not path.is_file():
            raise FileNotFoundError("harita bulunamadı")
        if path.stat().st_size > 32 * 1024 * 1024:
            raise ValueError("harita dosyası beklenen sınırdan büyük")
        try:
            with zipfile.ZipFile(path) as bundle:
                if sum(item.file_size for item in bundle.infolist()) > 8 * 1024 * 1024:
                    raise ValueError("harita arşivinin açılmış boyutu beklenen sınırdan büyük")
        except zipfile.BadZipFile as exc:
            raise ValueError("harita arşivi bozuk") from exc
        with np.load(path, allow_pickle=False) as archive:
            if "metadata" not in archive.files:
                raise ValueError("harita metadata kaydı eksik")
            metadata = json.loads(str(archive["metadata"].item()))
        if metadata.get("schema") != MAP_SCHEMA or metadata.get("map_id") != map_id:
            raise ValueError("harita şeması/kimliği geçersiz")
        return metadata

    def list_maps(self) -> list[dict]:
        self._ensure_directory()
        records: list[dict] = []
        for path in sorted(self.directory.glob("*.npz"), reverse=True):
            try:
                records.append(self.load_metadata(path.stem))
            except (OSError, ValueError, KeyError, json.JSONDecodeError):
                continue
        return records
