from __future__ import annotations

import asyncio
import importlib.util
import unittest
from pathlib import Path

import httpx


ROOT = Path(__file__).resolve().parents[1]
APP_PATH = ROOT / "apex_kinematics_lab" / "app.py"

SPEC = importlib.util.spec_from_file_location("apex_kinematics_backend", APP_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"Kinematics backend yüklenemedi: {APP_PATH}")
BACKEND = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BACKEND)


class _ErrorClient:
    async def get(self, url: str, **_: object) -> httpx.Response:
        request = httpx.Request("GET", url)
        return httpx.Response(502, request=request)


class KinematicsBackendTests(unittest.TestCase):
    def test_cache_freshness_rejects_missing_future_and_stale_samples(self) -> None:
        now = 100.0
        self.assertFalse(BACKEND._cache_is_fresh(0.0, now))
        self.assertFalse(BACKEND._cache_is_fresh(now + 0.01, now))
        self.assertFalse(BACKEND._cache_is_fresh(now - BACKEND.LIVE_STALE_S - 0.01, now))
        self.assertTrue(BACKEND._cache_is_fresh(now - BACKEND.LIVE_STALE_S / 2, now))

    def test_empty_exception_message_uses_exception_class(self) -> None:
        self.assertEqual(BACKEND._exception_text(TimeoutError()), "TimeoutError")
        self.assertEqual(BACKEND._exception_text(RuntimeError("upstream kapalı")), "upstream kapalı")

    def test_upstream_http_error_is_visible(self) -> None:
        original = BACKEND._live_client

        async def fake_live_client() -> _ErrorClient:
            return _ErrorClient()

        BACKEND._live_client = fake_live_client
        try:
            with self.assertRaisesRegex(RuntimeError, r"/telemetry HTTP 502"):
                asyncio.run(BACKEND._get_json("/telemetry"))
        finally:
            BACKEND._live_client = original


if __name__ == "__main__":
    unittest.main()
