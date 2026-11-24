"""Start Command Center preconfigured for the hardware-free dummy stack."""

from __future__ import annotations

import os

import uvicorn


def main() -> None:
    os.environ["ESP32_URL"] = "http://127.0.0.1:8081"
    os.environ["CAMERA_STREAM_URL"] = "http://127.0.0.1:8888"
    os.environ["APEX_SIM_MODE"] = "1"
    uvicorn.run("apex_server:app", host="127.0.0.1", port=8080)


if __name__ == "__main__":
    main()
