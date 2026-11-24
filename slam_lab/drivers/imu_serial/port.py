"""Open NodeMCU-32S serial without leaving ESP32 auto-reset lines asserted."""

from __future__ import annotations

import serial


def open_esp32_serial(port: str, baudrate: int, timeout: float = 0.5) -> serial.Serial:
    # NodeMCU boards use CP2102 DTR/RTS for EN/GPIO0 auto-reset. Opening a
    # Serial(port=...) applies pyserial's default asserted states immediately;
    # on this board that held the ESP32 silent after a mapper restart. Set both
    # inactive before the OS descriptor is opened.
    stream = serial.Serial(port=None, baudrate=baudrate, timeout=timeout)
    stream.dtr = False
    stream.rts = False
    stream.port = port
    stream.open()
    return stream
