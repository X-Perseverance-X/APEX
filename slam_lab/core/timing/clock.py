"""ESP32 micros() unwrap and host receive timestamp checks."""

from __future__ import annotations


class EspClock:
    def __init__(self) -> None:
        self._previous: int | None = None
        self._epoch = 0

    def unwrap_us(self, micros32: int) -> int:
        if not 0 <= micros32 <= 0xFFFFFFFF:
            raise ValueError("ESP timestamp must be uint32 microseconds")
        if self._previous is not None:
            if micros32 < self._previous and self._previous - micros32 > 0x80000000:
                self._epoch += 1 << 32
            elif micros32 < self._previous:
                raise ValueError("ESP timestamp moved backwards without rollover")
        self._previous = micros32
        return self._epoch + micros32


def ordered_ns(previous: int | None, current: int) -> bool:
    return current >= 0 and (previous is None or current > previous)

