#!/usr/bin/env python3
"""Read-only USB serial inventory with duplicate identity warnings."""

from __future__ import annotations

import glob
import os
import subprocess
from collections import defaultdict


FIELDS = (
    "ID_VENDOR_ID",
    "ID_MODEL_ID",
    "ID_VENDOR",
    "ID_MODEL",
    "ID_SERIAL",
    "ID_PATH",
    "DEVLINKS",
)


def properties(device: str) -> dict[str, str]:
    result = subprocess.run(
        ["udevadm", "info", "--query=property", f"--name={device}"],
        check=False,
        capture_output=True,
        text=True,
    )
    values: dict[str, str] = {}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition("=")
        if separator and key in FIELDS:
            values[key] = value
    return values


def main() -> int:
    devices = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not devices:
        print("No USB serial devices detected.")
        return 1

    identities: dict[tuple[str, str, str], list[str]] = defaultdict(list)
    for device in devices:
        info = properties(device)
        identity = (
            info.get("ID_VENDOR_ID", "?"),
            info.get("ID_MODEL_ID", "?"),
            info.get("ID_SERIAL", "?"),
        )
        identities[identity].append(device)
        print(f"[{device}]")
        for field in FIELDS:
            print(f"  {field}={info.get(field, '<missing>')}")

    for identity, matches in identities.items():
        if len(matches) > 1:
            joined = ", ".join(matches)
            print(
                "WARNING: duplicate USB identity "
                f"vid:pid:serial={identity}; devices={joined}. "
                "Do not trust /dev/serial/by-id alone; use by-path plus "
                "protocol verification."
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

