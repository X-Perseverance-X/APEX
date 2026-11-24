#!/usr/bin/env python3
"""Read one APEX LiDAR WebSocket message without issuing commands."""

from __future__ import annotations

import argparse
import asyncio
import json

import websockets


async def probe(url: str, timeout: float) -> None:
    async with websockets.connect(url, open_timeout=timeout) as socket:
        raw = await asyncio.wait_for(socket.recv(), timeout)
    message = json.loads(raw)
    summary = {key: message.get(key) for key in ("type", "online", "enabled", "applied", "slam")}
    summary["points"] = len(message.get("points") or [])
    summary["map_points"] = len(message.get("map_points") or [])
    print(json.dumps(summary, ensure_ascii=False))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("url")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    asyncio.run(probe(args.url, args.timeout))


if __name__ == "__main__":
    main()
