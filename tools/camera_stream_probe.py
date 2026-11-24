#!/usr/bin/env python3
"""Measure an MJPEG stream without decoding frames."""

from __future__ import annotations

import argparse
import statistics
import time
import urllib.request


def probe(url: str, duration: float) -> tuple[int, int, float, list[float]]:
    request = urllib.request.Request(url, headers={"User-Agent": "APEX-stream-probe/1"})
    frames = 0
    total_bytes = 0
    tail = b""
    frame_times: list[float] = []
    started = time.monotonic()
    with urllib.request.urlopen(request, timeout=max(5.0, duration + 2.0)) as response:
        content_type = response.headers.get("Content-Type", "")
        marker = "boundary="
        boundary_name = content_type.split(marker, 1)[1].split(";", 1)[0].strip().strip('"') if marker in content_type else "FRAME"
        boundary = ("--" + boundary_name).encode("ascii", "replace")
        while time.monotonic() - started < duration:
            chunk = response.read(65536)
            if not chunk:
                break
            total_bytes += len(chunk)
            data = tail + chunk
            found = data.count(boundary)
            frames += found
            if found:
                now = time.monotonic()
                frame_times.extend([now] * found)
            tail = data[-(len(boundary) - 1) :]
    elapsed = time.monotonic() - started
    gaps = [b - a for a, b in zip(frame_times, frame_times[1:]) if b > a]
    return frames, total_bytes, elapsed, gaps


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("url")
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()
    frames, total_bytes, elapsed, gaps = probe(args.url, args.duration)
    print(f"frames={frames}")
    print(f"elapsed_s={elapsed:.3f}")
    print(f"fps={frames / elapsed:.2f}")
    print(f"mbps={(total_bytes * 8 / elapsed) / 1_000_000:.2f}")
    if gaps:
        print(f"gap_mean_ms={statistics.fmean(gaps) * 1000:.1f}")
        print(f"gap_p95_ms={sorted(gaps)[min(len(gaps) - 1, int(len(gaps) * 0.95))] * 1000:.1f}")
        print(f"gap_max_ms={max(gaps) * 1000:.1f}")


if __name__ == "__main__":
    main()
