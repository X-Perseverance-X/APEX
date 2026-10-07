#!/usr/bin/env python3
"""Reprocess saved IMU/scans into fresh odometry and occupancy state."""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from replay.session import merged_events
from slam_runtime.pipeline import Processor


class ReplayRunner:
    """Drive the same processor used live, but from immutable session files."""

    def __init__(self, session: Path, processor: Processor, speed: float) -> None:
        self.session = session
        self.processor = processor
        self.speed = speed
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.processor.state["playback"] = {"status": "READY", "speed": speed,
                                            "session": session.name}

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, name="session-replay", daemon=True)
        self.thread.start()

    def _run(self) -> None:
        previous_ns = None
        try:
            for stamp_ns, kind, item in merged_events(self.session):
                if previous_ns is not None:
                    delay = max(0.0, (stamp_ns - previous_ns) * 1e-9 / self.speed)
                    if self.stop_event.wait(delay):
                        break
                elif self.stop_event.is_set():
                    break
                with self.processor.lock:
                    self.processor.state["playback"]["status"] = "PLAYING"
                if kind == "imu":
                    self.processor.process_imu(item)
                else:
                    self.processor.process_scan(item)
                previous_ns = stamp_ns
            with self.processor.lock:
                self.processor.state["playback"]["status"] = (
                    "STOPPED" if self.stop_event.is_set() else "COMPLETE"
                )
        except Exception as error:
            with self.processor.lock:
                self.processor.state["playback"]["status"] = f"ERROR: {error}"

    def close(self) -> None:
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=3)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    parser.add_argument("--serve", action="store_true", help="Replay into the live dashboard without sensors")
    parser.add_argument("--port", type=int, default=8766, help="Dashboard port with --serve")
    parser.add_argument("--speed", type=float, default=1.0, help="Replay speed multiplier")
    args = parser.parse_args()
    if args.speed <= 0:
        parser.error("--speed must be positive")
    session = args.session.resolve()
    hardware = yaml.safe_load((session / "hardware_snapshot.yaml").read_text())
    frames = yaml.safe_load((session / "frames_snapshot.yaml").read_text())
    calibration = yaml.safe_load((session / "calibration_snapshot.yaml").read_text())
    processor = Processor(hardware, frames, calibration)
    if args.serve:
        import uvicorn
        from visualization.web.server import create_app

        runner = ReplayRunner(session, processor, args.speed)
        runner.start()
        print(f"REPLAY=http://<PI_IP>:{args.port}/  session={session.name} speed={args.speed}x",
              flush=True)
        try:
            uvicorn.run(create_app(runner), host=hardware["network"]["bind"],
                        port=args.port, log_level="warning")
        finally:
            runner.close()
        return 0
    counts = {"imu": 0, "lidar": 0}
    for _, kind, item in merged_events(session):
        counts[kind] += 1
        if kind == "imu":
            processor.process_imu(item)
        else:
            processor.process_scan(item)
    snapshot = processor.snapshot()
    print(json.dumps({
        "session": str(session), "counts": counts,
        "mapping_status": snapshot["slam"]["mapping_status"],
        "pose": {key: snapshot["slam"][key] for key in ("x_m", "y_m", "yaw_deg")},
        "map_cell_count": len(snapshot["slam"]["map_cells"]),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
