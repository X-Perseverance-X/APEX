#!/usr/bin/env python3
"""Run sensor acquisition, session logging and network dashboard."""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main() -> int:
    from slam_runtime.pipeline import LiveRunner
    from visualization.web.server import create_app
    import uvicorn

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-record", action="store_true")
    args = parser.parse_args()
    runner = LiveRunner(record=not args.no_record)
    runner.start()
    if runner.writer:
        print(f"SESSION={runner.writer.directory}", flush=True)
    network = runner.hardware["network"]
    print(f"DASHBOARD=http://<PI_IP>:{network['port']}/", flush=True)
    for warning in runner.processor.state["warnings"]:
        print(warning, flush=True)

    # Uvicorn 0.54 restores and re-raises SIGTERM after its own shutdown.
    # Use a Python exception so our finally block still flushes the session.
    def terminate(_signum, _frame):
        raise SystemExit(143)

    previous_term_handler = signal.signal(signal.SIGTERM, terminate)
    try:
        uvicorn.run(create_app(runner), host=network["bind"], port=network["port"],
                    log_level="warning")
    finally:
        try:
            runner.close()
        finally:
            signal.signal(signal.SIGTERM, previous_term_handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
