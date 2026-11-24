Session recording is implemented in `slam_runtime/session.py` to avoid
shadowing Python's standard-library `logging` package. Each run creates a
timestamped directory under `sessions/`.
