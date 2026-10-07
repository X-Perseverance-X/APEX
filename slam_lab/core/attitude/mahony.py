"""Gyro integration with gravity feedback; yaw is relative and drifts."""

from __future__ import annotations

import math

from core.frames.geometry import Quaternion


class AttitudeEstimator:
    def __init__(self, gravity_gain: float = 1.0) -> None:
        self.q = Quaternion()
        self.gravity_gain = gravity_gain
        self.bias_rad_s = (0.0, 0.0, 0.0)
        self.last_time_ns: int | None = None

    def update(
        self,
        timestamp_ns: int,
        accel_body_m_s2: tuple[float, float, float],
        gyro_body_rad_s: tuple[float, float, float],
    ) -> Quaternion:
        if self.last_time_ns is None:
            self.last_time_ns = timestamp_ns
            return self.q
        dt = (timestamp_ns - self.last_time_ns) * 1e-9
        self.last_time_ns = timestamp_ns
        if dt <= 0 or dt > 0.5:
            return self.q

        gx, gy, gz = (g - b for g, b in zip(gyro_body_rad_s, self.bias_rad_s))
        norm = math.sqrt(sum(v * v for v in accel_body_m_s2))
        if 7.0 <= norm <= 12.5:
            measured = tuple(v / norm for v in accel_body_m_s2)
            # Body-frame direction of world +z predicted by current attitude.
            q = self.q.normalized()
            predicted = Quaternion(q.w, -q.x, -q.y, -q.z).rotate((0.0, 0.0, 1.0))
            error = (
                measured[1] * predicted[2] - measured[2] * predicted[1],
                measured[2] * predicted[0] - measured[0] * predicted[2],
                measured[0] * predicted[1] - measured[1] * predicted[0],
            )
            gx += self.gravity_gain * error[0]
            gy += self.gravity_gain * error[1]
            gz += self.gravity_gain * error[2]

        qdot = self.q * Quaternion(0.0, gx, gy, gz)
        self.q = Quaternion(
            self.q.w + 0.5 * qdot.w * dt,
            self.q.x + 0.5 * qdot.x * dt,
            self.q.y + 0.5 * qdot.y * dt,
            self.q.z + 0.5 * qdot.z * dt,
        ).normalized()
        return self.q

