"""Measure structural support and impacts in the offline stair task."""

from __future__ import annotations

import mujoco
import numpy as np


class BodySupport:
    def __init__(self, model, chassis, terrain):
        self.chassis = np.isin(np.arange(model.ngeom), chassis)
        self.terrain = np.isin(np.arange(model.ngeom), terrain)
        self.wrench = np.zeros(6)
        self.reset(model)

    def reset(self, model):
        self.weight_n = float(model.body_mass.sum() * abs(model.opt.gravity[2]))
        self.xy = None
        self.support_load = self.peak_support_load = self.peak_impact_load = 0.0
        self.support_steps = self.side_steps = self.side_streak = 0
        self.hard_impact = False

    def observe(self, model, data, contacts):
        self.xy = None
        self.support_load = 0.0
        a, b = contacts.geom.T
        chassis_pair = self.chassis[a] | self.chassis[b]
        i = np.flatnonzero(chassis_pair & (contacts.dist <= 0.001))
        upward = 0.0
        weighted_xy = np.zeros(2) if len(i) else None
        impact = 0.0
        side = False
        for j in i:
            c = contacts[int(j)]
            other = b[j] if self.chassis[a[j]] else a[j]
            # MuJoCo's normal points from geom 1 to geom 2.
            nz = float(c.frame[2]) * (-1 if self.chassis[a[j]] else 1)
            mujoco.mj_contactForce(model, data, int(j), self.wrench)
            force = float(self.wrench[0]) / self.weight_n
            if self.terrain[other] and nz > 0.72 and c.pos[2] < data.qpos[2] - 0.012:
                upward += force * nz
                weighted_xy += force * nz * c.pos[:2]
            else:
                side |= force > 0.045
                impact += force
        self.support_load = upward
        if upward >= 0.05:
            self.xy = weighted_xy / upward
            self.support_steps += 1
        self.side_steps += side
        self.side_streak = self.side_streak + 1 if side else 0
        self.peak_support_load = max(self.peak_support_load, upward)
        self.peak_impact_load = max(self.peak_impact_load, impact)
        dt = model.opt.timestep
        self.hard_impact |= impact > 1.50 or upward > 2.30 or self.side_streak * dt > 0.36
