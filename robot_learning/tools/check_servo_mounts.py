"""Check the leg servo cases against the MARK_2 holder meshes. Simulation model only.

For every leg this reports, in millimetres:

* the angle between the servo shaft and the joint axis (must be 0),
* where the joint axis sits along the case's 40 mm length, measured from the
  case centre (a real 40 mm servo has its shaft about 10 mm from the centre),
* how many holder-mesh vertices lie inside the case (must be 0),
* the four mounting-bore centres of the holder, measured in the case frame,
  next to the case's own bore pattern (+-24 mm along the length, +-5 mm across).

Run from the repository root:

    python robot_learning/tools/check_servo_mounts.py
"""

from __future__ import annotations

import sys

import mujoco
import numpy as np

from apex_rl.contract import LEG_ORDER
from apex_rl.model import build_xml

# Bore corners measured on the original right-hand MARK_2 assembly, in the link
# frame, millimetres: (x, z) of the four bolt holes that carry each servo.
BORES = {
    "femur": ("coxa", "femur_DS3240", [(64, -14), (74, -14), (64, 34.1), (74, 34.1)]),
    "tibia": (
        "tibia",
        "tibia_BLS3225",
        [(-9.46, -11.46), (-14.58, -2.87), (31.9, 13.19), (26.78, 21.78)],
    ),
}


def _rotation(quat):
    out = np.empty(9)
    mujoco.mju_quat2Mat(out, quat)
    return out.reshape(3, 3)


def _vertices(model, geom_id):
    mesh = model.geom_dataid[geom_id]
    start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
    vertices = model.mesh_vert[start : start + count]
    # MuJoCo stores mesh vertices in its own aligned frame; the geom pose already
    # carries the offset, so only the geom pose is applied here.
    return vertices @ _rotation(model.geom_quat[geom_id]).T + model.geom_pos[geom_id]


def main() -> int:
    model = mujoco.MjModel.from_xml_string(build_xml())
    failures = 0
    print(f"{'leg':4} {'joint':6} {'axis err':>8} {'shaft off':>9} {'in-case':>8} {'bore err':>9}")
    for leg in LEG_ORDER:
        name = leg.lower()
        side = 1 if leg.startswith("R") else -1
        for joint, (mesh, case, corners) in BORES.items():
            case_id = model.geom(f"{name}_{case}_case").id
            holder = model.geom(f"{name}_{mesh}_cad").id
            axes = _rotation(model.geom_quat[case_id])
            half = model.geom_size[case_id]
            centre = model.geom_pos[case_id]
            joint_axis = model.jnt_axis[model.joint(f"{name}_{joint}_joint").id]
            origin = (
                np.zeros(3)
                if joint == "tibia"
                else np.array([model.body_pos[model.body(f"{name}_femur").id][0], 0, 0])
            )
            height_axis = axes[:, 2]
            axis_error = np.degrees(np.arccos(np.clip(abs(height_axis @ joint_axis), -1, 1)))
            shaft = (origin - centre) @ axes[:, 0] * 1000
            inside = int(
                np.all(
                    np.abs((_vertices(model, holder) - centre) @ axes) < half - 0.0005, axis=1
                ).sum()
            )
            bores = np.array([[x, -20 * side, z] for x, z in corners])
            local = (bores - centre * 1000) @ axes
            pattern = np.linalg.norm(np.subtract(corners[2], corners[0])) / 2
            bore_error = max(
                np.abs(np.abs(local[:, 0]) - pattern).max(), np.abs(np.abs(local[:, 1]) - 5.0).max()
            )
            bad = axis_error > 0.5 or abs(abs(shaft) - 10.0) > 0.5 or inside or bore_error > 0.05
            failures += bool(bad)
            print(
                f"{leg:4} {joint:6} {axis_error:7.2f}° {shaft:+8.2f} {inside:8d} {bore_error:8.3f}"
                + ("  <-- check" if bad else "")
            )
    print(
        "all leg servo cases agree with the holders"
        if not failures
        else f"{failures} case(s) disagree"
    )
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
