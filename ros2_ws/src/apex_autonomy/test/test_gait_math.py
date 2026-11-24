import math
import pytest

from apex_autonomy.gait_math import solve_leg_ik, tripod_targets


MOUNTS = [math.radians(value) for value in (-61.699, -90.0, -118.301, 61.699, 90.0, 118.301)]
PHASES = [0.0, 0.5, 0.0, 0.5, 0.0, 0.5]


def test_firmware_stand_matches_existing_urdf_angles():
    coxa, femur, tibia = solve_leg_ik(MOUNTS[0], 0.150, 0.0, 0.0, -0.100)
    assert abs(coxa) < 1e-9
    assert femur == pytest.approx(0.723981492, abs=1e-6)
    assert tibia == pytest.approx(-2.276991532, abs=1e-6)


def test_tripods_are_opposite_groups():
    targets = tripod_targets(0.1, 0.5, 0.0, 0.0, MOUNTS, PHASES)
    swing = [item[1] for item in targets]
    assert swing == [True, False, True, False, True, False]


def test_full_command_stays_inside_urdf_joint_limits():
    for step in range(100):
        targets = tripod_targets(step / 100.0, 1.0, 0.0, 0.6, MOUNTS, PHASES)
        for (coxa, femur, tibia), _swing in targets:
            assert math.radians(-85) <= coxa <= math.radians(85)
            assert math.radians(-85) <= femur <= math.radians(85)
            assert math.radians(-175) <= tibia <= math.radians(-5)
