import itertools

import mujoco
import pytest

from apex_rl import curriculum
from apex_rl.home_flight import HOME_FLIGHT, HomeFlightCourse
from apex_rl.locomotion import COURSES, LocomotionEnv


def test_home_flight_matches_the_measured_first_step_and_tread_depths():
    course = COURSES["home-flight-up-5"]
    assert isinstance(course, HomeFlightCourse)
    edges = HOME_FLIGHT.lower_edges(course.edge_x)
    assert edges[1] - edges[0] == pytest.approx(0.30)
    assert all(b - a == pytest.approx(0.27) for a, b in itertools.pairwise(edges[1:]))
    assert course.height(edges[0] - 0.01) == 0.0
    assert course.height(edges[0] + 0.01) == pytest.approx(0.20)
    assert course.height(edges[1] + 0.01) == pytest.approx(0.40)
    turn = HOME_FLIGHT.turn_x(course.edge_x)
    # Winders rise in turn around the banister corner, then the upper flight goes left.
    assert course.height(turn + 0.20, -0.40) == pytest.approx(0.20 * 6)
    assert course.height(turn + 0.80, 0.40) == pytest.approx(0.20 * 8)
    assert course.height(turn + 0.45, 0.45 + 0.10) == pytest.approx(0.20 * 9)
    assert HOME_FLIGHT.total_rise == pytest.approx(2.80)


def test_home_flight_scene_loads_with_every_terrain_part_and_keeps_the_stage_list():
    env = LocomotionEnv("home-flight-preview")
    try:
        env.reset(seed=1)
        for name in COURSES["home-flight-preview"].terrain_names:
            assert env.model.geom(name).id >= 0
        assert env.model.geom("course_winder_3").type == mujoco.mjtGeom.mjGEOM_MESH
    finally:
        env.close()
    # The home staircase is measured separately; the training stage list is unchanged.
    assert not any(name.startswith("home-flight") for name in curriculum.STAGES)
