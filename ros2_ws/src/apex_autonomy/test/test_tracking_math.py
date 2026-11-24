from apex_autonomy.tracking_math import forward_command, turn_command


def test_forward_stops_inside_follow_distance():
    assert forward_command(1.0, 0.0) == 0.0


def test_forward_reduces_when_target_is_off_axis():
    assert forward_command(2.0, 0.8) < forward_command(2.0, 0.0)


def test_turn_is_gain_scheduled_and_saturated():
    assert abs(turn_command(0.5, 3.0)) > abs(turn_command(0.5, 1.0))
    assert turn_command(1.0, 10.0) == 0.85
