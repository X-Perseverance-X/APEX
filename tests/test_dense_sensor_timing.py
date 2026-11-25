"""Camera and LiDAR timestamps must be compared in a shared time domain."""

import pytest

from tools.live_dense_rgbd_worker import sensor_pair_skew_s


def test_sensor_age_compensates_for_sequential_http_requests():
    # At LiDAR response, the scan is 100 ms old. The camera response arrives
    # 60 ms later and carries a 170 ms frame-age value: only 10 ms apart.
    assert sensor_pair_skew_s(0.100, 0.170, 0.060) == pytest.approx(0.010)


def test_sensor_age_rejects_invalid_values():
    with pytest.raises(ValueError):
        sensor_pair_skew_s(0.1, float("nan"), 0.06)
