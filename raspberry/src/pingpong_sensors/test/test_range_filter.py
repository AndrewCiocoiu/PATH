"""Run: colcon test --packages-select pingpong_sensors   (or plain: pytest raspberry/src/pingpong_sensors)"""
from pingpong_sensors.range_filter import RangeFilter


def test_needs_enough_readings():
    f = RangeFilter(window=5, min_valid=3)
    assert f.update(2.0) == (0.0, False)
    assert f.update(2.1) == (0.0, False)
    r, valid = f.update(2.2)
    assert valid and r == 2.1


def test_median_rejects_a_spike():
    f = RangeFilter()
    for m in (2.0, 2.0, 0.3, 2.0, 2.0):  # background or noise for one reading
        r, valid = f.update(m)
    assert valid and r == 2.0


def test_out_of_range_and_missing_readings_count_as_invalid():
    f = RangeFilter(window=5, max_m=4.0, min_valid=3)
    for m in (2.0, None, 9.0, None, 2.0):
        r, valid = f.update(m)
    assert not valid
