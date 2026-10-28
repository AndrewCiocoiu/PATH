"""Run: colcon test --packages-select pingpong_control   (or plain: pytest raspberry/src/pingpong_control)"""
import math

from pingpong_control.ballistics import G, elevation_angle, time_of_flight
from pingpong_control.fire_permission import FireConfig, FireInputs, check


def test_elevation_hits_target():
    v, d, h = 8.0, 2.5, 0.2
    a = elevation_angle(d, h, v)
    t = time_of_flight(d, a, v)
    height_at_target = v * math.sin(a) * t - 0.5 * G * t ** 2
    assert abs(height_at_target - h) < 1e-6


def test_out_of_reach():
    assert elevation_angle(100.0, 0.0, 5.0) is None


def ready_inputs():
    return FireInputs(state_is_aim=True, software_armed=True, hardware_armed=True,
                      heartbeat_ok=True, target_confidence=0.9, track_age=20, aim_ok_cycles=5,
                      range_valid=True, range_m=2.5, launcher_ready=True)


def test_fire_permitted_when_everything_ok():
    assert check(ready_inputs(), FireConfig()) == []


def test_each_condition_blocks():
    cfg = FireConfig(no_fire_zones=[(-0.1, 0.1, -1.0, 1.0)])
    for field, value in [('hardware_armed', False), ('estop', True), ('range_m', 9.0),
                         ('launcher_ready', False), ('time_since_last_shot', 0.5),
                         ('turret_rate', 1.0)]:
        i = ready_inputs()
        setattr(i, field, value)
        assert check(i, cfg), field
    assert check(ready_inputs(), cfg) == ['barrel in no-fire zone']
