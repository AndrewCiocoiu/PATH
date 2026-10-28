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
    assert elevation_angle(3.0, 0.0, 5.0, drag_k=0.14) is None  # 5 m/s cannot reach 3 m with drag


def simulate_height(d, elev, v, k, dt=1e-5):
    """2-D simulation with quadratic drag: height and time when the ball reaches d."""
    x = y = t = 0.0
    vx, vy = v * math.cos(elev), v * math.sin(elev)
    while x < d:
        s = math.hypot(vx, vy)
        vx -= k * s * vx * dt
        vy -= (G + k * s * vy) * dt
        x += vx * dt
        y += vy * dt
        t += dt
    return y, t


def test_drag_model_matches_simulation():
    v, k = 7.9, 0.14  # 40 mm ball, launcher_calculations.tex
    for d in (1.0, 2.0, 3.0):
        a = elevation_angle(d, 0.0, v, drag_k=k)
        y, t = simulate_height(d, a, v, k)
        assert abs(math.degrees(math.atan2(y, d))) < 0.3, d   # hits within 0.3 deg
        assert abs(time_of_flight(d, a, v, drag_k=k) - t) < 0.01, d


def test_drag_raises_the_barrel():
    assert elevation_angle(3.0, 0.0, 7.9, drag_k=0.14) > elevation_angle(3.0, 0.0, 7.9) + 0.08


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
