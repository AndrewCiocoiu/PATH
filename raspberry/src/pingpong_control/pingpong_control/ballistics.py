"""Ballistics for the aim solver (stage S5). Pure Python, no ROS: easy to unit test.

OWNER: Andrei. Muzzle speed is measured in T19.

Air drag: the ball slows down as  dv/dt = -k |v| v,  k = rho * Cd * A / (2 m)
(40 mm ball: k = 0.14 1/m; mechanics/BARREL/40 mm ball/launcher_calculations.tex).
For flat shots (up to ~25 deg) the path has a closed form:
    y(x) = x tan(a) - g / (v cos a)^2 * ((e^(2kx) - 1) / (4k^2) - x / (2k))
which is a quadratic in tan(a). k = 0 gives the drag-free parabola.
Checked against a 2-D simulation: within 0.3 deg up to 3 m (test_logic.py).
"""
import math

G = 9.81


def _drop_factor(distance, drag_k):
    """Integral term of y(x) above, times 1: gives drop = g * factor / (v cos a)^2."""
    if drag_k * distance < 1e-6:
        return distance ** 2 / 2 + drag_k * distance ** 3 / 3
    k = drag_k
    return math.expm1(2 * k * distance) / (4 * k * k) - distance / (2 * k)


def elevation_angle(distance, height, muzzle_speed, g=G, drag_k=0.0):
    """Barrel elevation [rad] to hit a point `distance` m away horizontally and
    `height` m above the barrel exit. Flat (low) trajectory. None if out of reach."""
    if distance <= 0:
        return None
    a = g * _drop_factor(distance, drag_k) / muzzle_speed ** 2
    # a tan^2 - d tan + (a + h) = 0, low solution
    disc = distance ** 2 - 4 * a * (a + height)
    if disc < 0:
        return None
    return math.atan((distance - math.sqrt(disc)) / (2 * a))


def time_of_flight(distance, elevation, muzzle_speed, drag_k=0.0):
    vx = muzzle_speed * math.cos(elevation)
    if drag_k * distance < 1e-6:
        return distance / vx
    return math.expm1(drag_k * distance) / (drag_k * vx)

# TODO(Andrei): once T19 gives real data, replace or correct this with a lookup table
# range -> (elevation, time of flight) measured on the test range.
