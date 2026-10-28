"""Ballistics for the aim solver (stage S5). Pure Python, no ROS: easy to unit test.

OWNER: Andrei. Muzzle speed is measured in T19; drag is ignored for now.
"""
import math

G = 9.81


def elevation_angle(distance, height, muzzle_speed, g=G):
    """Barrel elevation [rad] to hit a point `distance` m away horizontally and
    `height` m above the barrel exit. Flat (low) trajectory. None if out of reach."""
    v2 = muzzle_speed ** 2
    disc = v2 ** 2 - g * (g * distance ** 2 + 2 * height * v2)
    if disc < 0 or distance <= 0:
        return None
    return math.atan((v2 - math.sqrt(disc)) / (g * distance))


def time_of_flight(distance, elevation, muzzle_speed):
    return distance / (muzzle_speed * math.cos(elevation))

# TODO(Andrei): add air drag (a 2.7 g ball slows down a lot) once T19 gives real data,
# e.g. a lookup table range -> (elevation, time of flight) measured on the test range.
