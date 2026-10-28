"""Range filter for the VL53L1X (stage S2). Pure Python, no ROS: easy to unit test.

OWNER: Razvi. A reading is a distance in metres, or None when the sensor gave nothing.
Output: median of the last valid readings, valid only when most of the window is valid.
"""
from collections import deque
from statistics import median


class RangeFilter:

    def __init__(self, window=5, min_m=0.05, max_m=4.0, min_valid=3):
        self.readings = deque(maxlen=window)
        self.min_m, self.max_m, self.min_valid = min_m, max_m, min_valid

    def update(self, meters):
        """Add one reading. Returns (range_m, valid)."""
        ok = meters is not None and self.min_m <= meters <= self.max_m
        self.readings.append(meters if ok else None)
        good = [r for r in self.readings if r is not None]
        if len(good) < self.min_valid:
            return 0.0, False
        return median(good), True
