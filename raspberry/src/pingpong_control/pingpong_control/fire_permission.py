"""Fire permission (doc section 10). Pure Python, no ROS: easy to unit test.

OWNER: Andrei.
check() returns the list of conditions that are NOT met. Empty list = fire permitted.
The mission manager shows the first reason in /system/state.message, which makes
"why doesn't it shoot?" easy to answer during testing.

Conditions marked (ESP32) are checked AGAIN in the firmware, independently of the Pi.
"""
from dataclasses import dataclass, field


@dataclass
class FireConfig:
    min_confidence: float = 0.6
    min_track_age: int = 10
    max_aim_error_rad: float = 0.0087        # 0.5 degrees
    min_aim_ok_cycles: int = 3
    range_min_m: float = 1.0
    range_max_m: float = 4.0
    max_settle_rate: float = 0.05            # rad/s
    moving_target_rate: float = 0.1          # rad/s, above this the target counts as moving
    cooldown_s: float = 2.0
    no_fire_zones: list = field(default_factory=list)  # [(yaw_min, yaw_max, pitch_min, pitch_max)]


@dataclass
class FireInputs:
    state_is_aim: bool = False
    software_armed: bool = False
    hardware_armed: bool = False
    estop: bool = False
    fault_active: bool = False
    heartbeat_ok: bool = False
    target_confidence: float = 0.0
    track_age: int = 0
    aim_ok_cycles: int = 0
    range_valid: bool = False
    range_m: float = 0.0
    turret_rate: float = 0.0                 # largest axis speed [rad/s]
    target_rate: float = 0.0                 # largest target angular speed [rad/s]
    launcher_ready: bool = False
    barrel_yaw: float = 0.0
    barrel_pitch: float = 0.0
    time_since_last_shot: float = 1e9


def in_no_fire_zone(yaw, pitch, zones):
    return any(y0 <= yaw <= y1 and p0 <= pitch <= p1 for (y0, y1, p0, p1) in zones)


def check(i: FireInputs, c: FireConfig):
    failed = []
    if not i.state_is_aim:
        failed.append('state is not AIM')
    if not i.hardware_armed:
        failed.append('hardware arm switch OFF')
    if not i.software_armed:
        failed.append('software not armed')
    if i.estop:
        failed.append('emergency stop active')
    if i.fault_active:
        failed.append('fault active')
    if not i.heartbeat_ok:
        failed.append('ESP32 heartbeat not healthy')
    if i.target_confidence < c.min_confidence:
        failed.append(f'target confidence {i.target_confidence:.2f} < {c.min_confidence}')
    if i.track_age < c.min_track_age:
        failed.append(f'track age {i.track_age} < {c.min_track_age}')
    if i.aim_ok_cycles < c.min_aim_ok_cycles:
        failed.append('aim error not small for enough cycles')
    if not i.range_valid:
        failed.append('range not valid')
    elif not c.range_min_m <= i.range_m <= c.range_max_m:
        failed.append(f'range {i.range_m:.2f} m outside window')
    moving = i.target_rate > c.moving_target_rate
    if not moving and i.turret_rate > c.max_settle_rate:
        failed.append('turret not settled')
    if not i.launcher_ready:
        failed.append('launcher not READY')
    if in_no_fire_zone(i.barrel_yaw, i.barrel_pitch, c.no_fire_zones):
        failed.append('barrel in no-fire zone')
    if i.time_since_last_shot < c.cooldown_s:
        failed.append('cool-down')
    return failed
