# A fault report. Published on /system/faults.
# The same codes are used in the ESP32 firmware. Add new codes at the end, never renumber.
uint8 WARNING=0
uint8 ERROR=1      # mission manager goes to FAULT
uint8 FATAL=2

uint16 ESTOP=1             # emergency stop pressed (mission manager goes to ESTOP)
uint16 HEARTBEAT_LOST=2    # ESP32 got no TurretCommand for 250 ms
uint16 HOMING_TIMEOUT=3
uint16 LAUNCHER_JAM=4
uint16 NODE_SILENT=5       # a topic stopped arriving (safety_monitor_node)
uint16 SELF_TEST_FAILED=6
uint16 FIRE_TIMEOUT=7      # no FIRED within 1 s after FIRE
uint16 BOOT_TIMEOUT=8
uint16 FIRE_REFUSED=9      # ESP32 interlock refused a FIRE (WARNING)
uint16 AT_LIMIT=10         # axis hit a soft limit (WARNING)

std_msgs/Header header
uint16 code
uint8 severity
string source      # node or module that raised it
string message
bool active        # false = fault cleared
