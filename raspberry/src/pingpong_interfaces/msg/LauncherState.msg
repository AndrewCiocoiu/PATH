# Launcher state from the ESP32. Published on /launcher/state on change and at 5 Hz.
uint8 EMPTY=0
uint8 LOADING=1
uint8 LOADED=2
uint8 COCKING=3
uint8 READY=4
uint8 FIRING=5
uint8 FIRED=6
uint8 JAM=7

std_msgs/Header header
uint8 state
bool ball_present
bool cocked
bool magazine_empty
bool arm_switch      # hardware arm switch, read by the ESP32
uint32 shots_fired
uint32 last_cmd_seq  # seq of the last accepted LauncherCommand (acknowledge)
