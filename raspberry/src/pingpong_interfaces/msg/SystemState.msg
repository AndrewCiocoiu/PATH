# Mission manager state (doc section 9.2). Published on /system/state.
uint8 BOOT=0
uint8 SELF_TEST=1
uint8 HOMING=2
uint8 IDLE=3
uint8 SEARCH=4
uint8 TRACK=5
uint8 AIM=6
uint8 FIRE=7
uint8 VERIFY=8
uint8 RELOAD=9
uint8 FAULT=10
uint8 ESTOP=11

std_msgs/Header header
uint8 state
string state_name
bool armed
bool estop
uint16 fault_code
string message       # human readable, e.g. why firing is not permitted yet
