# Bt state
uint8 IDLE=0
uint8 RUNNING=1
uint8 SUCCESS=2
uint8 FAILURE=3
uint8 UNKNOWN=4
# Bt msg
std_msgs/Header header
string behavior_xml
uint8 state