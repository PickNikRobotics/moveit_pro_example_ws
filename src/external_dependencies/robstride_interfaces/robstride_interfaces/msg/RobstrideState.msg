std_msgs/Header header

int32 comm_state
int32[] id
bool[] enabled
int32[] run_state
uint32[] fault_bits

int32 COMM_STATE_OK = 0
int32 COMM_STATE_BUS_NOT_OPEN = -1
int32 COMM_STATE_SEND_FAIL = -2

# run_state: 0 = reset, 1 = calibration, 2 = motor (running)
# fault_bits: see RobstrideSDK robstride_sdk::FaultBit for bit meanings
