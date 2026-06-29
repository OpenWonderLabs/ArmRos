# 命令执行结果（由驱动节点 publish）

builtin_interfaces/Time stamp

bool success

# 错误码（与 onero_define.h::MoveResult / onero_interface_c.h::ONERO_ERR_* 对齐）：
#   0  = OK
#  -1  = INVALID_PARAMS
#  -2  = IK_FAILED
#  -3  = COLLISION_DETECTED
#  -4  = EXECUTION_FAILED
#  -5  = TIMEOUT
#  -6  = INTERRUPTED
#  -7  = JOINT_LIMIT_EXCEEDED
#  -8  = BUSY  (该臂已有运动命令在执行)
int32 error_code
string error_message
