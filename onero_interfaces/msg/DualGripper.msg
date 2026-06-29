# DualGripper - 双臂夹爪状态/命令复用消息
# 字段命名与 DualMoveJ 一致（展开 left/right，不嵌套）
#
# 命令场景：driver 仅读取 left_position / right_position
# 状态场景：driver 周期发布到 /onero_arm/dual_arm/gripper_state 时填充全部 8 个字段

float32 left_position       # 0~100 %（0=全闭, 100=全开）
float32 left_velocity       # %/s
float32 left_force          # N
int32   left_error_code     # SDK 错误码；0 = 无错误

float32 right_position
float32 right_velocity
float32 right_force
int32   right_error_code
