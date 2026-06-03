# 机械臂状态消息

# 时间戳
builtin_interfaces/Time stamp

# 机械臂型号
string robot_model

# 当前关节位置（弧度）
float32[] joint_positions

# 当前关节速度（rad/s）
float32[] joint_velocities

# 当前关节力矩（N·m）
float32[] joint_torques

# 当前末端位姿
geometry_msgs/Pose end_effector_pose

# 机械臂状态
# 0: 空闲
# 1: 运动中
# 2: 错误
uint8 status

# 错误码（status=2时有效）
int32 error_code

# 错误描述
string error_message

