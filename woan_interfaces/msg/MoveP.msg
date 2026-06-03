# MoveP - 位姿透传

# 目标位姿
geometry_msgs/Pose pose

# 轨迹连接标志
# 0: 立即规划并执行
# 1: 与下一条轨迹一起规划，不立即执行
uint8 trajectory_connect

# 速度缩放因子 (0.05~5.0)
float32 speed_scale

