# MoveJ - 关节空间运动

# 目标关节角度（弧度）
float32[7] joint_positions

# 轨迹连接标志
# 0: 立即规划并执行
# 1: 与下一条轨迹一起规划，不立即执行
uint8 trajectory_connect

# 速度缩放因子（0.01~5.0）
# - < 1.0: 降速
# - = 1.0: 正常速度
# - > 1.0: 提速
float32 speed_scale
