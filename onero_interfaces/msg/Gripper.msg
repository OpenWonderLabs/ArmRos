# Gripper - 夹爪状态/命令复用消息
#
# 命令场景：driver 订阅时仅读取 position 字段，作为目标位置 (0~100 %)，
#           其余字段忽略。driver 内部用 move_position 平滑下发。
# 状态场景：driver 周期发布到 *_state 话题时填充所有字段，position 为当前位置。

float32 position        # 0~100 %（0=全闭, 100=全开；默认上电为全开）
float32 velocity        # %/s        （状态字段）
float32 force           # N          （状态字段）
int32   error_code      # SDK 错误码（状态字段；0 = 无错误）
