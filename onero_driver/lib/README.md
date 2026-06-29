# OneroArm 运行库与第三方组件

此目录包含 `onero_driver` 运行所需的预编译运行库，包括 `libonero_api.so` 以及其依赖的第三方动态库。

## 文件说明

```text
lib/
├── lib_install.sh
├── libonero_api.so                       # 指向当前架构的 .so（安装脚本生成）
├── linux_x86_64_v1.0.0/
│   ├── libonero_api.so
│   └── third_party/lib/x86_64-linux-gnu/
│       ├── librbdl.so*
│       ├── libpinocchio_*.so*
│       └── libhpp-fcl.so
├── linux_aarch64_v1.0.0/
│   ├── libonero_api.so
│   └── third_party/lib/aarch64-linux-gnu/
│       ├── librbdl.so*
│       ├── libpinocchio_*.so*
│       └── libhpp-fcl.so
└── linux_riscv64_v1.0.0/
    ├── libonero_api.so
    └── third_party/lib/riscv64-linux-gnu/
        ├── librbdl.so*
        ├── libpinocchio_*.so*
        └── libhpp-fcl.so
```

## RISC-V (riscv64) 编译说明

riscv64 工具链较新（GCC 15 + Boost 1.90），编译时需注意以下两点，否则会失败。

**1. 先 source 预装 ROS2 Humble**：

```bash
source /opt/ros/humble/setup.bash
```

**2. 编译整个工作区时，裸跑 `colcon build` 会失败**，需加 `-include cstdint` 并跳过本板编不了的包：

```bash
colcon build \
  --packages-skip onero_gazebo onero_control a1_l_config a1_r_config dual_config onero_bringup \
  --cmake-args -DCMAKE_CXX_FLAGS="-include cstdint"
```

- `-include cstdint`：GCC 15 下 ROS2 生成代码普遍缺 `<cstdint>`，会报 `'uint8_t' does not name a type`，全局预包含即可解决。
- `--packages-skip`：`onero_gazebo`（缺 gazebo）及 moveit2 线相关包在本板需额外依赖。

## 第三方组件许可

`libonero_api.so` 为 OneRobotics 自有运行库，采用 MIT License（详见 `../../LICENSES/LICENSE`）。随包携带的第三方组件保留其上游许可，**不随本工程变更为 MIT**。

下表给出每个 `.so` 对应的组件名与许可证类型；许可证完整正文统一存放于 `../../LICENSES/third_party/` 下：

| 组件 | 文件 | 上游许可 | 许可证全文 |
| ---- | ---- | -------- | ---------- |
| RBDL | `librbdl.so*`, `librbdl_urdfreader.so*` | zlib License | `../../LICENSES/third_party/RBDL.LICENSE` |
| Pinocchio | `libpinocchio_default.so*`, `libpinocchio_collision.so*`, `libpinocchio_parsers.so*` | BSD 2-Clause | `../../LICENSES/third_party/Pinocchio.LICENSE` |
| Pinocchio (子组件) | `pinocchio/math/multiprecision.hpp` | Boost Software License 1.0 | `../../LICENSES/third_party/Boost.LICENSE` |
| hpp-fcl | `libhpp-fcl.so` | BSD License | `../../LICENSES/third_party/hpp-fcl.LICENSE` |
| hpp-fcl (子组件) | `hpp/fcl/collision_utility.h` | LGPL-3.0-or-later | `../../LICENSES/third_party/hpp-fcl.LGPL-3.0.txt` |

> ⚠️ **LGPL-3.0 重分发义务**：分发本运行库时，必须随附 `../../LICENSES/third_party/hpp-fcl.LGPL-3.0.txt` 完整文本，并提供 hpp-fcl 上游源码获取方式（https://github.com/humanoid-path-planner/hpp-fcl ）。`libonero_api.so` 与 `libhpp-fcl.so` 之间为动态链接，符合 LGPL 动态链接条件。

发布源码包、二进制包或 SDK 时，请保留上述第三方组件的版权声明、许可文本和免责声明，并随附整个 `../../LICENSES/` 目录。
