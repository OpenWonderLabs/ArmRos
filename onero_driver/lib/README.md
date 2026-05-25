# OneroArm 运行库与第三方组件

此目录包含 `onero_driver` 运行所需的预编译运行库，包括 `libonero_api.so` 以及其依赖的第三方动态库。

## 文件说明

```text
lib/
├── lib_install.sh
└── linux_x86_64_v1.0.0/
    ├── libonero_api.so
    └── third_party/lib/x86_64-linux-gnu/
        ├── librbdl.so*
        ├── libpinocchio_*.so*
        └── libhpp-fcl.so
```

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
