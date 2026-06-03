#pragma once

#include <stdint.h>
#include <vector>
#include <array>

namespace woan_api {

// Interrupt control: callback returns true when motion should stop (e.g. Ctrl+C).
typedef bool (*woan_interrupt_check_fn)(void* ctx);

// Configuration structure for robot initialization
struct woan_config_t {
    char device[256];          // Serial device path (e.g., "/dev/ttyACM0")
    char robot_model[64];      // Robot model name (e.g., "x1_l", "x1_r")
    int dof=7;                   // Degrees of freedom (6 or 7)
    int baud_rate;             // Serial baud rate (default: 921600)
    char urdf_path[512];       // URDF file path (optional, can be empty for default)
    char version[32] = "";     // Robot version (e.g., "v3.2", "A1")
    // PD控制参数（从配置文件读取）
    double mit_kp[7];          // MIT模式PD参数-比例系数（最多7个关节）
    double mit_kd[7];          // MIT模式PD参数-微分系数（最多7个关节）
    // 中断控制：运动循环中检查，nullptr 表示不检查
    woan_interrupt_check_fn interrupt_check;
    void* interrupt_ctx;
    char model_description_path[512] = "../../woan_description"; // Path to woan_description package
    char slcan_type[32] = "canable"; // SLCAN protocol type ("canable" or "damiao")
    bool is_teleop_leader = false; // Whether this is a teleoperation leader arm
};

// ===== 运动控制默认参数 =====
constexpr double DEFAULT_MAX_VEL = 1.0;       // rad/s
constexpr double DEFAULT_MAX_ACC = DEFAULT_MAX_VEL / 3;       // rad/s²
constexpr double DEFAULT_MAX_JERK = DEFAULT_MAX_ACC / 3;     // rad/s³
constexpr double DEFAULT_TIME_STEP = 0.01;    // s

// ===== PD 参数预设 =====
struct PDGains {
    std::vector<double> kps;
    std::vector<double> kds;
};

// DOF=7 默认增益
inline PDGains GetPDGains7DOF() {
    return {
        // {300.0, 200.0, 200.0, 200.0, 100.0, 200.0, 100.0},
        // {4.5, 4.5, 4.5, 4.5, 1.0, 1.0, 1.0}
        // 水平放置A1
        {300.0, 200.0, 210.0, 210.0, 80.0, 80.0, 80.0},
        {4.0, 4.0, 1.5, 1.5, 0.8, 0.8, 0.8}
        // 主从臂遥操作Kp,Kd参考值
        // {200.0, 200.0, 150.0, 100.0, 20.0, 20.0, 20.0},
        // {5.0, 5.0, 5.0, 2.0, 1.0, 0.2, 0.2}
    };
}

// DOF=6 默认增益
inline PDGains GetPDGains6DOF() {
    return {
        {200.0, 200.0, 100.0, 30.0, 50.0, 20.0},
        {8.0, 8.0, 5.0, 2.0, 2.0, 0.2}
    };
}

// Robot handle (opaque pointer)
typedef struct woan_robot_handle woan_robot_handle;

// Move API result codes
enum class MoveResult : int {
    SUCCESS = 0,
    INVALID_PARAMS = -1,
    IK_FAILED = -2,
    COLLISION_DETECTED = -3,
    EXECUTION_FAILED = -4,
    TIMEOUT = -5,
};

// Motion modes
enum class MotionMode : uint8_t {
    MOVEJ = 0,              // Joint space motion
    MOVEL_STANDARD = 1,     // Cartesian linear (standard PD control)
    MOVEL_MIT = 2,          // Cartesian linear (MIT force control)
    MOVEP = 3,              // Pose transmission
};

// Trajectory connection flag
enum class TrajectoryConnect : uint8_t {
    EXECUTE_NOW = 0,        // Execute immediately
    BUFFER = 1,             // Buffer waypoint, don't execute
};

// Joint target (array)
using JointArray = std::vector<double>;

// Cartesian pose
struct Pose {
    double x, y, z;         // Position (meters)
    double qw, qx, qy, qz;  // Quaternion orientation
};

// Arm state from motors (position with zero bias, velocity and torque without)
struct ArmStateFromMotor {
    JointArray positions;   // Joint positions (rad) with zero bias applied
    JointArray velocities;  // Joint velocities (rad/s)
    JointArray torques;     // Joint torques (N·m)
};

// Trajectory point (for trajectory execution with dynamics)
struct TrajectoryPoint {
    JointArray position;      // Joint positions (radians)
    JointArray velocity;      // Joint velocities (rad/s)
    JointArray acceleration;  // Joint accelerations (rad/s²)
};

}  // namespace woan_api
