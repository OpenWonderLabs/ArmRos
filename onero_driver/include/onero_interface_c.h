// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

/**
 * @file onero_interface_c.h
 * @brief Pure C Interface for OneroArm (Windows/Linux Compatible)
 *
 * 该头文件保证可被纯 C 编译器解析，仅依赖 <stdint.h>、<stdbool.h>
 * 与下方内联定义的 ONERO_API 导出宏。
 * C++ 类型与实现细节请见 onero_interface_c.cpp。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#if !defined(ONERO_API)
    #if defined(_WIN32)
        #if defined(ONERO_STATIC)
            #define ONERO_API
        #elif defined(oneroarm_c_EXPORTS)
            #define ONERO_API __declspec(dllexport)
        #else
            #define ONERO_API __declspec(dllimport)
        #endif
    #else
        #define ONERO_API __attribute__((visibility("default")))
    #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Constants
// ============================================================================

#define ONERO_MAX_JOINT_COUNT 16
#define ONERO_PATH_MAX 512
#define ONERO_NAME_MAX 64
#define ONERO_VER_MAX 32

// ============================================================================
// Data Types
// ============================================================================

/**
 * @brief Opaque handle for the robot instance
 */
typedef void* onero_handle;

/**
 * @brief C-compatible configuration structure
 *
 * 字段语义与 C++ 侧 onero_config_t（onero_define.h）一一对应，但**不保证字段顺序
 * 与内存布局一致**：本结构由 onero_interface_c.cpp::to_cpp_config() 做按字段拷贝
 * 转换，调用方应通过 onero_create_robot 接口传入而非 reinterpret_cast。
 *
 * 调用方约束：所有未使用的关节槽位（mit_kp/mit_kd 中超出 dof 的元素）必须为 0；
 * 推荐用 `onero_config_t cfg = {0};` 或 `memset(&cfg, 0, sizeof(cfg))` 进行
 * 零初始化后再按需赋值，避免栈/堆未初始化值被误判为"用户传入的 PD 增益"。
 */
typedef struct {
    char device[256];
    char robot_model[64];
    int dof;
    int baud_rate;
    char urdf_path[512];
    char version[32];
    char mount_orientation[32];
    char model_description_path[512];
    double mit_kp[7];
    double mit_kd[7];
} onero_config_t;

/**
 * @brief C-compatible Joint Array
 */
typedef struct {
    double data[ONERO_MAX_JOINT_COUNT];
    int count; // Actual number of joints used (6 or 7)
} onero_joint_array_t;

/**
 * @brief C-compatible Pose
 */
typedef struct {
    double x, y, z;         // Position
    double qw, qx, qy, qz;  // Orientation (Quaternion)
} onero_pose_t;

/**
 * @brief C-compatible arm state read from motors
 *
 * 字段语义对齐 C++ 侧 onero_api::ArmStateFromMotor。
 * 失败路径下 count == 0，调用方应据此判定。
 */
typedef struct {
    onero_joint_array_t positions;
    onero_joint_array_t velocities;
    onero_joint_array_t torques;
} onero_arm_state_t;

/**
 * @brief C-compatible force-position control parameter
 */
typedef struct {
    bool force_position_flag;
    int direction;
    double force;
} onero_force_position_t;

/**
 * @brief C-compatible trajectory point (for send_trajectory)
 */
typedef struct {
    onero_joint_array_t position;
    onero_joint_array_t velocity;
    onero_joint_array_t acceleration;
} onero_traj_point_t;

/**
 * @brief Drag-teaching state codes (mirrors onero_api::DragTeachingState)
 */
typedef enum {
    ONERO_DRAG_TEACHING_IDLE      = 0,
    ONERO_DRAG_TEACHING_RECORDING = 1,
    ONERO_DRAG_TEACHING_REPLAYING = 2,
} onero_drag_teaching_state_t;

/**
 * @brief Opaque handle for the drag-teaching instance
 */
typedef void* onero_drag_teaching_handle;

// ============================================================================
// API Functions
// ============================================================================

// ----------------------------------------------------------------------------
// 返回值约定（C ABI 表，与 onero_interface_cpp.h 中 C++ 注释表对齐）：
//
//   * int 返回 = MoveResult（0 = SUCCESS，负数 = 错误码，详见
//     onero_define.h::MoveResult）：
//       - onero_movej / movel / movep / force_position_movel
//       - onero_send_trajectory_point / onero_send_trajectory
//       - onero_set_force_position_control
//       - onero_stop_force_position_control
//       - onero_cancel_trajectory
//       - onero_execute_buffered_trajectory
//       - onero_clear_trajectory_buffer
//       - onero_enable_motors / onero_disable_motors
//       - onero_drag_teaching_enable_motors
//       - onero_drag_teaching_start_recording / onero_drag_teaching_stop_recording
//       - onero_drag_teaching_start_replay   / onero_drag_teaching_stop_replay
//       - onero_drag_teaching_handle_command
//
//   * bool 返回（true = 成功）：
//       - onero_drag_teaching_initialize
//       - onero_drag_teaching_set_hardware
//       - onero_drag_teaching_is_initialized
//       - onero_is_hardware_connected
//
//   * 数据对象返回（count==0 / 全零 Pose / IDLE 表示失败）：
//       - onero_get_joint_positions / _from_motors
//       - onero_get_joint_velocities
//       - onero_get_arm_state_from_motor / _cached
//       - onero_get_end_effector_pose
//       - onero_drag_teaching_get_state
//
//   * 句柄返回（NULL 表示失败）：
//       - onero_create_robot
//       - onero_drag_teaching_create
// ----------------------------------------------------------------------------

// All functions now prefixed with ONERO_API

// --- Lifecycle ---

ONERO_API onero_handle onero_create_robot(const onero_config_t* config);

ONERO_API void onero_destroy_robot(onero_handle handle);

// --- Control ---

ONERO_API int onero_enable_motors(onero_handle handle);

ONERO_API int onero_disable_motors(onero_handle handle);

// --- Motion ---

ONERO_API int onero_movej(onero_handle handle,
                            const onero_joint_array_t* target,
                            double speed_scale,
                            uint8_t trajectory_connect);

ONERO_API int onero_movel(onero_handle handle,
                            const onero_pose_t* pose,
                            double speed_scale,
                            uint8_t trajectory_connect);

ONERO_API int onero_force_position_movel(onero_handle handle,
                                           const onero_pose_t* pose,
                                           double speed_scale,
                                           uint8_t trajectory_connect);

ONERO_API int onero_movep(onero_handle handle,
                            const onero_pose_t* pose,
                            double speed_scale,
                            uint8_t trajectory_connect);

// --- Buffer Management ---

ONERO_API int onero_execute_buffered_trajectory(onero_handle handle);

ONERO_API int onero_clear_trajectory_buffer(onero_handle handle);

// --- State Query ---

ONERO_API onero_joint_array_t onero_get_joint_positions(onero_handle handle);

ONERO_API onero_joint_array_t onero_get_joint_positions_from_motors(onero_handle handle);

ONERO_API onero_pose_t onero_get_end_effector_pose(onero_handle handle);

ONERO_API int onero_send_trajectory_point(onero_handle handle,
                                            const onero_joint_array_t* positions,
                                            const onero_joint_array_t* velocities);

// --- Force-Position Control ---

ONERO_API int onero_set_force_position_control(onero_handle handle,
                                                   const onero_force_position_t* params);

ONERO_API int onero_stop_force_position_control(onero_handle handle);

// --- Extended State Query ---

ONERO_API onero_joint_array_t onero_get_joint_velocities(onero_handle handle);

ONERO_API onero_arm_state_t onero_get_arm_state_from_motor(onero_handle handle);

ONERO_API onero_arm_state_t onero_get_arm_state_cached(onero_handle handle);

ONERO_API bool onero_is_hardware_connected(onero_handle handle);

// --- Trajectory ---

/**
 * @brief Send a full trajectory (sequence of trajectory points) for execution.
 * @param handle Robot handle.
 * @param points Pointer to an array of onero_traj_point_t (count == @p num_points).
 * @param num_points Number of trajectory points (>= 0).
 * @return 0 on success, negative on error.
 */
ONERO_API int onero_send_trajectory(onero_handle handle,
                                        const onero_traj_point_t* points,
                                        int num_points);

ONERO_API int onero_cancel_trajectory(onero_handle handle);

// --- Raw CAN Frame API ---
//
// 与电机/夹爪共用同一根 SLCAN 串口链路。SDK 会拦截发到保留 ID
// （电机 0x01-0x07 / 夹爪 / 操纵杆 / 0x7FF）的帧并返回错误码，不下发到总线。
// 接收端：所有非保留 ID 的入站帧会派发到已注册回调。
//
// 线程语义：回调在 SDK 接收路径（即调用方运动控制线程）上同步执行；
// 回调内禁止重入 SDK 任何发送/运动控制方法，否则可能与外层 receive() 自锁。
// payload 指针仅在回调执行期间有效；若需保留请自行拷贝。
//
// 错误码（详见 onero_define.h::ONERO_ERR_RAW_FRAME_*）：
//   0 = 成功；负数 = 失败（INVALID_LEN / INVALID_ID / RESERVED_ID /
//   PORT_NOT_OPEN / SEND_FAILED）

/**
 * @brief 用户接收回调签名（C ABI）
 * @param can_id 11-bit 标准帧 ID
 * @param data   payload 指针，仅在本回调执行期间有效（不要持有，需自行拷贝）
 * @param len    payload 长度（0..8）
 * @param user_data 注册时透传的指针（SDK 不解释）
 */
typedef void (*onero_can_frame_callback_t)(uint16_t can_id,
                                           const uint8_t* data,
                                           uint8_t len,
                                           void* user_data);

/**
 * @brief 发送一帧自定义 CAN 报文（11-bit 标准帧）。同步发送：返回时 payload 已被拷贝，调用方可释放 data。
 * @param can_id 11-bit 标准帧 ID，范围 [0, 0x7FF]，且不能落在保留集
 * @param data   payload 指针；len==0 时可为 NULL
 * @param len    payload 长度，0..8
 * @return 0=成功；负数错误码见 onero_define.h
 */
ONERO_API int onero_send_can_frame(onero_handle handle,
                                   uint16_t can_id,
                                   const uint8_t* data,
                                   uint8_t len);

/**
 * @brief 注册接收回调；所有非保留 ID 的入站帧都会派发到 cb。
 *        重复调用会替换前一个回调；cb==NULL 等价于 onero_clear_can_frame_callback。
 */
ONERO_API int onero_register_can_frame_callback(onero_handle handle,
                                                onero_can_frame_callback_t cb,
                                                void* user_data);

/**
 * @brief 清除已注册的接收回调（user_data 不再被引用）。
 */
ONERO_API int onero_clear_can_frame_callback(onero_handle handle);

/**
 * @brief 在 timeout_ms 内主动从 SLCAN 拉帧并 dispatch 到已注册回调。
 *        电机/夹爪帧仍走原解析路径。timeout_ms == 0 等价于一次非阻塞 try-recv。
 *        典型用法：在 onero_movej 等运动控制空闲期主动调用，避免 SLCAN
 *        rx 缓冲区累积。
 */
ONERO_API int onero_pump_can_bus(onero_handle handle, int timeout_ms);

// ============================================================================
// Drag Teaching API
// ============================================================================

ONERO_API onero_drag_teaching_handle onero_drag_teaching_create(void);

ONERO_API void onero_drag_teaching_destroy(onero_drag_teaching_handle handle);

ONERO_API bool onero_drag_teaching_initialize(onero_drag_teaching_handle handle,
                                         int dof,
                                         const char* record_file,
                                         double time_step);

ONERO_API bool onero_drag_teaching_set_hardware(onero_drag_teaching_handle handle,
                                           const char* device,
                                           const char* urdf_path,
                                           const char* robot_model,
                                           const char* mount_orientation);

ONERO_API int onero_drag_teaching_enable_motors(onero_drag_teaching_handle handle,
                                           bool restore_to_zero);

ONERO_API int onero_drag_teaching_start_recording(onero_drag_teaching_handle handle);

ONERO_API int onero_drag_teaching_stop_recording(onero_drag_teaching_handle handle);

ONERO_API void onero_drag_teaching_set_replay_file(onero_drag_teaching_handle handle,
                                              const char* replay_file);

ONERO_API int onero_drag_teaching_start_replay(onero_drag_teaching_handle handle);

ONERO_API int onero_drag_teaching_stop_replay(onero_drag_teaching_handle handle);

ONERO_API int onero_drag_teaching_handle_command(onero_drag_teaching_handle handle, int cmd);

ONERO_API void onero_drag_teaching_timer_callback(onero_drag_teaching_handle handle);

ONERO_API onero_drag_teaching_state_t onero_drag_teaching_get_state(onero_drag_teaching_handle handle);

ONERO_API bool onero_drag_teaching_is_initialized(onero_drag_teaching_handle handle);

ONERO_API void onero_drag_teaching_update_joint_state(onero_drag_teaching_handle handle,
                                                 const onero_joint_array_t* position,
                                                 const onero_joint_array_t* velocity,
                                                 const onero_joint_array_t* effort);

#ifdef __cplusplus
}
#endif
