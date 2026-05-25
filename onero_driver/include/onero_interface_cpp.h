// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#pragma once

#include "onero_define.h"

#include <vector>
#include <string>

#if !defined(ONERO_CPP_API)
    #if defined(_WIN32)
        #if defined(ONERO_STATIC)
            #define ONERO_CPP_API
        #elif defined(oneroarm_core_EXPORTS)
            #define ONERO_CPP_API __declspec(dllexport)
        #else
            #define ONERO_CPP_API __declspec(dllimport)
        #endif
    #else
        #define ONERO_CPP_API __attribute__((visibility("default")))
    #endif
#endif

namespace onero_api {

// ============================================================================
// 返回值约定（不同 API 沿用不同语义，调用方需对照表理解）：
//
//   * 强转自 MoveResult enum（0 = SUCCESS，负数 = 各类错误，详见
//     onero_define.h）：
//       - movej / movel / movep / force_position_movel
//       - send_trajectory_point / send_trajectory
//       - set_force_position_control / stop_force_position_control
//       - cancel_trajectory
//       - execute_buffered_trajectory / clear_trajectory_buffer
//       - enable_motors / disable_motors
//
//   * bool（true = 成功）：
//       - OneroDragTeaching::initialize
//       - OneroDragTeaching::set_hardware
//       - OneroDragTeaching::is_initialized
//       - OneroArm::is_hardware_connected
//
//   * int（0 = 成功，非 0 = 错误）：
//       - OneroDragTeaching::enable_motors
//       - OneroDragTeaching::start_recording / stop_recording
//       - OneroDragTeaching::start_replay   / stop_replay
//       - OneroDragTeaching::handle_command
//       - OneroArm::send_can_frame / register_can_frame_callback /
//         clear_can_frame_callback / pump_can_bus（错误码定义见 onero_define.h
//         中的 ONERO_ERR_RAW_FRAME_*）
//
//   * 数据对象（空 vector / 全零 Pose / count==0 表示失败，调用方需自行
//     防御）：
//       - get_joint_positions / get_joint_positions_from_motors
//       - get_joint_velocities
//       - get_arm_state_from_motor / get_arm_state_cached
//       - get_end_effector_pose
//       - OneroDragTeaching::get_state（失败回退为 IDLE）
//
// 现状沿用历史语义不做统一以保持 ABI 稳定；后续若引入 onero_status_t
// 单一返回码 enum，会在 SOVERSION major bump 时一并完成。
// ============================================================================

/**
 * @class OneroArm
 * @brief OneroArm 主控实例（C++ 公开入口，与 Python 同名同形）
 *
 * 构造时按 config 创建底层 robot handle，析构时自动释放——RAII 取代
 * 历史的 `OneroAPI::create_robot/destroy_robot` 静态对子。
 * 拷贝禁用、可移动；移动后源对象 handle_ == nullptr，析构无副作用。
 */
class ONERO_CPP_API OneroArm {
public:
    explicit OneroArm(const onero_config_t& config);
    ~OneroArm();

    OneroArm(const OneroArm&) = delete;
    OneroArm& operator=(const OneroArm&) = delete;

    OneroArm(OneroArm&& other) noexcept;
    OneroArm& operator=(OneroArm&& other) noexcept;

    /**
     * @brief 构造是否成功（false 表示 robot handle 创建失败，所有方法将以错误码退出）
     */
    bool valid() const { return handle_ != nullptr; }

    int enable_motors();
    int disable_motors();

    int movej(const JointArray& target, double speed_scale = 1.0, uint8_t trajectory_connect = 0);
    int movel(const Pose& pose, double speed_scale = 1.0, uint8_t trajectory_connect = 0);
    int movep(const Pose& pose, double speed_scale = 1.0, uint8_t trajectory_connect = 0);
    int force_position_movel(const Pose& pose, double speed_scale = 1.0, uint8_t trajectory_connect = 0);

    int set_force_position_control(const ForcePosition& force_position);
    int stop_force_position_control();

    int execute_buffered_trajectory();
    int clear_trajectory_buffer();

    JointArray get_joint_positions();
    JointArray get_joint_positions_from_motors();
    JointArray get_joint_velocities();
    ArmStateFromMotor get_arm_state_from_motor();
    ArmStateFromMotor get_arm_state_cached();
    Pose get_end_effector_pose();

    int send_trajectory_point(const JointArray& positions, const JointArray& velocities);
    int send_trajectory(const std::vector<TrajectoryPoint>& trajectory);
    int cancel_trajectory();

    bool is_hardware_connected();

    // ========== 原始 CAN 帧 API ==========
    // 与电机/夹爪共用同一根 SLCAN 链路。受保留 ID 集（电机 / 夹爪 /
    // 操纵杆 / 0x7FF）保护：发送命中保留集会直接返回错误码，不下发到总线。
    //
    // 线程语义：注册的回调在 SDK 接收路径（即调用方运动控制线程）上
    // 同步执行；回调内禁止重入 SDK 任何发送/运动控制方法，否则可能
    // 与外层 receive() 自锁。

    /**
     * @brief 发送一帧自定义 CAN 报文（11-bit 标准帧）。
     * @param can_id 11-bit 标准帧 ID，范围 [0, 0x7FF]，且不能落在保留集
     * @param payload 数据载荷指针；len==0 时可为 nullptr
     * @param len 0..8
     * @return 0=成功；负数错误码见 onero_define.h（ONERO_ERR_RAW_FRAME_*）
     */
    int send_can_frame(uint16_t can_id, const uint8_t* payload, uint8_t len);

    /**
     * @brief 注册接收回调；所有非保留 ID 的入站帧都会派发到 cb。
     *        重复调用会替换前一个回调；nullptr 等价于清除。
     */
    int register_can_frame_callback(CanFrameCallback cb);

    /**
     * @brief 清除已注册的接收回调。
     */
    int clear_can_frame_callback();

    /**
     * @brief 在 timeout_ms 内主动从 SLCAN 拉帧并 dispatch 到已注册回调。
     *        电机/夹爪帧仍走原解析路径。timeout_ms == 0 等价于一次非阻塞 try-recv。
     *        典型用法：在 movej 等运动控制空闲期主动调用，避免 SLCAN
     *        rx 缓冲区累积。
     */
    int pump_can_bus(int timeout_ms);

    /**
     * @brief 暴露底层 handle 给 C ABI / 其他内部桥接代码
     * @internal 用户代码不应使用
     */
    onero_robot_handle* native_handle() const { return handle_; }

private:
    onero_robot_handle* handle_ = nullptr;
};

/**
 * @class OneroDragTeaching
 * @brief 拖动示教实例（C++ 公开入口，与 Python 同名同形）
 *
 * 构造时分配 drag teaching handle，析构时释放。方法名与 Python 完全一致；
 * 不再带 `drag_teaching_` 前缀（类名已经表达了这个子空间）。
 */
class ONERO_CPP_API OneroDragTeaching {
public:
    OneroDragTeaching();
    ~OneroDragTeaching();

    OneroDragTeaching(const OneroDragTeaching&) = delete;
    OneroDragTeaching& operator=(const OneroDragTeaching&) = delete;

    OneroDragTeaching(OneroDragTeaching&& other) noexcept;
    OneroDragTeaching& operator=(OneroDragTeaching&& other) noexcept;

    bool valid() const { return handle_ != nullptr; }

    bool initialize(int dof, const std::string& record_file, double time_step = 0.01);
    bool set_hardware(const std::string& device,
                      const std::string& urdf_path,
                      const std::string& robot_model,
                      const std::string& mount_orientation = "horizontal");
    int  enable_motors(bool restore_to_zero = true);
    int  start_recording();
    int  stop_recording();
    void set_replay_file(const std::string& replay_file);
    int  start_replay();
    int  stop_replay();
    int  handle_command(int cmd);
    void timer_callback();
    DragTeachingState get_state();
    bool is_initialized();
    void update_joint_state(const JointArray& position,
                            const JointArray& velocity,
                            const JointArray& effort);

    /**
     * @brief 暴露底层 handle 给 C ABI / 其他内部桥接代码
     * @internal 用户代码不应使用
     */
    onero_drag_teaching_handle* native_handle() const { return handle_; }

private:
    onero_drag_teaching_handle* handle_ = nullptr;
};

}  // namespace onero_api
