#pragma once

#include "woan_api/woan_define.h"
#include "woan_api/serial_port.h"
#include "woan_api/damiao.h"
#include "woan_api/path_smoother.h"
#include "woan_api/encryption.h"
#include "woan_api/woan_platform.h"
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <string>
#include <fstream>
#include <mutex>
#include <vector>
#include <memory>
#include <chrono>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <cstdint>
#include <thread>

namespace woan_api {

/**
 * @class DragTeaching
 * @brief 拖动示教功能：零力拖动、轨迹记录、轨迹回放
 * 
 * 提供机械臂拖动示教的核心功能：
 * - 零力拖动模式：通过重力补偿实现零力拖动
 * - 轨迹记录：记录关节位置、速度、力矩到txt文件
 * - 轨迹回放：从txt文件读取并回放轨迹
 */
class WOAN_CPP_API DragTeaching {
public:
    /**
     * @brief 状态枚举
     */
    enum class State {
        IDLE,       // 空闲状态
        RECORDING,  // 正在记录
        REPLAYING   // 正在回放
    };

    /**
     * @brief 构造函数
     */
    DragTeaching();

    /**
     * @brief 析构函数
     */
    ~DragTeaching();

    /**
     * @brief 初始化（简化版，依赖外部 woan_driver）
     * @param dof 自由度
     * @param record_file 记录文件路径
     * @param time_step 时间步长（默认0.01秒）
     * @return true if successful, false otherwise
     */
    bool initialize(int dof,
                    const std::string& record_file,
                    double time_step = 0.01);

    /**
     * @brief 设置硬件资源（用于零力拖动控制）
     * @param device 串口设备路径（如 "/dev/ttyACM0"）
     * @param urdf_path URDF模型文件路径
     * @param robot_model 机器人型号（如 "x1_l", "x1_r"）
     * @return true if successful, false otherwise
     */
    bool set_hardware(const std::string& device,
                      const std::string& urdf_path,
                      const std::string& robot_model);

    /**
     * @brief 使能电机（类似WoanAPI::enable_motors）
     * @param restore_to_zero 是否恢复到零位（默认true，与ROS版本行为一致）
     * @return 0 on success, negative on failure
     */
    int enable_motors(bool restore_to_zero = true);

    /**
     * @brief 开始零力拖动记录
     * @return 0 on success, negative on failure
     */
    int start_recording();

    /**
     * @brief 停止记录
     * @return 0 on success, negative on failure
     */
    int stop_recording();

    /**
     * @brief 设置回放文件路径
     * @param replay_file 回放文件路径
     */
    void set_replay_file(const std::string& replay_file);

    /**
     * @brief 开始轨迹回放
     * @return 0 on success, negative on failure
     */
    int start_replay();

    /**
     * @brief 停止回放
     * @return 0 on success, negative on failure
     */
    int stop_replay();

    /**
     * @brief 处理命令（统一命令处理接口）
     * @param cmd 命令ID: 0=停止, 1=开始记录, 2=停止记录, 3=开始回放
     * @return 0 on success, negative on failure
     */
    int handle_command(int cmd);

    /**
     * @brief 定时器回调（由外部定期调用）
     * 根据当前状态执行相应的操作：
     * - RECORDING: 读取关节状态，计算重力补偿，记录到文件
     * - REPLAYING: 从文件读取轨迹点并执行
     */
    void timer_callback();

    /**
     * @brief 获取当前状态
     * @return 当前状态
     */
    State get_state() const { return state_; }

    /**
     * @brief 检查是否已初始化
     * @return true if initialized
     */
    bool is_initialized() const { return initialized_; }

    /**
     * @brief 获取时间步长
     * @return 时间步长（秒）
     */
    double get_time_step() const { return time_step_; }

    /**
     * @brief 更新关节状态（从 ROS2 话题调用）
     * @param position 关节位置
     * @param velocity 关节速度
     * @param effort 关节力矩/力
     */
    void update_joint_state(const std::vector<double>& position,
                           const std::vector<double>& velocity,
                           const std::vector<double>& effort);

private:
    // 记录相关
    void record_joint_state(const Eigen::VectorXd& pos, 
                           const Eigen::VectorXd& vel,
                           const Eigen::VectorXd& tau);
    bool write_file_header();               // 写入文件头

    // 回放相关
    bool read_trajectory_point(Eigen::VectorXd& pos, 
                               Eigen::VectorXd& vel,
                               Eigen::VectorXd& tau,
                               double& timestamp);
    bool read_file_header();                // 读取并验证文件头

    // 成员变量
    bool initialized_;                      // 是否已初始化
    State state_;                           // 当前状态
    int dof_;                               // 自由度
    std::string record_file_;               // 记录文件路径
    double time_step_;                      // 时间步长
    std::vector<double> default_joints_zero_;  // 零点位置

    // 当前关节状态缓存（从 ROS2 话题更新）
    std::vector<double> current_position_;  // 当前位置
    std::vector<double> current_velocity_;  // 当前速度
    std::vector<double> current_torque_;    // 当前力矩

    // 文件操作
    std::ofstream record_file_stream_;      // 记录文件流
    std::ifstream replay_file_stream_;      // 回放文件流
    std::mutex file_mutex_;                 // 文件操作互斥锁
    std::mutex control_mutex_;              // 控制操作互斥锁
    TrajectoryEncryption encryption_;       // 轨迹数据加密工具
    bool file_header_written_;              // 文件头是否已写入

    // 回放状态
    bool replay_file_opened_;               // 回放文件是否已打开
    Eigen::VectorXd last_replay_vel_;       // 上次回放速度（用于计算加速度）
    double next_trajectory_timestamp_;      // 下一个轨迹点的时间戳（用于时间同步）
    double last_trajectory_timestamp_;      // 上一个轨迹点的时间戳（用于计算时间间隔）
    bool replay_trajectory_started_;        // 轨迹回放是否已开始（用于准确计算回放时间，排除恢复到零位的时间）
    double last_replayed_timestamp_;        // 最后一个已回放的轨迹点的时间戳（用于计算回放总时长）
    
    // 时间记录
    std::chrono::steady_clock::time_point recording_start_time_;  // 记录开始时间
    std::chrono::steady_clock::time_point replay_start_time_;     // 回放开始时间

    // 零力拖动相关硬件资源
    std::shared_ptr<SerialPort> serial_port_;                    // 串口通信
    std::unique_ptr<damiao::Motor_Control> motor_control_;       // 达妙电机控制器
    std::unique_ptr<damiao::Motor[]> motors_;                    // 电机数组
    std::unique_ptr<RigidBodyDynamics::Model> rbdl_model_;       // RBDL动力学模型
    std::vector<double> gravity_scale_;                          // 重力补偿系数（不同关节不同）
    bool hardware_initialized_;                                  // 硬件是否已初始化
    
    // 路径规划和运动控制参数
    std::unique_ptr<PathSmoother[]> path_smoothers_;            // 路径平滑器数组
    double max_vel_;                                             // 最大速度 (rad/s)
    double max_acc_;                                             // 最大加速度 (rad/s²)
    double max_jerk_;                                            // 最大加加速度 (rad/s³)
    std::vector<double> kps_;                                    // PD控制参数 kp
    std::vector<double> kds_;                                    // PD控制参数 kd
    
    // 零力拖动控制方法
    void apply_zero_force_control();                             // 应用零力拖动控制
    Eigen::VectorXd get_current_position_from_motors();          // 从电机读取当前位置
    
    // 恢复零点方法
    int restore_arm();                                            // 恢复到零点（使用平滑路径，速度50%）
    void restore_arm_internal(const Eigen::VectorXd& start, const Eigen::VectorXd& goal);  // 内部实现
    Eigen::VectorXd get_current_velocity_from_motors() const;    // 从电机读取当前速度
};

}  // namespace woan_api

