#pragma once

#include "woan_define.h"
#include "woan_api/damiao.h"
#include "woan_api/serial_port.h"
#include "woan_api/robot_model.h"
#include "woan_api/path_smoother.h"
#include "woan_api/minimum_jerk_trajectory.h"
#include "woan_api/cubic_spline.h"
#include "woan_api/ArmDynamics.h"
#include "woan_api/woan_platform.h"
#include "woan_api/encryption.h"
#include "woan_api/trajectory_logger.h"
#include "woan_api/cubic_spline.h"

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <thread>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <fstream>
#include <cstring>
#include <algorithm>
#include <stdexcept>
#include <cerrno>
#include <fcntl.h>
#include <sstream>
#include <cstdint>
#include <cmath>
#include <filesystem>

namespace woan_api {

// Forward declaration
class DragTeaching;

/**
 * @class WoanCore
 * @brief Core robot control logic (non-ROS, pure C++)
 * 
 * This is the heart of the SO library - handles all motion control,
 * kinematics without any ROS dependencies.
 */
class WoanCore {
    friend class DragTeaching;  // Allow DragTeaching to access private members

public:
    WoanCore();
    ~WoanCore();

    /**
     * Initialize the robot with configuration
     */
    bool initialize(const woan_config_t& config);

    /**
     * Shutdown and cleanup
     */
    void shutdown();

    /**
     * Motor control
     */
    int enable_motors();
    int disable_motors();

    /**
     * Move API implementations
     */
    int movej(const JointArray& target,
              double speed_scale = 1.0,
              uint8_t trajectory_connect = 0);

    int movel(const Pose& pose,
              double speed_scale = 1.0,
              uint8_t trajectory_connect = 0);

    int movep(const Pose& pose,
              double speed_scale = 1.0,
              uint8_t trajectory_connect = 0);

    /**
     * Trajectory buffering
     */
    int executeBufferedTrajectories();
    int clear_trajectory_buffer();

    /**
     * Leader arm gravity compensation
     */
    int ArmGravityCompensation();
    void StopGravityCompensation();

    /**
     * State queries
     */
    JointArray get_joint_positions();
    JointArray get_joint_positions_from_motors();
    JointArray get_joint_velocities();
    ArmStateFromMotor get_arm_state_from_motor() const;  // 从电机读取位置（含零偏差）、速度、力矩
    Pose get_end_effector_pose();

    /**
     * Check if hardware (serial port) is connected
     * @return true if serial port is valid, false otherwise
     */
    bool is_hardware_connected() const;

    /**
     * MoveIt integration - Send trajectory point
     */
    int send_trajectory_point(const JointArray& positions, const JointArray& velocities);

    /**
     * Send complete trajectory with inverse dynamics compensation
     * @param trajectory Vector of trajectory points (each containing position, velocity, acceleration)
     * @return 0 on success, -1 on failure
     */
    int send_trajectory(const std::vector<TrajectoryPoint>& trajectory);

    /**
     * Cancel ongoing trajectory execution in send_trajectory
     * @return Status code(0 on success, -1 on failure)
     */
    int cancel_trajectory();

private:
    // ========== Configuration ==========
    woan_config_t config_;
    bool initialized_;
    int dof_;

    // ========== Hardware Interface ==========
    // 【复制自 arm_control.cpp 第1697-1704行】
    std::shared_ptr<SerialPort> serial_port_;                    // 串口通信
    std::unique_ptr<damiao::Motor_Control> motor_control_;       // 达妙电机控制器
    std::unique_ptr<damiao::Motor[]> motors_;                    // 电机数组
    std::unique_ptr<damiao::Gripper> gripper_;                   // 夹爪
    mutable std::mutex motor_state_mutex_;                       // 保护 get_arm_state_from_motor，一次仅允许一个查询
    std::vector<double> position_cache_;                         // 最近一次位置缓存（由重力补偿线程更新）

    // ========== Trajectory Planning ==========
    // 【复制自 arm_control.cpp 第1701行】
    std::unique_ptr<PathSmoother[]> path_smoothers_;             // S曲线路径平滑器

    // ========== Kinematics & Dynamics Models ==========
    // 【复制自 arm_control.cpp 第1698-1699行】
    std::unique_ptr<RigidBodyDynamics::Model> rbdl_model_;       // RBDL动力学模型
    std::unique_ptr<woan::RobotModel> robot_model_;          // Pinocchio运动学模型

    // ========== Control Parameters ==========
    // 【复制自 arm_control.cpp 第1712-1717行】
    double max_vel_;                                              // 最大速度
    double max_acc_;                                              // 最大加速度
    double max_jerk_;                                             // 最大加加速度
    double time_step_;                                            // 控制周期
    double control_frequency_hz_;                                 // 控制频率（Hz），可自由修改
    std::vector<double> kps_;                                     // PD控制-比例系数
    std::vector<double> kds_;                                     // PD控制-微分系数
    std::vector<double> torque_scales_;                           // 力矩缩放倍率（7个关节）

    // ========== Moveit Parameters ==========
    bool stop_{false};                                              // 用来接收moveit action的停止信号

    // ========== MoveL Specific Parameters ==========
    // 【复制自 arm_control.cpp 第1705-1708行】
    double base_cartesian_speed_;                                 // 基础笛卡尔速度 (m/s)
    double base_cartesian_acc_;                                   // 基础笛卡尔加速度 (m/s²)
    double movel_max_jerk_;                                       // MoveL加加速度
    double movel_time_step_;                                      // MoveL时间步长

    // ========== Calibration Parameters ==========
    Eigen::VectorXd zero_bias_;                                   // 零点偏差向量（用于角度转换）

    // ========== Trajectory Buffer ==========
    struct TrajectoryWaypoint {
        std::string type;                 // "movej", "movel", "movep"
        std::vector<double> joint_target; // 关节空间目标
        double speed_scale;               // 速度缩放
        Pose cart_pose;                   // 笛卡尔位姿（MoveL/MoveP）
        std::vector<double> kp_values;    // MoveL PD参数
        std::vector<double> kd_values;
        float max_jerk;
        Eigen::VectorXd end_velocity;     // 新增：该段结束后的速度
    };
    std::vector<TrajectoryWaypoint> trajectory_buffer_;
    std::mutex trajectory_mutex_;

    // ========== Spline Interpolation ==========
    std::vector<CubicSpline> joint_splines_;                  // 每个关节一个样条插值器
    std::vector<Eigen::VectorXd> joint_pos_buffer_;           // 预分配内存
    std::vector<Eigen::VectorXd> joint_vel_buffer_;
    std::vector<Eigen::VectorXd> joint_acc_buffer_;
    Eigen::VectorXd last_trajectory_end_velocity_;            // 上一段轨迹的理论终止速度

    // ========== Gravity Compensation ==========
    bool is_gravity_compensation_running_{false};
    std::thread gravity_compensation_thread_;
    std::unique_ptr<ArmDynamics> LeaderArmDynamics_;

    // ========== Helper Functions ==========
    /**
     * @brief Restore arm to default safe posture after enabling motors
     * @return 0 on success, negative on failure
     */
    int restore_arm();
    
    /**
     * @brief Get current joint positions from real motors (with zero_bias applied)
     */
    Eigen::VectorXd getCurrentPosition() const;
    
    /**
     * @brief Get current joint velocities
     * 【复制自 arm_control.cpp 第1632-1640行】
     */
    Eigen::VectorXd getCurrentVelocity() const;
    
    /**
     * @brief Internal MoveJ implementation (S-curve trajectory)
     * @param log_planned_data 是否记录规划数据（默认true，restore_arm时设为false）
     * @return true 正常完成，false 被 interrupt_check 中断
     */
    bool movej_internal(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, bool log_planned_data = true);

    /** @return true 当 interrupt_check 存在且返回 true（如 Ctrl+C） */
    bool isInterruptRequested() const;

    /**
     * @brief 更新位置缓存（用于重力补偿线程）
     */
    void updatePositionCache(const std::vector<double>& positions);

    // ========== 初始化相关方法 ==========
    /**
     * @brief 初始化控制参数
     */
    void initControlParameters();

    /**
     * @brief 初始化硬件（串口、电机、夹爪）
     */
    bool initHardware();

    /**
     * @brief 初始化模型（RBDL、Pinocchio）
     */
    void initModels();
    
    /**
     * @brief 初始化样条插值相关缓冲区
     */
    void initializeSplineBuffers(int max_dof);

    /**
     * @brief 查找基础URDF路径（根据robot_model确定目录：A1或v3.2）
     */
    std::string findBaseUrdfPath(const char* robot_model);

    /**
     * @brief 查找URDF文件路径（根据robot_model自动确定型号和左右臂）
     */
    std::string findUrdfPath(const char* robot_model, const char* urdf_path_config);

    /**
     * @brief 查找模型目录路径（用于Pinocchio，根据robot_model确定）
     */
    std::string findModelDir(const char* robot_model);

    // ========== 轨迹管理方法 ==========
    /**
     * @brief 缓冲轨迹点
     */
    bool bufferTrajectory(const std::string& type, const JointArray& joint_target,
                         double speed_scale,
                         const Pose& cart_pose = {0, 0, 0, 1, 0, 0, 0});

    /**
     * @brief 应用速度缩放（保存原值并应用新值）
     */
    double applySpeedScale(double speed_scale);

    /**
     * @brief 恢复速度缩放
     */
    void restoreSpeedScale(double original_max_vel);

    /**
     * @brief 获取轨迹起始速度（优先使用理论值）
     */
    Eigen::VectorXd getTrajectoryEndVelocity();
    
    /**
     * @brief 更新轨迹理论终止速度
     */
    void updateTrajectoryEndVelocity(const Eigen::VectorXd& end_vel);
    
    /**
     * @brief 清除速度状态（停止后调用）
     */
    void clearTrajectoryEndVelocity();

    // ========== MoveL规划方法 ==========
    /**
     * @brief 规划笛卡尔路径（直线插值）
     */
    std::vector<woan::Posture> planCartesianPath(
        const woan::Transform& current_transform,
        const Eigen::Vector3d& target_pos,
        const Eigen::Quaterniond& target_quat,
        double effective_step_size);

    /**
     * @brief 规划笛卡尔路径（简单等间距插值，不涉及时间）
     * @param num_points 路径点数量，默认100个
     * @return 返回等间距插值的笛卡尔路径点
     */
    std::vector<woan::Posture> planCartesianPath(
        const woan::Transform& current_transform,
        const Eigen::Vector3d& target_pos,
        const Eigen::Quaterniond& target_quat,
        size_t num_points);

    /**
     * @brief 规划笛卡尔路径（考虑梯形速度曲线，自动计算duration）
     * @return 返回路径和duration（通过引用参数返回）
     */
    std::vector<woan::Posture> planCartesianPath(
        const woan::Transform& current_transform,
        const Eigen::Vector3d& target_pos,
        const Eigen::Quaterniond& target_quat,
        double distance,
        double target_speed,
        double max_acc,
        double dt,
        double& duration,
        std::vector<double>* time_points = nullptr);

    /**
     * @brief 验证路径是否为直线
     */
    bool validateStraightLine(const std::vector<woan::Posture>& cartesian_path);

    /**
     * @brief 计算IK路径
     */
    bool computeIKPath(const std::vector<woan::Posture>& cartesian_path,
                      const Eigen::VectorXd& current_joints,
                      std::vector<Eigen::VectorXd>& joint_path);

    /**
     * @brief 计算IK轨迹，包含位置、速度和加速度
     */
    bool computeIKTrajectory(const std::vector<woan::Posture>& cartesian_path,
                            const Eigen::VectorXd& current_joints,
                            const Eigen::VectorXd& direction,
                            double target_speed,
                            double max_acc,
                            double total_distance,
                            double duration,
                            double dt,
                            std::vector<Eigen::VectorXd>& joint_pos_path,
                            std::vector<Eigen::VectorXd>& joint_vel_path,
                            std::vector<Eigen::VectorXd>& joint_acc_path);
                            
    /**
     * @brief 使用三次样条插值生成平滑关节轨迹
     */
    bool generateSplineTrajectory(
        const std::vector<Eigen::VectorXd>& joint_via_points,
        const std::vector<double>& time_points,
        const Eigen::VectorXd& start_velocity,
        const Eigen::VectorXd& end_velocity,
        double dt,
        std::vector<Eigen::VectorXd>& joint_pos_path,
        std::vector<Eigen::VectorXd>& joint_vel_path,
        std::vector<Eigen::VectorXd>& joint_acc_path);
        
    /**
     * @brief 预处理时间点（合并过近的点）
     */
    std::vector<double> preprocessTimePoints(const std::vector<double>& t_in,
                                           std::vector<Eigen::VectorXd>& via_points);

    /**
     * @brief 计算MoveL轨迹时长
     */
    double computeMoveLDuration(const std::vector<Eigen::VectorXd>& joint_path,
                               double speed_scale);

    // ========== 参数管理方法 ==========
    /**
     * @brief 对力矩进行系数缩放处理
     * @param tau 动力学计算出的原始力矩
     */
    void applyTorqueScaling(Eigen::VectorXd& tau);

    /**
     * @brief 精确时间控制：等待直到达到控制频率间隔
     * @param loop_start_time 循环开始时间
     * @return 返回当前时间，用于下一次循环
     */
    std::chrono::steady_clock::time_point waitForControlInterval(
        std::chrono::steady_clock::time_point loop_start_time) const;
};

}  // namespace woan_api