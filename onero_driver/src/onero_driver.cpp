// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include <pinocchio/fwd.hpp>  // ensure pinocchio precedes any Boost from rclcpp

#include "rclcpp/rclcpp.hpp"
#include "onero_interfaces/msg/move_j.hpp"
#include "onero_interfaces/msg/move_l.hpp"
#include "onero_interfaces/msg/move_p.hpp"
#include "onero_interfaces/msg/arm_state.hpp"
#include "onero_interfaces/msg/set_force_position.hpp"
#include "onero_interfaces/srv/end_effector_pose.hpp" // 替换为你的包名
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "onero_interface_cpp.h"
#include "onero_define.h"

#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <fstream> // 用于文件操作
#include <algorithm>
#include <cctype>
#include <unistd.h>  // for usleep

namespace {

std::string normalizeMountOrientation(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (value == "horizontal" || value == "vertical") {
        return value;
    }
    return "";
}

}

class OneroDriverNode : public rclcpp::Node {
public:
    OneroDriverNode() : Node("onero_driver_node"), is_executing_(false), should_cancel_(false), time_step_(0.01) {  // time_step_是debug用的                       
        this->declare_parameter<int>("dof", 14);
        this->declare_parameter<double>("state_pub_rate", 100.0);
        this->declare_parameter<std::string>("mount_orientation", "horizontal");

        dof_ = this->get_parameter("dof").as_int();
        double state_pub_rate = this->get_parameter("state_pub_rate").as_double();
        mount_orientation_ = normalizeMountOrientation(
            this->get_parameter("mount_orientation").as_string());
        if (mount_orientation_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid mount_orientation parameter, fallback to 'horizontal'. "
                        "Supported values: horizontal, vertical");
            mount_orientation_ = "horizontal";
        }

        if(dof_ == 7){
            arm_status_ = 0;

            this->declare_parameter<std::string>("robot_model", "a1_l");
            this->declare_parameter<std::string>("device", "/dev/ttyACM0");

            robot_model_ = this->get_parameter("robot_model").as_string();
            device_ = this->get_parameter("device").as_string();
        }
        else if(dof_ == 14){
            left_arm_status_= 0;
            right_arm_status_= 0;

            this->declare_parameter<std::string>("robot_model", "A1_dual");
            this->declare_parameter<std::string>("symbol_left", "a1_l");
            this->declare_parameter<std::string>("device_left", "/dev/ttyACM0");
            this->declare_parameter<std::string>("symbol_right", "a1_r");
            this->declare_parameter<std::string>("device_right", "/dev/ttyACM1");

            robot_model_ = this->get_parameter("robot_model").as_string();
            symbol_left_ = this->get_parameter("symbol_left").as_string();
            device_left_ = this->get_parameter("device_left").as_string();
            symbol_right_ = this->get_parameter("symbol_right").as_string();
            device_right_ = this->get_parameter("device_right").as_string();
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Unsupported DOF: %d. Only 7 or 14 are supported.", dof_);
            throw std::runtime_error("Unsupported DOF");
        }
        
        if (!initializeRobot()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize robot!");
            return;
        }
        
        //单臂 or 双臂
        movej_sub_ = this->create_subscription<onero_interfaces::msg::MoveJ>(
            "/onero_driver/movej_cmd", 10,
            std::bind(&OneroDriverNode::movejCallback, this, std::placeholders::_1));
        //仅单臂
        movel_sub_ = this->create_subscription<onero_interfaces::msg::MoveL>(
            "/onero_driver/movel_cmd", 10,
            std::bind(&OneroDriverNode::movelCallback, this, std::placeholders::_1));
        force_position_movel_sub_ = this->create_subscription<onero_interfaces::msg::MoveL>(
            "/onero_driver/force_position_movel_cmd", 10,
            std::bind(&OneroDriverNode::forcePositionMovelCallback, this, std::placeholders::_1));
        //仅单臂
        force_position_control_sub_ = this->create_subscription<onero_interfaces::msg::SetForcePosition>(
            "/onero_driver/force_position_control_cmd", 10,
            std::bind(&OneroDriverNode::ForcePositionControlCallback, this, std::placeholders::_1));

        stop_force_postion_sub_= this->create_subscription<std_msgs::msg::Bool>(
            "/onero_driver/stop_force_postion_cmd", 10,
            std::bind(&OneroDriverNode::StopForcePostionControlCallback, this, std::placeholders::_1));

        //仅单臂
        movep_sub_ = this->create_subscription<onero_interfaces::msg::MoveP>(
            "/onero_driver/movep_cmd", 10,
            std::bind(&OneroDriverNode::movepCallback, this, std::placeholders::_1));
        
        movej_result_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/onero_driver/movej_result", 10);
        
        movel_result_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/onero_driver/movel_result", 10);
        force_position_movel_result_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/onero_driver/force_position_movel_result", 10);
        
        movep_result_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/onero_driver/movep_result", 10);
        
        force_position_control_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/onero_driver/force_position_control_result", 10);
        stop_force_position_control_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/onero_driver/stop_force_position_control_result", 10);
        
        //单臂的状态发布
        arm_state_pub_ = this->create_publisher<onero_interfaces::msg::ArmState>(
        "/onero_driver/arm_state", 10);

        //双臂需左右臂状态发布
        left_arm_state_pub_ = this->create_publisher<onero_interfaces::msg::ArmState>(
            "/onero_driver/left_arm_state", 10);
        right_arm_state_pub_ = this->create_publisher<onero_interfaces::msg::ArmState>(
            "/onero_driver/right_arm_state", 10);
        
        //发布 /joint_states 供 MoveIt 使用
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        
        clear_buffer_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/onero_driver/clear_trajectory_buffer",
            std::bind(&OneroDriverNode::clearBufferCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // 双臂：订阅规划手臂状态（由onero_control_dual发布）
        planning_arms_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/moveit_planning_arms_status", 10,
            std::bind(&OneroDriverNode::planningArmsCallback, this, std::placeholders::_1));
        
        // ========== 订阅完整轨迹和执行命令 ==========
        trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/moveit_trajectory", 10,
            std::bind(&OneroDriverNode::trajectoryCallback, this, std::placeholders::_1));
        
        ///*send_trajectory_point的方式*/
        // execute_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        //     "/moveit_execute", 10,
        //     std::bind(&OneroDriverNode::executeCallback, this, std::placeholders::_1));

        /*send_trajectory的方式*/
        execute_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/moveit_execute", 10,
            std::bind(&OneroDriverNode::executeCallback, this, std::placeholders::_1));
        
        cancel_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/moveit_cancel", 10,
            std::bind(&OneroDriverNode::cancelCallback, this, std::placeholders::_1));
        
        // 传统模式：执行成功的状态反馈
        execution_result_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/onero_driver/trajectory_execution_result", 10);
        
        // 新增DEBUG：订阅测试轨迹（用于replay_trajectory_from_file）
        test_trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/test_trajectory", 10,
            std::bind(&OneroDriverNode::testTrajectoryCallback, this, std::placeholders::_1));
        
        //RCLCPP_INFO(this->get_logger(), "已订阅 /test_trajectory 用于轨迹测试");
        
        // 新增DEBUG：订阅完整轨迹并使用send_trajectory执行
        test_send_trajectory_full_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/test_send_trajectory_full", 10,
            std::bind(&OneroDriverNode::testSendTrajectoryFullCallback, this, std::placeholders::_1));
        
        //RCLCPP_INFO(this->get_logger(), "已订阅 /test_send_trajectory_full 用于完整轨迹执行");
        
        auto state_period = std::chrono::duration<double>(1.0 / state_pub_rate);
        state_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(state_period),
            std::bind(&OneroDriverNode::publishArmState, this));
        
        get_end_pose_service_ = this->create_service<onero_interfaces::srv::EndEffectorPose>(
            "end_effector_pose",
            std::bind(&OneroDriverNode::GetEndPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
    
    ~OneroDriverNode() {
        // 停止执行线程
        should_cancel_ = true;
        if (execution_thread_.joinable()) {
            execution_thread_.join();
        }

        // 安全策略：节点退出时仅释放句柄，不调用 disable_motors。
        // 运行期间机械臂可能停留在任意位姿，若失能将丧失关节保持力矩，
        // 在重力作用下发生坠落，存在伤人或损坏设备的风险。
        // 退出后请通过急停按钮或物理电源开关进行下电。
        // unique_ptr 析构会调用 OneroArm::~OneroArm() = destroy_robot。
    }
    
private:
    std::unique_ptr<onero_api::OneroArm> robot_arm_;        // 单臂
    std::unique_ptr<onero_api::OneroArm> robot_arm_left_;   // 双臂：左臂
    std::unique_ptr<onero_api::OneroArm> robot_arm_right_;  // 双臂：右臂
    rclcpp::Subscription<onero_interfaces::msg::MoveJ>::SharedPtr movej_sub_;
    rclcpp::Subscription<onero_interfaces::msg::MoveL>::SharedPtr movel_sub_;
    rclcpp::Subscription<onero_interfaces::msg::MoveL>::SharedPtr force_position_movel_sub_;
    rclcpp::Subscription<onero_interfaces::msg::SetForcePosition>::SharedPtr force_position_control_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_force_postion_sub_;
    rclcpp::Subscription<onero_interfaces::msg::MoveP>::SharedPtr movep_sub_;
    
    // MoveIt输出订阅
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planning_arms_sub_;  // 双臂：订阅规划手臂状态
    
    // 方案C：订阅完整轨迹和控制命令
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr execute_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_result_pub_;
    
    // 新增DEBUG：测试轨迹订阅（用于replay_trajectory_from_file）
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr test_trajectory_sub_;
    
    // 新增DEBUG：完整轨迹执行订阅（使用send_trajectory）
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr test_send_trajectory_full_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr movej_result_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr movel_result_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr force_position_movel_result_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr movep_result_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr force_position_control_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_force_position_control_pub_;

    
    rclcpp::Publisher<onero_interfaces::msg::ArmState>::SharedPtr arm_state_pub_; // 单臂状态
    rclcpp::Publisher<onero_interfaces::msg::ArmState>::SharedPtr left_arm_state_pub_;  // 双臂
    rclcpp::Publisher<onero_interfaces::msg::ArmState>::SharedPtr right_arm_state_pub_; // 双臂
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_; 
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_buffer_srv_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::Service<onero_interfaces::srv::EndEffectorPose>::SharedPtr get_end_pose_service_;

    
    std::string device_; // 单臂
    int arm_status_; // 单臂

    std::string robot_model_;
    std::string mount_orientation_;
    std::string symbol_left_;
    std::string device_left_;
    std::string symbol_right_;
    std::string device_right_;
    int dof_;
    int left_arm_status_; // 双臂
    int right_arm_status_; // 双臂
    std::string planning_arms_status_;  // 双臂： "left", "right", "dual", 或 "none"
    mutable std::mutex planning_arms_status_mutex_;
    
    // 双臂： 仅用于警告信息发布：只连了一只手臂的警告信号
    bool warn_signal_ = false;
    bool last_warn_signal_ = false;
    
    // 方案C：轨迹缓冲相关变量
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_buffer_;
    rclcpp::Time trajectory_start_time_;
    std::atomic<bool> is_executing_;
    std::atomic<bool> should_cancel_;
    std::mutex trajectory_mutex_;
    std::thread execution_thread_;
    double time_step_;  // debug： 控制周期，单位：秒（默认0.01s = 10ms）

    static bool interruptRequested(void* ctx) {
        (void)ctx;
        return !rclcpp::ok();
    }
    
    bool initializeRobot() {
        if(dof_ == 7){
            try {
                onero_api::onero_config_t config{};
                
                // 从参数读取串口设备名称
                strncpy(config.device, device_.c_str(), sizeof(config.device) - 1);
                config.device[sizeof(config.device) - 1] = '\0';  // 确保 null 终止
                
                // 传递robot_model参数（从ROS参数获取）
                strncpy(config.robot_model, robot_model_.c_str(), sizeof(config.robot_model) - 1);
                config.robot_model[sizeof(config.robot_model) - 1] = '\0';  // 确保 null 终止
                strncpy(config.mount_orientation, mount_orientation_.c_str(), sizeof(config.mount_orientation) - 1);
                config.mount_orientation[sizeof(config.mount_orientation) - 1] = '\0';
                
                config.dof = dof_;
                config.baud_rate = 921600;
                config.urdf_path[0] = '\0';  // 使用默认路径

                // 从配置文件读取MIT模式PD参数
                std::vector<double> mit_kp_values = this->declare_parameter<std::vector<double>>(
                    "mit_mode.default_kp_values", std::vector<double>(7, 100.0));
                std::vector<double> mit_kd_values = this->declare_parameter<std::vector<double>>(
                    "mit_mode.default_kd_values", std::vector<double>(7, 0.5));

                if ((int)mit_kp_values.size() < dof_ || (int)mit_kd_values.size() < dof_) {
                    RCLCPP_ERROR(this->get_logger(),
                        "mit_mode.default_kp_values/default_kd_values size (%zu/%zu) < dof (%d)",
                        mit_kp_values.size(), mit_kd_values.size(), dof_);
                    return false;
                }

                for (int i = 0; i < dof_; i++) {
                    config.mit_kp[i] = mit_kp_values[i];
                }
                for (int i = 0; i < dof_; i++) {
                    config.mit_kd[i] = mit_kd_values[i];
                }
                config.interrupt_check = &OneroDriverNode::interruptRequested;
                config.interrupt_ctx = this;

                robot_arm_ = std::make_unique<onero_api::OneroArm>(config);
                if (!robot_arm_->valid()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create robot handle");
                    robot_arm_.reset();
                    return false;
                }

                int result = robot_arm_->enable_motors();
                if (result != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to enable motors (code: %d)", result);
                    robot_arm_.reset();
                    return false;
                }
                return true;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception during initialization: %s", e.what());
                return false;
            }
        }

        else { // 双臂:14个自由度
            try {
                // ========== 初始化左臂 ==========
                onero_api::onero_config_t config_left{};
                
                strncpy(config_left.device, device_left_.c_str(), sizeof(config_left.device) - 1);
                config_left.device[sizeof(config_left.device) - 1] = '\0';
                
                strncpy(config_left.robot_model, symbol_left_.c_str(), sizeof(config_left.robot_model) - 1);
                config_left.robot_model[sizeof(config_left.robot_model) - 1] = '\0';
                strncpy(config_left.mount_orientation, mount_orientation_.c_str(), sizeof(config_left.mount_orientation) - 1);
                config_left.mount_orientation[sizeof(config_left.mount_orientation) - 1] = '\0';
                
                config_left.dof = dof_ / 2;  // 单臂7自由度
                config_left.baud_rate = 921600;
                config_left.urdf_path[0] = '\0';

                // 读取左臂MIT模式PD参数
                std::vector<double> mit_kp_left = this->declare_parameter<std::vector<double>>(
                    "mit_mode.left_kp_values", std::vector<double>(dof_ / 2, 100.0));
                std::vector<double> mit_kd_left = this->declare_parameter<std::vector<double>>(
                    "mit_mode.left_kd_values", std::vector<double>(dof_ / 2, 0.5));

                if ((int)mit_kp_left.size() < dof_ / 2 || (int)mit_kd_left.size() < dof_ / 2) {
                    RCLCPP_ERROR(this->get_logger(),
                        "mit_mode.left_kp_values/left_kd_values size (%zu/%zu) < dof/2 (%d)",
                        mit_kp_left.size(), mit_kd_left.size(), dof_ / 2);
                    return false;
                }

                for (int i = 0; i < dof_ / 2; i++) {
                    config_left.mit_kp[i] = mit_kp_left[i];
                    config_left.mit_kd[i] = mit_kd_left[i];
                }
                config_left.interrupt_check = &OneroDriverNode::interruptRequested;
                config_left.interrupt_ctx = this;

                robot_arm_left_ = std::make_unique<onero_api::OneroArm>(config_left);
                if (!robot_arm_left_->valid()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create left arm robot handle");
                    robot_arm_left_.reset();
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "Left arm handle created successfully");

                int result_left = robot_arm_left_->enable_motors();
                if (result_left != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to enable left arm motors (code: %d)", result_left);
                    robot_arm_left_.reset();
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "Left arm enabled: %s on %s",
                            symbol_left_.c_str(), device_left_.c_str());
                
                // ========== 初始化右臂 ==========
                RCLCPP_INFO(this->get_logger(), "Starting right arm initialization...");
                RCLCPP_INFO(this->get_logger(), "Right arm device: %s", device_right_.c_str());
                
                onero_api::onero_config_t config_right{};
                
                strncpy(config_right.device, device_right_.c_str(), sizeof(config_right.device) - 1);
                config_right.device[sizeof(config_right.device) - 1] = '\0';
                
                strncpy(config_right.robot_model, symbol_right_.c_str(), sizeof(config_right.robot_model) - 1);
                config_right.robot_model[sizeof(config_right.robot_model) - 1] = '\0';
                strncpy(config_right.mount_orientation, mount_orientation_.c_str(), sizeof(config_right.mount_orientation) - 1);
                config_right.mount_orientation[sizeof(config_right.mount_orientation) - 1] = '\0';
                
                config_right.dof = dof_ / 2;  // 单臂7自由度
                config_right.baud_rate = 921600;
                config_right.urdf_path[0] = '\0';

                // 读取右臂MIT模式PD参数
                std::vector<double> mit_kp_right = this->declare_parameter<std::vector<double>>(
                    "mit_mode.right_kp_values", std::vector<double>(dof_ / 2, 100.0));
                std::vector<double> mit_kd_right = this->declare_parameter<std::vector<double>>(
                    "mit_mode.right_kd_values", std::vector<double>(dof_ / 2, 0.5));

                if ((int)mit_kp_right.size() < dof_ / 2 || (int)mit_kd_right.size() < dof_ / 2) {
                    RCLCPP_ERROR(this->get_logger(),
                        "mit_mode.right_kp_values/right_kd_values size (%zu/%zu) < dof/2 (%d)",
                        mit_kp_right.size(), mit_kd_right.size(), dof_ / 2);
                    // 左臂在此前已 enable_motors 成功 → 回滚必须显式 disable_motors，
                    // 因为 unique_ptr.reset() 仅触发 destroy_robot，不会 disable。
                    robot_arm_left_->disable_motors();
                    robot_arm_left_.reset();
                    return false;
                }

                for (int i = 0; i < dof_ / 2; i++) {
                    config_right.mit_kp[i] = mit_kp_right[i];
                    config_right.mit_kd[i] = mit_kd_right[i];
                }
                config_right.interrupt_check = &OneroDriverNode::interruptRequested;
                config_right.interrupt_ctx = this;

                robot_arm_right_ = std::make_unique<onero_api::OneroArm>(config_right);
                if (!robot_arm_right_->valid()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create right arm robot handle (device: %s)",
                                device_right_.c_str());
                    // 清理左臂（已 enable_motors）
                    robot_arm_right_.reset();
                    robot_arm_left_->disable_motors();
                    robot_arm_left_.reset();
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "Right arm handle created successfully");

                int result_right = robot_arm_right_->enable_motors();
                if (result_right != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to enable right arm motors (code: %d, device: %s)",
                                result_right, device_right_.c_str());
                    // 右臂 enable_motors 失败：右臂未启用，直接 reset（=destroy）
                    robot_arm_right_.reset();
                    // 清理左臂（已 enable_motors）
                    robot_arm_left_->disable_motors();
                    robot_arm_left_.reset();
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "Right arm enabled: %s on %s",
                            symbol_right_.c_str(), device_right_.c_str());
                return true;
            } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception during initialization: %s", e.what());
                    return false;
            }
        }
    }
    
    void movejCallback(const onero_interfaces::msg::MoveJ::SharedPtr msg) {
        if(dof_ == 7){
            if (!robot_arm_) {
                std_msgs::msg::Bool result;
                result.data = false;
                movej_result_pub_->publish(result);
                return;
            }
        
            try {
                // Convert float32[] to double vector
                onero_api::JointArray joint_array(msg->joint.begin(), msg->joint.end());
                
                int ret = robot_arm_->movej(joint_array, msg->speed_scale, 
                                        msg->trajectory_connect);
                     
                std_msgs::msg::Bool result;
                result.data = (ret == 0);
                movej_result_pub_->publish(result);
                arm_status_ = 0;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "MoveJ exception: %s", e.what());
                std_msgs::msg::Bool result;
                result.data = false;
                movej_result_pub_->publish(result);
            }
        }

        if(dof_ == 14){
            if (!robot_arm_left_ || !robot_arm_right_) {
                std_msgs::msg::Bool result;
                result.data = false;
                movej_result_pub_->publish(result);
                return;
            }
        
            try {
                // Convert float32[] to double vector
                onero_api::JointArray joint_array(msg->joint.begin(), msg->joint.end());
                
                // 双臂模式：收到了14个关节角度，假设前7个是左臂，后7个是右臂
                if (joint_array.size() == static_cast<size_t>(dof_)) {
                    onero_api::JointArray left_joints(joint_array.begin(), joint_array.begin() + dof_ / 2);
                    onero_api::JointArray right_joints(joint_array.begin() + dof_ / 2, joint_array.end());
                    
                    // 使用多线程同时执行左右臂运动
                    std::atomic<int> ret_left{-1};
                    std::atomic<int> ret_right{-1};
                    
                    std::thread left_thread([this, &ret_left, left_joints, msg]() {
                        ret_left = robot_arm_left_->movej(left_joints, msg->speed_scale, 
                                                msg->trajectory_connect);
                    });
                    
                    std::thread right_thread([this, &ret_right, right_joints, msg]() {
                        ret_right = robot_arm_right_->movej(right_joints, msg->speed_scale, 
                                                msg->trajectory_connect);
                    });
                    
                    // 等待两个线程完成
                    left_thread.join();
                    right_thread.join();
                    
                    std_msgs::msg::Bool result;
                    result.data = (ret_left == 0 && ret_right == 0);
                    movej_result_pub_->publish(result);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "MoveJ: Expected 14 joints, got %zu", joint_array.size());
                    std_msgs::msg::Bool result;
                    result.data = false;
                    movej_result_pub_->publish(result);
                }
                left_arm_status_ = 0;
                right_arm_status_ = 0;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "MoveJ exception: %s", e.what());
                std_msgs::msg::Bool result;
                result.data = false;
                movej_result_pub_->publish(result);
            } 
        }
    }
    
    void movelCallback(const onero_interfaces::msg::MoveL::SharedPtr msg) {
        if(dof_ == 7){
            if (!robot_arm_) {
                std_msgs::msg::Bool result;
                result.data = false;
                movel_result_pub_->publish(result);
                return;
            }
            
            try {
                onero_api::Pose pose;
                pose.x = msg->pose.position.x;
                pose.y = msg->pose.position.y;
                pose.z = msg->pose.position.z;
                pose.qw = msg->pose.orientation.w;
                pose.qx = msg->pose.orientation.x;
                pose.qy = msg->pose.orientation.y;
                pose.qz = msg->pose.orientation.z;
                
                int ret = robot_arm_->movel(pose, msg->speed_scale,
                                        msg->trajectory_connect);
                std_msgs::msg::Bool result;
                result.data = (ret == 0);
                movel_result_pub_->publish(result);
                arm_status_ = 0;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "MoveL exception: %s", e.what());
                std_msgs::msg::Bool result;
                result.data = false;
                movel_result_pub_->publish(result);
            }
        }

        if(dof_ == 14){ 
            if (!robot_arm_left_ || !robot_arm_right_) {
                std_msgs::msg::Bool result;
                result.data = false;
                movel_result_pub_->publish(result);
                return;
            }
        
            try {
                onero_api::Pose pose;
                pose.x = msg->pose.position.x;
                pose.y = msg->pose.position.y;
                pose.z = msg->pose.position.z;
                pose.qw = msg->pose.orientation.w;
                pose.qx = msg->pose.orientation.x;
                pose.qy = msg->pose.orientation.y;
                pose.qz = msg->pose.orientation.z;
                
                // 双臂模式：这里只控制左臂，根据需求可以改为右臂或两个都控制
                int ret = robot_arm_left_->movel(pose, msg->speed_scale,
                                        msg->trajectory_connect);
                std_msgs::msg::Bool result;
                result.data = (ret == 0);
                movel_result_pub_->publish(result);
                left_arm_status_ = 0;
                right_arm_status_ = 0;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "MoveL exception: %s", e.what());
                std_msgs::msg::Bool result;
                result.data = false;
                movel_result_pub_->publish(result);
            }
        }  
    }

    void forcePositionMovelCallback(const onero_interfaces::msg::MoveL::SharedPtr msg) {
        if(dof_ == 7){
            if (!robot_arm_) {
                std_msgs::msg::Bool result;
                result.data = false;
                force_position_movel_result_pub_->publish(result);
                return;
            }

            try {
                onero_api::Pose pose;
                pose.x = msg->pose.position.x;
                pose.y = msg->pose.position.y;
                pose.z = msg->pose.position.z;
                pose.qw = msg->pose.orientation.w;
                pose.qx = msg->pose.orientation.x;
                pose.qy = msg->pose.orientation.y;
                pose.qz = msg->pose.orientation.z;

                int ret = robot_arm_->force_position_movel(pose, msg->speed_scale,
                                                        msg->trajectory_connect);
                std_msgs::msg::Bool result;
                result.data = (ret == 0);
                force_position_movel_result_pub_->publish(result);
                arm_status_ = 0;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "ForcePositionMoveL exception: %s", e.what());
                std_msgs::msg::Bool result;
                result.data = false;
                force_position_movel_result_pub_->publish(result);
            }
        }

        if(dof_ == 14){
            if (!robot_arm_left_ || !robot_arm_right_) {
                std_msgs::msg::Bool result;
                result.data = false;
                force_position_movel_result_pub_->publish(result);
                return;
            }

            try {
                onero_api::Pose pose;
                pose.x = msg->pose.position.x;
                pose.y = msg->pose.position.y;
                pose.z = msg->pose.position.z;
                pose.qw = msg->pose.orientation.w;
                pose.qx = msg->pose.orientation.x;
                pose.qy = msg->pose.orientation.y;
                pose.qz = msg->pose.orientation.z;

                int ret = robot_arm_left_->force_position_movel(pose, msg->speed_scale,
                                                        msg->trajectory_connect);
                std_msgs::msg::Bool result;
                result.data = (ret == 0);
                force_position_movel_result_pub_->publish(result);
                left_arm_status_ = 0;
                right_arm_status_ = 0;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "ForcePositionMoveL exception: %s", e.what());
                std_msgs::msg::Bool result;
                result.data = false;
                force_position_movel_result_pub_->publish(result);
            }
        }
    }
    

    void ForcePositionControlCallback(const onero_interfaces::msg::SetForcePosition::SharedPtr msg) 
    {
        if(dof_ == 7)
        {
            if (!robot_arm_) 
            {
                std_msgs::msg::Bool result;
                result.data = false;
                force_position_control_pub_->publish(result);
                return;
            }
            
            try 
            {
                onero_api::ForcePosition force_position;
                force_position.force_position_flag = true;
                force_position.direction = msg->direction;
                force_position.force = msg->force;

                int ret = robot_arm_->set_force_position_control(force_position);
                std_msgs::msg::Bool result;
                result.data = (ret == 0);
                force_position_control_pub_->publish(result);
                arm_status_ = 0;
            } catch (const std::exception& e) 
            {
                RCLCPP_ERROR(this->get_logger(), "ForcePositionControl exception: %s", e.what());
                std_msgs::msg::Bool result;
                result.data = false;
                force_position_control_pub_->publish(result);
            }
        }

        // if(dof_ == 14)
        // { 
        //     if (!robot_arm_left_ || !robot_arm_right_) 
        //     {
        //         std_msgs::msg::Bool result;
        //         result.data = false;
        //         movel_result_pub_->publish(result);
        //         return;
        //     }
        
        //     try {
        //         onero_api::Pose pose;
        //         pose.x = msg->pose.position.x;
        //         pose.y = msg->pose.position.y;
        //         pose.z = msg->pose.position.z;
        //         pose.qw = msg->pose.orientation.w;
        //         pose.qx = msg->pose.orientation.x;
        //         pose.qy = msg->pose.orientation.y;
        //         pose.qz = msg->pose.orientation.z;
                
        //         // 双臂模式：这里只控制左臂，根据需求可以改为右臂或两个都控制
        //         int ret = robot_arm_left_->movel(pose, msg->speed_scale,
        //                                 msg->trajectory_connect);
        //         std_msgs::msg::Bool result;
        //         result.data = (ret == 0);
        //         movel_result_pub_->publish(result);
        //         left_arm_status_ = 0;
        //         right_arm_status_ = 0;
        //     } catch (const std::exception& e) 
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "MoveL exception: %s", e.what());
        //         std_msgs::msg::Bool result;
        //         result.data = false;
        //         movel_result_pub_->publish(result);
        //     }
        // }  
    }

    void StopForcePostionControlCallback(const std_msgs::msg::Bool::SharedPtr msg) 
    {
        (void)msg;
        if(dof_ == 7)
        {
            if (!robot_arm_) 
            {
                std_msgs::msg::Bool result;
                result.data = false;
                stop_force_position_control_pub_->publish(result);
                return;
            }
            
            try 
            {                                                  
                int ret = robot_arm_->stop_force_position_control();
                std_msgs::msg::Bool result;
                result.data = (ret == 0);
                stop_force_position_control_pub_->publish(result);
                arm_status_ = 0;
            } catch (const std::exception& e) 
            {
                RCLCPP_ERROR(this->get_logger(), "StopForcePostionControl exception: %s", e.what());
                std_msgs::msg::Bool result;
                result.data = false;
                force_position_control_pub_->publish(result);
            }
        }

        // if(dof_ == 14)
        // { 
        //     if (!robot_arm_left_ || !robot_arm_right_) 
        //     {
        //         std_msgs::msg::Bool result;
        //         result.data = false;
        //         movel_result_pub_->publish(result);
        //         return;
        //     }
        
        //     try {
        //         onero_api::Pose pose;
        //         pose.x = msg->pose.position.x;
        //         pose.y = msg->pose.position.y;
        //         pose.z = msg->pose.position.z;
        //         pose.qw = msg->pose.orientation.w;
        //         pose.qx = msg->pose.orientation.x;
        //         pose.qy = msg->pose.orientation.y;
        //         pose.qz = msg->pose.orientation.z;
                
        //         // 双臂模式：这里只控制左臂，根据需求可以改为右臂或两个都控制
        //         int ret = robot_arm_left_->movel(pose, msg->speed_scale,
        //                                 msg->trajectory_connect);
        //         std_msgs::msg::Bool result;
        //         result.data = (ret == 0);
        //         movel_result_pub_->publish(result);
        //         left_arm_status_ = 0;
        //         right_arm_status_ = 0;
        //     } catch (const std::exception& e) 
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "MoveL exception: %s", e.what());
        //         std_msgs::msg::Bool result;
        //         result.data = false;
        //         movel_result_pub_->publish(result);
        //     }
        // }  
    }


    void movepCallback(const onero_interfaces::msg::MoveP::SharedPtr msg) {
        if(dof_ == 7){
            if (!robot_arm_) {
                std_msgs::msg::Bool result;
                result.data = false;
                movep_result_pub_->publish(result);
                return;
            }
        
            try {
                onero_api::Pose pose;
                pose.x = msg->pose.position.x;
                pose.y = msg->pose.position.y;
                pose.z = msg->pose.position.z;
                pose.qw = msg->pose.orientation.w;
                pose.qx = msg->pose.orientation.x;
                pose.qy = msg->pose.orientation.y;
                pose.qz = msg->pose.orientation.z;
                
                int ret = robot_arm_->movep(pose, msg->speed_scale, 
                                        msg->trajectory_connect);
                
                // 打印目标位姿和当前位姿
                // onero_api::Pose current_pose = robot_arm_->get_end_effector_pose();
                // RCLCPP_INFO(this->get_logger(), 
                //     "MoveP finished. Target Pose: [x: %.4f, y: %.4f, z: %.4f, w: %.4f, x: %.4f, y: %.4f, z: %.4f]",
                //     pose.x, pose.y, pose.z, pose.qw, pose.qx, pose.qy, pose.qz);
                // RCLCPP_INFO(this->get_logger(), 
                //     "Current Pose:  [x: %.4f, y: %.4f, z: %.4f, w: %.4f, x: %.4f, y: %.4f, z: %.4f]",
                //     current_pose.x, current_pose.y, current_pose.z, current_pose.qw, current_pose.qx, current_pose.qy, current_pose.qz);

                std_msgs::msg::Bool result;
                result.data = (ret == 0);
                movep_result_pub_->publish(result);
                arm_status_ = 0;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "MoveP exception: %s", e.what());
                std_msgs::msg::Bool result;
                result.data = false;
                movep_result_pub_->publish(result);
            }

        }

        if(dof_ == 14){
            if (!robot_arm_left_ || !robot_arm_right_) {
                std_msgs::msg::Bool result;
                result.data = false;
                movep_result_pub_->publish(result);
                return;
            }
            
            try {
                onero_api::Pose pose;
                pose.x = msg->pose.position.x;
                pose.y = msg->pose.position.y;
                pose.z = msg->pose.position.z;
                pose.qw = msg->pose.orientation.w;
                pose.qx = msg->pose.orientation.x;
                pose.qy = msg->pose.orientation.y;
                pose.qz = msg->pose.orientation.z;
                
                // 双臂模式：这里只控制左臂，根据需求可以改为右臂或两个都控制
                int ret = robot_arm_left_->movep(pose, msg->speed_scale, 
                                        msg->trajectory_connect);
                std_msgs::msg::Bool result;
                result.data = (ret == 0);
                movep_result_pub_->publish(result);
                left_arm_status_ = 0;
                right_arm_status_ = 0;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "MoveP exception: %s", e.what());
                std_msgs::msg::Bool result;
                result.data = false;
                movep_result_pub_->publish(result);
            }
        }
        
    }
    
    void clearBufferCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        if(dof_ == 7){
            if (!robot_arm_) {
                response->success = false;
                response->message = "Robot not initialized";
                return;
            }
            
            try {
                int ret = robot_arm_->clear_trajectory_buffer();
                response->success = (ret == 0);
                response->message = response->success ? "Buffer cleared" : "Failed to clear buffer";
            } catch (const std::exception& e) {
                response->success = false;
                response->message = std::string("Exception: ") + e.what();
            }
        }
        if(dof_ == 14){
            if (!robot_arm_left_ || !robot_arm_right_) {
                response->success = false;
                response->message = "Robot not initialized";
                return;
            }
            
            try {
                int ret_left = robot_arm_left_->clear_trajectory_buffer();
                int ret_right = robot_arm_right_->clear_trajectory_buffer();
                
                response->success = (ret_left == 0 && ret_right == 0);
                if (response->success) {
                    response->message = "Both arms buffer cleared";
                } else {
                    response->message = "Failed to clear buffer: left=" + std::to_string(ret_left) + 
                                        ", right=" + std::to_string(ret_right);
                }
            } catch (const std::exception& e) {
                response->success = false;
                response->message = std::string("Exception: ") + e.what();
            }
        }
    }
    
    void publishArmState() {
        if(dof_ == 7){
            if (!robot_arm_) return;
        
            try {
                // 根据硬件连接状态选择读取方式
                bool hardware_connected = robot_arm_->is_hardware_connected();
                onero_api::JointArray positions;
                
                if (hardware_connected) {
                    // 连接真实硬件：从电机读取实际位置、速度、力矩
                    onero_api::ArmStateFromMotor motor_state = robot_arm_->get_arm_state_from_motor();
                    positions = motor_state.positions;
                    // RCLCPP_INFO(
                    //     this->get_logger(),
                    //     "\033[32mPublishing state from hardware\033[0m"
                    // );
                    
                    // 发布 ArmState
                    auto state = onero_interfaces::msg::ArmState();
                    state.robot_model = robot_model_;
                    state.status = arm_status_;
                    
                    if (motor_state.positions.size() == static_cast<size_t>(dof_)) {
                        state.joint_positions.assign(motor_state.positions.begin(), motor_state.positions.end());
                    }
                    if (motor_state.velocities.size() == static_cast<size_t>(dof_)) {
                        state.joint_velocities.assign(motor_state.velocities.begin(), motor_state.velocities.end());
                    }
                    if (motor_state.torques.size() == static_cast<size_t>(dof_)) {
                        state.joint_torques.assign(motor_state.torques.begin(), motor_state.torques.end());
                    }
                    
                    onero_api::Pose ee_pose = robot_arm_->get_end_effector_pose();
                    state.end_effector_pose.position.x = ee_pose.x;
                    state.end_effector_pose.position.y = ee_pose.y;
                    state.end_effector_pose.position.z = ee_pose.z;
                    state.end_effector_pose.orientation.w = ee_pose.qw;
                    state.end_effector_pose.orientation.x = ee_pose.qx;
                    state.end_effector_pose.orientation.y = ee_pose.qy;
                    state.end_effector_pose.orientation.z = ee_pose.qz;
                    
                    arm_state_pub_->publish(state);
                    
                    // 发布 JointState 供 MoveIt 使用
                    auto joint_state = sensor_msgs::msg::JointState();
                    joint_state.header.stamp = this->now();
                    joint_state.header.frame_id = robot_model_ + "_arm";
                    
                    // 生成关节名称
                    for (int i = 0; i < dof_; i++) {
                        joint_state.name.push_back("joint" + std::to_string(i+1) + "-" + robot_model_);
                    }
                    
                    // 设置关节位置、速度、力矩（使用实际数据）
                    if (motor_state.positions.size() == static_cast<size_t>(dof_)) {
                        joint_state.position.assign(motor_state.positions.begin(), motor_state.positions.end());
                    }
                    if (motor_state.velocities.size() == static_cast<size_t>(dof_)) {
                        joint_state.velocity.assign(motor_state.velocities.begin(), motor_state.velocities.end());
                    }
                    if (motor_state.torques.size() == static_cast<size_t>(dof_)) {
                        joint_state.effort.assign(motor_state.torques.begin(), motor_state.torques.end());
                    }
                    
                    joint_state_pub_->publish(joint_state);
                    
                } else {
                    // 模拟环境：使用规划位置（缓存位置），速度和力矩为空
                    positions = robot_arm_->get_joint_positions();
                    
                    // 发布 ArmState
                    auto state = onero_interfaces::msg::ArmState();
                    state.robot_model = robot_model_;
                    state.status = arm_status_;
                    
                    if (positions.size() == static_cast<size_t>(dof_)) {
                        state.joint_positions.assign(positions.begin(), positions.end());
                    }
                    
                    onero_api::Pose ee_pose = robot_arm_->get_end_effector_pose();
                    state.end_effector_pose.position.x = ee_pose.x;
                    state.end_effector_pose.position.y = ee_pose.y;
                    state.end_effector_pose.position.z = ee_pose.z;
                    state.end_effector_pose.orientation.w = ee_pose.qw;
                    state.end_effector_pose.orientation.x = ee_pose.qx;
                    state.end_effector_pose.orientation.y = ee_pose.qy;
                    state.end_effector_pose.orientation.z = ee_pose.qz;
                    
                    arm_state_pub_->publish(state);
                    
                    // 发布 JointState 供 MoveIt 使用
                    auto joint_state = sensor_msgs::msg::JointState();
                    joint_state.header.stamp = this->now();
                    joint_state.header.frame_id = robot_model_ + "_arm";
                    
                    // 生成关节名称
                    for (int i = 0; i < dof_; i++) {
                        joint_state.name.push_back("joint" + std::to_string(i+1) + "-" + robot_model_);
                    }
                    
                    // 设置关节位置（速度和力矩为空）
                    if (positions.size() == static_cast<size_t>(dof_)) {
                        joint_state.position.assign(positions.begin(), positions.end());
                    }
                    joint_state.velocity.assign(dof_, 0.0);
                    joint_state.effort.assign(dof_, 0.0);
                    
                    joint_state_pub_->publish(joint_state);
                }
                
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Error publishing state: %s", e.what());
            }

        }
        if(dof_ == 14){
            if (!robot_arm_left_ || !robot_arm_right_) return;
        
            try {
                // 根据硬件连接状态选择读取方式
                bool hardware_connected_left = robot_arm_left_->is_hardware_connected();
                bool hardware_connected_right = robot_arm_right_->is_hardware_connected();

                if (hardware_connected_left && hardware_connected_right) {
                    /*  左臂和右臂都有连接真实硬件  */
                    //从仅有单臂连接切换到双臂连接真实硬件，取消警告
                    warn_signal_ = false;
                    if(last_warn_signal_ == true && warn_signal_ == false){
                        RCLCPP_INFO(this->get_logger(), "双臂均连接真实硬件。");
                        last_warn_signal_ = false;
                    }

                    onero_api::JointArray positions_dual;
                    onero_api::JointArray velocities_dual;
                    onero_api::JointArray torques_dual;

                    // 连接真实硬件（左臂）：从电机读取实际位置、速度、力矩
                    onero_api::ArmStateFromMotor motor_state_left = robot_arm_left_->get_arm_state_from_motor();

                    //发布 ArmState（左臂）
                    auto left_state = onero_interfaces::msg::ArmState();
                    left_state.robot_model = symbol_left_;
                    left_state.status = left_arm_status_;

                    if (motor_state_left.positions.size() == static_cast<size_t>(dof_/2)) {
                        left_state.joint_positions.assign(motor_state_left.positions.begin(), motor_state_left.positions.end());
                    }
                    if (motor_state_left.velocities.size() == static_cast<size_t>(dof_/2)) {
                        left_state.joint_velocities.assign(motor_state_left.velocities.begin(), motor_state_left.velocities.end());
                    }
                    if (motor_state_left.torques.size() == static_cast<size_t>(dof_/2)) {
                        left_state.joint_torques.assign(motor_state_left.torques.begin(), motor_state_left.torques.end());
                    }

                    // 末端位姿（左臂）
                    onero_api::Pose left_ee_pose = robot_arm_left_->get_end_effector_pose();
                    left_state.end_effector_pose.position.x = left_ee_pose.x;
                    left_state.end_effector_pose.position.y = left_ee_pose.y;
                    left_state.end_effector_pose.position.z = left_ee_pose.z;
                    left_state.end_effector_pose.orientation.w = left_ee_pose.qw;
                    left_state.end_effector_pose.orientation.x = left_ee_pose.qx;
                    left_state.end_effector_pose.orientation.y = left_ee_pose.qy;
                    left_state.end_effector_pose.orientation.z = left_ee_pose.qz;
                    
                    left_arm_state_pub_->publish(left_state);

                    // 连接真实硬件（右臂）：从电机读取实际位置、速度、力矩
                    onero_api::ArmStateFromMotor motor_state_right = robot_arm_right_->get_arm_state_from_motor();

                    //发布 ArmState（右臂）
                    auto right_state = onero_interfaces::msg::ArmState();
                    right_state.robot_model = symbol_right_;
                    right_state.status = right_arm_status_;

                    if (motor_state_right.positions.size() == static_cast<size_t>(dof_/2)) {
                        right_state.joint_positions.assign(motor_state_right.positions.begin(), motor_state_right.positions.end());
                    }
                    if (motor_state_right.velocities.size() == static_cast<size_t>(dof_/2)) {
                        right_state.joint_velocities.assign(motor_state_right.velocities.begin(), motor_state_right.velocities.end());
                    }
                    if (motor_state_right.torques.size() == static_cast<size_t>(dof_/2)) {
                        right_state.joint_torques.assign(motor_state_right.torques.begin(), motor_state_right.torques.end());
                    }

                    // 末端位姿（右臂）
                    onero_api::Pose right_ee_pose = robot_arm_right_->get_end_effector_pose();
                    right_state.end_effector_pose.position.x = right_ee_pose.x;
                    right_state.end_effector_pose.position.y = right_ee_pose.y;
                    right_state.end_effector_pose.position.z = right_ee_pose.z;
                    right_state.end_effector_pose.orientation.w = right_ee_pose.qw;
                    right_state.end_effector_pose.orientation.x = right_ee_pose.qx;
                    right_state.end_effector_pose.orientation.y = right_ee_pose.qy;
                    right_state.end_effector_pose.orientation.z = right_ee_pose.qz;
                    
                    right_arm_state_pub_->publish(right_state);

                    // 发布 JointState 供 MoveIt 使用（双臂合并）
                    auto joint_state = sensor_msgs::msg::JointState();
                    joint_state.header.stamp = this->now();
                    joint_state.header.frame_id = robot_model_;

                    // 生成关节名称：左臂 + 右臂
                    for (int i = 0; i < dof_ / 2; i++) {
                        joint_state.name.push_back("joint" + std::to_string(i+1) + "-" + symbol_left_);
                    }
                    for (int i = 0; i < dof_ / 2; i++) {
                        joint_state.name.push_back("joint" + std::to_string(i+1) + "-" + symbol_right_);
                    }

                    // 合并位置、速度、力矩
                    positions_dual.insert(positions_dual.end(), motor_state_left.positions.begin(), motor_state_left.positions.end());
                    positions_dual.insert(positions_dual.end(), motor_state_right.positions.begin(), motor_state_right.positions.end());

                    velocities_dual.insert(velocities_dual.end(), motor_state_left.velocities.begin(), motor_state_left.velocities.end());
                    velocities_dual.insert(velocities_dual.end(), motor_state_right.velocities.begin(), motor_state_right.velocities.end());

                    torques_dual.insert(torques_dual.end(), motor_state_left.torques.begin(), motor_state_left.torques.end());
                    torques_dual.insert(torques_dual.end(), motor_state_right.torques.begin(), motor_state_right.torques.end());

                    if (positions_dual.size() == static_cast<size_t>(dof_)) {
                        joint_state.position.assign(positions_dual.begin(), positions_dual.end());
                    }
                    if (velocities_dual.size() == static_cast<size_t>(dof_)) {
                        joint_state.velocity.assign(velocities_dual.begin(), velocities_dual.end());
                    }
                    if (torques_dual.size() == static_cast<size_t>(dof_)) {
                        joint_state.effort.assign(torques_dual.begin(), torques_dual.end());
                    }
                    joint_state_pub_->publish(joint_state);   
                }
                else if(!hardware_connected_left && !hardware_connected_right){
                    /*  左臂右臂未连接真实硬件  */
                    ////从仅有单臂连接切换到双臂切换到模拟模式，取消警告
                    warn_signal_ = false;
                    if(last_warn_signal_ == true && warn_signal_ == false){
                    RCLCPP_INFO(this->get_logger(), "检测到双臂均未连接真实硬件，已切换至模拟模式。");
                    last_warn_signal_ = false;
                    }

                    onero_api::JointArray positions_left;
                    onero_api::JointArray positions_right;
                    onero_api::JointArray positions_dual;

                    // 模拟环境（左臂未连接真实硬件）：使用规划位置（缓存位置），速度和力矩为空
                    positions_left = robot_arm_left_->get_joint_positions();
                    
                    //发布 ArmState（左臂）
                    auto left_state = onero_interfaces::msg::ArmState();
                    left_state.robot_model = symbol_left_;
                    left_state.status = left_arm_status_;

                    if (positions_left.size() == static_cast<size_t>(dof_/2)) {
                        left_state.joint_positions.assign(positions_left.begin(), positions_left.end());
                    }

                    // 末端位姿（左臂）
                    onero_api::Pose left_ee_pose = robot_arm_left_->get_end_effector_pose();
                    left_state.end_effector_pose.position.x = left_ee_pose.x;
                    left_state.end_effector_pose.position.y = left_ee_pose.y;
                    left_state.end_effector_pose.position.z = left_ee_pose.z;
                    left_state.end_effector_pose.orientation.w = left_ee_pose.qw;
                    left_state.end_effector_pose.orientation.x = left_ee_pose.qx;
                    left_state.end_effector_pose.orientation.y = left_ee_pose.qy;
                    left_state.end_effector_pose.orientation.z = left_ee_pose.qz;
                    
                    left_arm_state_pub_->publish(left_state);

                    // 模拟环境（右臂未连接真实硬件）：使用规划位置（缓存位置），速度和力矩为空
                    positions_right = robot_arm_right_->get_joint_positions();

                    // 发布 ArmState（右臂）
                    auto right_state = onero_interfaces::msg::ArmState();
                    right_state.robot_model = symbol_right_;
                    right_state.status = right_arm_status_;
                    
                    if (positions_right.size() == static_cast<size_t>(dof_/2)) {
                        right_state.joint_positions.assign(positions_right.begin(), positions_right.end());
                    }
                    
                    // 末端位姿（右臂）
                    onero_api::Pose right_ee_pose = robot_arm_right_->get_end_effector_pose();
                    right_state.end_effector_pose.position.x = right_ee_pose.x;
                    right_state.end_effector_pose.position.y = right_ee_pose.y;
                    right_state.end_effector_pose.position.z = right_ee_pose.z;
                    right_state.end_effector_pose.orientation.w = right_ee_pose.qw;
                    right_state.end_effector_pose.orientation.x = right_ee_pose.qx;
                    right_state.end_effector_pose.orientation.y = right_ee_pose.qy;
                    right_state.end_effector_pose.orientation.z = right_ee_pose.qz;
                    
                    right_arm_state_pub_->publish(right_state);

                    // 发布 JointState 供 MoveIt 使用（双臂合并）
                    auto joint_state = sensor_msgs::msg::JointState();
                    joint_state.header.stamp = this->now();
                    joint_state.header.frame_id = robot_model_;

                    // 生成关节名称：左臂 + 右臂
                    for (int i = 0; i < dof_ / 2; i++) {
                        joint_state.name.push_back("joint" + std::to_string(i+1) + "-" + symbol_left_);
                    }
                    for (int i = 0; i < dof_ / 2; i++) {
                        joint_state.name.push_back("joint" + std::to_string(i+1) + "-" + symbol_right_);
                    }

                    // 合并位置, 速度和力矩为0
                    positions_dual.insert(positions_dual.end(), positions_left.begin(), positions_left.end());
                    positions_dual.insert(positions_dual.end(), positions_right.begin(), positions_right.end());

                    if (positions_dual.size() == static_cast<size_t>(dof_)) {
                        joint_state.position.assign(positions_dual.begin(), positions_dual.end());
                    }
                    joint_state.velocity.assign(dof_, 0.0);           
                    joint_state.effort.assign(dof_, 0.0);
                    
                    joint_state_pub_->publish(joint_state);   
                }
                else {
                /* 非法情况只报一次警告： 只有一个臂连接真实硬件 */
                warn_signal_ = true;
                if(last_warn_signal_ == false && warn_signal_ == true){
                    RCLCPP_WARN(this->get_logger(), "检测到只有一只手臂连接了真实硬件，请检查连接状态！");
                    last_warn_signal_ = true;
                }   
                }       
                
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Error publishing state: %s", e.what());
            }
        }
        
    }
    
    // 双臂：规划手臂状态回调（接收onero_control_dual发布的规划信息）
    void planningArmsCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string new_status = msg->data;
        {
            std::lock_guard<std::mutex> lock(planning_arms_status_mutex_);
            planning_arms_status_ = new_status;
        }
        RCLCPP_INFO(this->get_logger(), "收到规划手臂状态: %s", new_status.c_str());
    }
    
    // ========== 方案C：轨迹缓冲和精准定时驱动执行 ==========
    
    // 接收完整轨迹并缓冲
    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        
        // 清空旧轨迹
        trajectory_buffer_.clear();
        
        // 存储新轨迹
        trajectory_buffer_ = msg->points;
        is_executing_ = false;  // 等待执行命令

        // // 将接收到的轨迹写入文件
        // static std::ofstream ofs("arm_record.txt", std::ios::trunc); // 使用 trunc 清空旧文件
        // if (ofs.is_open()) {
        //     int dof = dof_; // 使用成员变量 dof_
        //     for (const auto& point : trajectory_buffer_) {
        //         // 写入位置
        //         for (int i = 0; i < dof; ++i) {
        //             ofs << point.positions[i] << (i == dof - 1 ? "" : ",");
        //         }
        //         ofs << " "; // 位置和速度之间的分隔符

        //         // 写入速度
        //         for (int i = 0; i < dof; ++i) {
        //             ofs << point.velocities[i] << (i == dof - 1 ? "" : ",");
        //         }
        //         ofs << " "; // 速度和加速度之间的分隔符

        //         // 写入加速度
        //         for (int i = 0; i < dof; ++i) {
        //             ofs << point.accelerations[i] << (i == dof - 1 ? "" : ",");
        //         }
        //         ofs << "\n"; // 每个点占一行
        //     }
        //     ofs.flush();
        //     RCLCPP_INFO(this->get_logger(), "轨迹已写入 arm_record.txt (包含时间戳)");
        // } else {
        //     RCLCPP_ERROR(this->get_logger(), "无法打开 arm_record.txt 进行写入");
        // }
        
        RCLCPP_INFO(this->get_logger(), "接收到完整轨迹: %zu 个点", trajectory_buffer_.size());        // 调试：打印前几个点的信息
        if (!trajectory_buffer_.empty()) {
            const auto& first_point = trajectory_buffer_.front();
            const auto& last_point = trajectory_buffer_.back();
            double total_time = rclcpp::Duration(last_point.time_from_start).seconds();
            
            RCLCPP_INFO(this->get_logger(), "   轨迹时长: %.2f 秒", total_time);
            RCLCPP_INFO(this->get_logger(), "   第一个点位置维度: %zu, 速度维度: %zu", 
                        first_point.positions.size(), first_point.velocities.size());
        }
    }
    

    /*send_trajectory的方式，开始执行轨迹,executeTrajectoryFullLoop*/
    void executeCallback(const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;  // 未使用的参数
        
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        
        if (trajectory_buffer_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "轨迹缓冲为空，无法执行！");
            return;
        }
        
        if (is_executing_) {
            RCLCPP_WARN(this->get_logger(), "轨迹正在执行中，忽略重复执行命令");
            return;
        }
        
        // 等待之前的线程结束
        if (execution_thread_.joinable()) {
            execution_thread_.join();
        }
        
        // 重置执行状态
        is_executing_ = true;
        
        RCLCPP_INFO(this->get_logger(), "开始执行轨迹（独立线程 + 精确10ms循环）");
        
        // 启动执行线程
        execution_thread_ = std::thread(&OneroDriverNode::executeTrajectoryFullLoop, this);
    }

    /*send_trajectory方式，executeTrajectoryFullLoop：使用完整轨迹一次性下发*/
    void executeTrajectoryFullLoop() {
        if(dof_ == 7){
            if (!robot_arm_) {
                RCLCPP_ERROR(this->get_logger(), "机器人未初始化");
                is_executing_ = false;
                auto msg = std_msgs::msg::Bool();
                msg.data = false;
                execution_result_pub_->publish(msg);
                return;
            }
        }
        if(dof_ == 14){
            if (!robot_arm_left_ || !robot_arm_right_) {
                RCLCPP_ERROR(this->get_logger(), "机器人未初始化");
                is_executing_ = false;
                auto msg = std_msgs::msg::Bool();
                msg.data = false;
                execution_result_pub_->publish(msg);
                return;
            }
       }
    
        // 获取轨迹副本（避免长时间持有锁）
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> local_trajectory;
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            local_trajectory = trajectory_buffer_;
        }
        
        if (local_trajectory.empty()) {
            RCLCPP_ERROR(this->get_logger(), "轨迹为空");
            is_executing_ = false;
            auto msg = std_msgs::msg::Bool();
            msg.data = false;
            execution_result_pub_->publish(msg);
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "========== 使用 send_trajectory 执行完整轨迹 ==========");
        RCLCPP_INFO(this->get_logger(), "轨迹点数: %zu，控制周期: %.3f ms", 
                    local_trajectory.size(), time_step_ * 1000.0);
        
        // 检查第一个点的数据有效性
        const auto& first_point = local_trajectory.front();
        if (first_point.positions.size() != static_cast<size_t>(dof_) ||
            first_point.velocities.size() != static_cast<size_t>(dof_) ||
            first_point.accelerations.size() != static_cast<size_t>(dof_)) {
            RCLCPP_ERROR(this->get_logger(), 
                "轨迹数据维度错误: pos=%zu, vel=%zu, acc=%zu (期望 %d)", 
                first_point.positions.size(), first_point.velocities.size(), 
                first_point.accelerations.size(), dof_);
            is_executing_ = false;
            auto msg = std_msgs::msg::Bool();
            msg.data = false;
            execution_result_pub_->publish(msg);
            return;
        }
        
        auto start_time = std::chrono::high_resolution_clock::now();//debug,jingyi
        
        if(dof_ == 7){
            try {
                // ========== 构建单臂的完整轨迹 ==========
                std::vector<onero_api::TrajectoryPoint> trajectory;
                trajectory.reserve(local_trajectory.size());
                
                for (const auto& point : local_trajectory) {
                    onero_api::TrajectoryPoint traj_point;
                    traj_point.position = point.positions;
                    traj_point.velocity = point.velocities;
                    traj_point.acceleration = point.accelerations;
                    trajectory.push_back(traj_point);
                }

                RCLCPP_INFO(this->get_logger(), "轨迹构建完成: %zu 个点", trajectory.size());

                // ========== 执行轨迹 ==========
                int result = robot_arm_->send_trajectory(trajectory);
                if (result == 0) {
                    RCLCPP_INFO(this->get_logger(), "轨迹执行完成");
                    auto msg = std_msgs::msg::Bool();
                    msg.data = true;
                    execution_result_pub_->publish(msg);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "轨迹执行失败 (code: %d)", result);
                    is_executing_ = false;
                    auto msg = std_msgs::msg::Bool();
                    msg.data = false;
                    execution_result_pub_->publish(msg);
                    return;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "执行完整轨迹异常: %s", e.what());
                is_executing_ = false;
                auto msg = std_msgs::msg::Bool();
                msg.data = false;
                execution_result_pub_->publish(msg);
                return;
            }
        }

        if(dof_ == 14){
            std::string planning_arms_status;
            {
                std::lock_guard<std::mutex> lock(planning_arms_status_mutex_);
                planning_arms_status = planning_arms_status_;
            }
            try {
                // ========== 构建左臂和右臂的完整轨迹 ==========
                std::vector<onero_api::TrajectoryPoint> trajectory_left;
                std::vector<onero_api::TrajectoryPoint> trajectory_right;
                trajectory_left.reserve(local_trajectory.size());
                trajectory_right.reserve(local_trajectory.size());
                
                for (const auto& point : local_trajectory) {
                    // 左臂轨迹点（前7个关节）
                    onero_api::TrajectoryPoint traj_point_left;
                    traj_point_left.position.assign(point.positions.begin(), point.positions.begin() + dof_ / 2);
                    traj_point_left.velocity.assign(point.velocities.begin(), point.velocities.begin() + dof_ / 2);
                    traj_point_left.acceleration.assign(point.accelerations.begin(), point.accelerations.begin() + dof_ / 2);
                    trajectory_left.push_back(traj_point_left);
                    
                    // 右臂轨迹点（后7个关节）
                    onero_api::TrajectoryPoint traj_point_right;
                    traj_point_right.position.assign(point.positions.begin() + dof_ / 2, point.positions.end());
                    traj_point_right.velocity.assign(point.velocities.begin() + dof_ / 2, point.velocities.end());
                    traj_point_right.acceleration.assign(point.accelerations.begin() + dof_ / 2, point.accelerations.end());
                    trajectory_right.push_back(traj_point_right);
                }
                
                RCLCPP_INFO(this->get_logger(), "轨迹构建完成: 左臂 %zu 个点, 右臂 %zu 个点", 
                            trajectory_left.size(), trajectory_right.size());
                
                // ========== 根据规划手臂状态执行轨迹 ==========
                if (planning_arms_status == "dual") {
                    // 双臂同时执行：启动两个线程
                    RCLCPP_INFO(this->get_logger(), "双臂模式：同时执行左右臂轨迹");
                    
                    std::thread left_thread([this, trajectory_left]() {
                        int result = robot_arm_left_->send_trajectory(trajectory_left);
                        if (result == 0) {
                            RCLCPP_INFO(this->get_logger(), "左臂轨迹执行完成");
                            auto msg = std_msgs::msg::Bool();
                            msg.data = true;
                            execution_result_pub_->publish(msg);
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "左臂轨迹执行失败 (code: %d)", result);
                            is_executing_ = false;
                            auto msg = std_msgs::msg::Bool();
                            msg.data = false;
                            execution_result_pub_->publish(msg);
                            return;
                        }
                    });
                    
                    std::thread right_thread([this, trajectory_right]() {
                        int result = robot_arm_right_->send_trajectory(trajectory_right);
                        if (result == 0) {
                            RCLCPP_INFO(this->get_logger(), "右臂轨迹执行完成");
                            auto msg = std_msgs::msg::Bool();
                            msg.data = true;
                            execution_result_pub_->publish(msg);
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "右臂轨迹执行失败 (code: %d)", result);
                            is_executing_ = false;
                            auto msg = std_msgs::msg::Bool();
                            msg.data = false;
                            execution_result_pub_->publish(msg);
                            return;
                        }
                    });
                    
                    // 等待两个线程完成
                    left_thread.join();
                    right_thread.join();
                    
                } else if (planning_arms_status == "left") {
                    // 仅左臂执行
                    RCLCPP_INFO(this->get_logger(), "左臂模式：执行左臂轨迹");
                    int result = robot_arm_left_->send_trajectory(trajectory_left);
                    if (result == 0) {
                        RCLCPP_INFO(this->get_logger(), "左臂轨迹执行完成");
                        auto msg = std_msgs::msg::Bool();
                        msg.data = true;
                        execution_result_pub_->publish(msg);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "左臂轨迹执行失败 (code: %d)", result);
                        is_executing_ = false;
                        auto msg = std_msgs::msg::Bool();
                        msg.data = false;
                        execution_result_pub_->publish(msg);
                        return;
                    }
                    
                } else if (planning_arms_status == "right") {
                    // 仅右臂执行
                    RCLCPP_INFO(this->get_logger(), "右臂模式：执行右臂轨迹");
                    int result = robot_arm_right_->send_trajectory(trajectory_right);
                    if (result == 0) {
                        RCLCPP_INFO(this->get_logger(), "右臂轨迹执行完成");
                        auto msg = std_msgs::msg::Bool();
                        msg.data = true;
                        execution_result_pub_->publish(msg);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "右臂轨迹执行失败 (code: %d)", result);
                        is_executing_ = false;
                        auto msg = std_msgs::msg::Bool();
                        msg.data = false;
                        execution_result_pub_->publish(msg);
                        return;
                    }
                    
                } else {
                    RCLCPP_WARN(this->get_logger(), "未知的规划手臂状态（双臂的起始和目标一致）: %s", planning_arms_status.c_str());
                    is_executing_ = false;
                    auto msg = std_msgs::msg::Bool();
                    msg.data = false;
                    execution_result_pub_->publish(msg);
                    return;
                }
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "执行完整轨迹异常: %s", e.what());
                is_executing_ = false;
                auto msg = std_msgs::msg::Bool();
                msg.data = false;
                execution_result_pub_->publish(msg);
                return;
            }
        }

        // ========== 执行完成统计，Debug ==========
        auto end_time = std::chrono::high_resolution_clock::now();
        double total_time = std::chrono::duration<double>(end_time - start_time).count();
        
        RCLCPP_INFO(this->get_logger(), 
            "========================================");
        RCLCPP_INFO(this->get_logger(), 
            "完整轨迹执行完成！");
        RCLCPP_INFO(this->get_logger(), 
            "   轨迹点数: %zu", local_trajectory.size());
        RCLCPP_INFO(this->get_logger(), 
            "   总耗时: %.2f 秒", total_time);
        RCLCPP_INFO(this->get_logger(), 
            "========================================");

        is_executing_ = false;//执行中标志为真，用来拒绝执行多次按下执行按钮
    }
    
     /*send_trajectory的方式，取消执行*/
    void cancelCallback(const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;  // 未使用的参数
        
        if (!is_executing_) {
            RCLCPP_INFO(this->get_logger(), "当前没有执行中的轨迹");
            return;
        }

        if(dof_ == 7){
            int result = robot_arm_->cancel_trajectory();
            if (result == 0) {
                RCLCPP_WARN(this->get_logger(), "轨迹执行取消信号已发送");
            } else {
                RCLCPP_ERROR(this->get_logger(), "轨迹取消失败 (code: %d)", result);
            }
        }

        if(dof_ == 14){
            std::string planning_arms_status;
            {
                std::lock_guard<std::mutex> lock(planning_arms_status_mutex_);
                planning_arms_status = planning_arms_status_;
            }
            //根据规划的手臂来取消对应的轨迹
            if (planning_arms_status == "left") {
                int result = robot_arm_left_->cancel_trajectory();
                if (result == 0) {
                    RCLCPP_WARN(this->get_logger(), "左臂轨迹执行取消信号已发送");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "左臂轨迹取消失败 (code: %d)", result);
                }
            } else if (planning_arms_status == "right") {
                 int result = robot_arm_right_->cancel_trajectory();
                if (result == 0) {
                    RCLCPP_WARN(this->get_logger(), "右臂轨迹执行取消信号已发送");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "右臂轨迹取消失败 (code: %d)", result);
                }
            } else if (planning_arms_status == "dual") {
                int result_left = robot_arm_left_->cancel_trajectory();
                int result_right = robot_arm_right_->cancel_trajectory();
                if (result_left == 0 && result_right == 0) {
                    RCLCPP_WARN(this->get_logger(), "双臂轨迹执行取消信号已发送");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "双臂轨迹取消失败 (left code: %d, right code: %d)", result_left, result_right);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "未知的规划手臂状态（双臂的起始和目标一致）: %s", planning_arms_status.c_str());
            }

        }
    }
    
    // ========== 新增DEBUG：测试轨迹回调（使用send_trajectory_point API）==========
    void testTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        if (!robot_arm_left_ || !robot_arm_right_) {
            RCLCPP_ERROR(this->get_logger(), "机器人未初始化");
            return;
        }
        
        if (msg->points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "接收到空轨迹");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "========== 开始执行测试轨迹 ==========");
        RCLCPP_INFO(this->get_logger(), "轨迹点数: %zu", msg->points.size());
        
        // 检查第一个点的数据维度
        const auto& first_point = msg->points.front();
        RCLCPP_INFO(this->get_logger(), "第一个点 - 位置数: %zu, 速度数: %zu, 加速度数: %zu",
                    first_point.positions.size(), 
                    first_point.velocities.size(),
                    first_point.accelerations.size());
        
        // 启动独立线程执行测试轨迹（不阻塞主线程）
        std::thread([this, msg]() {
            auto start_time = std::chrono::high_resolution_clock::now();
            size_t executed_points = 0;
            
            for (size_t i = 0; i < msg->points.size(); ++i) {
                const auto& point = msg->points[i];
                
                // 检查数据维度（支持7自由度和14自由度）
                int expected_dof = point.positions.size();
                bool is_left_arm_only = (expected_dof == 7);
                
                if (expected_dof != 7 && expected_dof != 14) {
                    RCLCPP_ERROR(this->get_logger(), 
                        "轨迹点 %zu 数据维度错误: %d (期望 7 或 14)", i, expected_dof);
                    break;
                }
                
                if (point.velocities.size() != static_cast<size_t>(expected_dof) ||
                    point.accelerations.size() != static_cast<size_t>(expected_dof)) {
                    RCLCPP_ERROR(this->get_logger(), 
                        "轨迹点 %zu 速度/加速度维度不匹配: pos=%d, vel=%zu, acc=%zu",
                        i, expected_dof, point.velocities.size(), point.accelerations.size());
                    break;
                }
                
                try {
                    if (is_left_arm_only) {
                        // 7自由度模式：只控制左臂
                        onero_api::JointArray positions(point.positions.begin(), point.positions.end());
                        onero_api::JointArray velocities(point.velocities.begin(), point.velocities.end());
                        onero_api::JointArray accelerations(point.accelerations.begin(), point.accelerations.end());
                        
                        int result = robot_arm_left_->send_trajectory_point(
                                                                    positions,
                                                                    velocities);
                        // int result = robot_arm_right_->send_trajectory_point( 
                        //                                             positions, 
                        //                                             velocities,
                        //                                             accelerations);//jingyi
                        if (result != 0) {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "左臂轨迹点发送失败 (code: %d)", result);
                        }
                    } else {
                        // 14自由度模式：控制双臂
                        onero_api::JointArray positions_left(point.positions.begin(), 
                                                            point.positions.begin() + 7);
                        onero_api::JointArray velocities_left(point.velocities.begin(), 
                                                             point.velocities.begin() + 7);
                        onero_api::JointArray accelerations_left(point.accelerations.begin(), 
                                                                point.accelerations.begin() + 7);
                        
                        onero_api::JointArray positions_right(point.positions.begin() + 7, 
                                                             point.positions.end());
                        onero_api::JointArray velocities_right(point.velocities.begin() + 7, 
                                                              point.velocities.end());
                        onero_api::JointArray accelerations_right(point.accelerations.begin() + 7, 
                                                                 point.accelerations.end());
                        
                        int result_left = robot_arm_left_->send_trajectory_point(
                                                                         positions_left,
                                                                         velocities_left);
                        int result_right = robot_arm_right_->send_trajectory_point(
                                                                          positions_right,
                                                                          velocities_right);
                        
                        if (result_left != 0 || result_right != 0) {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "双臂轨迹点发送失败 (left: %d, right: %d)", result_left, result_right);
                        }
                    }
                    
                    executed_points++;
                    
                    // 进度输出（每100个点）
                    if (executed_points % 100 == 0 || (i + 1) >= msg->points.size()) {
                        double progress = (double)(i + 1) / msg->points.size() * 100.0;
                        auto now = std::chrono::high_resolution_clock::now();
                        double elapsed = std::chrono::duration<double>(now - start_time).count();
                        RCLCPP_INFO(this->get_logger(), 
                            "测试轨迹执行进度: %.1f%% (%zu/%zu), 已用时: %.2f秒", 
                            progress, i + 1, msg->points.size(), elapsed);
                    }
                    
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "发送测试轨迹点 %zu 异常: %s", i, e.what());
                    break;
                }
                
                // 精确10ms sleep控制
                if (i < msg->points.size() - 1) {
                    auto target_time = start_time + std::chrono::microseconds(
                        static_cast<long long>((i + 1) * this->time_step_ * 1000000));
                    auto now = std::chrono::high_resolution_clock::now();
                    auto remaining_time = std::chrono::duration_cast<std::chrono::microseconds>(
                        target_time - now);
                    
                    if (remaining_time.count() > 0) {
                        usleep(static_cast<unsigned int>(remaining_time.count()));
                    } else if (remaining_time.count() < -10000) {
                        if (i % 100 == 0) {
                            RCLCPP_WARN(this->get_logger(), 
                                "测试轨迹点 %zu 执行延迟 %.2f ms", 
                                i, -remaining_time.count() / 1000.0);
                        }
                    }
                }
            }
            
            // 执行完成
            auto end_time = std::chrono::high_resolution_clock::now();
            double total_time = std::chrono::duration<double>(end_time - start_time).count();
            
            RCLCPP_INFO(this->get_logger(), 
                "✅ 测试轨迹执行完成！共执行 %zu 个点，耗时 %.2f 秒", 
                executed_points, total_time);
        }).detach();  // 分离线程，让它独立执行
    }
    
    // ========== 新增DEBUG：完整轨迹执行回调（使用send_trajectory API）==========
    void testSendTrajectoryFullCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        if (!robot_arm_left_ || !robot_arm_right_) {
            RCLCPP_ERROR(this->get_logger(), "机器人未初始化");
            return;
        }
        
        if (msg->points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "接收到空轨迹");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "========== 开始执行完整轨迹（send_trajectory API）==========");
        RCLCPP_INFO(this->get_logger(), "轨迹点数: %zu", msg->points.size());
        
        // 检查第一个点的数据维度
        const auto& first_point = msg->points.front();
        RCLCPP_INFO(this->get_logger(), "第一个点 - 位置数: %zu, 速度数: %zu, 加速度数: %zu",
                    first_point.positions.size(), 
                    first_point.velocities.size(),
                    first_point.accelerations.size());
        
        // 启动独立线程执行（不阻塞主线程）
        std::thread([this, msg]() {
            auto start_time = std::chrono::high_resolution_clock::now();
            
            // 检查数据维度（支持7自由度和14自由度）
            int expected_dof = msg->points.front().positions.size();
            bool is_left_arm_only = (expected_dof == 7);
            
            if (expected_dof != 7 && expected_dof != 14) {
                RCLCPP_ERROR(this->get_logger(), 
                    "轨迹数据维度错误: %d (期望 7 或 14)", expected_dof);
                return;
            }
            
            try {
                if (is_left_arm_only) {
                    // ========== 7自由度模式：只控制左臂/或测右臂 ==========
                    std::vector<onero_api::TrajectoryPoint> trajectory;
                    trajectory.reserve(msg->points.size());
                    
                    for (const auto& point : msg->points) {
                        onero_api::TrajectoryPoint traj_point;
                        traj_point.position.assign(point.positions.begin(), point.positions.end());
                        traj_point.velocity.assign(point.velocities.begin(), point.velocities.end());
                        traj_point.acceleration.assign(point.accelerations.begin(), point.accelerations.end());
                        trajectory.push_back(traj_point);
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "准备发送左臂轨迹: %zu 个点", trajectory.size());
                    
                    int result = robot_arm_left_->send_trajectory(trajectory);
                    // int result = robot_arm_right_->send_trajectory(trajectory);//jingyi

                    auto end_time = std::chrono::high_resolution_clock::now();
                    double total_time = std::chrono::duration<double>(end_time - start_time).count();
                    
                    if (result == 0) {
                        RCLCPP_INFO(this->get_logger(), 
                            "✅ 左臂完整轨迹执行完成！共 %zu 个点，耗时 %.2f 秒", 
                            trajectory.size(), total_time);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), 
                            "❌ 左臂完整轨迹执行失败 (code: %d)，耗时 %.2f 秒", 
                            result, total_time);
                    }
                    
                } else {
                    // ========== 14自由度模式：控制双臂 ==========
                    std::vector<onero_api::TrajectoryPoint> trajectory_left;
                    std::vector<onero_api::TrajectoryPoint> trajectory_right;
                    trajectory_left.reserve(msg->points.size());
                    trajectory_right.reserve(msg->points.size());
                    
                    for (const auto& point : msg->points) {
                        // 左臂轨迹点
                        onero_api::TrajectoryPoint traj_point_left;
                        traj_point_left.position.assign(point.positions.begin(), point.positions.begin() + 7);
                        traj_point_left.velocity.assign(point.velocities.begin(), point.velocities.begin() + 7);
                        traj_point_left.acceleration.assign(point.accelerations.begin(), point.accelerations.begin() + 7);
                        trajectory_left.push_back(traj_point_left);
                        
                        // 右臂轨迹点
                        onero_api::TrajectoryPoint traj_point_right;
                        traj_point_right.position.assign(point.positions.begin() + 7, point.positions.end());
                        traj_point_right.velocity.assign(point.velocities.begin() + 7, point.velocities.end());
                        traj_point_right.acceleration.assign(point.accelerations.begin() + 7, point.accelerations.end());
                        trajectory_right.push_back(traj_point_right);
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "准备发送双臂轨迹: 左臂 %zu 个点, 右臂 %zu 个点", 
                                trajectory_left.size(), trajectory_right.size());
                    
                    // 启动两个线程同时执行左右臂轨迹
                    std::thread left_thread([this, trajectory_left]() {
                        int result = robot_arm_left_->send_trajectory(trajectory_left);
                        if (result == 0) {
                            RCLCPP_INFO(this->get_logger(), "✅ 左臂轨迹执行完成");
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "❌ 左臂轨迹执行失败 (code: %d)", result);
                        }
                    });
                    
                    std::thread right_thread([this, trajectory_right]() {
                        int result = robot_arm_right_->send_trajectory(trajectory_right);
                        if (result == 0) {
                            RCLCPP_INFO(this->get_logger(), "✅ 右臂轨迹执行完成");
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "❌ 右臂轨迹执行失败 (code: %d)", result);
                        }
                    });
                    
                    // 等待两个线程完成
                    left_thread.join();
                    right_thread.join();
                    
                    auto end_time = std::chrono::high_resolution_clock::now();
                    double total_time = std::chrono::duration<double>(end_time - start_time).count();
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "✅ 双臂完整轨迹执行完成！共 %zu 个点，耗时 %.2f 秒", 
                        trajectory_left.size(), total_time);
                }
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "执行完整轨迹异常: %s", e.what());
            }
        }).detach();  // 分离线程，让它独立执行
    }

    // 服务回调函数
    void GetEndPoseCallback(
        const std::shared_ptr<onero_interfaces::srv::EndEffectorPose::Request> request,
        std::shared_ptr<onero_interfaces::srv::EndEffectorPose::Response> response)
    {
        (void)request; // 请求为空

        if(dof_ == 7)
        {
            if (!robot_arm_) return;
        
            try {                                                                                                                                                                    
                onero_api::Pose ee_pose = robot_arm_->get_end_effector_pose();
                response->pose.position.x = ee_pose.x;
                response->pose.position.y = ee_pose.y;
                response->pose.position.z = ee_pose.z;
                response->pose.orientation.w = ee_pose.qw;
                response->pose.orientation.x = ee_pose.qx;
                response->pose.orientation.y = ee_pose.qy;
                response->pose.orientation.z = ee_pose.qz;                                                                              
            } catch (const std::exception& e) 
            {
                RCLCPP_WARN(this->get_logger(), "Error publishing state: %s", e.what());
            }
        }

        // RCLCPP_DEBUG(this->get_logger(), "Sent Pose: x=%.2f, y=%.2f, z=%.2f", 
        //                 response->pose.position.x, 
        //                 response->pose.position.y, 
        //                 response->pose.position.z);
    }


};



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OneroDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
