#include <pinocchio/fwd.hpp>  // ensure pinocchio precedes any Boost from rclcpp

#include "rclcpp/rclcpp.hpp"
#include "woan_interfaces/msg/move_j.hpp"
#include "woan_interfaces/msg/move_l.hpp"
#include "woan_interfaces/msg/move_p.hpp"
#include "woan_interfaces/msg/arm_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "woan_api/woan_interface.h"
#include "woan_api/woan_define.h"

#include <thread>
#include <mutex>
#include <vector>
#include <fstream> // 用于文件操作
#include <unistd.h>  // for usleep

using woan_api::WoanAPI;

class WoanDriverNode : public rclcpp::Node {
public:
    WoanDriverNode() : Node("woan_driver_node"), is_executing_(false), should_cancel_(false), time_step_(0.01) {  // time_step_是debug用的                       
        this->declare_parameter<int>("dof", 14);
        this->declare_parameter<double>("state_pub_rate", 100.0);

        dof_ = this->get_parameter("dof").as_int();      
        double state_pub_rate = this->get_parameter("state_pub_rate").as_double();

        if(dof_ == 7){
            robot_handle_ = nullptr;
            arm_status_ = 0;

            this->declare_parameter<std::string>("robot_model", "x1_l");
            this->declare_parameter<std::string>("device", "/dev/ttyACM0");

            robot_model_ = this->get_parameter("robot_model").as_string();
            device_ = this->get_parameter("device").as_string();

            RCLCPP_INFO(this->get_logger(), "Initializing WoanDriver: %s (DOF: %d)", 
            robot_model_.c_str(), dof_);
        }
        else if(dof_ == 14){
            robot_handle_left_= nullptr;
            robot_handle_right_= nullptr;
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

            RCLCPP_INFO(this->get_logger(), "Initializing WoanDriver using SO library: %s (DOF: %d), Controlled Arm: %s, %s", 
                    robot_model_.c_str(), dof_, symbol_left_.c_str(), symbol_right_.c_str());
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
        movej_sub_ = this->create_subscription<woan_interfaces::msg::MoveJ>(
            "/woan_driver/movej_cmd", 10,
            std::bind(&WoanDriverNode::movejCallback, this, std::placeholders::_1));
        //仅单臂
        movel_sub_ = this->create_subscription<woan_interfaces::msg::MoveL>(
            "/woan_driver/movel_cmd", 10,
            std::bind(&WoanDriverNode::movelCallback, this, std::placeholders::_1));
        //仅单臂
        movep_sub_ = this->create_subscription<woan_interfaces::msg::MoveP>(
            "/woan_driver/movep_cmd", 10,
            std::bind(&WoanDriverNode::movepCallback, this, std::placeholders::_1));
        
        movej_result_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/woan_driver/movej_result", 10);
        
        movel_result_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/woan_driver/movel_result", 10);
        
        movep_result_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/woan_driver/movep_result", 10);
        
        //单臂的状态发布
        arm_state_pub_ = this->create_publisher<woan_interfaces::msg::ArmState>(
        "/woan_driver/arm_state", 10);

        //双臂需左右臂状态发布
        left_arm_state_pub_ = this->create_publisher<woan_interfaces::msg::ArmState>(
            "/woan_driver/left_arm_state", 10);
        right_arm_state_pub_ = this->create_publisher<woan_interfaces::msg::ArmState>(
            "/woan_driver/right_arm_state", 10);
        
        //发布 /joint_states 供 MoveIt 使用
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        
        clear_buffer_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/woan_driver/clear_trajectory_buffer",
            std::bind(&WoanDriverNode::clearBufferCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // 双臂：订阅规划手臂状态（由woan_control_dual发布）
        planning_arms_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/moveit_planning_arms_status", 10,
            std::bind(&WoanDriverNode::planningArmsCallback, this, std::placeholders::_1));
        
        // ========== 订阅完整轨迹和执行命令 ==========
        trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/moveit_trajectory", 10,
            std::bind(&WoanDriverNode::trajectoryCallback, this, std::placeholders::_1));
        
        ///*send_trajectory_point的方式*/
        // execute_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        //     "/moveit_execute", 10,
        //     std::bind(&WoanDriverNode::executeCallback, this, std::placeholders::_1));

        /*send_trajectory的方式*/
        execute_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/moveit_execute", 10,
            std::bind(&WoanDriverNode::executeCallback, this, std::placeholders::_1));
        
        cancel_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/moveit_cancel", 10,
            std::bind(&WoanDriverNode::cancelCallback, this, std::placeholders::_1));
        
        // 传统模式：执行成功的状态反馈
        execution_result_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/woan_driver/trajectory_execution_result", 10);
        
        // 新增DEBUG：订阅测试轨迹（用于replay_trajectory_from_file）
        test_trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/test_trajectory", 10,
            std::bind(&WoanDriverNode::testTrajectoryCallback, this, std::placeholders::_1));
        
        //RCLCPP_INFO(this->get_logger(), "已订阅 /test_trajectory 用于轨迹测试");
        
        // 新增DEBUG：订阅完整轨迹并使用send_trajectory执行
        test_send_trajectory_full_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/test_send_trajectory_full", 10,
            std::bind(&WoanDriverNode::testSendTrajectoryFullCallback, this, std::placeholders::_1));
        
        //RCLCPP_INFO(this->get_logger(), "已订阅 /test_send_trajectory_full 用于完整轨迹执行");
        
        auto state_period = std::chrono::duration<double>(1.0 / state_pub_rate);
        state_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(state_period),
            std::bind(&WoanDriverNode::publishArmState, this));
        
        //RCLCPP_INFO(this->get_logger(), "WoanDriver initialized successfully using SO library");
    }
    
    ~WoanDriverNode() {
        // 停止执行线程
        should_cancel_ = true;
        if (execution_thread_.joinable()) {
            execution_thread_.join();
        }

        // 单臂
        if(dof_ == 7 && robot_handle_){  
            WoanAPI::disable_motors(robot_handle_);
            WoanAPI::destroy_robot(robot_handle_);
        }
        
        // 双臂
        if(dof_ == 14){
            if (robot_handle_left_) {
                WoanAPI::disable_motors(robot_handle_left_);
                WoanAPI::destroy_robot(robot_handle_left_);
            }
            if (robot_handle_right_) {
                WoanAPI::disable_motors(robot_handle_right_);
                WoanAPI::destroy_robot(robot_handle_right_);
            }
        }
    }
    
private:
    woan_api::woan_robot_handle* robot_handle_;      // 单臂句柄
    woan_api::woan_robot_handle* robot_handle_left_;   // 双臂：左臂句柄
    woan_api::woan_robot_handle* robot_handle_right_;  // 双臂：右臂句柄
    rclcpp::Subscription<woan_interfaces::msg::MoveJ>::SharedPtr movej_sub_;
    rclcpp::Subscription<woan_interfaces::msg::MoveL>::SharedPtr movel_sub_;
    rclcpp::Subscription<woan_interfaces::msg::MoveP>::SharedPtr movep_sub_;
    
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
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr movep_result_pub_;
    rclcpp::Publisher<woan_interfaces::msg::ArmState>::SharedPtr arm_state_pub_; // 单臂状态
    rclcpp::Publisher<woan_interfaces::msg::ArmState>::SharedPtr left_arm_state_pub_;  // 双臂
    rclcpp::Publisher<woan_interfaces::msg::ArmState>::SharedPtr right_arm_state_pub_; // 双臂
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_; 
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_buffer_srv_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    
    std::string device_; // 单臂
    int arm_status_; // 单臂

    std::string robot_model_;
    std::string symbol_left_;
    std::string device_left_;
    std::string symbol_right_;
    std::string device_right_;
    int dof_;
    int left_arm_status_; // 双臂
    int right_arm_status_; // 双臂
    std::string planning_arms_status_;  // 双臂： "left", "right", "dual", 或 "none"
    
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
                woan_api::woan_config_t config;
                // 使用 memset 初始化整个结构体为 0，确保所有字符串都以 null 结尾
                memset(&config, 0, sizeof(config));
                
                // 从参数读取串口设备名称
                strncpy(config.device, device_.c_str(), sizeof(config.device) - 1);
                config.device[sizeof(config.device) - 1] = '\0';  // 确保 null 终止
                
                // 传递robot_model参数（从ROS参数获取）
                strncpy(config.robot_model, robot_model_.c_str(), sizeof(config.robot_model) - 1);
                config.robot_model[sizeof(config.robot_model) - 1] = '\0';  // 确保 null 终止
                
                config.dof = dof_;
                config.baud_rate = 921600;
                config.urdf_path[0] = '\0';  // 使用默认路径
                
                // 从配置文件读取MIT模式PD参数
                std::vector<double> mit_kp_values = this->declare_parameter<std::vector<double>>(
                    "mit_mode.default_kp_values", std::vector<double>(7, 100.0));
                std::vector<double> mit_kd_values = this->declare_parameter<std::vector<double>>(
                    "mit_mode.default_kd_values", std::vector<double>(7, 0.5));
                
                for (int i = 0; i < dof_; i++) {
                    config.mit_kp[i] = mit_kp_values[i];
                }
                for (int i = 0; i < dof_; i++) {
                    config.mit_kd[i] = mit_kd_values[i];
                }
                config.interrupt_check = &WoanDriverNode::interruptRequested;
                config.interrupt_ctx = this;
                
                robot_handle_ = WoanAPI::create_robot(config);
                if (!robot_handle_) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create robot handle");
                    return false;
                }
                
                int result = WoanAPI::enable_motors(robot_handle_);
                if (result != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to enable motors (code: %d)", result);
                    WoanAPI::destroy_robot(robot_handle_);
                    robot_handle_ = nullptr;
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
                woan_api::woan_config_t config_left;
                memset(&config_left, 0, sizeof(config_left));
                
                strncpy(config_left.device, device_left_.c_str(), sizeof(config_left.device) - 1);
                config_left.device[sizeof(config_left.device) - 1] = '\0';
                
                strncpy(config_left.robot_model, symbol_left_.c_str(), sizeof(config_left.robot_model) - 1);
                config_left.robot_model[sizeof(config_left.robot_model) - 1] = '\0';
                
                config_left.dof = dof_ / 2;  // 单臂7自由度
                config_left.baud_rate = 921600;
                config_left.urdf_path[0] = '\0';
                
                // 读取左臂MIT模式PD参数
                std::vector<double> mit_kp_left = this->declare_parameter<std::vector<double>>(
                    "mit_mode.left_kp_values", std::vector<double>(dof_ / 2, 100.0));
                std::vector<double> mit_kd_left = this->declare_parameter<std::vector<double>>(
                    "mit_mode.left_kd_values", std::vector<double>(dof_ / 2, 0.5));
                
                for (int i = 0; i < dof_ / 2; i++) {
                    config_left.mit_kp[i] = mit_kp_left[i];
                    config_left.mit_kd[i] = mit_kd_left[i];
                }
                config_left.interrupt_check = &WoanDriverNode::interruptRequested;
                config_left.interrupt_ctx = this;
                
                robot_handle_left_ = WoanAPI::create_robot(config_left);
                if (!robot_handle_left_) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create left arm robot handle");
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "Left arm handle created successfully");
                
                int result_left = WoanAPI::enable_motors(robot_handle_left_);
                if (result_left != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to enable left arm motors (code: %d)", result_left);
                    WoanAPI::destroy_robot(robot_handle_left_);
                    robot_handle_left_ = nullptr;
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "Left arm enabled: %s on %s", 
                            symbol_left_.c_str(), device_left_.c_str());
                
                // ========== 初始化右臂 ==========
                RCLCPP_INFO(this->get_logger(), "Starting right arm initialization...");
                RCLCPP_INFO(this->get_logger(), "Right arm device: %s", device_right_.c_str());
                
                woan_api::woan_config_t config_right;
                memset(&config_right, 0, sizeof(config_right));
                
                strncpy(config_right.device, device_right_.c_str(), sizeof(config_right.device) - 1);
                config_right.device[sizeof(config_right.device) - 1] = '\0';
                
                strncpy(config_right.robot_model, symbol_right_.c_str(), sizeof(config_right.robot_model) - 1);
                config_right.robot_model[sizeof(config_right.robot_model) - 1] = '\0';
                
                config_right.dof = dof_ / 2;  // 单臂7自由度
                config_right.baud_rate = 921600;
                config_right.urdf_path[0] = '\0';
                
                // 读取右臂MIT模式PD参数
                std::vector<double> mit_kp_right = this->declare_parameter<std::vector<double>>(
                    "mit_mode.right_kp_values", std::vector<double>(dof_ / 2, 100.0));
                std::vector<double> mit_kd_right = this->declare_parameter<std::vector<double>>(
                    "mit_mode.right_kd_values", std::vector<double>(dof_ / 2, 0.5));
                
                for (int i = 0; i < dof_ / 2; i++) {
                    config_right.mit_kp[i] = mit_kp_right[i];
                    config_right.mit_kd[i] = mit_kd_right[i];
                }
                config_right.interrupt_check = &WoanDriverNode::interruptRequested;
                config_right.interrupt_ctx = this;
                
                robot_handle_right_ = WoanAPI::create_robot(config_right);
                if (!robot_handle_right_) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create right arm robot handle (device: %s)", 
                                device_right_.c_str());
                    // 清理左臂
                    WoanAPI::disable_motors(robot_handle_left_);
                    WoanAPI::destroy_robot(robot_handle_left_);
                    robot_handle_left_ = nullptr;
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "Right arm handle created successfully");
                
                int result_right = WoanAPI::enable_motors(robot_handle_right_);
                if (result_right != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to enable right arm motors (code: %d, device: %s)", 
                                result_right, device_right_.c_str());
                    WoanAPI::destroy_robot(robot_handle_right_);
                    robot_handle_right_ = nullptr;
                    // 清理左臂
                    WoanAPI::disable_motors(robot_handle_left_);
                    WoanAPI::destroy_robot(robot_handle_left_);
                    robot_handle_left_ = nullptr;
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
    
    void movejCallback(const woan_interfaces::msg::MoveJ::SharedPtr msg) {
        if(dof_ == 7){
            if (!robot_handle_) {
                std_msgs::msg::Bool result;
                result.data = false;
                movej_result_pub_->publish(result);
                return;
            }
        
            try {
                // Convert float32[] to double vector
                woan_api::JointArray joint_array(msg->joint.begin(), msg->joint.end());
                
                int ret = WoanAPI::movej(robot_handle_, joint_array, msg->speed_scale, 
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
            if (!robot_handle_left_ || !robot_handle_right_) {
                std_msgs::msg::Bool result;
                result.data = false;
                movej_result_pub_->publish(result);
                return;
            }
        
            try {
                // Convert float32[] to double vector
                woan_api::JointArray joint_array(msg->joint.begin(), msg->joint.end());
                
                // 双臂模式：收到了14个关节角度，假设前7个是左臂，后7个是右臂
                if (joint_array.size() == static_cast<size_t>(dof_)) {
                    woan_api::JointArray left_joints(joint_array.begin(), joint_array.begin() + dof_ / 2);
                    woan_api::JointArray right_joints(joint_array.begin() + dof_ / 2, joint_array.end());
                    
                    // 使用多线程同时执行左右臂运动
                    std::atomic<int> ret_left{-1};
                    std::atomic<int> ret_right{-1};
                    
                    std::thread left_thread([this, &ret_left, left_joints, msg]() {
                        ret_left = WoanAPI::movej(robot_handle_left_, left_joints, msg->speed_scale, 
                                                msg->trajectory_connect);
                    });
                    
                    std::thread right_thread([this, &ret_right, right_joints, msg]() {
                        ret_right = WoanAPI::movej(robot_handle_right_, right_joints, msg->speed_scale, 
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
    
    void movelCallback(const woan_interfaces::msg::MoveL::SharedPtr msg) {
        if(dof_ == 7){
            if (!robot_handle_) {
                std_msgs::msg::Bool result;
                result.data = false;
                movel_result_pub_->publish(result);
                return;
            }
            
            try {
                woan_api::Pose pose;
                pose.x = msg->pose.position.x;
                pose.y = msg->pose.position.y;
                pose.z = msg->pose.position.z;
                pose.qw = msg->pose.orientation.w;
                pose.qx = msg->pose.orientation.x;
                pose.qy = msg->pose.orientation.y;
                pose.qz = msg->pose.orientation.z;
                
                int ret = WoanAPI::movel(robot_handle_, pose, msg->speed_scale,
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
            if (!robot_handle_left_ || !robot_handle_right_) {
                std_msgs::msg::Bool result;
                result.data = false;
                movel_result_pub_->publish(result);
                return;
            }
        
            try {
                woan_api::Pose pose;
                pose.x = msg->pose.position.x;
                pose.y = msg->pose.position.y;
                pose.z = msg->pose.position.z;
                pose.qw = msg->pose.orientation.w;
                pose.qx = msg->pose.orientation.x;
                pose.qy = msg->pose.orientation.y;
                pose.qz = msg->pose.orientation.z;
                
                // 双臂模式：这里只控制左臂，根据需求可以改为右臂或两个都控制
                int ret = WoanAPI::movel(robot_handle_left_, pose, msg->speed_scale,
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
    
    void movepCallback(const woan_interfaces::msg::MoveP::SharedPtr msg) {
        if(dof_ == 7){
            if (!robot_handle_) {
                std_msgs::msg::Bool result;
                result.data = false;
                movep_result_pub_->publish(result);
                return;
            }
        
            try {
                woan_api::Pose pose;
                pose.x = msg->pose.position.x;
                pose.y = msg->pose.position.y;
                pose.z = msg->pose.position.z;
                pose.qw = msg->pose.orientation.w;
                pose.qx = msg->pose.orientation.x;
                pose.qy = msg->pose.orientation.y;
                pose.qz = msg->pose.orientation.z;
                
                int ret = WoanAPI::movep(robot_handle_, pose, msg->speed_scale, 
                                        msg->trajectory_connect);
                
                // 打印目标位姿和当前位姿
                woan_api::Pose current_pose = WoanAPI::get_end_effector_pose(robot_handle_);
                RCLCPP_INFO(this->get_logger(), 
                    "MoveP finished. Target Pose: [x: %.4f, y: %.4f, z: %.4f, w: %.4f, x: %.4f, y: %.4f, z: %.4f]",
                    pose.x, pose.y, pose.z, pose.qw, pose.qx, pose.qy, pose.qz);
                RCLCPP_INFO(this->get_logger(), 
                    "Current Pose:  [x: %.4f, y: %.4f, z: %.4f, w: %.4f, x: %.4f, y: %.4f, z: %.4f]",
                    current_pose.x, current_pose.y, current_pose.z, current_pose.qw, current_pose.qx, current_pose.qy, current_pose.qz);

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
            if (!robot_handle_left_ || !robot_handle_right_) {
                std_msgs::msg::Bool result;
                result.data = false;
                movep_result_pub_->publish(result);
                return;
            }
            
            try {
                woan_api::Pose pose;
                pose.x = msg->pose.position.x;
                pose.y = msg->pose.position.y;
                pose.z = msg->pose.position.z;
                pose.qw = msg->pose.orientation.w;
                pose.qx = msg->pose.orientation.x;
                pose.qy = msg->pose.orientation.y;
                pose.qz = msg->pose.orientation.z;
                
                // 双臂模式：这里只控制左臂，根据需求可以改为右臂或两个都控制
                int ret = WoanAPI::movep(robot_handle_left_, pose, msg->speed_scale, 
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
            if (!robot_handle_) {
                response->success = false;
                response->message = "Robot not initialized";
                return;
            }
            
            try {
                int ret = WoanAPI::clear_trajectory_buffer(robot_handle_);
                response->success = (ret == 0);
                response->message = response->success ? "Buffer cleared" : "Failed to clear buffer";
            } catch (const std::exception& e) {
                response->success = false;
                response->message = std::string("Exception: ") + e.what();
            }
        }
        if(dof_ == 14){
            if (!robot_handle_left_ || !robot_handle_right_) {
                response->success = false;
                response->message = "Robot not initialized";
                return;
            }
            
            try {
                int ret_left = WoanAPI::clear_trajectory_buffer(robot_handle_left_);
                int ret_right = WoanAPI::clear_trajectory_buffer(robot_handle_right_);
                
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
            if (!robot_handle_) return;
        
            try {
                // 根据硬件连接状态选择读取方式
                bool hardware_connected = WoanAPI::is_hardware_connected(robot_handle_);
                woan_api::JointArray positions;
                
                if (hardware_connected) {
                    // 连接真实硬件：从电机读取实际位置、速度、力矩
                    woan_api::ArmStateFromMotor motor_state = WoanAPI::get_arm_state_from_motor(robot_handle_);
                    positions = motor_state.positions;
                    // RCLCPP_INFO(
                    //     this->get_logger(),
                    //     "\033[32mPublishing state from hardware\033[0m"
                    // );
                    
                    // 发布 ArmState
                    auto state = woan_interfaces::msg::ArmState();
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
                    
                    woan_api::Pose ee_pose = WoanAPI::get_end_effector_pose(robot_handle_);
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
                    positions = WoanAPI::get_joint_positions(robot_handle_);
                    
                    // 发布 ArmState
                    auto state = woan_interfaces::msg::ArmState();
                    state.robot_model = robot_model_;
                    state.status = arm_status_;
                    
                    if (positions.size() == static_cast<size_t>(dof_)) {
                        state.joint_positions.assign(positions.begin(), positions.end());
                    }
                    
                    woan_api::Pose ee_pose = WoanAPI::get_end_effector_pose(robot_handle_);
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
            if (!robot_handle_left_ || !robot_handle_right_) return;
        
            try {
                // 根据硬件连接状态选择读取方式
                bool hardware_connected_left = WoanAPI::is_hardware_connected(robot_handle_left_);
                bool hardware_connected_right = WoanAPI::is_hardware_connected(robot_handle_right_);

                if (hardware_connected_left && hardware_connected_right) {
                    /*  左臂和右臂都有连接真实硬件  */
                    //从仅有单臂连接切换到双臂连接真实硬件，取消警告
                    warn_signal_ = false;
                    if(last_warn_signal_ == true && warn_signal_ == false){
                        RCLCPP_INFO(this->get_logger(), "双臂均连接真实硬件。");
                        last_warn_signal_ = false;
                    }

                    woan_api::JointArray positions_dual;
                    woan_api::JointArray velocities_dual;
                    woan_api::JointArray torques_dual;

                    // 连接真实硬件（左臂）：从电机读取实际位置、速度、力矩
                    woan_api::ArmStateFromMotor motor_state_left = WoanAPI::get_arm_state_from_motor(robot_handle_left_);

                    //发布 ArmState（左臂）
                    auto left_state = woan_interfaces::msg::ArmState();
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
                    woan_api::Pose left_ee_pose = WoanAPI::get_end_effector_pose(robot_handle_left_);
                    left_state.end_effector_pose.position.x = left_ee_pose.x;
                    left_state.end_effector_pose.position.y = left_ee_pose.y;
                    left_state.end_effector_pose.position.z = left_ee_pose.z;
                    left_state.end_effector_pose.orientation.w = left_ee_pose.qw;
                    left_state.end_effector_pose.orientation.x = left_ee_pose.qx;
                    left_state.end_effector_pose.orientation.y = left_ee_pose.qy;
                    left_state.end_effector_pose.orientation.z = left_ee_pose.qz;
                    
                    left_arm_state_pub_->publish(left_state);

                    // 连接真实硬件（右臂）：从电机读取实际位置、速度、力矩
                    woan_api::ArmStateFromMotor motor_state_right = WoanAPI::get_arm_state_from_motor(robot_handle_right_);

                    //发布 ArmState（右臂）
                    auto right_state = woan_interfaces::msg::ArmState();
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
                    woan_api::Pose right_ee_pose = WoanAPI::get_end_effector_pose(robot_handle_right_);
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

                    woan_api::JointArray positions_left;
                    woan_api::JointArray positions_right;
                    woan_api::JointArray positions_dual;

                    // 模拟环境（左臂未连接真实硬件）：使用规划位置（缓存位置），速度和力矩为空
                    positions_left = WoanAPI::get_joint_positions(robot_handle_left_);
                    
                    //发布 ArmState（左臂）
                    auto left_state = woan_interfaces::msg::ArmState();
                    left_state.robot_model = symbol_left_;
                    left_state.status = left_arm_status_;

                    if (positions_left.size() == static_cast<size_t>(dof_/2)) {
                        left_state.joint_positions.assign(positions_left.begin(), positions_left.end());
                    }

                    // 末端位姿（左臂）
                    woan_api::Pose left_ee_pose = WoanAPI::get_end_effector_pose(robot_handle_left_);
                    left_state.end_effector_pose.position.x = left_ee_pose.x;
                    left_state.end_effector_pose.position.y = left_ee_pose.y;
                    left_state.end_effector_pose.position.z = left_ee_pose.z;
                    left_state.end_effector_pose.orientation.w = left_ee_pose.qw;
                    left_state.end_effector_pose.orientation.x = left_ee_pose.qx;
                    left_state.end_effector_pose.orientation.y = left_ee_pose.qy;
                    left_state.end_effector_pose.orientation.z = left_ee_pose.qz;
                    
                    left_arm_state_pub_->publish(left_state);

                    // 模拟环境（右臂未连接真实硬件）：使用规划位置（缓存位置），速度和力矩为空
                    positions_right = WoanAPI::get_joint_positions(robot_handle_right_);

                    // 发布 ArmState（右臂）
                    auto right_state = woan_interfaces::msg::ArmState();
                    right_state.robot_model = symbol_right_;
                    right_state.status = right_arm_status_;
                    
                    if (positions_right.size() == static_cast<size_t>(dof_/2)) {
                        right_state.joint_positions.assign(positions_right.begin(), positions_right.end());
                    }
                    
                    // 末端位姿（右臂）
                    woan_api::Pose right_ee_pose = WoanAPI::get_end_effector_pose(robot_handle_right_);
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
    
    // 双臂：规划手臂状态回调（接收woan_control_dual发布的规划信息）
    void planningArmsCallback(const std_msgs::msg::String::SharedPtr msg) {
        planning_arms_status_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "收到规划手臂状态: %s", planning_arms_status_.c_str());
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
    
    // /*send_trajectory_point的方式，开始执行轨迹*/
    // void executeCallback(const std_msgs::msg::Empty::SharedPtr msg) {
    //     (void)msg;  // 未使用的参数
        
    //     std::lock_guard<std::mutex> lock(trajectory_mutex_);
        
    //     if (trajectory_buffer_.empty()) {
    //         RCLCPP_ERROR(this->get_logger(), "轨迹缓冲为空，无法执行！");
    //         return;
    //     }
        
    //     if (is_executing_) {
    //         RCLCPP_WARN(this->get_logger(), "轨迹正在执行中，忽略重复执行命令");
    //         return;
    //     }
        
    //     // 等待之前的线程结束
    //     if (execution_thread_.joinable()) {
    //         execution_thread_.join();
    //     }
        
    //     // 重置执行状态
    //     should_cancel_ = false;
    //     is_executing_ = true;
        
    //     RCLCPP_INFO(this->get_logger(), "开始执行轨迹（独立线程 + 精确10ms循环）");
        
    //     // 启动执行线程
    //     execution_thread_ = std::thread(&WoanDriverNode::executeTrajectoryLoop, this);
    // }

    // /* send_trajectory_point的方式，executeTrajectoryLoop：轨迹执行循环（独立线程，精确10ms间隔）*/
    // void executeTrajectoryLoop() {
    //     if (!robot_handle_left_ || !robot_handle_right_) {
    //         RCLCPP_ERROR(this->get_logger(), "机器人未初始化");
    //         is_executing_ = false;
    //         return;
    //     }
        
    //     // 获取轨迹副本（避免长时间持有锁）
    //     std::vector<trajectory_msgs::msg::JointTrajectoryPoint> local_trajectory;
    //     {
    //         std::lock_guard<std::mutex> lock(trajectory_mutex_);
    //         local_trajectory = trajectory_buffer_;
    //     }
        
    //     if (local_trajectory.empty()) {
    //         RCLCPP_ERROR(this->get_logger(), "轨迹为空");
    //         is_executing_ = false;
    //         return;
    //     }
        
    //     RCLCPP_INFO(this->get_logger(), "轨迹执行线程启动，共 %zu 个点，控制周期: %.3f ms", 
    //                 local_trajectory.size(), time_step_ * 1000.0);
        
    //     size_t executed_points = 0;
        
    //     // 在第一个点发送前立即记录开始时间（减少线程启动延迟影响）
    //     auto start_time = std::chrono::high_resolution_clock::now();
        
    //     // for循环 + usleep精确控制时间，每个轨迹点都执行
    //     for (size_t i = 0; i < local_trajectory.size(); ++i) {
    //         // 检查取消信号
    //         if (should_cancel_) {
    //             RCLCPP_WARN(this->get_logger(), "轨迹执行已取消（已执行 %zu/%zu 个点）", 
    //                         executed_points, local_trajectory.size());
    //             break;
    //         }
            
    //         const auto& point = local_trajectory[i];
            
    //         // 检查数据有效性
    //         if (point.positions.size() != static_cast<size_t>(dof_) ||
    //             point.velocities.size() != static_cast<size_t>(dof_)||
    //             point.accelerations.size() != static_cast<size_t>(dof_)) {
    //             RCLCPP_ERROR(this->get_logger(), 
    //                 "轨迹点 %zu 数据无效: pos=%zu, vel=%zu acc=%zu (期望 %d)", 
    //                 i, point.positions.size(), point.velocities.size(), point.accelerations.size(), dof_);
    //             break;
    //         }
            
    //         try {
    //             // 提取左臂数据（前7个关节）
    //             woan_api::JointArray positions_left(dof_ / 2);
    //             woan_api::JointArray velocities_left(dof_ / 2);
    //             woan_api::JointArray accelerations_left(dof_ / 2);
                
    //             for (int j = 0; j < dof_ / 2; ++j) {
    //                 positions_left[j] = point.positions[j];
    //                 velocities_left[j] = point.velocities[j];
    //                 accelerations_left[j] = point.accelerations[j];
    //             }
                
    //             // 提取右臂数据（后7个关节）
    //             woan_api::JointArray positions_right(dof_ / 2);
    //             woan_api::JointArray velocities_right(dof_ / 2);
    //             woan_api::JointArray accelerations_right(dof_ / 2);
                
    //             for (int j = 0; j < dof_ / 2; ++j) {
    //                 positions_right[j] = point.positions[dof_ / 2 + j];
    //                 velocities_right[j] = point.velocities[dof_ / 2 + j];
    //                 accelerations_right[j] = point.accelerations[dof_ / 2 + j];

    //             }
                
    //             // 根据规划手臂状态发送轨迹点
    //             if (planning_arms_status_ == "left" || planning_arms_status_ == "dual") {
    //                 int result = WoanAPI::send_trajectory_point(robot_handle_left_, 
    //                                                             positions_left, 
    //                                                             velocities_left,
    //                                                             accelerations_left);
    //                 if (result != 0) {
    //                     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //                         "左臂轨迹点发送失败 (code: %d)", result);
    //                 }
    //             }
                
    //             if (planning_arms_status_ == "right" || planning_arms_status_ == "dual") {
    //                 int result = WoanAPI::send_trajectory_point(robot_handle_right_, 
    //                                                             positions_right, 
    //                                                             velocities_right,
    //                                                             accelerations_right);
    //                 if (result != 0) {
    //                     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //                         "右臂轨迹点发送失败 (code: %d)", result);
    //                 }
    //             }
                
    //             executed_points++;
                
    //             // 进度输出（每100个点）
    //             if (executed_points % 100 == 0 || (i + 1) >= local_trajectory.size()) {
    //                 double progress = (double)(i + 1) / local_trajectory.size() * 100.0;
    //                 auto now = std::chrono::high_resolution_clock::now();
    //                 double elapsed = std::chrono::duration<double>(now - start_time).count();
    //                 RCLCPP_INFO(this->get_logger(), 
    //                     "执行进度: %.1f%% (%zu/%zu), 已用时: %.2f秒", 
    //                     progress, i + 1, local_trajectory.size(), elapsed);
    //             }
                
    //         } catch (const std::exception& e) {
    //             RCLCPP_ERROR(this->get_logger(), "发送轨迹点 %zu 异常: %s", i, e.what());
    //             break;
    //         }
            
    //         // ========== 精确sleep控制周期（补偿发送耗时）==========
    //         // 最后一个点不需要sleep（已经完成任务）
    //         if (i < local_trajectory.size() - 1) {
    //             // 计算下一个点的目标时间
    //             auto target_time = start_time + std::chrono::microseconds(static_cast<long long>((i + 1) * time_step_ * 1000000));
    //             auto now = std::chrono::high_resolution_clock::now();
                
    //             // 计算还需要sleep的时间
    //             auto remaining_time = std::chrono::duration_cast<std::chrono::microseconds>(target_time - now);
                
    //             if (remaining_time.count() > 0) {
    //                 // 还没到目标时间，sleep剩余时间
    //                 usleep(static_cast<unsigned int>(remaining_time.count()));
    //             } else if (remaining_time.count() < -10000) {
    //                 // 如果延迟超过10ms，输出警告（每100个点一次）
    //                 if (i % 100 == 0) {
    //                     RCLCPP_WARN(this->get_logger(), 
    //                         "轨迹点 %zu 执行延迟 %.2f ms", 
    //                         i, -remaining_time.count() / 1000.0);
    //                 }
    //             }
    //         }
    //     }
        
    //     // 执行完成
    //     auto end_time = std::chrono::high_resolution_clock::now();
    //     double total_time = std::chrono::duration<double>(end_time - start_time).count();
        
    //     if (should_cancel_) {
    //         RCLCPP_WARN(this->get_logger(), 
    //             "轨迹执行已取消，共执行 %zu/%zu 个点，耗时 %.2f 秒", 
    //             executed_points, local_trajectory.size(), total_time);
    //     } else {
    //         RCLCPP_INFO(this->get_logger(), 
    //             "轨迹执行完成！共执行 %zu 个点，耗时 %.2f 秒", 
    //             executed_points, total_time);
    //     }
        
    //     is_executing_ = false;
    // }

    // /*send_trajectory_point的方式，取消执行*/
    // void cancelCallback(const std_msgs::msg::Empty::SharedPtr msg) {
    //     (void)msg;  // 未使用的参数
        
    //     if (!is_executing_) {
    //         RCLCPP_INFO(this->get_logger(), "当前没有执行中的轨迹");
    //         return;
    //     }
        
    //     should_cancel_ = true;
    //     RCLCPP_WARN(this->get_logger(), "轨迹执行取消信号已发送");
    // }

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
        execution_thread_ = std::thread(&WoanDriverNode::executeTrajectoryFullLoop, this);
    }

    /*send_trajectory方式，executeTrajectoryFullLoop：使用完整轨迹一次性下发*/
    void executeTrajectoryFullLoop() {
        if(dof_ == 7){
            if (!robot_handle_) {
                RCLCPP_ERROR(this->get_logger(), "机器人未初始化");
                is_executing_ = false;
                auto msg = std_msgs::msg::Bool();
                msg.data = false;
                execution_result_pub_->publish(msg);
                return;
            }
        }
        if(dof_ == 14){
            if (!robot_handle_left_ || !robot_handle_right_) {
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
                std::vector<woan_api::TrajectoryPoint> trajectory;
                trajectory.reserve(local_trajectory.size());
                
                for (const auto& point : local_trajectory) {
                    woan_api::TrajectoryPoint traj_point;
                    traj_point.position = point.positions;
                    traj_point.velocity = point.velocities;
                    traj_point.acceleration = point.accelerations;
                    trajectory.push_back(traj_point);
                }

                RCLCPP_INFO(this->get_logger(), "轨迹构建完成: %zu 个点", trajectory.size());

                // ========== 执行轨迹 ==========
                int result = WoanAPI::send_trajectory(robot_handle_, trajectory);
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
            try {
                // ========== 构建左臂和右臂的完整轨迹 ==========
                std::vector<woan_api::TrajectoryPoint> trajectory_left;
                std::vector<woan_api::TrajectoryPoint> trajectory_right;
                trajectory_left.reserve(local_trajectory.size());
                trajectory_right.reserve(local_trajectory.size());
                
                for (const auto& point : local_trajectory) {
                    // 左臂轨迹点（前7个关节）
                    woan_api::TrajectoryPoint traj_point_left;
                    traj_point_left.position.assign(point.positions.begin(), point.positions.begin() + dof_ / 2);
                    traj_point_left.velocity.assign(point.velocities.begin(), point.velocities.begin() + dof_ / 2);
                    traj_point_left.acceleration.assign(point.accelerations.begin(), point.accelerations.begin() + dof_ / 2);
                    trajectory_left.push_back(traj_point_left);
                    
                    // 右臂轨迹点（后7个关节）
                    woan_api::TrajectoryPoint traj_point_right;
                    traj_point_right.position.assign(point.positions.begin() + dof_ / 2, point.positions.end());
                    traj_point_right.velocity.assign(point.velocities.begin() + dof_ / 2, point.velocities.end());
                    traj_point_right.acceleration.assign(point.accelerations.begin() + dof_ / 2, point.accelerations.end());
                    trajectory_right.push_back(traj_point_right);
                }
                
                RCLCPP_INFO(this->get_logger(), "轨迹构建完成: 左臂 %zu 个点, 右臂 %zu 个点", 
                            trajectory_left.size(), trajectory_right.size());
                
                // ========== 根据规划手臂状态执行轨迹 ==========
                if (planning_arms_status_ == "dual") {
                    // 双臂同时执行：启动两个线程
                    RCLCPP_INFO(this->get_logger(), "双臂模式：同时执行左右臂轨迹");
                    
                    std::thread left_thread([this, trajectory_left]() {
                        int result = WoanAPI::send_trajectory(robot_handle_left_, trajectory_left);
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
                        int result = WoanAPI::send_trajectory(robot_handle_right_, trajectory_right);
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
                    
                } else if (planning_arms_status_ == "left") {
                    // 仅左臂执行
                    RCLCPP_INFO(this->get_logger(), "左臂模式：执行左臂轨迹");
                    int result = WoanAPI::send_trajectory(robot_handle_left_, trajectory_left);
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
                    
                } else if (planning_arms_status_ == "right") {
                    // 仅右臂执行
                    RCLCPP_INFO(this->get_logger(), "右臂模式：执行右臂轨迹");
                    int result = WoanAPI::send_trajectory(robot_handle_right_, trajectory_right);
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
                    RCLCPP_WARN(this->get_logger(), "未知的规划手臂状态（双臂的起始和目标一致）: %s", planning_arms_status_.c_str());
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
            int result = WoanAPI::cancel_trajectory(robot_handle_);
            if (result == 0) {
                RCLCPP_WARN(this->get_logger(), "轨迹执行取消信号已发送");
            } else {
                RCLCPP_ERROR(this->get_logger(), "轨迹取消失败 (code: %d)", result);
            }
        }

        if(dof_ == 14){
            //根据规划的手臂来取消对应的轨迹
            if (planning_arms_status_ == "left") {
                int result = WoanAPI::cancel_trajectory(robot_handle_left_);
                if (result == 0) {
                    RCLCPP_WARN(this->get_logger(), "左臂轨迹执行取消信号已发送");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "左臂轨迹取消失败 (code: %d)", result);
                }
            } else if (planning_arms_status_ == "right") {
                 int result = WoanAPI::cancel_trajectory(robot_handle_right_);
                if (result == 0) {
                    RCLCPP_WARN(this->get_logger(), "右臂轨迹执行取消信号已发送");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "右臂轨迹取消失败 (code: %d)", result);
                }
            } else if (planning_arms_status_ == "dual") {
                int result_left = WoanAPI::cancel_trajectory(robot_handle_left_);
                int result_right = WoanAPI::cancel_trajectory(robot_handle_right_);
                if (result_left == 0 && result_right == 0) {
                    RCLCPP_WARN(this->get_logger(), "双臂轨迹执行取消信号已发送");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "双臂轨迹取消失败 (left code: %d, right code: %d)", result_left, result_right);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "未知的规划手臂状态（双臂的起始和目标一致）: %s", planning_arms_status_.c_str());
            }

        }
    }
    
    // ========== 新增DEBUG：测试轨迹回调（使用send_trajectory_point API）==========
    void testTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        if (!robot_handle_left_ || !robot_handle_right_) {
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
                        woan_api::JointArray positions(point.positions.begin(), point.positions.end());
                        woan_api::JointArray velocities(point.velocities.begin(), point.velocities.end());
                        woan_api::JointArray accelerations(point.accelerations.begin(), point.accelerations.end());
                        
                        int result = WoanAPI::send_trajectory_point(robot_handle_left_, 
                                                                    positions, 
                                                                    velocities,
                                                                    accelerations);
                        // int result = WoanAPI::send_trajectory_point(robot_handle_right_, 
                        //                                             positions, 
                        //                                             velocities,
                        //                                             accelerations);//jingyi
                        if (result != 0) {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "左臂轨迹点发送失败 (code: %d)", result);
                        }
                    } else {
                        // 14自由度模式：控制双臂
                        woan_api::JointArray positions_left(point.positions.begin(), 
                                                            point.positions.begin() + 7);
                        woan_api::JointArray velocities_left(point.velocities.begin(), 
                                                             point.velocities.begin() + 7);
                        woan_api::JointArray accelerations_left(point.accelerations.begin(), 
                                                                point.accelerations.begin() + 7);
                        
                        woan_api::JointArray positions_right(point.positions.begin() + 7, 
                                                             point.positions.end());
                        woan_api::JointArray velocities_right(point.velocities.begin() + 7, 
                                                              point.velocities.end());
                        woan_api::JointArray accelerations_right(point.accelerations.begin() + 7, 
                                                                 point.accelerations.end());
                        
                        int result_left = WoanAPI::send_trajectory_point(robot_handle_left_, 
                                                                         positions_left, 
                                                                         velocities_left,
                                                                         accelerations_left);
                        int result_right = WoanAPI::send_trajectory_point(robot_handle_right_, 
                                                                          positions_right, 
                                                                          velocities_right,
                                                                          accelerations_right);
                        
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
        if (!robot_handle_left_ || !robot_handle_right_) {
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
                    std::vector<woan_api::TrajectoryPoint> trajectory;
                    trajectory.reserve(msg->points.size());
                    
                    for (const auto& point : msg->points) {
                        woan_api::TrajectoryPoint traj_point;
                        traj_point.position.assign(point.positions.begin(), point.positions.end());
                        traj_point.velocity.assign(point.velocities.begin(), point.velocities.end());
                        traj_point.acceleration.assign(point.accelerations.begin(), point.accelerations.end());
                        trajectory.push_back(traj_point);
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "准备发送左臂轨迹: %zu 个点", trajectory.size());
                    
                    int result = WoanAPI::send_trajectory(robot_handle_left_, trajectory);
                    // int result = WoanAPI::send_trajectory(robot_handle_right_, trajectory);//jingyi

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
                    std::vector<woan_api::TrajectoryPoint> trajectory_left;
                    std::vector<woan_api::TrajectoryPoint> trajectory_right;
                    trajectory_left.reserve(msg->points.size());
                    trajectory_right.reserve(msg->points.size());
                    
                    for (const auto& point : msg->points) {
                        // 左臂轨迹点
                        woan_api::TrajectoryPoint traj_point_left;
                        traj_point_left.position.assign(point.positions.begin(), point.positions.begin() + 7);
                        traj_point_left.velocity.assign(point.velocities.begin(), point.velocities.begin() + 7);
                        traj_point_left.acceleration.assign(point.accelerations.begin(), point.accelerations.begin() + 7);
                        trajectory_left.push_back(traj_point_left);
                        
                        // 右臂轨迹点
                        woan_api::TrajectoryPoint traj_point_right;
                        traj_point_right.position.assign(point.positions.begin() + 7, point.positions.end());
                        traj_point_right.velocity.assign(point.velocities.begin() + 7, point.velocities.end());
                        traj_point_right.acceleration.assign(point.accelerations.begin() + 7, point.accelerations.end());
                        trajectory_right.push_back(traj_point_right);
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "准备发送双臂轨迹: 左臂 %zu 个点, 右臂 %zu 个点", 
                                trajectory_left.size(), trajectory_right.size());
                    
                    // 启动两个线程同时执行左右臂轨迹
                    std::thread left_thread([this, trajectory_left]() {
                        int result = WoanAPI::send_trajectory(robot_handle_left_, trajectory_left);
                        if (result == 0) {
                            RCLCPP_INFO(this->get_logger(), "✅ 左臂轨迹执行完成");
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "❌ 左臂轨迹执行失败 (code: %d)", result);
                        }
                    });
                    
                    std::thread right_thread([this, trajectory_right]() {
                        int result = WoanAPI::send_trajectory(robot_handle_right_, trajectory_right);
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
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WoanDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
