#pragma once

#include "rclcpp/rclcpp.hpp"
#include "woan_interfaces/msg/move_j.hpp"
#include "woan_interfaces/msg/move_l.hpp"
#include "woan_interfaces/msg/move_p.hpp"
#include "woan_interfaces/msg/arm_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace woan {

/**
 * @brief WoanArm机器人驱动节点
 * 
 * 提供ROS2话题接口，接收Move API命令并转发给arm_control执行。
 * 发布机械臂状态信息。
 */
class WoanDriver : public rclcpp::Node {
public:
    WoanDriver();
    ~WoanDriver();
    
private:
    // ========== 回调函数 ==========
    
    /**
     * @brief MoveJ命令回调
     */
    void movejCallback(const woan_interfaces::msg::MoveJ::SharedPtr msg);
    
    /**
     * @brief MoveL命令回调
     */
    void movelCallback(const woan_interfaces::msg::MoveL::SharedPtr msg);
    
    /**
     * @brief MoveP命令回调
     */
    void movepCallback(const woan_interfaces::msg::MoveP::SharedPtr msg);
    
    /**
     * @brief joint_states回调 - 用于发布arm_state
     */
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    /**
     * @brief 定时发布机械臂状态
     */
    void publishArmState();
    
    /**
     * @brief 获取机械臂状态请求回调（请求-响应模式）
     */
    void getArmStateCallback(const std_msgs::msg::Empty::SharedPtr msg);
    
    // ========== 订阅器 ==========
    rclcpp::Subscription<woan_interfaces::msg::MoveJ>::SharedPtr movej_sub_;
    rclcpp::Subscription<woan_interfaces::msg::MoveL>::SharedPtr movel_sub_;
    rclcpp::Subscription<woan_interfaces::msg::MoveP>::SharedPtr movep_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr get_arm_state_cmd_sub_;
    
    // ========== 发布器 ==========
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr movej_result_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr movel_result_pub_;
    rclcpp::Publisher<woan_interfaces::msg::ArmState>::SharedPtr arm_state_pub_;
    
    // 转发到arm_control的发布器
    rclcpp::Publisher<woan_interfaces::msg::MoveJ>::SharedPtr arm_movej_pub_;
    rclcpp::Publisher<woan_interfaces::msg::MoveL>::SharedPtr arm_movel_pub_;
    rclcpp::Publisher<woan_interfaces::msg::MoveP>::SharedPtr arm_movep_pub_;
    
    // 订阅arm_control的结果
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_movej_result_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_movel_result_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_movep_result_sub_;
    
    // ========== 定时器 ==========
    rclcpp::TimerBase::SharedPtr state_timer_;
    
    // ========== 状态变量 ==========
    sensor_msgs::msg::JointState latest_joint_state_;
    std::string robot_model_;
    int dof_;
    std::atomic<uint8_t> arm_status_{0};  // 0:空闲 1:运动中 2:错误
    std::mutex state_mutex_;
};

} // namespace woan

