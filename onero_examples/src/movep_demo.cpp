// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include "rclcpp/rclcpp.hpp"
#include "onero_interfaces/msg/command_result.hpp"
#include "onero_interfaces/msg/move_p.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief MoveP示例节点
 *
 * 仅在单臂模式下运行：
 *   ros2 launch onero_driver a1_l_driver.launch.py
 *   ros2 run onero_examples movep_demo
 * 注意：实体机运行前请确认机械臂采用单臂安装方式；双臂安装方式需先重新评估轨迹，避免与桌面干涉。
 *
 * 演示如何使用 MoveP API 进行笛卡尔空间点到点运动。
 */
class MovePDemo : public rclcpp::Node {
public:
    MovePDemo() : Node("movep_demo_node") {
        RCLCPP_INFO(this->get_logger(), "MoveP Demo Node Starting...");
        
        // 创建发布器
        movep_pub_ = this->create_publisher<onero_interfaces::msg::MoveP>(
            "/onero_arm/movep", 10);
        
        // 创建订阅器（接收结果）
        result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/movep_result", 10,
            std::bind(&MovePDemo::resultCallback, this, std::placeholders::_1));
        
        // 延迟2秒后执行
        timer_ = this->create_wall_timer(
            2s, std::bind(&MovePDemo::executeMoveP, this));
        
        RCLCPP_INFO(this->get_logger(), "MoveP Demo Node Initialized");
        RCLCPP_INFO(this->get_logger(),
                    "Target pose verified with a1_l_driver.launch.py and a1_r_driver.launch.py in sim");
        RCLCPP_INFO(this->get_logger(), "Will send MoveP command in 2 seconds...");
    }
    
private:
    void executeMoveP() {
        // 停止定时器（只执行一次）
        timer_->cancel();
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Sending MoveP command...");
        
        // 构造MoveP消息
        auto msg = onero_interfaces::msg::MoveP();

        // Verified in sim with a1_l_driver.launch.py and a1_r_driver.launch.py.
        msg.pose.position.x = -0.4786;
        msg.pose.position.y = 0.2144;
        msg.pose.position.z = 0.2552;
        // ROS geometry_msgs/Quaternion 字段顺序为 x,y,z,w。
        msg.pose.orientation.x = -0.6082;
        msg.pose.orientation.y = -0.2328;
        msg.pose.orientation.z = 0.3607;
        msg.pose.orientation.w = 0.6677;

        // 设置参数
        msg.trajectory_connect = 0;   // 立即执行（不缓冲）
        msg.speed_scale = 0.8;        // 速度缩放80%

        // 打印目标位姿
        RCLCPP_INFO(this->get_logger(), "Target pose:");
        RCLCPP_INFO(this->get_logger(), "  Position: [%.4f, %.4f, %.4f] m",
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Orientation (quaternion x,y,z,w): [%.4f, %.4f, %.4f, %.4f]",
                    msg.pose.orientation.x, msg.pose.orientation.y,
                    msg.pose.orientation.z, msg.pose.orientation.w);
        
        // 发布命令
        movep_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "MoveP command sent successfully");
        RCLCPP_INFO(this->get_logger(), "Waiting for result...");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }
    
    void resultCallback(const onero_interfaces::msg::CommandResult::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "========================================");
        if (msg->success) {
            RCLCPP_INFO(this->get_logger(), "✓ MoveP executed successfully!");
        } else {
            RCLCPP_WARN(this->get_logger(), "✗ MoveP execution failed: %s", msg->error_message.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        // 延迟后关闭节点
        auto shutdown_timer = this->create_wall_timer(
            1s, []() { rclcpp::shutdown(); });
    }
    
    rclcpp::Publisher<onero_interfaces::msg::MoveP>::SharedPtr movep_pub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr result_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MovePDemo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
