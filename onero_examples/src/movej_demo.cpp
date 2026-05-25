// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include "rclcpp/rclcpp.hpp"
#include "onero_interfaces/msg/move_j.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief MoveJ示例节点
 * 
 * 演示如何使用MoveJ API控制机械臂在关节空间运动。
 */
class MoveJDemo : public rclcpp::Node {
public:
    MoveJDemo() : Node("movej_demo_node") {
        RCLCPP_INFO(this->get_logger(), "MoveJ Demo Node Starting...");
        
        // 创建发布器
        movej_pub_ = this->create_publisher<onero_interfaces::msg::MoveJ>(
            "/onero_driver/movej_cmd", 10);
        
        // 创建订阅器（接收结果）
        result_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/onero_driver/movej_result", 10,
            std::bind(&MoveJDemo::resultCallback, this, std::placeholders::_1));
        
        // 延迟2秒后执行（等待其他节点启动）
        timer_ = this->create_wall_timer(
            2s, std::bind(&MoveJDemo::executeMoveJ, this));
        
        RCLCPP_INFO(this->get_logger(), "MoveJ Demo Node Initialized");
        RCLCPP_INFO(this->get_logger(), "Will send MoveJ command in 2 seconds...");
    }
    
private:
    void executeMoveJ() {
        // 停止定时器（只执行一次）
        timer_->cancel();
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Sending MoveJ command...");
        
        // 构造MoveJ消息
        auto msg = onero_interfaces::msg::MoveJ();
        
        // 目标关节角度（弧度）
        // 全零位置
        msg.joint = { 0,  0.0,  0.0,  -1.57,  0.0,  -1.57,  0.0};
        
        msg.speed_scale = 0.8;  //修改scale的限幅
        msg.trajectory_connect = 0;  // 立即执行
        
        // 打印目标关节角度
        RCLCPP_INFO(this->get_logger(), "Target joint angles (rad):");
        for (size_t i = 0; i < msg.joint.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  Joint %zu: %.3f", i, msg.joint[i]);
        }
        RCLCPP_INFO(this->get_logger(), "Speed scale: %.2f",msg.speed_scale);
        
        // 发布命令
        movej_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "MoveJ command sent successfully");
        RCLCPP_INFO(this->get_logger(), "Waiting for result...");
    }
    
    void resultCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "========================================");
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "✓ MoveJ executed successfully!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "✗ MoveJ execution failed!");
        }
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        // 延迟后关闭节点
        auto shutdown_timer = this->create_wall_timer(
            1s, []() { rclcpp::shutdown(); });
    }
    
    rclcpp::Publisher<onero_interfaces::msg::MoveJ>::SharedPtr movej_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr result_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveJDemo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

