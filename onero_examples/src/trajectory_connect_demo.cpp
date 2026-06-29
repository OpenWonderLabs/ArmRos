// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include <rclcpp/rclcpp.hpp>
#include "onero_interfaces/msg/command_result.hpp"
#include "onero_interfaces/msg/move_j.hpp"
#include "onero_interfaces/msg/move_l.hpp"
#include "onero_interfaces/msg/move_p.hpp"
#include "onero_define.h"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

/**
 * @brief 轨迹连接示例节点
 *
 * 仅适用于 a1_l 单臂模式：
 *   ros2 launch onero_driver a1_l_driver.launch.py
 *   ros2 run onero_examples trajectory_connect_demo
 * 注意：实体机运行前请确认机械臂采用单臂安装方式；双臂安装方式需先重新评估轨迹，避免与桌面干涉。
 *
 * 演示使用 trajectory_connect 缓冲 MoveJ、MoveL，并用 MoveP 触发混合轨迹执行。
 */
class TrajectoryConnectDemo : public rclcpp::Node
{
public:
    TrajectoryConnectDemo() : Node("trajectory_connect_demo")
    {
        // 创建发布器
        movej_pub_ = this->create_publisher<onero_interfaces::msg::MoveJ>(
            "/onero_arm/movej", 10);
        movel_pub_ = this->create_publisher<onero_interfaces::msg::MoveL>(
            "/onero_arm/movel", 10);
        movep_pub_ = this->create_publisher<onero_interfaces::msg::MoveP>(
            "/onero_arm/movep", 10);
        
        // 创建结果订阅
        movej_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/movej_result", 10,
            [this](const onero_interfaces::msg::CommandResult::SharedPtr msg) {
                if (current_api_ == "MoveJ") {
                    handleResult("MoveJ", msg);
                }
            });
        
        movel_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/movel_result", 10,
            [this](const onero_interfaces::msg::CommandResult::SharedPtr msg) {
                if (current_api_ == "MoveL") {
                    handleResult("MoveL", msg);
                }
            });
        
        movep_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/movep_result", 10,
            [this](const onero_interfaces::msg::CommandResult::SharedPtr msg) {
                if (current_api_ == "MoveP") {
                    handleResult("MoveP", msg);
                }
            });
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Trajectory Connect Demo");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "This demo shows how to use trajectory_connect");
        RCLCPP_INFO(this->get_logger(), "to send multiple waypoints using all Move APIs:");
        RCLCPP_INFO(this->get_logger(), "- MoveJ: Joint space motion");
        RCLCPP_INFO(this->get_logger(), "- MoveL: Cartesian space motion");
        RCLCPP_INFO(this->get_logger(), "- MoveP: Cartesian point-to-point motion");
        RCLCPP_INFO(this->get_logger(), "========================================\n");
    }
    
    void run()
    {
        std::this_thread::sleep_for(1s);
        
        // ========== 混合轨迹连接演示: MoveJ -> MoveL -> MoveP ==========
        RCLCPP_INFO(this->get_logger(), "\n[Test] Trajectory blending: MoveJ -> MoveL -> MoveP");
        RCLCPP_INFO(this->get_logger(), "========================================");
        const bool ok = testMixedTrajectoryConnect();
        
        RCLCPP_INFO(this->get_logger(), "\n========================================");
        if (ok) {
            RCLCPP_INFO(this->get_logger(), "✓ Trajectory Connect Demo Completed!");
            RCLCPP_INFO(this->get_logger(), "All Move APIs (MoveJ/MoveL/MoveP) tested.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Trajectory Connect Demo failed.");
        }
        RCLCPP_INFO(this->get_logger(), "========================================");
    }
    
private:
    // ========== 测试 混合轨迹连接：MoveJ -> MoveL -> MoveP ==========
    bool testMixedTrajectoryConnect()
    {
        // 1) MoveJ：缓冲一个关节目标（trajectory_connect=1）
        RCLCPP_INFO(this->get_logger(), "  Buffering MoveJ waypoint...");
        std::vector<float> j1 = {0.0f, 0.8f, 0.0f, -0.9f, 0.0f, 0.0f, 0.0f};
        current_api_ = "MoveJ";
        sendMoveJ(j1, 1);
        if (!waitForResult(2.0) || !last_result_success_) return false;
        std::this_thread::sleep_for(200ms);

        // 2) MoveL：缓冲一个直线位姿（trajectory_connect=1）
        RCLCPP_INFO(this->get_logger(), "  Buffering MoveL waypoint...");
        current_api_ = "MoveL";
        sendMoveL(-0.4786f, 0.2144f, 0.2552f, 1);
        if (!waitForResult(2.0) || !last_result_success_) return false;
        std::this_thread::sleep_for(200ms);

        // 3) MoveP：触发执行，作为最终段（trajectory_connect=0）
        RCLCPP_INFO(this->get_logger(), "  Triggering execution with MoveP (final waypoint)...");
        current_api_ = "MoveP";
        sendMoveP(-0.307447f, 0.260748f, 0.474408f, 0);
        return waitForResult(60.0) && last_result_success_;
    }
    
    // ========== 发送函数 ==========
    void sendMoveJ(const std::vector<float>& joint_angles, uint8_t trajectory_connect)
    {
        auto msg = onero_interfaces::msg::MoveJ();
        if (joint_angles.size() != msg.joint_positions.size()) {
            RCLCPP_ERROR(this->get_logger(),
                "joint_angles size %zu != %zu", joint_angles.size(), msg.joint_positions.size());
            return;
        }
        std::copy(joint_angles.begin(), joint_angles.end(), msg.joint_positions.begin());
        msg.speed_scale = 0.5f;  // 慢速，安全
        msg.trajectory_connect = trajectory_connect;

        result_received_ = false;
        last_result_success_ = false;
        current_command_is_buffer_ =
            trajectory_connect == static_cast<uint8_t>(onero_api::TrajectoryConnect::BUFFER);
        movej_pub_->publish(msg);
    }
    
    void sendMoveL(float x, float y, float z, uint8_t trajectory_connect)
    {
        auto msg = onero_interfaces::msg::MoveL();
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = z;
        msg.pose.orientation.x = -0.6082;
        msg.pose.orientation.y = -0.2328;
        msg.pose.orientation.z = 0.3607;
        msg.pose.orientation.w = 0.6677;
        msg.speed_scale = 0.8f;
        msg.trajectory_connect = trajectory_connect;
        
        result_received_ = false;
        last_result_success_ = false;
        current_command_is_buffer_ =
            trajectory_connect == static_cast<uint8_t>(onero_api::TrajectoryConnect::BUFFER);
        movel_pub_->publish(msg);
    }
    
    void sendMoveP(float x, float y, float z, uint8_t trajectory_connect)
    {
        auto msg = onero_interfaces::msg::MoveP();
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = z;
        msg.pose.orientation.x = -0.524384;
        msg.pose.orientation.y = 0.045629;
        msg.pose.orientation.z = 0.474378;
        msg.pose.orientation.w = 0.705624;
        msg.trajectory_connect = trajectory_connect;
        msg.speed_scale = 0.3f;
        
        result_received_ = false;
        last_result_success_ = false;
        current_command_is_buffer_ =
            trajectory_connect == static_cast<uint8_t>(onero_api::TrajectoryConnect::BUFFER);
        movep_pub_->publish(msg);
    }
    
    bool waitForResult(double timeout_sec)
    {
        auto start = std::chrono::steady_clock::now();
        while (!result_received_) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(50ms);
            
            auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed > timeout_sec) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for result");
                return false;
            }
        }
        return true;
    }

    void handleResult(const char* api, const onero_interfaces::msg::CommandResult::SharedPtr& msg)
    {
        const char* phase = current_command_is_buffer_ ? "buffer ack" : "execution result";
        RCLCPP_INFO(this->get_logger(), "%s %s %s: %s %s",
                   msg->success ? "✓" : "X", api, phase,
                   msg->success ? "SUCCESS" : "FAILED", msg->error_message.c_str());
        last_result_success_ = msg->success;
        result_received_ = true;
    }
    
    rclcpp::Publisher<onero_interfaces::msg::MoveJ>::SharedPtr movej_pub_;
    rclcpp::Publisher<onero_interfaces::msg::MoveL>::SharedPtr movel_pub_;
    rclcpp::Publisher<onero_interfaces::msg::MoveP>::SharedPtr movep_pub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr movej_result_sub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr movel_result_sub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr movep_result_sub_;
    bool result_received_ = false;
    bool last_result_success_ = false;
    bool current_command_is_buffer_ = false;
    std::string current_api_ = "";
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TrajectoryConnectDemo>();
    node->run();
    
    rclcpp::shutdown();
    return 0;
}
