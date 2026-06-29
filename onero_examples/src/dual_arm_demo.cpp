// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include "rclcpp/rclcpp.hpp"
#include "onero_interfaces/msg/command_result.hpp"
#include "onero_interfaces/msg/dual_move_j.hpp"
#include "onero_interfaces/msg/move_j.hpp"
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief 双臂综合示例节点
 *
 * 仅在双臂模式（a1_dual_driver.launch.py）下运行：
 *   ros2 launch onero_driver a1_dual_driver.launch.py
 *   ros2 run onero_examples dual_arm_demo
 *
 * 按顺序演示三段动作：
 *   1. 双臂同步运动     —— 发布 DualMoveJ 到 /onero_arm/dual_arm/movej
 *   2. 双臂下单独控左臂 —— 发布 MoveJ 到 /onero_arm/left_arm/movej
 *   3. 双臂下单独控右臂 —— 发布 MoveJ 到 /onero_arm/right_arm/movej
 *
 * 每段都订阅对应的 *_result，等收到 success 回调后再触发下一段。
 */
class DualArmDemo : public rclcpp::Node {
public:
    DualArmDemo() : Node("dual_arm_demo_node"), step_(0) {
        RCLCPP_INFO(this->get_logger(), "Dual Arm Demo Node Starting...");

        dual_movej_pub_ = this->create_publisher<onero_interfaces::msg::DualMoveJ>(
            "/onero_arm/dual_arm/movej", 10);
        left_movej_pub_ = this->create_publisher<onero_interfaces::msg::MoveJ>(
            "/onero_arm/left_arm/movej", 10);
        right_movej_pub_ = this->create_publisher<onero_interfaces::msg::MoveJ>(
            "/onero_arm/right_arm/movej", 10);

        dual_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/dual_arm/movej_result", 10,
            std::bind(&DualArmDemo::dualResultCallback, this, std::placeholders::_1));
        left_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/left_arm/movej_result", 10,
            std::bind(&DualArmDemo::leftResultCallback, this, std::placeholders::_1));
        right_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/right_arm/movej_result", 10,
            std::bind(&DualArmDemo::rightResultCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            2s, std::bind(&DualArmDemo::startSequence, this));

        RCLCPP_INFO(this->get_logger(), "Will start sequence in 2 seconds...");
        RCLCPP_INFO(this->get_logger(), "Sequence: dual sync -> left only -> right only");
    }

private:
    void startSequence() {
        timer_->cancel();
        sendDualSync();
    }

    void sendDualSync() {
        step_ = 1;
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "[Step 1/3] DualMoveJ -> /onero_arm/dual_arm/movej");
        RCLCPP_INFO(this->get_logger(), "  Both arms move synchronously to a symmetric pose.");

        auto msg = onero_interfaces::msg::DualMoveJ();
        // 左臂目标
        msg.left_joint = { 0.0f, 0.0f, 0.0f, -1.57f, 0.0f, 1.57f, 0.0f };
        // 右臂目标（对称姿态）
        msg.right_joint = { 0.0f, 0.0f, 0.0f, 1.57f, 0.0f, -1.57f, 0.0f };
        msg.speed_scale = 0.5f;

        dual_movej_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "  DualMoveJ command sent. Waiting for result...");
    }

    void sendLeftOnly() {
        step_ = 2;
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "[Step 2/3] MoveJ -> /onero_arm/left_arm/movej (left arm only)");

        auto msg = onero_interfaces::msg::MoveJ();
        msg.joint_positions = { 0.3f, 0.0f, 0.0f, -1.57f, 0.0f, 1.57f, 0.0f };
        msg.speed_scale = 0.5f;
        msg.trajectory_connect = 0;

        left_movej_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "  Left-arm MoveJ command sent. Waiting for result...");
    }

    void sendRightOnly() {
        step_ = 3;
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "[Step 3/3] MoveJ -> /onero_arm/right_arm/movej (right arm only)");

        auto msg = onero_interfaces::msg::MoveJ();
        msg.joint_positions = { -0.3f, 0.0f, 0.0f, 1.57f, 0.0f, -1.57f, 0.0f };
        msg.speed_scale = 0.5f;
        msg.trajectory_connect = 0;

        right_movej_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "  Right-arm MoveJ command sent. Waiting for result...");
    }

    void dualResultCallback(const onero_interfaces::msg::CommandResult::SharedPtr msg) {
        if (step_ != 1) return;
        logResult("DualMoveJ", msg);
        if (msg->success) {
            sendLeftOnly();
        } else {
            shutdown();
        }
    }

    void leftResultCallback(const onero_interfaces::msg::CommandResult::SharedPtr msg) {
        if (step_ != 2) return;
        logResult("Left-arm MoveJ", msg);
        if (msg->success) {
            sendRightOnly();
        } else {
            shutdown();
        }
    }

    void rightResultCallback(const onero_interfaces::msg::CommandResult::SharedPtr msg) {
        if (step_ != 3) return;
        logResult("Right-arm MoveJ", msg);
        RCLCPP_INFO(this->get_logger(), "========================================");
        if (msg->success) {
            RCLCPP_INFO(this->get_logger(), "Dual Arm Demo Completed.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Dual Arm Demo Aborted.");
        }
        shutdown();
    }

    void logResult(const char* tag, const onero_interfaces::msg::CommandResult::SharedPtr& msg) {
        if (msg->success) {
            RCLCPP_INFO(this->get_logger(), "  ✓ %s succeeded", tag);
        } else {
            RCLCPP_ERROR(this->get_logger(), "  ✗ %s failed: %s", tag, msg->error_message.c_str());
        }
    }

    void shutdown() {
        if (!shutdown_timer_) {
            shutdown_timer_ = this->create_wall_timer(1s, []() { rclcpp::shutdown(); });
        }
    }

    rclcpp::Publisher<onero_interfaces::msg::DualMoveJ>::SharedPtr dual_movej_pub_;
    rclcpp::Publisher<onero_interfaces::msg::MoveJ>::SharedPtr left_movej_pub_;
    rclcpp::Publisher<onero_interfaces::msg::MoveJ>::SharedPtr right_movej_pub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr dual_result_sub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr left_result_sub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr right_result_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
    int step_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualArmDemo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
