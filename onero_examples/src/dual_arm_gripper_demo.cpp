// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include "rclcpp/rclcpp.hpp"
#include "onero_interfaces/msg/command_result.hpp"
#include "onero_interfaces/msg/dual_gripper.hpp"
#include "onero_interfaces/msg/dual_move_j.hpp"
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief 双臂带夹爪示例节点
 *
 * 仅在双臂模式 + 左右两侧都启用夹爪时运行：
 *   ros2 launch onero_driver a1_dual_driver.launch.py left_gripper:=true right_gripper:=true
 *   ros2 run onero_examples dual_arm_gripper_demo
 *
 * 按顺序演示三段动作：
 *   1. DualMoveJ → 对称 home 位姿
 *   2. 双爪同步全闭 (left/right_position = 0)
 *   3. 双爪同步全开 (left/right_position = 100)
 *
 * 注：夹爪默认上电就是「全开」状态，所以演示从 close 开始 → 再回到 open。
 *
 * 走 /onero_arm/dual_arm/gripper 同步话题，driver 在内部 wall-clock 上同时
 * 下发左右两侧。一条 DualGripper 消息一次同步左右。
 */
class DualArmGripperDemo : public rclcpp::Node {
public:
    DualArmGripperDemo() : Node("dual_arm_gripper_demo_node"), step_(0) {
        RCLCPP_INFO(this->get_logger(), "Dual Arm Gripper Demo Starting...");

        dual_movej_pub_ = this->create_publisher<onero_interfaces::msg::DualMoveJ>(
            "/onero_arm/dual_arm/movej", 10);
        dual_gripper_pub_ = this->create_publisher<onero_interfaces::msg::DualGripper>(
            "/onero_arm/dual_arm/gripper", 10);

        dual_movej_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/dual_arm/movej_result", 10,
            std::bind(&DualArmGripperDemo::dualMovejResultCallback, this, std::placeholders::_1));
        dual_gripper_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/dual_arm/gripper_result", 10,
            std::bind(&DualArmGripperDemo::dualGripperResultCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            2s, std::bind(&DualArmGripperDemo::startSequence, this));

        RCLCPP_INFO(this->get_logger(),
            "Sequence: DualMoveJ home -> close both grippers -> open both grippers");
        RCLCPP_INFO(this->get_logger(), "Will start in 2 seconds...");
    }

private:
    void startSequence() {
        timer_->cancel();
        sendDualMoveJ();
    }

    void sendDualMoveJ() {
        step_ = 1;
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "[1/3] DualMoveJ -> symmetric home");

        auto msg = onero_interfaces::msg::DualMoveJ();
        msg.left_joint  = { 0.0f, 0.0f, 0.0f, -1.57f, 0.0f, 1.57f, 0.0f };
        msg.right_joint = { 0.0f, 0.0f, 0.0f, 1.57f, 0.0f, -1.57f, 0.0f };
        msg.speed_scale = 0.5f;
        dual_movej_pub_->publish(msg);
    }

    void sendDualGripper(float position, int step, const char* label) {
        step_ = step;
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "[%d/3] %s (left/right position=%.1f%%)",
                    step, label, position);

        auto msg = onero_interfaces::msg::DualGripper();
        msg.left_position  = position;
        msg.right_position = position;
        dual_gripper_pub_->publish(msg);
    }

    void dualMovejResultCallback(const onero_interfaces::msg::CommandResult::SharedPtr msg) {
        if (step_ != 1) return;
        if (!msg->success) {
            RCLCPP_ERROR(this->get_logger(), "DualMoveJ failed: %s", msg->error_message.c_str());
            scheduleShutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "DualMoveJ done.");
        sendDualGripper(0.0f, 2, "Close both grippers");
    }

    void dualGripperResultCallback(const onero_interfaces::msg::CommandResult::SharedPtr msg) {
        if (step_ != 2 && step_ != 3) return;
        if (!msg->success) {
            RCLCPP_ERROR(this->get_logger(), "DualGripper failed at step %d: %s",
                         step_, msg->error_message.c_str());
            scheduleShutdown();
            return;
        }
        if (step_ == 2) {
            RCLCPP_INFO(this->get_logger(), "Both grippers closed.");
            sendDualGripper(100.0f, 3, "Open both grippers");
        } else {
            RCLCPP_INFO(this->get_logger(), "Both grippers opened. Demo complete.");
            scheduleShutdown();
        }
    }

    void scheduleShutdown() {
        shutdown_timer_ = this->create_wall_timer(
            1s, []() { rclcpp::shutdown(); });
    }

    int step_;
    rclcpp::Publisher<onero_interfaces::msg::DualMoveJ>::SharedPtr dual_movej_pub_;
    rclcpp::Publisher<onero_interfaces::msg::DualGripper>::SharedPtr dual_gripper_pub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr dual_movej_result_sub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr dual_gripper_result_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualArmGripperDemo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
