// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include "rclcpp/rclcpp.hpp"
#include "onero_interfaces/msg/command_result.hpp"
#include "onero_interfaces/msg/gripper.hpp"
#include "onero_interfaces/msg/move_j.hpp"
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief 单臂带夹爪示例节点
 *
 * 仅在单臂模式 + 启用夹爪时运行：
 *   ros2 launch onero_driver a1_l_driver.launch.py gripper:=true
 *   ros2 run onero_examples single_arm_gripper_demo
 * 注意：实体机运行前请确认机械臂采用单臂安装方式；双臂安装方式需先重新评估轨迹，避免与桌面干涉。
 *
 * 按顺序演示三段动作：
 *   1. MoveJ → home 位姿
 *   2. 夹爪全闭 (position = 0)
 *   3. 夹爪全开 (position = 100)
 *
 * 注：夹爪默认上电就是「全开」状态，所以演示从 close 开始 → 再回到 open。
 *
 * 每段都订阅对应的 *_result，收到 success 回调后再触发下一段；任一段失败则
 * 打印错误并退出。
 */
class SingleArmGripperDemo : public rclcpp::Node {
public:
    SingleArmGripperDemo() : Node("single_arm_gripper_demo_node"), step_(0) {
        RCLCPP_INFO(this->get_logger(), "Single Arm Gripper Demo Starting...");

        movej_pub_ = this->create_publisher<onero_interfaces::msg::MoveJ>(
            "/onero_arm/movej", 10);
        gripper_pub_ = this->create_publisher<onero_interfaces::msg::Gripper>(
            "/onero_arm/gripper", 10);

        movej_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/movej_result", 10,
            std::bind(&SingleArmGripperDemo::movejResultCallback, this, std::placeholders::_1));
        gripper_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/gripper_result", 10,
            std::bind(&SingleArmGripperDemo::gripperResultCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            2s, std::bind(&SingleArmGripperDemo::startSequence, this));

        RCLCPP_INFO(this->get_logger(), "Sequence: MoveJ home -> close gripper -> open gripper");
        RCLCPP_INFO(this->get_logger(), "Will start in 2 seconds...");
    }

private:
    void startSequence() {
        timer_->cancel();
        sendMoveJ();
    }

    void sendMoveJ() {
        step_ = 1;
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "[1/3] MoveJ -> home");

        auto msg = onero_interfaces::msg::MoveJ();
        msg.joint_positions = { 0.0f, 0.0f, 0.0f, -1.57f, 0.0f, -1.57f, 0.0f };
        msg.speed_scale = 0.5f;
        msg.trajectory_connect = 0;
        movej_pub_->publish(msg);
    }

    void sendGripper(float position, int step, const char* label) {
        step_ = step;
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "[%d/3] %s (position=%.1f%%)", step, label, position);

        auto msg = onero_interfaces::msg::Gripper();
        msg.position = position;
        gripper_pub_->publish(msg);
    }

    void movejResultCallback(const onero_interfaces::msg::CommandResult::SharedPtr msg) {
        if (step_ != 1) return;
        if (!msg->success) {
            RCLCPP_ERROR(this->get_logger(), "MoveJ failed: %s", msg->error_message.c_str());
            scheduleShutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "MoveJ done.");
        sendGripper(0.0f, 2, "Close gripper");
    }

    void gripperResultCallback(const onero_interfaces::msg::CommandResult::SharedPtr msg) {
        if (step_ != 2 && step_ != 3) return;
        if (!msg->success) {
            RCLCPP_ERROR(this->get_logger(), "Gripper failed at step %d: %s",
                         step_, msg->error_message.c_str());
            scheduleShutdown();
            return;
        }
        if (step_ == 2) {
            RCLCPP_INFO(this->get_logger(), "Gripper closed.");
            sendGripper(100.0f, 3, "Open gripper");
        } else {
            RCLCPP_INFO(this->get_logger(), "Gripper opened. Demo complete.");
            scheduleShutdown();
        }
    }

    void scheduleShutdown() {
        shutdown_timer_ = this->create_wall_timer(
            1s, []() { rclcpp::shutdown(); });
    }

    int step_;
    rclcpp::Publisher<onero_interfaces::msg::MoveJ>::SharedPtr movej_pub_;
    rclcpp::Publisher<onero_interfaces::msg::Gripper>::SharedPtr gripper_pub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr movej_result_sub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr gripper_result_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SingleArmGripperDemo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
