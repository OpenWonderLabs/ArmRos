// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include "rclcpp/rclcpp.hpp"
#include "onero_interfaces/msg/command_result.hpp"
#include "onero_interfaces/msg/move_l.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief MoveL示例节点
 */
class MoveLDemo : public rclcpp::Node {
public:
    MoveLDemo() : Node("movel_demo_node") {
        RCLCPP_INFO(this->get_logger(), "MoveL Demo Node Starting...");

        movel_pub_ = this->create_publisher<onero_interfaces::msg::MoveL>(
            "/onero_arm/movel", 10);

        result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/movel_result", 10,
            std::bind(&MoveLDemo::resultCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            2s, std::bind(&MoveLDemo::executeMoveL, this));

        RCLCPP_INFO(this->get_logger(), "MoveL Demo Node Initialized");
        RCLCPP_INFO(this->get_logger(),
                    "Target pose verified with a1_l_driver.launch.py and a1_r_driver.launch.py in sim");
        RCLCPP_INFO(this->get_logger(), "Will send MoveL command in 2 seconds...");
    }

private:
    void executeMoveL() {
        timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Sending MoveL command...");

        auto msg = onero_interfaces::msg::MoveL();

        // Verified in sim with a1_l_driver.launch.py and a1_r_driver.launch.py.
        msg.pose.position.x = -0.4786;
        msg.pose.position.y = 0.2144;
        msg.pose.position.z = 0.2552;

        msg.pose.orientation.x = -0.6082;
        msg.pose.orientation.y = -0.2328;
        msg.pose.orientation.z = 0.3607;
        msg.pose.orientation.w = 0.6677;

        msg.speed_scale = 0.8;
        msg.trajectory_connect = 0;

        RCLCPP_INFO(this->get_logger(), "Target pose:");
        RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f] m",
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Orientation (quaternion x,y,z,w): [%.3f, %.3f, %.3f, %.3f]",
                    msg.pose.orientation.x, msg.pose.orientation.y,
                    msg.pose.orientation.z, msg.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "Speed Scale: %.3f", msg.speed_scale);

        movel_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "MoveL command sent successfully");
        RCLCPP_INFO(this->get_logger(), "Waiting for result...");
    }

    void resultCallback(const onero_interfaces::msg::CommandResult::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "========================================");
        if (msg->success) {
            RCLCPP_INFO(this->get_logger(), "✓ MoveL executed successfully!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "✗ MoveL execution failed: %s", msg->error_message.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "========================================");

        auto shutdown_timer = this->create_wall_timer(
            1s, []() { rclcpp::shutdown(); });
    }

    rclcpp::Publisher<onero_interfaces::msg::MoveL>::SharedPtr movel_pub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr result_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveLDemo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
