// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include "rclcpp/rclcpp.hpp"
#include "onero_interfaces/msg/command_result.hpp"
#include "onero_interfaces/msg/move_p.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <vector>

using namespace std::chrono_literals;

namespace {

struct Waypoint {
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
};

std::vector<Waypoint> makeWaypoints() {
    // Screened in a1_l simulation. Quaternion order here matches geometry_msgs: x,y,z,w.
    return {
        {-0.29906555728559786,  0.19061056916493688, 0.54581449331356047,
         -0.09161293270344323,  0.22448114213969839, 0.66183579755165534, 0.70935792408649057},
        {-0.44966991656879418,  0.12001145052012660, 0.45489377182934820,
         -0.29528149191149045, -0.22597168280766958, 0.13671169667145860, 0.91818056562791928},
        {-0.21650168194786162,  0.18305783153440255, 0.56216260955952368,
          0.14796435630969368, -0.00791007098182435, -0.03178211563834282, 0.98845024010542482},
        {-0.06036199338134762,  0.26290762210053048, 0.59972357692906220,
         -0.00330802665714571,  0.20802603214958235, -0.50131343058007971, 0.83988039102467948},
        {-0.04994023275892231, -0.20206990849809464, 0.59991841122305045,
          0.59115580644444488,  0.15332162583941683, -0.23266317752697568, 0.75689836661207510},
    };
}

}  // namespace

/**
 * @brief MoveP 连续途经点示例节点
 *
 * 仅在单臂模式下运行：
 *   ros2 launch onero_driver a1_l_driver.launch.py
 *   ros2 run onero_examples movep_via_points_demo
 * 注意：实体机运行前请确认机械臂采用单臂安装方式；双臂安装方式需先重新评估轨迹，避免与桌面干涉。
 *
 * 本示例只通过 ROS2 话题下发 MoveP 命令。前 N-1 个点使用 trajectory_connect=1 缓冲，
 * 最后一个点使用 trajectory_connect=0 触发驱动执行整段连续轨迹。
 */
class MovePViaPointsDemo : public rclcpp::Node {
public:
    MovePViaPointsDemo()
        : Node("movep_via_points_demo_node"),
          waypoints_(makeWaypoints()) {
        speed_scale_ = this->declare_parameter<double>("speed_scale", 1.2);

        movep_pub_ = this->create_publisher<onero_interfaces::msg::MoveP>(
            "/onero_arm/movep", 10);

        result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/movep_result", 10,
            std::bind(&MovePViaPointsDemo::resultCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            2s, std::bind(&MovePViaPointsDemo::startMovePViaPoints, this));

        RCLCPP_INFO(this->get_logger(), "MoveP via-points demo node initialized");
        RCLCPP_INFO(this->get_logger(),
                    "Default driver launch: ros2 launch onero_driver a1_l_driver.launch.py");
        RCLCPP_INFO(this->get_logger(), "Will send %zu MoveP waypoints in 2 seconds...",
                    waypoints_.size());
    }

private:
    void startMovePViaPoints() {
        timer_->cancel();

        if (waypoints_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No MoveP waypoints configured");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Sending MoveP via-points command sequence...");
        RCLCPP_INFO(this->get_logger(),
                    "Using trajectory_connect: buffer first %zu point(s), execute on final point",
                    waypoints_.size() > 0 ? waypoints_.size() - 1 : 0);
        RCLCPP_INFO(this->get_logger(), "Speed scale: %.2f", speed_scale_);

        current_index_ = 0;
        publishCurrentWaypoint();
    }

    void publishCurrentWaypoint() {
        if (current_index_ >= waypoints_.size()) {
            return;
        }

        const auto& point = waypoints_[current_index_];
        auto msg = onero_interfaces::msg::MoveP();
        msg.pose.position.x = point.x;
        msg.pose.position.y = point.y;
        msg.pose.position.z = point.z;
        msg.pose.orientation.x = point.qx;
        msg.pose.orientation.y = point.qy;
        msg.pose.orientation.z = point.qz;
        msg.pose.orientation.w = point.qw;
        msg.speed_scale = speed_scale_;
        msg.trajectory_connect = isFinalWaypoint() ? kExecuteNow : kBufferAndConnect;

        RCLCPP_INFO(this->get_logger(),
                    "Publishing waypoint %zu/%zu: pos=[%.4f, %.4f, %.4f], "
                    "quat_xyzw=[%.4f, %.4f, %.4f, %.4f], trajectory_connect=%u",
                    current_index_ + 1, waypoints_.size(),
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y,
                    msg.pose.orientation.z, msg.pose.orientation.w,
                    static_cast<unsigned int>(msg.trajectory_connect));

        movep_pub_->publish(msg);
    }

    void resultCallback(const onero_interfaces::msg::CommandResult::SharedPtr msg) {
        if (current_index_ >= waypoints_.size()) {
            return;
        }

        const bool final_waypoint = isFinalWaypoint();
        const char* phase = final_waypoint ? "execution result" : "buffer ack";

        if (!msg->success) {
            RCLCPP_ERROR(this->get_logger(), "MoveP waypoint %zu %s failed: %s",
                         current_index_ + 1, phase, msg->error_message.c_str());
            shutdownSoon();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "MoveP waypoint %zu %s succeeded",
                    current_index_ + 1, phase);

        if (final_waypoint) {
            RCLCPP_INFO(this->get_logger(), "MoveP via-points command sequence completed");
            RCLCPP_INFO(this->get_logger(), "========================================");
            shutdownSoon();
            return;
        }

        ++current_index_;
        publishCurrentWaypoint();
    }

    bool isFinalWaypoint() const {
        return current_index_ + 1 >= waypoints_.size();
    }

    void shutdownSoon() {
        shutdown_timer_ = this->create_wall_timer(
            1s, []() { rclcpp::shutdown(); });
    }

    static constexpr uint8_t kExecuteNow = 0;
    static constexpr uint8_t kBufferAndConnect = 1;

    rclcpp::Publisher<onero_interfaces::msg::MoveP>::SharedPtr movep_pub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr result_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;

    std::vector<Waypoint> waypoints_;
    size_t current_index_{0};
    double speed_scale_{1.2};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MovePViaPointsDemo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
