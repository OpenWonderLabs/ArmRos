// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "onero_interfaces/msg/arm_state.hpp"
#include "onero_interfaces/msg/command_result.hpp"
#include "onero_interfaces/msg/move_j.hpp"
#include "onero_interfaces/msg/move_l.hpp"
#include "onero_interfaces/msg/move_p.hpp"
#include "onero_interfaces/srv/end_effector_pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "onero_interface_cpp.h"

#include <memory>
#include <string>

namespace onero {

struct PerArmInterface {
    std::string sub_ns;  // "" for a namespaced single-arm node, left_arm/right_arm for dual mode.
    std::unique_ptr<onero_api::OneroArm> arm;

    rclcpp::Subscription<onero_interfaces::msg::MoveJ>::SharedPtr movej_sub;
    rclcpp::Subscription<onero_interfaces::msg::MoveL>::SharedPtr movel_sub;
    rclcpp::Subscription<onero_interfaces::msg::MoveP>::SharedPtr movep_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub;

    rclcpp::Publisher<onero_interfaces::msg::CommandResult>::SharedPtr movej_result_pub;
    rclcpp::Publisher<onero_interfaces::msg::CommandResult>::SharedPtr movel_result_pub;
    rclcpp::Publisher<onero_interfaces::msg::CommandResult>::SharedPtr movep_result_pub;
    rclcpp::Publisher<onero_interfaces::msg::CommandResult>::SharedPtr stop_result_pub;
    rclcpp::Publisher<onero_interfaces::msg::ArmState>::SharedPtr arm_state_pub;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_buffer_srv;
    rclcpp::Service<onero_interfaces::srv::EndEffectorPose>::SharedPtr get_end_pose_srv;
};

}  // namespace onero
