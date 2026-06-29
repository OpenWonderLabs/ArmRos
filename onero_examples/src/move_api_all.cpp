// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include "rclcpp/rclcpp.hpp"
#include "onero_interfaces/msg/command_result.hpp"
#include "onero_interfaces/msg/move_j.hpp"
#include "onero_interfaces/msg/move_l.hpp"
#include "onero_interfaces/msg/move_p.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <chrono>
#include <thread>
#include <vector>
#include <string>

using namespace std::chrono_literals;

/**
 * @brief Move API All演示节点
 *
 * 仅在单臂模式下运行：
 *   ros2 launch onero_driver a1_l_driver.launch.py
 *   ros2 run onero_examples move_all
 * 注意：实体机运行前请确认机械臂采用单臂安装方式；双臂安装方式需先重新评估轨迹，避免与桌面干涉。
 *
 * 演示如何顺序使用所有Move API：
 * - MoveJ: 关节空间运动
 * - MoveL: 笛卡尔空间运动
 * - MoveP: 笛卡尔空间点到点运动
 * 每个API完全执行后才开始下一个
 */
class MoveApiAllDemo : public rclcpp::Node {
public:
    MoveApiAllDemo() : Node("move_api_all_demo"), result_received_(false), current_api_("") {
        // 创建发布器
        movej_pub_ = this->create_publisher<onero_interfaces::msg::MoveJ>("/onero_arm/movej", 10);
        movel_pub_ = this->create_publisher<onero_interfaces::msg::MoveL>("/onero_arm/movel", 10);
        movep_pub_ = this->create_publisher<onero_interfaces::msg::MoveP>("/onero_arm/movep", 10);
        
        // 创建结果订阅
        movej_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/movej_result", 10,
            [this](const onero_interfaces::msg::CommandResult::SharedPtr msg) { resultCallback("MoveJ", msg); });
        
        movel_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/movel_result", 10,
            [this](const onero_interfaces::msg::CommandResult::SharedPtr msg) { resultCallback("MoveL", msg); });
        
        movep_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
            "/onero_arm/movep_result", 10,
            [this](const onero_interfaces::msg::CommandResult::SharedPtr msg) { resultCallback("MoveP", msg); });
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Move API All Demo");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "This demo shows how to use all Move APIs sequentially:");
        RCLCPP_INFO(this->get_logger(), "- MoveJ: Joint space motion");
        RCLCPP_INFO(this->get_logger(), "- MoveL: Cartesian space motion");
        RCLCPP_INFO(this->get_logger(), "- MoveP: Cartesian point-to-point motion");
        RCLCPP_INFO(this->get_logger(), "MoveL/MoveP target pose is verified in A1-L and A1-R sim");
        RCLCPP_INFO(this->get_logger(), "Each API will execute completely before the next one starts");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }
    
    void run() {
        // 等待1秒
        std::this_thread::sleep_for(1s);
        
        // 第一步：MoveJ运动
        RCLCPP_INFO(this->get_logger(), "\n[Step 1] Executing MoveJ motion...");
        RCLCPP_INFO(this->get_logger(), "========================================");
        executeMoveJ();

        // 等待1秒确保稳定
        RCLCPP_INFO(this->get_logger(), "Waiting 1s to settle...");
        std::this_thread::sleep_for(1s);

        // 第二步：MoveL运动
        RCLCPP_INFO(this->get_logger(), "\n[Step 2] Executing MoveL motion...");
        RCLCPP_INFO(this->get_logger(), "========================================");
        executeMoveL();

        // 等待1秒确保稳定
        RCLCPP_INFO(this->get_logger(), "Waiting 1s to settle...");
        std::this_thread::sleep_for(1s);

        // 第三步：MoveP运动
        RCLCPP_INFO(this->get_logger(), "\n[Step 3] Executing MoveP motion...");
        RCLCPP_INFO(this->get_logger(), "========================================");
        executeMoveP();
        
        RCLCPP_INFO(this->get_logger(), "\n========================================");
        RCLCPP_INFO(this->get_logger(), "✓ Move API All Demo Completed!");
        RCLCPP_INFO(this->get_logger(), "All Move APIs (MoveJ/MoveL/MoveP) executed sequentially.");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }
    
private:
    void executeMoveJ() {
        RCLCPP_INFO(this->get_logger(), "  Sending MoveJ command...");
        std::vector<float> j1 = {0.0f, 0.2f, 0.0f, -0.5f, 0.0f, 0.0f, 0.0f};
        current_api_ = "MoveJ";
        sendMoveJ(j1);
        waitForResult(10.0);
    }
    
    void executeMoveL() {
        RCLCPP_INFO(this->get_logger(), "  Sending MoveL command...");
        current_api_ = "MoveL";
        sendMoveL(-0.4786, 0.2144, 0.2552);
        waitForResult(15.0);
    }
    
    void executeMoveP() {
        RCLCPP_INFO(this->get_logger(), "  Sending MoveP command...");
        current_api_ = "MoveP";
        sendMoveP(-0.4786, 0.2144, 0.2552);
        waitForResult(15.0);
    }
    
    void sendMoveJ(const std::vector<float>& joint_angles) {
        auto msg = std::make_shared<onero_interfaces::msg::MoveJ>();
        if (joint_angles.size() != msg->joint_positions.size()) {
            RCLCPP_ERROR(this->get_logger(),
                "joint_angles size %zu != %zu", joint_angles.size(), msg->joint_positions.size());
            return;
        }
        std::copy(joint_angles.begin(), joint_angles.end(), msg->joint_positions.begin());
        msg->speed_scale = 0.8;
        msg->trajectory_connect = 0;  // 不使用轨迹连接，立即执行

        result_received_ = false;
        movej_pub_->publish(*msg);
    }
    
    void sendMoveL(double x, double y, double z) {
        auto msg = std::make_shared<onero_interfaces::msg::MoveL>();
        msg->pose.position.x = x;
        msg->pose.position.y = y;
        msg->pose.position.z = z;
        msg->pose.orientation.x = -0.6082;
        msg->pose.orientation.y = -0.2328;
        msg->pose.orientation.z = 0.3607;
        msg->pose.orientation.w = 0.6677;
        msg->speed_scale = 0.8;
        msg->trajectory_connect = 0;  
        
        result_received_ = false;
        movel_pub_->publish(*msg);
    }
    
    void sendMoveP(double x, double y, double z) {
        auto msg = std::make_shared<onero_interfaces::msg::MoveP>();
        msg->pose.position.x = x;
        msg->pose.position.y = y;
        msg->pose.position.z = z;
        msg->pose.orientation.x = -0.6082;
        msg->pose.orientation.y = -0.2328;
        msg->pose.orientation.z = 0.3607;
        msg->pose.orientation.w = 0.6677;
        msg->trajectory_connect = 0;  // 不使用轨迹连接，立即执行
        msg->speed_scale = 0.8;
        
        result_received_ = false;
        movep_pub_->publish(*msg);
    }
    
    void resultCallback(const std::string& api_type, const onero_interfaces::msg::CommandResult::SharedPtr msg) {
        if (current_api_ == api_type) {
            RCLCPP_INFO(this->get_logger(), "✓ %s result: %s %s",
                       api_type.c_str(), msg->success ? "SUCCESS" : "FAILED",
                       msg->error_message.c_str());
            result_received_ = true;
        }
    }
    
    void waitForResult(double timeout_sec) {
        auto start_time = this->now();
        while (!result_received_) {
            rclcpp::spin_some(this->shared_from_this());
            std::this_thread::sleep_for(50ms);
            
            auto elapsed = (this->now() - start_time).seconds();
            if (elapsed > timeout_sec) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for result");
                break;
            }
        }
    }
    
    // 发布器和订阅器
    rclcpp::Publisher<onero_interfaces::msg::MoveJ>::SharedPtr movej_pub_;
    rclcpp::Publisher<onero_interfaces::msg::MoveL>::SharedPtr movel_pub_;
    rclcpp::Publisher<onero_interfaces::msg::MoveP>::SharedPtr movep_pub_;
    
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr movej_result_sub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr movel_result_sub_;
    rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr movep_result_sub_;
    
    // 状态变量
    bool result_received_;
    std::string current_api_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MoveApiAllDemo>();
    node->run();
    
    rclcpp::shutdown();
    return 0;
}
