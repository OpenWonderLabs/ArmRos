#include <rclcpp/rclcpp.hpp>
#include "woan_interfaces/msg/move_j.hpp"
#include "woan_interfaces/msg/move_l.hpp"
#include "woan_interfaces/msg/move_p.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class TrajectoryConnectDemo : public rclcpp::Node
{
public:
    TrajectoryConnectDemo() : Node("trajectory_connect_demo")
    {
        // 创建发布器
        movej_pub_ = this->create_publisher<woan_interfaces::msg::MoveJ>(
            "/woan_driver/movej_cmd", 10);
        movel_pub_ = this->create_publisher<woan_interfaces::msg::MoveL>(
            "/woan_driver/movel_cmd", 10);
        movep_pub_ = this->create_publisher<woan_interfaces::msg::MoveP>(
            "/woan_driver/movep_cmd", 10);
        
        // 创建结果订阅
        movej_result_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/woan_driver/movej_result", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (current_api_ == "MoveJ") {
                    RCLCPP_INFO(this->get_logger(), "✓ MoveJ result: %s", 
                               msg->data ? "SUCCESS" : "FAILED");
                    result_received_ = true;
                }
            });
        
        movel_result_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/woan_driver/movel_result", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (current_api_ == "MoveL") {
                    RCLCPP_INFO(this->get_logger(), "✓ MoveL result: %s", 
                               msg->data ? "SUCCESS" : "FAILED");
                    result_received_ = true;
                }
            });
        
        movep_result_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/woan_driver/movep_result", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (current_api_ == "MoveP") {
                    RCLCPP_INFO(this->get_logger(), "✓ MoveP result: %s", 
                               msg->data ? "SUCCESS" : "FAILED");
                    result_received_ = true;
                }
            });
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Trajectory Connect Demo");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "This demo shows how to use trajectory_connect");
        RCLCPP_INFO(this->get_logger(), "to send multiple waypoints using all Move APIs:");
        RCLCPP_INFO(this->get_logger(), "- MoveJ: Joint space motion");
        RCLCPP_INFO(this->get_logger(), "- MoveL: Cartesian space motion");
        RCLCPP_INFO(this->get_logger(), "- MoveP: Pose transmission");
        RCLCPP_INFO(this->get_logger(), "========================================\n");
    }
    
    void run()
    {
        std::this_thread::sleep_for(1s);
        
        // ========== 混合轨迹连接演示: MoveJ -> MoveL -> MoveP ==========
        RCLCPP_INFO(this->get_logger(), "\n[测试] 混合轨迹连接: MoveJ -> MoveL -> MoveP");
        RCLCPP_INFO(this->get_logger(), "========================================");
        testMixedTrajectoryConnect();
        
        RCLCPP_INFO(this->get_logger(), "\n========================================");
        RCLCPP_INFO(this->get_logger(), "✓ Trajectory Connect Demo Completed!");
        RCLCPP_INFO(this->get_logger(), "All Move APIs (MoveJ/MoveL/MoveP) tested.");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }
    
private:
    // ========== 测试 混合轨迹连接：MoveJ -> MoveL -> MoveP ==========
    void testMixedTrajectoryConnect()
    {
        // 1) MoveJ：缓冲一个关节目标（trajectory_connect=1）
        RCLCPP_INFO(this->get_logger(), "  Buffering MoveJ waypoint...");
        std::vector<float> j1 = {0.0f, 0.2f, 0.0f, -0.5f, 0.0f, 0.0f, 0.0f};
        current_api_ = "MoveJ";
        sendMoveJ(j1, 1);
        waitForResult(2.0);
        std::this_thread::sleep_for(200ms);

        // 2) MoveL：缓冲一个直线位姿（trajectory_connect=1）
        RCLCPP_INFO(this->get_logger(), "  Buffering MoveL waypoint...");
        current_api_ = "MoveL";
        sendMoveL(0.32f, 0.18f, 0.25f, 1);
        waitForResult(2.0);
        std::this_thread::sleep_for(200ms);

        // 3) MoveP：触发执行，作为最终段（trajectory_connect=0）
        RCLCPP_INFO(this->get_logger(), "  Triggering execution with MoveP (final waypoint)...");
        current_api_ = "MoveP";
        sendMoveP(0.28f, 0.15f, 0.22f, 0);
        waitForResult(20.0);
    }
    
    // ========== 发送函数 ==========
    void sendMoveJ(const std::vector<float>& joint_angles, uint8_t trajectory_connect)
    {
        auto msg = woan_interfaces::msg::MoveJ();
        msg.joint = joint_angles;
        msg.speed_scale = 0.5f;  // 慢速，安全
        msg.trajectory_connect = trajectory_connect;
        
        result_received_ = false;
        movej_pub_->publish(msg);
    }
    
    void sendMoveL(float x, float y, float z, uint8_t trajectory_connect)
    {
        auto msg = woan_interfaces::msg::MoveL();
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = z;
        msg.pose.orientation.w = 1.0;
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.speed_scale = 0.5f; 
        msg.trajectory_connect = trajectory_connect;
        
        result_received_ = false;
        movel_pub_->publish(msg);
    }
    
    void sendMoveP(float x, float y, float z, uint8_t trajectory_connect)
    {
        auto msg = woan_interfaces::msg::MoveP();
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = z;
        msg.pose.orientation.w = 1.0;
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.trajectory_connect = trajectory_connect;
        msg.speed_scale = 0.5f;
        
        result_received_ = false;
        movep_pub_->publish(msg);
    }
    
    void waitForResult(double timeout_sec)
    {
        auto start = std::chrono::steady_clock::now();
        while (!result_received_) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(50ms);
            
            auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed > timeout_sec) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for result");
                break;
            }
        }
    }
    
    rclcpp::Publisher<woan_interfaces::msg::MoveJ>::SharedPtr movej_pub_;
    rclcpp::Publisher<woan_interfaces::msg::MoveL>::SharedPtr movel_pub_;
    rclcpp::Publisher<woan_interfaces::msg::MoveP>::SharedPtr movep_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movej_result_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movel_result_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movep_result_sub_;
    bool result_received_ = false;
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

