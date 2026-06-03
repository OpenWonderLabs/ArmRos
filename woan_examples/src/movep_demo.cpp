#include "rclcpp/rclcpp.hpp"
#include "woan_interfaces/msg/move_p.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief MoveP示例节点
 * 
 * 演示如何使用MoveP API进行位姿透传控制。
 * MoveP适用于高频实时跟随场景，如视觉伺服。
 */
class MovePDemo : public rclcpp::Node {
public:
    MovePDemo() : Node("movep_demo_node") {
        RCLCPP_INFO(this->get_logger(), "MoveP Demo Node Starting...");
        
        // 创建发布器
        movep_pub_ = this->create_publisher<woan_interfaces::msg::MoveP>(
            "/woan_driver/movep_cmd", 10);
        
        // 创建订阅器（接收结果）
        result_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/woan_driver/movep_result", 10,
            std::bind(&MovePDemo::resultCallback, this, std::placeholders::_1));
        
        // 延迟2秒后执行
        timer_ = this->create_wall_timer(
            2s, std::bind(&MovePDemo::executeMoveP, this));
        
        RCLCPP_INFO(this->get_logger(), "MoveP Demo Node Initialized");
        RCLCPP_INFO(this->get_logger(), "Will send MoveP command in 2 seconds...");
    }
    
private:
    void executeMoveP() {
        // 停止定时器（只执行一次）
        timer_->cancel();
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Sending MoveP command...");
        
        // 构造MoveP消息
        auto msg = woan_interfaces::msg::MoveP();
        
        // 设置目标位姿
        //目标位姿x,y,z
        msg.pose.position.x = -0.051162;  
        msg.pose.position.y = -0.323413;    
        msg.pose.position.z = 0.446415;   
        //四元数w,x,y,z
        msg.pose.orientation.w = -0.344476;
        msg.pose.orientation.x = 0.222207;
        msg.pose.orientation.y = -0.476194;
        msg.pose.orientation.z = 0.777946;
        
        // 设置参数
        msg.trajectory_connect = 0;   // 立即执行（不缓冲）
        msg.speed_scale = 0.5;        // 速度缩放50%，执行速度降低（更平稳、更安全）
        
        // 打印目标位姿
        RCLCPP_INFO(this->get_logger(), "Target pose:");
        RCLCPP_INFO(this->get_logger(), "  Position: [%.4f, %.4f, %.4f] m",
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Orientation (quaternion): [%.4f, %.4f, %.4f, %.4f]",
                    msg.pose.orientation.w, msg.pose.orientation.x,
                    msg.pose.orientation.y, msg.pose.orientation.z);
        
        // 发布命令
        movep_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "MoveP command sent successfully");
        RCLCPP_INFO(this->get_logger(), "Waiting for result...");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }
    
    void resultCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "========================================");
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "✓ MoveP executed successfully!");
        } else {
            RCLCPP_WARN(this->get_logger(), "✗ MoveP execution failed");
        }
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        // 延迟后关闭节点
        auto shutdown_timer = this->create_wall_timer(
            1s, []() { rclcpp::shutdown(); });
    }
    
    rclcpp::Publisher<woan_interfaces::msg::MoveP>::SharedPtr movep_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr result_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MovePDemo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

