#include "rclcpp/rclcpp.hpp"
#include "woan_interfaces/msg/move_l.hpp"
#include "std_msgs/msg/bool.hpp"
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

        movel_pub_ = this->create_publisher<woan_interfaces::msg::MoveL>(
            "/woan_driver/movel_cmd", 10);

        result_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/woan_driver/movel_result", 10,
            std::bind(&MoveLDemo::resultCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            2s, std::bind(&MoveLDemo::executeMoveL, this));

        RCLCPP_INFO(this->get_logger(), "MoveL Demo Node Initialized");
        RCLCPP_INFO(this->get_logger(), "Will send MoveL command in 2 seconds...");
    }

private:
    void executeMoveL() {
        timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Sending MoveL command...");

        auto msg = woan_interfaces::msg::MoveL();

        msg.pose.position.x = 0.3;
        msg.pose.position.y = 0.3;
        msg.pose.position.z = 0.4;

        msg.pose.orientation.w = 1.0;
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;

        msg.speed_scale = 0.8;
        msg.trajectory_connect = 0;

        RCLCPP_INFO(this->get_logger(), "Target pose:");
        RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f] m",
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Orientation (quaternion): [%.3f, %.3f, %.3f, %.3f]",
                    msg.pose.orientation.w, msg.pose.orientation.x,
                    msg.pose.orientation.y, msg.pose.orientation.z);
        RCLCPP_INFO(this->get_logger(), "Speed Scale: %.3f", msg.speed_scale);

        movel_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "MoveL command sent successfully");
        RCLCPP_INFO(this->get_logger(), "Waiting for result...");
    }

    void resultCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "========================================");
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "✓ MoveL executed successfully!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "✗ MoveL execution failed");
        }
        RCLCPP_INFO(this->get_logger(), "========================================");

        auto shutdown_timer = this->create_wall_timer(
            1s, []() { rclcpp::shutdown(); });
    }

    rclcpp::Publisher<woan_interfaces::msg::MoveL>::SharedPtr movel_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr result_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveLDemo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

