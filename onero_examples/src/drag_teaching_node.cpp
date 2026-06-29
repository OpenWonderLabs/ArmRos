// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "onero_interfaces/msg/move_j.hpp"
#include "onero_interface_cpp.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <atomic>
#include <csignal>
#include <memory>
#include <string>
#include <cstdlib>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

namespace {
// SIGINT/SIGTERM handler 只置 atomic 标志，由 ROS executor 上的 wall timer 检测后
// 在普通线程上下文里走 graceful_shutdown（latch + stop_recording）。
// 不在 signal handler 里直接调串口/SDK，避免 signal-async 不安全。
std::atomic<bool> g_drag_shutdown_requested{false};
}  // namespace

extern "C" void dragTeachingSignalHandler(int) {
    g_drag_shutdown_requested.store(true);
}

/**
 * @brief 单臂拖动示教节点
 *
 * 本示例直接使用 SDK 控制机械臂；不要同时启动 onero_driver 控制同一设备。
 *
 * 单臂运行：
 *   ros2 launch onero_driver a1_l_driver.launch.py
 *   ros2 run onero_examples drag_teaching_node
 * 注意：实体机运行前请确认机械臂采用单臂安装方式；双臂安装方式需先重新评估拖动轨迹，避免与桌面干涉。
 *
 * 通过 /drag_teaching_cmd 控制开始、停止和回放拖动示教轨迹。
 */
class DragTeachingNode : public rclcpp::Node {
public:
    DragTeachingNode() : Node("drag_teaching_node") {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  Drag Teaching Node Starting");
        RCLCPP_INFO(this->get_logger(), "========================================");

        // 尝试从 onero_driver_node 读取参数，如果读取不到则使用默认值
        // 这样可以自动适配不同的机械臂型号，无需修改启动方式
        int dof = 7;
        std::string device = "/dev/ttyACM0";
        std::string robot_model = "a1_l";
        std::string urdf_path = "";
        std::string mount_orientation = "vertical";
        bool with_gripper = false;

        // 尝试从 onero_driver_node 读取参数
        try {
            auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "onero_driver_node");

            // 等待参数服务可用
            if (param_client->wait_for_service(std::chrono::seconds(2))) {
                // 读取参数
                std::vector<std::string> param_names = {
                    "robot_model", "dof", "device", "mount_orientation", "gripper.enabled"
                };
                auto params = param_client->get_parameters(param_names);

                for (const auto& param : params) {
                    if (param.get_name() == "robot_model") {
                        robot_model = param.as_string();
                    } else if (param.get_name() == "dof") {
                        dof = param.as_int();
                    } else if (param.get_name() == "device") {
                        device = param.as_string();
                    } else if (param.get_name() == "mount_orientation") {
                        mount_orientation = param.as_string();
                    } else if (param.get_name() == "gripper.enabled" &&
                               param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                        with_gripper = param.as_bool();
                    }
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "onero_driver_node parameter service unavailable, using defaults");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to read parameters from onero_driver_node, using defaults: %s", e.what());
        }

        // 声明本地参数（允许通过命令行覆盖）
        dof = this->declare_parameter<int>("dof", dof);
        device = this->declare_parameter<std::string>("device", device);
        robot_model = this->declare_parameter<std::string>("robot_model", robot_model);
        urdf_path = this->declare_parameter<std::string>("urdf_path", urdf_path);
        mount_orientation = this->declare_parameter<std::string>("mount_orientation", mount_orientation);
        with_gripper = this->declare_parameter<bool>("gripper", with_gripper);

        // 设置轨迹日志目录（使用相对路径）
        trajectory_log_dir_ = std::filesystem::path("./trajectory_log");
        // 确保目录存在
        if (!std::filesystem::exists(trajectory_log_dir_)) {
            std::filesystem::create_directories(trajectory_log_dir_);
            RCLCPP_INFO(this->get_logger(), "✓ Created trajectory log directory: %s", trajectory_log_dir_.string().c_str());
        }

        // 生成记录文件路径（带时间戳，使用相对路径）
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        std::string timestamp = ss.str();
        std::string record_file = trajectory_log_dir_.string() + "/drag_record_" + timestamp + ".dat";

        // 初始化 DragTeaching
        drag_teaching_ = std::make_unique<onero_api::OneroDragTeaching>();
        if (!drag_teaching_->valid() ||
            !drag_teaching_->initialize(dof, record_file, 0.01)) {
            RCLCPP_ERROR(this->get_logger(), "✗ Failed to initialize drag teaching!");
            drag_teaching_.reset();
            return;
        }

        // 设置硬件资源（用于零力拖动控制）
        if (!drag_teaching_->set_hardware(device, urdf_path, robot_model,
                                          mount_orientation, with_gripper)) {
            RCLCPP_ERROR(this->get_logger(), "✗ Hardware initialization failed!");
            RCLCPP_ERROR(this->get_logger(), "  Device: %s", device.c_str());
            RCLCPP_ERROR(this->get_logger(), "  Robot model: %s", robot_model.c_str());
            RCLCPP_ERROR(this->get_logger(), "  Mount orientation: %s", mount_orientation.c_str());
            RCLCPP_ERROR(this->get_logger(), "  With gripper: %s", with_gripper ? "true" : "false");
            RCLCPP_ERROR(this->get_logger(), "  Program will exit");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✓ Hardware resources initialized successfully");
        RCLCPP_INFO(this->get_logger(), "  Device: %s", device.c_str());
        RCLCPP_INFO(this->get_logger(), "  Robot model: %s", robot_model.c_str());
        RCLCPP_INFO(this->get_logger(), "  Mount orientation: %s", mount_orientation.c_str());
        RCLCPP_INFO(this->get_logger(), "  With gripper: %s", with_gripper ? "true" : "false");

        // 订阅关节状态（从 onero_driver 发布）
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(10),
            std::bind(&DragTeachingNode::joint_state_callback, this, std::placeholders::_1));

        // 订阅控制命令话题
        cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/drag_teaching_cmd", rclcpp::QoS(10),
            std::bind(&DragTeachingNode::cmd_callback, this, std::placeholders::_1));

        // 发布 MoveJ 命令（用于回放）
        movej_pub_ = this->create_publisher<onero_interfaces::msg::MoveJ>(
            "/onero_arm/movej", 10);

        // 创建定时器
        auto timer_period = std::chrono::duration<double>(0.01);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration_cast<std::chrono::microseconds>(timer_period)),
            std::bind(&DragTeachingNode::timer_callback, this));

        // ★ SIGINT 软中断检测 timer：每 200ms 检查一次 atomic 标志。
        //    检测到后在 executor 线程上下文（普通线程）调用 graceful_shutdown，
        //    不在 signal handler 里直接调串口/SDK。
        shutdown_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() {
                if (g_drag_shutdown_requested.load() && !shutdown_done_.exchange(true)) {
                    graceful_shutdown();
                    rclcpp::shutdown();
                }
            });

        // 显示简洁的欢迎信息
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  Drag Teaching Node Ready");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  This node provides robot arm drag teaching:");
        RCLCPP_INFO(this->get_logger(), "  1. Zero-force drag recording: manually drag the arm, auto-record trajectory");
        RCLCPP_INFO(this->get_logger(), "  2. Trajectory replay: replay currently recorded trajectory");
        RCLCPP_INFO(this->get_logger(), "  3. Select trajectory replay: choose and replay from history");
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        print_command_prompt();
        
        // 启动终端输入线程
        input_thread_running_ = true;
        input_thread_ = std::thread(&DragTeachingNode::input_thread_func, this);
    }

    ~DragTeachingNode() {
        input_thread_running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
        // unique_ptr 析构会调用 OneroDragTeaching::~OneroDragTeaching() = destroy_drag_teaching
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // 更新 DragTeaching 的关节状态缓存
        if (drag_teaching_ && drag_teaching_->is_initialized()) {
            drag_teaching_->update_joint_state(msg->position, msg->velocity, msg->effort);
        }
    }
    void cmd_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        // 从ROS2话题接收的命令也使用相同的处理逻辑
        execute_command(msg->data);
    }

    void timer_callback() {
        if (drag_teaching_ && drag_teaching_->is_initialized()) {
            drag_teaching_->timer_callback();

            // 检查回放状态变化，如果从REPLAYING变为IDLE，显示提示
            auto current_state = drag_teaching_->get_state();
            if (waiting_for_replay_completion_ &&
                last_state_ == onero_api::DragTeachingState::REPLAYING &&
                current_state == onero_api::DragTeachingState::IDLE) {
                waiting_for_replay_completion_ = false;
                if (rclcpp::ok()) {
                    print_command_prompt();
                }
            }
            last_state_ = current_state;
        }
    }

    // 终端输入处理线程
    void input_thread_func() {
        std::string line;
        while (input_thread_running_ && rclcpp::ok()) {
            std::getline(std::cin, line);
            if (!input_thread_running_ || !rclcpp::ok()) {
                break;
            }
            
            // 去除首尾空白字符
            size_t first = line.find_first_not_of(" \t\n\r");
            if (first != std::string::npos) {
                line.erase(0, first);
            }
            size_t last = line.find_last_not_of(" \t\n\r");
            if (last != std::string::npos) {
                line.erase(last + 1);
            }
            
            if (line.empty()) {
                continue;
            }
            
            // 尝试解析为数字
            try {
                int cmd = std::stoi(line);
                execute_command(cmd);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "✗ Invalid input: %s (please enter 0, 1, 2, 3, 4, or 5)", line.c_str());
                if (rclcpp::ok()) {
                    print_command_prompt();
                }
            }
        }
    }

    // 执行命令（从cmd_callback中提取的公共逻辑）
    void execute_command(int cmd) {
        if (cmd == 5) {
            RCLCPP_INFO(this->get_logger(), "✓ Exiting program");
            rclcpp::shutdown();
            std::exit(0);
        }
        
        // 功能4：选择并回放历史轨迹
        if (cmd == 4) {
            execute_select_and_replay();
            return;
        }
        
        int result = drag_teaching_->handle_command(cmd);
        
        // 根据命令类型显示不同的信息
        switch (cmd) {
            case 0:
                if (result == 0) {
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    RCLCPP_INFO(this->get_logger(), "✓ Command: Stop");
                    RCLCPP_INFO(this->get_logger(), "  All operations stopped");
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                } else {
                    RCLCPP_WARN(this->get_logger(), "✗ Command 0 (Stop) failed");
                }
                break;
                
            case 1:
                if (result == 0) {
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    RCLCPP_INFO(this->get_logger(), "✓ Command: Start Recording");
                    RCLCPP_INFO(this->get_logger(), "  Zero-force drag mode activated");
                    RCLCPP_INFO(this->get_logger(), "  You can now manually drag the robot arm");
                    RCLCPP_INFO(this->get_logger(), "  Recording trajectory...");
                    RCLCPP_INFO(this->get_logger(), "  Enter '2' to stop recording");
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "✗ Command 1 (Start Recording) failed");
                    RCLCPP_ERROR(this->get_logger(), "  Please ensure the robot is in IDLE state");
                }
                break;
                
            case 2:
                if (result == 0) {
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    RCLCPP_INFO(this->get_logger(), "✓ Command: Stop Recording");
                    RCLCPP_INFO(this->get_logger(), "  Recording stopped");
                    RCLCPP_INFO(this->get_logger(), "  Trajectory saved to file");
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "✗ Command 2 (Stop Recording) failed");
                    RCLCPP_ERROR(this->get_logger(), "  Please press '1' to start recording first");
                }
                break;
                
            case 3:
                if (result == 0) {
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    RCLCPP_INFO(this->get_logger(), "✓ Command: Start Replay");
                    RCLCPP_INFO(this->get_logger(), "  Replay started");
                    RCLCPP_INFO(this->get_logger(), "  Robot will first return to zero position");
                    RCLCPP_INFO(this->get_logger(), "  Then move to the first point and replay trajectory");
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    // 设置标志，等待回放完成后显示提示
                    waiting_for_replay_completion_ = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "✗ Command 3 (Start Replay) failed");
                    RCLCPP_ERROR(this->get_logger(), "  Please ensure the trajectory file exists");
                    RCLCPP_ERROR(this->get_logger(), "  And the robot is in IDLE state");
                }
                break;
                
            default:
                RCLCPP_WARN(this->get_logger(), "✗ Unknown command: %d", cmd);
                RCLCPP_WARN(this->get_logger(), "  Valid commands: 0=Stop, 1=Start Recording, 2=Stop Recording, 3=Replay Current, 4=Select History, 5=Exit");
                break;
        }
        
        // 如果不在等待回放完成，立即显示提示
        if (rclcpp::ok() && !waiting_for_replay_completion_) {
            print_command_prompt();
        }
    }

    void print_command_prompt() {
        RCLCPP_INFO(this->get_logger(), ">>> Enter command:");
        RCLCPP_INFO(this->get_logger(), "    0 - Stop all operations");
        RCLCPP_INFO(this->get_logger(), "    1 - Start zero-force drag recording");
        RCLCPP_INFO(this->get_logger(), "    2 - Stop recording");
        RCLCPP_INFO(this->get_logger(), "    3 - Replay current trajectory");
        RCLCPP_INFO(this->get_logger(), "    4 - Select history trajectory");
        RCLCPP_INFO(this->get_logger(), "    5 - Exit program");
        RCLCPP_INFO(this->get_logger(), ">>> ");
    }

    // 功能4：选择并回放历史轨迹
    void execute_select_and_replay() {
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "  Select History Trajectory File");
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        
        // 列出目录下的所有txt文件
        std::vector<std::filesystem::path> trajectory_files;
        if (std::filesystem::exists(trajectory_log_dir_)) {
            for (const auto& entry : std::filesystem::directory_iterator(trajectory_log_dir_)) {
                if (entry.is_regular_file() && entry.path().extension() == ".dat") {
                    trajectory_files.push_back(entry.path());
                }
            }
        }
        
        // 按文件名排序（最新的在前）
        std::sort(trajectory_files.begin(), trajectory_files.end(), 
                  [](const std::filesystem::path& a, const std::filesystem::path& b) {
                      return a.filename().string() > b.filename().string();
                  });
        
        if (trajectory_files.empty()) {
            RCLCPP_WARN(this->get_logger(), "  No trajectory files found");
            RCLCPP_WARN(this->get_logger(), "  Directory: %s", trajectory_log_dir_.string().c_str());
            RCLCPP_WARN(this->get_logger(), "  Please use feature 1 to record trajectory first");
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            if (rclcpp::ok()) {
                print_command_prompt();
            }
            return;
        }
        
        // 显示文件列表
        RCLCPP_INFO(this->get_logger(), "  Found %zu trajectory file(s):", trajectory_files.size());
        for (size_t i = 0; i < trajectory_files.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "    [%zu] %s",
                       i + 1,
                       trajectory_files[i].filename().string().c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "  Enter file number (1-%zu) or 0 to cancel:", trajectory_files.size());
        
        // 读取用户输入
        std::string line;
        std::getline(std::cin, line);
        
        // 去除首尾空白字符
        size_t first = line.find_first_not_of(" \t\n\r");
        if (first != std::string::npos) {
            line.erase(0, first);
        }
        size_t last = line.find_last_not_of(" \t\n\r");
        if (last != std::string::npos) {
            line.erase(last + 1);
        }
        
        if (line.empty() || line == "0") {
            RCLCPP_INFO(this->get_logger(), "  Cancelled");
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            if (rclcpp::ok()) {
                print_command_prompt();
            }
            return;
        }
        
        try {
            size_t index = std::stoul(line);
            if (index < 1 || index > trajectory_files.size()) {
                RCLCPP_ERROR(this->get_logger(), "  Invalid file number: %s", line.c_str());
                RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                if (rclcpp::ok()) {
                    print_command_prompt();
                }
                return;
            }
            
            // 设置回放文件并开始回放
            std::string selected_file = trajectory_files[index - 1].string();
            drag_teaching_->set_replay_file(selected_file);

            int result = drag_teaching_->handle_command(3);  // 命令3：开始回放
            if (result == 0) {
                RCLCPP_INFO(this->get_logger(), "✓ Starting replay: %s", trajectory_files[index - 1].filename().string().c_str());
                RCLCPP_INFO(this->get_logger(), "  Robot will first move to the trajectory start point");
                RCLCPP_INFO(this->get_logger(), "  Then replay trajectory");
                RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                // 设置标志，等待回放完成后显示提示
                waiting_for_replay_completion_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ Replay failed");
                RCLCPP_ERROR(this->get_logger(), "  Please ensure the robot is in IDLE state");
                RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                if (rclcpp::ok()) {
                    print_command_prompt();
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "  Invalid input: %s", line.c_str());
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            if (rclcpp::ok()) {
                print_command_prompt();
            }
        }
    }

    std::unique_ptr<onero_api::OneroDragTeaching> drag_teaching_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_sub_;
    rclcpp::Publisher<onero_interfaces::msg::MoveJ>::SharedPtr movej_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 终端输入线程
    std::thread input_thread_;
    std::atomic<bool> input_thread_running_{false};
    
    // 回放状态跟踪
    bool waiting_for_replay_completion_{false};
    onero_api::DragTeachingState last_state_{onero_api::DragTeachingState::IDLE};

    // 轨迹日志目录
    std::filesystem::path trajectory_log_dir_;

    // SIGINT/SIGTERM 软中断 timer，及防重复调用 latch 的标志
    rclcpp::TimerBase::SharedPtr shutdown_check_timer_;
    std::atomic<bool> shutdown_done_{false};

public:
    // 由 shutdown_check_timer_ 在普通线程上下文里调用：让拖动模式停下并 latch 当前位置。
    void graceful_shutdown() {
        if (drag_teaching_ && drag_teaching_->is_initialized()) {
            // handle_command(0) 内部走 stop_recording / stop_replay 路径，
            // 拖动状态下会触发 latch_current_position_locked。
            drag_teaching_->handle_command(0);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // ★ 注册 SIGINT/SIGTERM 软中断，置 atomic 标志后由 ROS timer 处理 latch
    std::signal(SIGINT, dragTeachingSignalHandler);
    std::signal(SIGTERM, dragTeachingSignalHandler);
    auto node = std::make_shared<DragTeachingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
