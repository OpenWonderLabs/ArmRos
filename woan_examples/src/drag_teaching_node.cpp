#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "woan_interfaces/msg/move_j.hpp"
#include "woan_api/drag_teaching.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <atomic>
#include <string>
#include <cstdlib>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

class DragTeachingNode : public rclcpp::Node {
public:
    DragTeachingNode() : Node("drag_teaching_node") {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  拖动示教节点启动中");
        RCLCPP_INFO(this->get_logger(), "========================================");

        // 尝试从 woan_driver_node 读取参数，如果读取不到则使用默认值
        // 这样可以自动适配不同的机械臂型号，无需修改启动方式
        int dof = 7;
        std::string device = "/dev/ttyACM0";
        std::string robot_model = "x1_l";
        std::string urdf_path = "";

        // 尝试从 woan_driver_node 读取参数
        try {
            auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "woan_driver_node");
            
            // 等待参数服务可用
            if (param_client->wait_for_service(std::chrono::seconds(2))) {
                // 读取参数
                std::vector<std::string> param_names = {"robot_model", "dof", "device"};
                auto params = param_client->get_parameters(param_names);
                
                for (const auto& param : params) {
                    if (param.get_name() == "robot_model") {
                        robot_model = param.as_string();
                        //RCLCPP_INFO(this->get_logger(), "从 woan_driver_node 读取 robot_model: %s", robot_model.c_str());
                    } else if (param.get_name() == "dof") {
                        dof = param.as_int();
                        //RCLCPP_INFO(this->get_logger(), "从 woan_driver_node 读取 dof: %d", dof);
                    } else if (param.get_name() == "device") {
                        device = param.as_string();
                        //RCLCPP_INFO(this->get_logger(), "从 woan_driver_node 读取 device: %s", device.c_str());
                    }
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "woan_driver_node 参数服务不可用，使用默认值");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "无法从 woan_driver_node 读取参数，使用默认值: %s", e.what());
        }

        // 声明本地参数（允许通过命令行覆盖）
        dof = this->declare_parameter<int>("dof", dof);
        device = this->declare_parameter<std::string>("device", device);
        robot_model = this->declare_parameter<std::string>("robot_model", robot_model);
        urdf_path = this->declare_parameter<std::string>("urdf_path", urdf_path);

        // 设置轨迹日志目录（使用相对路径）
        trajectory_log_dir_ = std::filesystem::path("./trajectory_log");
        // 确保目录存在
        if (!std::filesystem::exists(trajectory_log_dir_)) {
            std::filesystem::create_directories(trajectory_log_dir_);
            RCLCPP_INFO(this->get_logger(), "✓ 创建轨迹日志目录: %s", trajectory_log_dir_.string().c_str());
        }

        // 生成记录文件路径（带时间戳，使用相对路径）
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        std::string timestamp = ss.str();
        std::string record_file = trajectory_log_dir_.string() + "/drag_record_" + timestamp + ".dat";

        // 初始化 DragTeaching
        drag_teaching_ = std::make_unique<woan_api::DragTeaching>();
        if (!drag_teaching_->initialize(dof, record_file, 0.01)) {
            RCLCPP_ERROR(this->get_logger(), "✗ 初始化拖动示教失败！");
            return;
        }

        // 设置硬件资源（用于零力拖动控制）
        if (!drag_teaching_->set_hardware(device, urdf_path, robot_model)) {
            RCLCPP_ERROR(this->get_logger(), "✗ 硬件初始化失败！");
            RCLCPP_ERROR(this->get_logger(), "  设备: %s", device.c_str());
            RCLCPP_ERROR(this->get_logger(), "  机器人型号: %s", robot_model.c_str());
            RCLCPP_ERROR(this->get_logger(), "  程序将退出");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✓ 硬件资源初始化成功");
        RCLCPP_INFO(this->get_logger(), "  设备: %s", device.c_str());
        RCLCPP_INFO(this->get_logger(), "  机器人型号: %s", robot_model.c_str());

        // 订阅关节状态（从 woan_driver 发布）
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(10),
            std::bind(&DragTeachingNode::joint_state_callback, this, std::placeholders::_1));

        // 订阅控制命令话题
        cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/drag_teaching_cmd", rclcpp::QoS(10),
            std::bind(&DragTeachingNode::cmd_callback, this, std::placeholders::_1));

        // 发布 MoveJ 命令（用于回放）
        movej_pub_ = this->create_publisher<woan_interfaces::msg::MoveJ>(
            "/woan_driver/movej_cmd", 10);

        // 创建定时器
        auto timer_period = std::chrono::duration<double>(0.01);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration_cast<std::chrono::microseconds>(timer_period)),
            std::bind(&DragTeachingNode::timer_callback, this));

        // 显示简洁的欢迎信息
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  拖动示教节点就绪");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  本节点提供机械臂拖动示教功能：");
        RCLCPP_INFO(this->get_logger(), "  1. 零力拖动记录：手动拖动机械臂，自动记录轨迹");
        RCLCPP_INFO(this->get_logger(), "  2. 轨迹回放：回放当前记录的轨迹");
        RCLCPP_INFO(this->get_logger(), "  3. 选择轨迹回放：从历史记录中选择并回放轨迹");
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        print_command_prompt();
        
        // 启动终端输入线程
        input_thread_running_ = true;
        input_thread_ = std::thread(&DragTeachingNode::input_thread_func, this);
    }

    ~DragTeachingNode() {
        input_thread_running_ = false;
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
                last_state_ == woan_api::DragTeaching::State::REPLAYING &&
                current_state == woan_api::DragTeaching::State::IDLE) {
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
                RCLCPP_WARN(this->get_logger(), "✗ 无效输入: %s (请输入 0、1、2、3、4 或 5)", line.c_str());
                if (rclcpp::ok()) {
                    print_command_prompt();
                }
            }
        }
    }

    // 执行命令（从cmd_callback中提取的公共逻辑）
    void execute_command(int cmd) {
        if (cmd == 5) {
            RCLCPP_INFO(this->get_logger(), "✓ 退出程序");
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
                    RCLCPP_INFO(this->get_logger(), "✓ 命令: 停止");
                    RCLCPP_INFO(this->get_logger(), "  所有操作已停止");
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                } else {
                    RCLCPP_WARN(this->get_logger(), "✗ 命令 0 (停止) 失败");
                }
                break;
                
            case 1:
                if (result == 0) {
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    RCLCPP_INFO(this->get_logger(), "✓ 命令: 开始记录");
                    RCLCPP_INFO(this->get_logger(), "  零力拖动模式已激活");
                    RCLCPP_INFO(this->get_logger(), "  现在可以手动拖动机械臂了");
                    RCLCPP_INFO(this->get_logger(), "  正在记录轨迹...");
                    RCLCPP_INFO(this->get_logger(), "  输入 '2' 停止记录");
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "✗ 命令 1 (开始记录) 失败");
                    RCLCPP_ERROR(this->get_logger(), "  请确保机器人处于空闲状态");
                }
                break;
                
            case 2:
                if (result == 0) {
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    RCLCPP_INFO(this->get_logger(), "✓ 命令: 停止记录");
                    RCLCPP_INFO(this->get_logger(), "  记录已停止");
                    RCLCPP_INFO(this->get_logger(), "  轨迹已保存到文件");
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "✗ 命令 2 (停止记录) 失败");
                    RCLCPP_ERROR(this->get_logger(), "  请先按 '1' 开始记录");
                }
                break;
                
            case 3:
                if (result == 0) {
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    RCLCPP_INFO(this->get_logger(), "✓ 命令: 开始回放");
                    RCLCPP_INFO(this->get_logger(), "  回放已开始");
                    RCLCPP_INFO(this->get_logger(), "  机器人将先恢复到零点位置");
                    RCLCPP_INFO(this->get_logger(), "  然后移动到第一个点并回放轨迹");
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    // 设置标志，等待回放完成后显示提示
                    waiting_for_replay_completion_ = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "✗ 命令 3 (开始回放) 失败");
                    RCLCPP_ERROR(this->get_logger(), "  请确保轨迹文件存在");
                    RCLCPP_ERROR(this->get_logger(), "  且机器人处于空闲状态");
                }
                break;
                
            default:
                RCLCPP_WARN(this->get_logger(), "✗ 未知命令: %d", cmd);
                RCLCPP_WARN(this->get_logger(), "  有效命令: 0=停止, 1=开始记录, 2=停止记录, 3=回放当前轨迹, 4=选择历史轨迹, 5=退出");
                break;
        }
        
        // 如果不在等待回放完成，立即显示提示
        if (rclcpp::ok() && !waiting_for_replay_completion_) {
            print_command_prompt();
        }
    }

    void print_command_prompt() {
        RCLCPP_INFO(this->get_logger(), ">>> 请输入命令:");
        RCLCPP_INFO(this->get_logger(), "    0 - 停止所有操作");
        RCLCPP_INFO(this->get_logger(), "    1 - 开始零力拖动记录");
        RCLCPP_INFO(this->get_logger(), "    2 - 停止记录");
        RCLCPP_INFO(this->get_logger(), "    3 - 回放当前轨迹");
        RCLCPP_INFO(this->get_logger(), "    4 - 选择历史轨迹");
        RCLCPP_INFO(this->get_logger(), "    5 - 退出程序");
        RCLCPP_INFO(this->get_logger(), ">>> ");
    }

    // 功能4：选择并回放历史轨迹
    void execute_select_and_replay() {
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "  选择历史轨迹文件");
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
            RCLCPP_WARN(this->get_logger(), "  未找到轨迹文件");
            RCLCPP_WARN(this->get_logger(), "  目录: %s", trajectory_log_dir_.string().c_str());
            RCLCPP_WARN(this->get_logger(), "  请先使用功能1记录轨迹");
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            if (rclcpp::ok()) {
                print_command_prompt();
            }
            return;
        }
        
        // 显示文件列表
        RCLCPP_INFO(this->get_logger(), "  找到 %zu 个轨迹文件:", trajectory_files.size());
        for (size_t i = 0; i < trajectory_files.size(); i++) {
            auto file_size = std::filesystem::file_size(trajectory_files[i]);
            auto mod_time = std::filesystem::last_write_time(trajectory_files[i]);
            auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                mod_time - std::filesystem::file_time_type::clock::now() + std::chrono::system_clock::now());
            auto time_t = std::chrono::system_clock::to_time_t(sctp);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
            
            RCLCPP_INFO(this->get_logger(), "    [%zu] %s (%.2f KB, %s)", 
                       i + 1,
                       trajectory_files[i].filename().string().c_str(),
                       file_size / 1024.0,
                       ss.str().c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "  请输入文件编号 (1-%zu) 或 0 取消:", trajectory_files.size());
        
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
            RCLCPP_INFO(this->get_logger(), "  已取消");
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            if (rclcpp::ok()) {
                print_command_prompt();
            }
            return;
        }
        
        try {
            size_t index = std::stoul(line);
            if (index < 1 || index > trajectory_files.size()) {
                RCLCPP_ERROR(this->get_logger(), "  无效的文件编号: %s", line.c_str());
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
                RCLCPP_INFO(this->get_logger(), "✓ 开始回放: %s", trajectory_files[index - 1].filename().string().c_str());
                RCLCPP_INFO(this->get_logger(), "  机器人将先恢复到零点位置");
                RCLCPP_INFO(this->get_logger(), "  然后移动到第一个点并回放轨迹");
                RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                // 设置标志，等待回放完成后显示提示
                waiting_for_replay_completion_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ 回放失败");
                RCLCPP_ERROR(this->get_logger(), "  请确保机器人处于空闲状态");
                RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                if (rclcpp::ok()) {
                    print_command_prompt();
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "  无效输入: %s", line.c_str());
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            if (rclcpp::ok()) {
                print_command_prompt();
            }
        }
    }

    std::unique_ptr<woan_api::DragTeaching> drag_teaching_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_sub_;
    rclcpp::Publisher<woan_interfaces::msg::MoveJ>::SharedPtr movej_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 终端输入线程
    std::thread input_thread_;
    std::atomic<bool> input_thread_running_{false};
    
    // 回放状态跟踪
    bool waiting_for_replay_completion_{false};
    woan_api::DragTeaching::State last_state_{woan_api::DragTeaching::State::IDLE};
    
    // 轨迹日志目录
    std::filesystem::path trajectory_log_dir_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DragTeachingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

