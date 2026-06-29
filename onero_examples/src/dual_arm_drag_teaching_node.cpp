// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "onero_interface_cpp.h"
#include <chrono>
#include <thread>
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
#include <utility>
#include <fstream>
#include <optional>

using namespace std::chrono_literals;

namespace {
constexpr int kArmDof = 7;

bool ends_with(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() &&
           std::equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

// SIGINT/SIGTERM handler 只置 atomic 标志，由 wall timer 在 executor 线程上下文里 latch 双臂。
std::atomic<bool> g_dual_drag_shutdown_requested{false};
}  // namespace

extern "C" void dualDragTeachingSignalHandler(int) {
    g_dual_drag_shutdown_requested.store(true);
}

/**
 * @brief 双臂拖动示教节点
 *
 * 本示例直接使用 SDK 控制两臂；不要同时启动 onero_driver 控制同一设备。
 *
 * 双臂运行：
 *   ros2 launch onero_driver a1_dual_driver.launch.py
 *   ros2 run onero_examples dual_arm_drag_teaching_node
 *
 * 通过 /dual_drag_teaching_cmd 控制开始、停止和回放双臂拖动示教轨迹。
 */
class DualArmDragTeachingNode : public rclcpp::Node {
public:
    DualArmDragTeachingNode() : Node("dual_arm_drag_teaching_node") {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  Dual-Arm Drag Teaching Node Starting");
        RCLCPP_INFO(this->get_logger(), "========================================");

        // 参数默认值
        int dof_param = 14;
        std::string device_left = "/dev/ttyACM0";
        std::string device_right = "/dev/ttyACM1";
        std::string mount_orientation = "vertical";
        std::string driver_robot_model = "A1_dual";
        bool left_gripper = false;
        bool right_gripper = false;

        // 尝试从 onero_driver_node 读取参数
        try {
            auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "onero_driver_node");
            if (param_client->wait_for_service(std::chrono::seconds(2))) {
                std::vector<std::string> param_names = {
                    "robot_model", "dof", "device_left", "device_right", "mount_orientation",
                    "gripper.left_enabled", "gripper.right_enabled"
                };
                auto params = param_client->get_parameters(param_names);
                for (const auto& param : params) {
                    if (param.get_name() == "robot_model") {
                        driver_robot_model = param.as_string();
                    } else if (param.get_name() == "dof") {
                        dof_param = param.as_int();
                    } else if (param.get_name() == "device_left") {
                        device_left = param.as_string();
                    } else if (param.get_name() == "device_right") {
                        device_right = param.as_string();
                    } else if (param.get_name() == "mount_orientation") {
                        mount_orientation = param.as_string();
                    } else if (param.get_name() == "gripper.left_enabled" &&
                               param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                        left_gripper = param.as_bool();
                    } else if (param.get_name() == "gripper.right_enabled" &&
                               param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                        right_gripper = param.as_bool();
                    }
                }
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "onero_driver_node parameter service unavailable, using defaults");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to read parameters from onero_driver_node, using defaults: %s",
                        e.what());
        }

        // 允许命令行覆盖
        device_left = this->declare_parameter<std::string>("device_left", device_left);
        device_right = this->declare_parameter<std::string>("device_right", device_right);
        mount_orientation = this->declare_parameter<std::string>("mount_orientation", mount_orientation);
        left_gripper = this->declare_parameter<bool>("left_gripper", left_gripper);
        right_gripper = this->declare_parameter<bool>("right_gripper", right_gripper);
        std::string urdf_path_left = this->declare_parameter<std::string>("urdf_path_left", "");
        std::string urdf_path_right = this->declare_parameter<std::string>("urdf_path_right", "");

        if (dof_param != 14) {
            RCLCPP_WARN(this->get_logger(),
                        "Driver dof=%d (expected 14 for dual-arm). Continuing with 7 dof per arm.",
                        dof_param);
        }
        RCLCPP_INFO(this->get_logger(), "  Driver robot_model: %s", driver_robot_model.c_str());
        RCLCPP_INFO(this->get_logger(), "  Mount orientation : %s", mount_orientation.c_str());
        RCLCPP_INFO(this->get_logger(), "  Left  device      : %s", device_left.c_str());
        RCLCPP_INFO(this->get_logger(), "  Right device      : %s", device_right.c_str());
        RCLCPP_INFO(this->get_logger(), "  Left  gripper     : %s", left_gripper ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  Right gripper     : %s", right_gripper ? "true" : "false");

        // 准备日志目录：双臂记录放在子目录，避免与单臂历史混在一起。
        trajectory_log_dir_ = std::filesystem::path("./trajectory_log/dual_arm");
        if (!std::filesystem::exists(trajectory_log_dir_)) {
            std::filesystem::create_directories(trajectory_log_dir_);
            RCLCPP_INFO(this->get_logger(), "✓ Created trajectory log directory: %s",
                        trajectory_log_dir_.string().c_str());
        }

        // 同一时间戳，两臂成对生成记录文件
        assign_pair_files(make_timestamp());

        left_.side = "left";
        left_.robot_model = "a1_l";
        left_.device = device_left;
        left_.urdf_path = urdf_path_left;
        left_.with_gripper = left_gripper;
        left_.suffix = "-a1_l";

        right_.side = "right";
        right_.robot_model = "a1_r";
        right_.device = device_right;
        right_.urdf_path = urdf_path_right;
        right_.with_gripper = right_gripper;
        right_.suffix = "-a1_r";

        if (!init_arm(left_, mount_orientation) ||
            !init_arm(right_, mount_orientation)) {
            RCLCPP_ERROR(this->get_logger(), "✗ Dual-arm initialization failed, node will exit");
            return;
        }

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(10),
            std::bind(&DualArmDragTeachingNode::joint_state_callback, this, std::placeholders::_1));

        cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/dual_drag_teaching_cmd", rclcpp::QoS(10),
            std::bind(&DualArmDragTeachingNode::cmd_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            10ms, std::bind(&DualArmDragTeachingNode::timer_callback, this));

        // ★ SIGINT 软中断检测 timer：每 200ms 检查 atomic 标志；
        //    检测到则在普通线程上下文调用 graceful_shutdown 让双臂 latch。
        shutdown_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() {
                if (g_dual_drag_shutdown_requested.load() && !shutdown_done_.exchange(true)) {
                    graceful_shutdown();
                    rclcpp::shutdown();
                }
            });

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  Dual-Arm Drag Teaching Node Ready");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  All commands operate on BOTH arms simultaneously:");
        RCLCPP_INFO(this->get_logger(), "  1. Zero-force drag recording: drag both arms together");
        RCLCPP_INFO(this->get_logger(), "  2. Trajectory replay: both arms replay synchronously");
        RCLCPP_INFO(this->get_logger(), "  3. History replay: pick a recorded pair and replay");
        RCLCPP_INFO(this->get_logger(), "========================================");

        print_command_prompt();

        input_thread_running_ = true;
        input_thread_ = std::thread(&DualArmDragTeachingNode::input_thread_func, this);
    }

    ~DualArmDragTeachingNode() {
        input_thread_running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    struct ArmCtx {
        std::string side;            // "left" / "right"
        std::string robot_model;     // "a1_l" / "a1_r"
        std::string device;
        std::string urdf_path;
        bool with_gripper{false};
        std::string suffix;          // "-a1_l" / "-a1_r"
        std::filesystem::path record_file;
        std::unique_ptr<onero_api::OneroDragTeaching> dt;
        onero_api::DragTeachingState last_state{onero_api::DragTeachingState::IDLE};
    };

    bool init_arm(ArmCtx& arm, const std::string& mount_orientation) {
        arm.dt = std::make_unique<onero_api::OneroDragTeaching>();
        if (!arm.dt->valid() ||
            !arm.dt->initialize(kArmDof, arm.record_file.string(), 0.01)) {
            RCLCPP_ERROR(this->get_logger(), "✗ Failed to initialize drag teaching for %s arm!",
                         arm.side.c_str());
            arm.dt.reset();
            return false;
        }

        if (!arm.dt->set_hardware(arm.device, arm.urdf_path, arm.robot_model,
                                  mount_orientation, arm.with_gripper)) {
            RCLCPP_ERROR(this->get_logger(),
                         "✗ Hardware initialization failed for %s arm (device=%s, model=%s, gripper=%s)",
                         arm.side.c_str(), arm.device.c_str(), arm.robot_model.c_str(),
                         arm.with_gripper ? "true" : "false");
            arm.dt.reset();
            return false;
        }
        RCLCPP_INFO(this->get_logger(),
                    "✓ %s arm hardware ready (device=%s, model=%s, gripper=%s, record=%s)",
                    arm.side.c_str(), arm.device.c_str(), arm.robot_model.c_str(),
                    arm.with_gripper ? "true" : "false",
                    arm.record_file.filename().string().c_str());
        return true;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // 按关节名后缀分流，避免对发布顺序做硬编码假设
        onero_api::JointArray lp, lv, le, rp, rv, re;
        lp.reserve(kArmDof); lv.reserve(kArmDof); le.reserve(kArmDof);
        rp.reserve(kArmDof); rv.reserve(kArmDof); re.reserve(kArmDof);

        const bool has_vel = msg->velocity.size() == msg->name.size();
        const bool has_eff = msg->effort.size() == msg->name.size();
        const bool has_pos = msg->position.size() == msg->name.size();
        if (!has_pos) {
            return;
        }

        for (size_t i = 0; i < msg->name.size(); ++i) {
            const auto& n = msg->name[i];
            if (ends_with(n, left_.suffix)) {
                lp.push_back(msg->position[i]);
                lv.push_back(has_vel ? msg->velocity[i] : 0.0);
                le.push_back(has_eff ? msg->effort[i] : 0.0);
            } else if (ends_with(n, right_.suffix)) {
                rp.push_back(msg->position[i]);
                rv.push_back(has_vel ? msg->velocity[i] : 0.0);
                re.push_back(has_eff ? msg->effort[i] : 0.0);
            }
        }

        if (left_.dt && left_.dt->is_initialized() && lp.size() == kArmDof) {
            left_.dt->update_joint_state(lp, lv, le);
        }
        if (right_.dt && right_.dt->is_initialized() && rp.size() == kArmDof) {
            right_.dt->update_joint_state(rp, rv, re);
        }
    }

    void cmd_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        execute_command(msg->data);
    }

    void timer_callback() {
        bool any_initialized = false;
        for (ArmCtx* arm : {&left_, &right_}) {
            if (arm->dt && arm->dt->is_initialized()) {
                arm->dt->timer_callback();
                any_initialized = true;
            }
        }
        if (!any_initialized) {
            return;
        }

        const auto ls = left_.dt->get_state();
        const auto rs = right_.dt->get_state();
        if (waiting_for_replay_completion_ &&
            (left_.last_state == onero_api::DragTeachingState::REPLAYING ||
             right_.last_state == onero_api::DragTeachingState::REPLAYING) &&
            ls == onero_api::DragTeachingState::IDLE &&
            rs == onero_api::DragTeachingState::IDLE) {
            waiting_for_replay_completion_ = false;
            if (rclcpp::ok()) {
                print_command_prompt();
            }
        }
        left_.last_state = ls;
        right_.last_state = rs;
    }

    void input_thread_func() {
        std::string line;
        while (input_thread_running_ && rclcpp::ok()) {
            std::getline(std::cin, line);
            if (!input_thread_running_ || !rclcpp::ok()) {
                break;
            }
            trim(line);
            if (line.empty()) {
                continue;
            }
            try {
                int cmd = std::stoi(line);
                execute_command(cmd);
            } catch (const std::exception&) {
                RCLCPP_WARN(this->get_logger(),
                            "✗ Invalid input: %s (please enter 0, 1, 2, 3, 4, or 5)",
                            line.c_str());
                if (rclcpp::ok()) {
                    print_command_prompt();
                }
            }
        }
    }

    void execute_command(int cmd) {
        if (cmd == 5) {
            RCLCPP_INFO(this->get_logger(), "✓ Exiting program");
            rclcpp::shutdown();
            std::exit(0);
        }

        if (cmd == 4) {
            execute_select_and_replay();
            return;
        }

        // 0/1/2/3 通过 SDK dual API 一次性下发到两臂。cmd=3 走"barrier 同步 move +
        // 共享 t0 begin"的三阶段流程,避免左臂先 tick / 时间基准错开。
        const int rc = (left_.dt && right_.dt)
            ? onero_api::OneroDragTeaching::handle_command_dual(*left_.dt, *right_.dt, cmd)
            : -1;
        const bool ok = (rc == 0);

        switch (cmd) {
            case 0:
                if (ok) {
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    RCLCPP_INFO(this->get_logger(), "✓ Command: Stop (both arms)");
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "✗ Command 0 (Stop) failed (rc=%d)", rc);
                }
                break;

            case 1:
                if (ok) {
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    RCLCPP_INFO(this->get_logger(), "✓ Command: Start Recording (both arms)");
                    RCLCPP_INFO(this->get_logger(), "  Zero-force drag mode activated on both arms");
                    RCLCPP_INFO(this->get_logger(), "  Drag both arms manually to teach a coordinated motion");
                    RCLCPP_INFO(this->get_logger(), "  Enter '2' to stop recording");
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                } else {
                    RCLCPP_ERROR(this->get_logger(),
                                 "✗ Command 1 (Start Recording) failed (rc=%d)", rc);
                    RCLCPP_ERROR(this->get_logger(), "  Both arms must be IDLE to start recording");
                }
                break;

            case 2:
                if (ok) {
                    const bool manifest_ok = write_dual_manifest();
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    RCLCPP_INFO(this->get_logger(), "✓ Command: Stop Recording (both arms)");
                    RCLCPP_INFO(this->get_logger(), "  Trajectory pair saved: %s",
                                current_pair_timestamp_.c_str());
                    if (manifest_ok) {
                        RCLCPP_INFO(this->get_logger(), "  Dual manifest: %s",
                                    current_manifest_file_.filename().string().c_str());
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                                    "  Dual manifest was not written; data files are still saved");
                    }
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                } else {
                    RCLCPP_ERROR(this->get_logger(),
                                 "✗ Command 2 (Stop Recording) failed (rc=%d)", rc);
                    RCLCPP_ERROR(this->get_logger(), "  Press '1' to start recording first");
                }
                break;

            case 3:
                if (ok) {
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    RCLCPP_INFO(this->get_logger(), "✓ Command: Start Replay (both arms)");
                    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
                    waiting_for_replay_completion_ = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(),
                                 "✗ Command 3 (Start Replay) failed (rc=%d)", rc);
                    RCLCPP_ERROR(this->get_logger(), "  Ensure both trajectory files exist and arms are IDLE");
                }
                break;

            default:
                RCLCPP_WARN(this->get_logger(), "✗ Unknown command: %d", cmd);
                RCLCPP_WARN(this->get_logger(),
                            "  Valid commands: 0=Stop, 1=Start Recording, 2=Stop Recording, "
                            "3=Replay Current, 4=Select History, 5=Exit");
                break;
        }

        if (rclcpp::ok() && !waiting_for_replay_completion_) {
            print_command_prompt();
        }
    }

    void print_command_prompt() {
        RCLCPP_INFO(this->get_logger(), ">>> Enter command (operates on BOTH arms):");
        RCLCPP_INFO(this->get_logger(), "    0 - Stop all operations");
        RCLCPP_INFO(this->get_logger(), "    1 - Start zero-force drag recording");
        RCLCPP_INFO(this->get_logger(), "    2 - Stop recording");
        RCLCPP_INFO(this->get_logger(), "    3 - Replay current trajectory pair");
        RCLCPP_INFO(this->get_logger(), "    4 - Select history trajectory pair");
        RCLCPP_INFO(this->get_logger(), "    5 - Exit program");
        RCLCPP_INFO(this->get_logger(), ">>> ");
    }

    static std::string make_timestamp() {
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
        return ss.str();
    }

    std::filesystem::path dual_manifest_path(const std::string& timestamp) const {
        return trajectory_log_dir_ / ("drag_record_dual_" + timestamp + "_index.dat");
    }

    std::filesystem::path dual_data_path(const std::string& timestamp,
                                         const std::string& side) const {
        return trajectory_log_dir_ / ("drag_record_dual_" + timestamp + "_" + side + ".dat");
    }

    void assign_pair_files(const std::string& timestamp) {
        current_pair_timestamp_ = timestamp;
        left_.record_file = dual_data_path(timestamp, "left");
        right_.record_file = dual_data_path(timestamp, "right");
        current_manifest_file_ = dual_manifest_path(timestamp);
    }

    bool write_dual_manifest() const {
        if (current_pair_timestamp_.empty() ||
            !std::filesystem::exists(left_.record_file) ||
            !std::filesystem::exists(right_.record_file)) {
            return false;
        }

        std::ofstream out(current_manifest_file_, std::ios::trunc);
        if (!out.is_open()) {
            return false;
        }

        out << "ONERO_DUAL_TRAJ 1\n";
        out << "timestamp=" << current_pair_timestamp_ << "\n";
        out << "left=" << left_.record_file.filename().string() << "\n";
        out << "right=" << right_.record_file.filename().string() << "\n";
        return out.good();
    }

    struct TrajectoryPair {
        std::string timestamp;
        std::filesystem::path left;
        std::filesystem::path right;
    };

    static bool read_key_value(const std::string& line,
                               const std::string& key,
                               std::string& value) {
        const std::string prefix = key + "=";
        if (line.rfind(prefix, 0) != 0) {
            return false;
        }
        value = line.substr(prefix.size());
        return true;
    }

    std::optional<TrajectoryPair> read_dual_manifest(
            const std::filesystem::path& manifest_path) const {
        std::ifstream in(manifest_path);
        if (!in.is_open()) {
            return std::nullopt;
        }

        std::string line;
        if (!std::getline(in, line) || line != "ONERO_DUAL_TRAJ 1") {
            return std::nullopt;
        }

        std::string timestamp;
        std::string left_file;
        std::string right_file;
        while (std::getline(in, line)) {
            std::string value;
            if (read_key_value(line, "timestamp", value)) {
                timestamp = value;
            } else if (read_key_value(line, "left", value)) {
                left_file = value;
            } else if (read_key_value(line, "right", value)) {
                right_file = value;
            }
        }

        if (timestamp.empty()) {
            const std::string stem = manifest_path.stem().string();
            const std::string prefix = "drag_record_dual_";
            if (stem.rfind(prefix, 0) == 0) {
                timestamp = stem.substr(prefix.size());
                const std::string suffix = "_index";
                if (ends_with(timestamp, suffix)) {
                    timestamp.resize(timestamp.size() - suffix.size());
                }
            }
        }
        if (timestamp.empty() || left_file.empty() || right_file.empty()) {
            return std::nullopt;
        }

        const auto left_path = manifest_path.parent_path() / left_file;
        const auto right_path = manifest_path.parent_path() / right_file;
        if (!std::filesystem::is_regular_file(left_path) ||
            !std::filesystem::is_regular_file(right_path)) {
            return std::nullopt;
        }

        return TrajectoryPair{timestamp, left_path, right_path};
    }

    // 只按 *_index.dat 清单列出新版双臂轨迹，按时间戳降序。
    std::vector<TrajectoryPair> list_trajectory_pairs() const {
        std::vector<TrajectoryPair> result;

        const std::string manifest_prefix = "drag_record_dual_";
        if (std::filesystem::exists(trajectory_log_dir_)) {
            for (const auto& entry : std::filesystem::directory_iterator(trajectory_log_dir_)) {
                if (!entry.is_regular_file() || entry.path().extension() != ".dat") {
                    continue;
                }
                const std::string stem = entry.path().stem().string();
                if (stem.rfind(manifest_prefix, 0) != 0 || !ends_with(stem, "_index")) {
                    continue;
                }
                auto pair = read_dual_manifest(entry.path());
                if (pair) {
                    result.push_back(*pair);
                }
            }
        }

        std::sort(result.begin(), result.end(),
                  [](const auto& a, const auto& b) {
                      return a.timestamp > b.timestamp;
                  });
        return result;
    }

    void execute_select_and_replay() {
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "  Select History Trajectory Pair");
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");

        auto pairs = list_trajectory_pairs();
        if (pairs.empty()) {
            RCLCPP_WARN(this->get_logger(), "  No paired trajectory files found");
            RCLCPP_WARN(this->get_logger(), "  Directory: %s", trajectory_log_dir_.string().c_str());
            RCLCPP_WARN(this->get_logger(), "  Use feature 1 to record a pair first");
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            if (rclcpp::ok()) {
                print_command_prompt();
            }
            return;
        }

        RCLCPP_INFO(this->get_logger(), "  Found %zu trajectory pair(s):", pairs.size());
        for (size_t i = 0; i < pairs.size(); ++i) {
            const auto& pair = pairs[i];
            RCLCPP_INFO(this->get_logger(),
                        "    [%zu] %s",
                        i + 1, pair.timestamp.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "  Enter pair number (1-%zu) or 0 to cancel:",
                    pairs.size());

        std::string line;
        std::getline(std::cin, line);
        trim(line);

        if (line.empty() || line == "0") {
            RCLCPP_INFO(this->get_logger(), "  Cancelled");
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            if (rclcpp::ok()) {
                print_command_prompt();
            }
            return;
        }

        size_t index = 0;
        try {
            index = std::stoul(line);
        } catch (const std::exception&) {
            RCLCPP_ERROR(this->get_logger(), "  Invalid input: %s", line.c_str());
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            if (rclcpp::ok()) {
                print_command_prompt();
            }
            return;
        }
        if (index < 1 || index > pairs.size()) {
            RCLCPP_ERROR(this->get_logger(), "  Invalid pair number: %s", line.c_str());
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            if (rclcpp::ok()) {
                print_command_prompt();
            }
            return;
        }

        const auto& pair = pairs[index - 1];
        RCLCPP_INFO(this->get_logger(),
                    "  Selected pair: %s",
                    pair.timestamp.c_str());

        left_.dt->set_replay_file(pair.left.string());
        right_.dt->set_replay_file(pair.right.string());

        const int rc = onero_api::OneroDragTeaching::handle_command_dual(
            *left_.dt, *right_.dt, 3);
        if (rc == 0) {
            RCLCPP_INFO(this->get_logger(), "✓ Replaying pair: %s",
                        pair.timestamp.c_str());
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            waiting_for_replay_completion_ = true;
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "✗ Replay failed (rc=%d). Both arms must be IDLE.", rc);
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            if (rclcpp::ok()) {
                print_command_prompt();
            }
        }
    }

    static void trim(std::string& s) {
        const size_t first = s.find_first_not_of(" \t\n\r");
        if (first == std::string::npos) {
            s.clear();
            return;
        }
        const size_t last = s.find_last_not_of(" \t\n\r");
        s = s.substr(first, last - first + 1);
    }

    ArmCtx left_;
    ArmCtx right_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::thread input_thread_;
    std::atomic<bool> input_thread_running_{false};

    bool waiting_for_replay_completion_{false};
    std::filesystem::path trajectory_log_dir_;
    std::string current_pair_timestamp_;
    std::filesystem::path current_manifest_file_;

    rclcpp::TimerBase::SharedPtr shutdown_check_timer_;
    std::atomic<bool> shutdown_done_{false};

public:
    // 由 shutdown_check_timer_ 在普通线程上下文里调用：让双臂拖动同时停下并 latch。
    void graceful_shutdown() {
        if (left_.dt && left_.dt->is_initialized() &&
            right_.dt && right_.dt->is_initialized()) {
            // dual API 兜底:cmd=0 顺序停 recording+replay,无需同步
            (void)onero_api::OneroDragTeaching::handle_command_dual(*left_.dt, *right_.dt, 0);
            return;
        }
        // 任一侧未初始化时退化到逐臂调用,避免 dual API 在 nullptr 上抛错
        if (left_.dt && left_.dt->is_initialized()) {
            left_.dt->handle_command(0);
        }
        if (right_.dt && right_.dt->is_initialized()) {
            right_.dt->handle_command(0);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, dualDragTeachingSignalHandler);
    std::signal(SIGTERM, dualDragTeachingSignalHandler);
    auto node = std::make_shared<DualArmDragTeachingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
