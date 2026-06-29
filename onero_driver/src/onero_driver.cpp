// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include <pinocchio/fwd.hpp>

#include "rclcpp/rclcpp.hpp"
#include "onero_interfaces/msg/arm_state.hpp"
#include "onero_interfaces/msg/command_result.hpp"
#include "onero_interfaces/msg/dual_gripper.hpp"
#include "onero_interfaces/msg/dual_move_j.hpp"
#include "onero_interfaces/msg/dual_sync_state.hpp"
#include "onero_interfaces/msg/gripper.hpp"
#include "onero_interfaces/msg/move_j.hpp"
#include "onero_interfaces/msg/move_l.hpp"
#include "onero_interfaces/msg/move_p.hpp"
#include "onero_interfaces/srv/end_effector_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "onero_define.h"
#include "onero_interface_cpp.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using CommandResult = onero_interfaces::msg::CommandResult;

constexpr int kArmDof = 7;

std::string normalizeMountOrientation(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (value == "horizontal" || value == "vertical") {
        return value;
    }
    return "horizontal";
}

std::string codeToMessage(int code) {
    switch (code) {
        case 0: return "";
        case -1: return "INVALID_PARAMS";
        case -2: return "IK_FAILED";
        case -3: return "COLLISION_DETECTED";
        case -4: return "EXECUTION_FAILED";
        case -5: return "TIMEOUT";
        case -6: return "INTERRUPTED";
        case -7: return "JOINT_LIMIT_EXCEEDED";
        case -8: return "BUSY";
        default: return "UNKNOWN_ERROR";
    }
}

onero_api::Pose toApiPose(const geometry_msgs::msg::Pose& pose) {
    onero_api::Pose out;
    out.x = pose.position.x;
    out.y = pose.position.y;
    out.z = pose.position.z;
    out.qw = pose.orientation.w;
    out.qx = pose.orientation.x;
    out.qy = pose.orientation.y;
    out.qz = pose.orientation.z;
    return out;
}

void fillRosPose(const onero_api::Pose& in, geometry_msgs::msg::Pose& out) {
    out.position.x = in.x;
    out.position.y = in.y;
    out.position.z = in.z;
    out.orientation.w = in.qw;
    out.orientation.x = in.qx;
    out.orientation.y = in.qy;
    out.orientation.z = in.qz;
}

std::string normalizePlanningTarget(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (value == "left" || value == "left_arm") {
        return "left";
    }
    if (value == "right" || value == "right_arm") {
        return "right";
    }
    if (value == "dual" || value == "both" || value == "both_arms") {
        return "dual";
    }
    return value;
}

bool endsWith(const std::string& value, const std::string& suffix) {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string inferDriverMode(const std::string& robot_model, int dof) {
    std::string model = robot_model;
    std::transform(model.begin(), model.end(), model.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (model.find("dual") != std::string::npos) {
        return "dual";
    }
    if (endsWith(model, "_l") || model.find("left") != std::string::npos) {
        return "left";
    }
    if (endsWith(model, "_r") || model.find("right") != std::string::npos) {
        return "right";
    }
    if (dof == 14) {
        return "dual";
    }
    return "";
}

class StartBarrier {
public:
    explicit StartBarrier(int target) : target_(target) {}

    bool arriveAndWait(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mu_);
        ++waiting_;
        if (waiting_ >= target_) {
            cv_.notify_all();
            return !aborted_;
        }
        return cv_.wait_for(lock, timeout, [this]() {
            return waiting_ >= target_ || aborted_;
        }) && !aborted_;
    }

    void abort() {
        std::lock_guard<std::mutex> lock(mu_);
        aborted_ = true;
        cv_.notify_all();
    }

private:
    std::mutex mu_;
    std::condition_variable cv_;
    int waiting_{0};
    int target_{0};
    bool aborted_{false};
};

}  // namespace

class OneroDriverNode : public rclcpp::Node {
public:
    OneroDriverNode()
        : Node("onero_driver_node") {
        loadParams();
        setupArms();
        setupJointStatePub();
        setupMoveItBridge();
        setupStateTimer();
    }

    ~OneroDriverNode() override {
        // 1) 停所有 worker：set shutdown flag, notify, join
        for (auto& iface : arms_) {
            iface->shutdown_requested.store(true);
            iface->per_arm_should_cancel.store(true);
            {
                std::lock_guard<std::mutex> lk(iface->pending_mu);
                iface->pending_cv.notify_all();
            }
            if (iface->command_worker.joinable()) {
                iface->command_worker.join();
            }
        }
        // 2) worker 已停 → 总线安静 → 安全关闭夹爪。
        for (auto& iface : arms_) {
            if (iface->arm && iface->arm->has_gripper()) {
                iface->arm->gripper()->disable();
            }
        }
        // 3) 双臂调度线程
        dual_shutdown_.store(true);
        {
            std::lock_guard<std::mutex> lk(dual_pending_mu_);
            dual_pending_cv_.notify_all();
        }
        if (dual_scheduler_.joinable()) {
            dual_scheduler_.join();
        }
        // 4) MoveIt 轨迹执行线程
        moveit_should_cancel_.store(true);
        if (execution_thread_.joinable()) {
            execution_thread_.join();
        }
        // ★ 不调 disable_motors / cancel_trajectory；电机保持上一拍 PD 命令，
        //    OneroArm 析构会触发 OneroCore::shutdown() 释放串口（不再下电）。
    }

private:
    // -----------------------------------------------------------------
    // 工作线程任务定义：所有运动类命令统一通过 Job 投递给 per-arm worker。
    // -----------------------------------------------------------------
    struct Job {
        enum class Kind { MOVEJ, MOVEL, MOVEP };
        Kind kind{Kind::MOVEJ};
        onero_api::JointArray joints;          // MOVEJ 用
        onero_api::Pose pose{};                // MOVEL/MOVEP 用
        double speed_scale{1.0};
        uint8_t trajectory_connect{0};
        rclcpp::Publisher<CommandResult>::SharedPtr result_pub;  // 结果走哪个话题
    };

    struct DualJob {
        onero_api::JointArray left_target;
        onero_api::JointArray right_target;
        double speed_scale{1.0};
    };

    // PerArmInterface 状态机：
    //   运动 callback：成功投递任务后置 status=1；worker 完成后置 0（成功）或 2（失败）
    //   终止类（stop/dual_arm/stop）：执行后置 status=0
    struct PerArmInterface;
    struct InterruptCtx {
        OneroDriverNode* node{nullptr};
        PerArmInterface* iface{nullptr};
    };

    struct PerArmInterface {
        std::string sub_ns;
        std::string robot_model;
        std::string device;
        std::vector<std::string> joint_names;     // 长度 = kArmDof，空字符串则 fallback 默认命名
        std::unique_ptr<onero_api::OneroArm> arm;
        std::atomic<uint8_t> status{0};

        // 并发原语
        std::mutex command_mutex;                       // 互斥所有 SDK 写命令（worker 内持有）
        std::atomic<bool> busy{false};                  // 是否有运动命令正在执行（CAS 拦截）
        std::atomic<bool> per_arm_should_cancel{false}; // 由 stop / cancel callback 设置；SDK 通过 InterruptCtx 读
        std::atomic<bool> shutdown_requested{false};

        // 单槽 pending（不是队列）
        std::mutex pending_mu;
        std::condition_variable pending_cv;
        std::optional<Job> pending;

        // worker 线程
        std::thread command_worker;

        // SDK interrupt_ctx 槽位（生命周期与 iface 一致）
        std::unique_ptr<InterruptCtx> interrupt_ctx;

        rclcpp::Subscription<onero_interfaces::msg::MoveJ>::SharedPtr movej_sub;
        rclcpp::Subscription<onero_interfaces::msg::MoveL>::SharedPtr movel_sub;
        rclcpp::Subscription<onero_interfaces::msg::MoveP>::SharedPtr movep_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub;
        rclcpp::Subscription<onero_interfaces::msg::Gripper>::SharedPtr gripper_sub;

        rclcpp::Publisher<CommandResult>::SharedPtr movej_result_pub;
        rclcpp::Publisher<CommandResult>::SharedPtr movel_result_pub;
        rclcpp::Publisher<CommandResult>::SharedPtr movep_result_pub;
        rclcpp::Publisher<CommandResult>::SharedPtr stop_result_pub;
        rclcpp::Publisher<CommandResult>::SharedPtr gripper_result_pub;
        rclcpp::Publisher<onero_interfaces::msg::ArmState>::SharedPtr arm_state_pub;
        rclcpp::Publisher<onero_interfaces::msg::Gripper>::SharedPtr gripper_state_pub;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_buffer_srv;
        rclcpp::Service<onero_interfaces::srv::EndEffectorPose>::SharedPtr get_end_pose_srv;

        std::string topic(const std::string& leaf) const {
            return sub_ns.empty()
                ? "/onero_arm/" + leaf
                : "/onero_arm/" + sub_ns + "/" + leaf;
        }
    };

    // ★ Per-arm 中断检查：SDK 的控制环每个 tick 调一次，看到该 flag 立即 return INTERRUPTED。
    static bool interruptRequested(void* ctx) {
        if (!ctx) {
            return !rclcpp::ok();
        }
        auto* c = static_cast<InterruptCtx*>(ctx);
        if (!c->iface) {
            return !rclcpp::ok();
        }
        return c->iface->per_arm_should_cancel.load() || !rclcpp::ok();
    }

    // -----------------------------------------------------------------
    // 参数加载
    // -----------------------------------------------------------------
    void loadParams() {
        this->declare_parameter<int>("dof", 7);
        this->declare_parameter<double>("state_pub_rate", 100.0);
        this->declare_parameter<std::string>("mount_orientation", "horizontal");
        this->declare_parameter<std::string>("robot_model", "a1_l");
        this->declare_parameter<std::string>("device", "/dev/ttyACM0");
        this->declare_parameter<std::string>("device_left", "/dev/ttyACM0");
        this->declare_parameter<std::string>("device_right", "/dev/ttyACM1");
        this->declare_parameter<bool>("simulation_mode", false);

        // 夹爪参数：单/双臂启用开关分开声明，便于用户在对应 yaml 里只看到一种。
        // gripper.max_* 是 SDK move_position 的轨迹平滑参数，左右共用一份。
        this->declare_parameter<bool>("gripper.enabled",        false);
        this->declare_parameter<bool>("gripper.left_enabled",   false);
        this->declare_parameter<bool>("gripper.right_enabled",  false);
        this->declare_parameter<double>("gripper.max_vel",  100.0);
        this->declare_parameter<double>("gripper.max_acc",  250.0);
        this->declare_parameter<double>("gripper.max_jerk", 1000.0);

        // 关节命名参数化：默认空 → fallback 到 "joint{N}-{robot_model}"。
        // 用户可在 yaml 里覆盖以适配自己的 URDF。
        this->declare_parameter<std::vector<std::string>>(
            "joint_names", std::vector<std::string>{});
        this->declare_parameter<std::vector<std::string>>(
            "joint_names_left", std::vector<std::string>{});
        this->declare_parameter<std::vector<std::string>>(
            "joint_names_right", std::vector<std::string>{});

        dof_ = this->get_parameter("dof").as_int();
        state_pub_rate_ = this->get_parameter("state_pub_rate").as_double();
        mount_orientation_ = normalizeMountOrientation(this->get_parameter("mount_orientation").as_string());

        robot_model_ = this->get_parameter("robot_model").as_string();
        device_ = this->get_parameter("device").as_string();
        device_left_ = this->get_parameter("device_left").as_string();
        device_right_ = this->get_parameter("device_right").as_string();
        simulation_mode_ = this->get_parameter("simulation_mode").as_bool();

        gripper_enabled_single_ = this->get_parameter("gripper.enabled").as_bool();
        gripper_enabled_left_   = this->get_parameter("gripper.left_enabled").as_bool();
        gripper_enabled_right_  = this->get_parameter("gripper.right_enabled").as_bool();
        gripper_max_vel_  = this->get_parameter("gripper.max_vel").as_double();
        gripper_max_acc_  = this->get_parameter("gripper.max_acc").as_double();
        gripper_max_jerk_ = this->get_parameter("gripper.max_jerk").as_double();

        joint_names_single_ = this->get_parameter("joint_names").as_string_array();
        joint_names_left_  = this->get_parameter("joint_names_left").as_string_array();
        joint_names_right_ = this->get_parameter("joint_names_right").as_string_array();

        driver_mode_ = inferDriverMode(robot_model_, dof_);
        if (driver_mode_.empty()) {
            throw std::runtime_error(
                "Cannot infer driver mode from robot_model='" + robot_model_ +
                "'. Use a left/right model name such as a1_l/a1_r, or A1_dual with dof=14.");
        }
        if (driver_mode_ == "dual" && dof_ != 14) {
            RCLCPP_WARN(this->get_logger(), "robot_model=%s implies dual-arm mode; overriding dof to 14",
                        robot_model_.c_str());
            dof_ = 14;
        }
        if (driver_mode_ != "dual" && dof_ != kArmDof) {
            RCLCPP_WARN(this->get_logger(),
                        "single-arm robot_model expects dof=%d; overriding", kArmDof);
            dof_ = kArmDof;
        }
    }

    // -----------------------------------------------------------------
    // 启动臂 + worker
    // -----------------------------------------------------------------
    void setupArms() {
        if (driver_mode_ == "dual") {
            std::vector<double> left_init_pos;
            std::vector<double> right_init_pos;
            std::unique_ptr<PerArmInterface> left;
            std::unique_ptr<PerArmInterface> right;
            try {
                left  = makePerArmCreateAndEnable("left_arm",  "a1_l", device_left_,
                                                  /*left_arm_params=*/true,
                                                  gripper_enabled_left_,
                                                  left_init_pos);
                right = makePerArmCreateAndEnable("right_arm", "a1_r", device_right_,
                                                  /*left_arm_params=*/false,
                                                  gripper_enabled_right_,
                                                  right_init_pos);
            } catch (...) {
                // 启动阶段失败回滚:任一侧已 enable_motors 但还未运动,显式下电避免持续保持力矩。
                if (left  && left->arm)  left->arm->disable_motors();
                if (right && right->arm) right->arm->disable_motors();
                throw;
            }

            // 并发 restore — 两个独立 OneroArm 通过 SDK 内置 barrier 在同一 wall-clock
            // 时刻进入 restore_arm,避免先后 enable + restore 的串行延迟。
            onero_api::JointArray left_target(left_init_pos.begin(),
                                              left_init_pos.begin() + kArmDof);
            onero_api::JointArray right_target(right_init_pos.begin(),
                                               right_init_pos.begin() + kArmDof);
            std::vector<onero_api::OneroArm::RestoreTarget> tasks{
                {left->arm.get(),  left_target},
                {right->arm.get(), right_target}
            };
            int restore_code = onero_api::OneroArm::restore_arm_concurrent(tasks);
            if (restore_code != static_cast<int>(onero_api::MoveResult::SUCCESS)) {
                // 失败时两侧都 disable;SDK 不会主动中断对侧 restore,但此时已等齐返回,
                // 调 disable_motors 是安全的。
                if (left->arm)  left->arm->disable_motors();
                if (right->arm) right->arm->disable_motors();
                throw std::runtime_error("dual restore_arm_concurrent failed, code="
                                         + std::to_string(restore_code));
            }

            arms_.push_back(std::move(left));
            arms_.push_back(std::move(right));
            // worker + ROS 接口要在 restore 完成之后才绑定 — 避免 worker 还在跑
            // movej/movel 等 SDK 命令时被并发的 restore 抢串口。
            setupPerArmRosBindings(*arms_[0]);
            setupPerArmRosBindings(*arms_[1]);
            setupDualArmInterface();
            return;
        }

        // 单臂分支:复用 create+enable / restore / setup 三段,语义不变。
        std::vector<double> init_pos;
        std::string model = robot_model_;
        auto iface = makePerArmCreateAndEnable("", model, device_,
                                               driver_mode_ != "right",
                                               gripper_enabled_single_,
                                               init_pos);
        onero_api::JointArray target(init_pos.begin(), init_pos.begin() + kArmDof);
        int restore_ret = iface->arm->restore_arm(target);
        if (restore_ret != static_cast<int>(onero_api::MoveResult::SUCCESS)) {
            iface->arm->disable_motors();
            throw std::runtime_error("Failed to restore arm to initial pos for " + iface->robot_model
                                     + ", code=" + std::to_string(restore_ret));
        }
        arms_.push_back(std::move(iface));
        setupPerArmRosBindings(*arms_.back());
    }

    std::unique_ptr<PerArmInterface> makePerArmCreateAndEnable(
        const std::string& sub_ns,
        const std::string& robot_model,
        const std::string& device,
        bool left_arm_params,
        bool with_gripper,
        std::vector<double>& init_pos_out) {
        auto iface = std::make_unique<PerArmInterface>();
        iface->sub_ns = sub_ns;
        iface->robot_model = robot_model;
        iface->device = device;
        iface->interrupt_ctx = std::make_unique<InterruptCtx>();
        iface->interrupt_ctx->node = this;
        iface->interrupt_ctx->iface = iface.get();

        // 选 joint_names:单臂用 joint_names;双臂分别用 joint_names_left / right。
        const std::vector<std::string>& cfg_names =
            (driver_mode_ == "dual")
                ? (left_arm_params ? joint_names_left_ : joint_names_right_)
                : joint_names_single_;
        iface->joint_names.resize(kArmDof);
        for (int i = 0; i < kArmDof; ++i) {
            iface->joint_names[i] =
                (static_cast<int>(cfg_names.size()) > i && !cfg_names[i].empty())
                    ? cfg_names[i]
                    : ("joint" + std::to_string(i + 1) + "-" + robot_model);
        }

        onero_api::onero_config_t config{};
        std::strncpy(config.device, device.c_str(), sizeof(config.device) - 1);
        std::strncpy(config.robot_model, robot_model.c_str(), sizeof(config.robot_model) - 1);
        std::strncpy(config.mount_orientation, mount_orientation_.c_str(), sizeof(config.mount_orientation) - 1);
        config.dof = kArmDof;
        config.baud_rate = 921600;
        config.urdf_path[0] = '\0';
        config.interrupt_check = &OneroDriverNode::interruptRequested;
        config.interrupt_ctx = iface->interrupt_ctx.get();
        config.simulation_mode = simulation_mode_;
        config.with_gripper = with_gripper;

        // ★ mit_kp/mit_kd 默认 0:SDK 会 fallback 到内置默认值
        //   ([150,150,150,150,30,30,30] / [4,4,4,4,1,1,1])
        std::vector<double> kp;
        std::vector<double> kd;
        std::vector<double> initial_pos;
        if (driver_mode_ == "dual") {
            kp = this->declare_parameter<std::vector<double>>(
                left_arm_params ? "mit_mode.left_kp_values" : "mit_mode.right_kp_values",
                std::vector<double>(kArmDof, 0.0));
            kd = this->declare_parameter<std::vector<double>>(
                left_arm_params ? "mit_mode.left_kd_values" : "mit_mode.right_kd_values",
                std::vector<double>(kArmDof, 0.0));
            initial_pos = this->declare_parameter<std::vector<double>>(
                left_arm_params ? "left_initial_pos" : "right_initial_pos",
                std::vector<double>(kArmDof, 0.0));
        } else {
            kp = this->declare_parameter<std::vector<double>>(
                "mit_mode.default_kp_values", std::vector<double>(kArmDof, 0.0));
            kd = this->declare_parameter<std::vector<double>>(
                "mit_mode.default_kd_values", std::vector<double>(kArmDof, 0.0));
            initial_pos = this->declare_parameter<std::vector<double>>(
                "initial_pos", std::vector<double>(kArmDof, 0.0));
        }

        if (kp.size() < kArmDof || kd.size() < kArmDof) {
            throw std::runtime_error("mit_mode kp/kd arrays must contain at least 7 values");
        }
        if (initial_pos.size() < kArmDof) {
            throw std::runtime_error("initial_pos array must contain at least 7 values");
        }
        for (int i = 0; i < kArmDof; ++i) {
            config.mit_kp[i] = kp[i];
            config.mit_kd[i] = kd[i];
        }

        iface->arm = std::make_unique<onero_api::OneroArm>(config);
        if (!iface->arm->valid()) {
            throw std::runtime_error("Failed to create OneroArm for " + robot_model);
        }

        int ret = iface->arm->enable_motors();
        if (ret != static_cast<int>(onero_api::MoveResult::SUCCESS)) {
            throw std::runtime_error("Failed to enable motors for " + robot_model + ", code=" + std::to_string(ret));
        }

        // 启用夹爪。容错而非抛异常 —— 硬件问题不应阻塞整臂启动；
        // demo / 用户端通过 CommandResult.success=false 自行感知。
        if (with_gripper && iface->arm->has_gripper()) {
            int g_ret = iface->arm->gripper()->enable();
            if (g_ret != 0) {
                RCLCPP_WARN(this->get_logger(),
                    "gripper enable failed for %s, code=%d (continuing without gripper)",
                    robot_model.c_str(), g_ret);
            }
        }

        // 把 initial_pos 透出给 caller — 双臂 setupArms 用它构造 restore_arm_concurrent
        // 的 RestoreTarget 列表;单臂 setupArms 直接用它调 restore_arm。
        init_pos_out = std::move(initial_pos);

        return iface;
    }

    void setupPerArmRosBindings(PerArmInterface& iface) {
        PerArmInterface* raw = &iface;
        // 启动 worker 线程:必须在 arm/interrupt_ctx 都准备好之后。
        iface.command_worker = std::thread([this, raw]() { perArmWorkerLoop(*raw); });

        // ROS 接口绑定
        iface.movej_sub = this->create_subscription<onero_interfaces::msg::MoveJ>(
            raw->topic("movej"), 10,
            [this, raw](onero_interfaces::msg::MoveJ::SharedPtr msg) { movejCallback(*raw, msg); });
        iface.movel_sub = this->create_subscription<onero_interfaces::msg::MoveL>(
            raw->topic("movel"), 10,
            [this, raw](onero_interfaces::msg::MoveL::SharedPtr msg) { movelCallback(*raw, msg); });
        iface.movep_sub = this->create_subscription<onero_interfaces::msg::MoveP>(
            raw->topic("movep"), 10,
            [this, raw](onero_interfaces::msg::MoveP::SharedPtr msg) { movepCallback(*raw, msg); });
        iface.stop_sub = this->create_subscription<std_msgs::msg::Bool>(
            raw->topic("stop"), 10,
            [this, raw](std_msgs::msg::Bool::SharedPtr msg) { stopCallback(*raw, msg); });

        iface.movej_result_pub = this->create_publisher<CommandResult>(raw->topic("movej_result"), 10);
        iface.movel_result_pub = this->create_publisher<CommandResult>(raw->topic("movel_result"), 10);
        iface.movep_result_pub = this->create_publisher<CommandResult>(raw->topic("movep_result"), 10);
        iface.stop_result_pub = this->create_publisher<CommandResult>(raw->topic("stop_result"), 10);
        iface.arm_state_pub =
            this->create_publisher<onero_interfaces::msg::ArmState>(raw->topic("arm_state"), 10);

        iface.clear_buffer_srv = this->create_service<std_srvs::srv::Trigger>(
            raw->topic("clear_buffer"),
            [this, raw](std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                clearBufferCallback(*raw, request, response);
            });
        iface.get_end_pose_srv = this->create_service<onero_interfaces::srv::EndEffectorPose>(
            raw->topic("get_end_pose"),
            [this, raw](std::shared_ptr<onero_interfaces::srv::EndEffectorPose::Request> request,
                        std::shared_ptr<onero_interfaces::srv::EndEffectorPose::Response> response) {
                getEndPoseCallback(*raw, request, response);
            });

        // 夹爪话题：仅当该臂启用并真正构造出 OneroGripper 时绑定，
        // 没装夹爪的臂不会出现 /onero_arm[/xxx_arm]/gripper* 话题。
        if (iface.arm && iface.arm->has_gripper()) {
            iface.gripper_result_pub = this->create_publisher<CommandResult>(
                raw->topic("gripper_result"), 10);
            iface.gripper_state_pub = this->create_publisher<onero_interfaces::msg::Gripper>(
                raw->topic("gripper_state"), 10);
            iface.gripper_sub = this->create_subscription<onero_interfaces::msg::Gripper>(
                raw->topic("gripper"), 10,
                [this, raw](onero_interfaces::msg::Gripper::SharedPtr msg) {
                    gripperCallback(*raw, msg);
                });
        }
    }

    // -----------------------------------------------------------------
    // Worker：单线程串行处理一个 PerArmInterface 的运动命令
    // -----------------------------------------------------------------
    void perArmWorkerLoop(PerArmInterface& iface) {
        for (;;) {
            Job job;
            {
                std::unique_lock<std::mutex> lk(iface.pending_mu);
                iface.pending_cv.wait(lk, [&]{
                    return iface.pending.has_value() || iface.shutdown_requested.load();
                });
                if (iface.shutdown_requested.load()) {
                    return;
                }
                job = std::move(*iface.pending);
                iface.pending.reset();
            }

            int code = static_cast<int>(onero_api::MoveResult::EXECUTION_FAILED);
            {
                std::lock_guard<std::mutex> mu(iface.command_mutex);
                if (iface.arm) {
                    switch (job.kind) {
                        case Job::Kind::MOVEJ:
                            code = iface.arm->movej(job.joints, job.speed_scale, job.trajectory_connect);
                            break;
                        case Job::Kind::MOVEL:
                            code = iface.arm->movel(job.pose, job.speed_scale, job.trajectory_connect);
                            break;
                        case Job::Kind::MOVEP:
                            code = iface.arm->movep(job.pose, job.speed_scale, job.trajectory_connect);
                            break;
                    }
                }
            }
            iface.status.store(code == 0 ? 0 : 2);
            if (code == static_cast<int>(onero_api::MoveResult::SUCCESS)) {
                const bool buffered =
                    job.trajectory_connect == static_cast<uint8_t>(onero_api::TrajectoryConnect::BUFFER);
                publishCommandResult(job.result_pub, code, "", buffered ? "BUFFER_SUCCESS" : "");
            } else {
                onero_api::BufferedTrajectoryFailure buffered_failure;
                if (iface.arm) {
                    buffered_failure = iface.arm->get_last_buffered_trajectory_failure();
                }
                if (buffered_failure.valid) {
                    const std::string prefix =
                        "buffered " + bufferedTypeToName(buffered_failure.type) + ": ";
                    auto buffered_result_pub =
                        resultPublisherForBufferedType(iface, buffered_failure.type);
                    if (buffered_result_pub && buffered_result_pub != job.result_pub) {
                        publishCommandResult(buffered_result_pub, buffered_failure.error_code, prefix);
                    }
                    publishCommandResult(job.result_pub, code, prefix);
                } else {
                    publishCommandResult(job.result_pub, code);
                }
            }

            // 完成本次 Job：释放 BUSY，让下一条命令可以被接收
            iface.busy.store(false);
        }
    }

    // -----------------------------------------------------------------
    // 双臂调度线程
    // -----------------------------------------------------------------
    void setupDualArmInterface() {
        dual_movej_sub_ = this->create_subscription<onero_interfaces::msg::DualMoveJ>(
            "/onero_arm/dual_arm/movej", 10,
            std::bind(&OneroDriverNode::dualMoveJCallback, this, std::placeholders::_1));
        dual_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/onero_arm/dual_arm/stop", 10,
            std::bind(&OneroDriverNode::dualStopCallback, this, std::placeholders::_1));
        dual_movej_result_pub_ = this->create_publisher<CommandResult>("/onero_arm/dual_arm/movej_result", 10);
        dual_stop_result_pub_ = this->create_publisher<CommandResult>("/onero_arm/dual_arm/stop_result", 10);
        dual_sync_state_pub_ =
            this->create_publisher<onero_interfaces::msg::DualSyncState>("/onero_arm/dual_arm/sync_state", 10);

        // 双臂同步夹爪话题：仅当左右两侧都启用并真正构造出 OneroGripper 时才绑定。
        if (arms_.size() >= 2
            && arms_[0]->arm && arms_[0]->arm->has_gripper()
            && arms_[1]->arm && arms_[1]->arm->has_gripper()) {
            dual_gripper_sub_ = this->create_subscription<onero_interfaces::msg::DualGripper>(
                "/onero_arm/dual_arm/gripper", 10,
                std::bind(&OneroDriverNode::dualGripperCallback, this, std::placeholders::_1));
            dual_gripper_result_pub_ = this->create_publisher<CommandResult>(
                "/onero_arm/dual_arm/gripper_result", 10);
            dual_gripper_state_pub_ = this->create_publisher<onero_interfaces::msg::DualGripper>(
                "/onero_arm/dual_arm/gripper_state", 10);
        }

        dual_scheduler_ = std::thread(&OneroDriverNode::dualSchedulerLoop, this);
    }

    void dualSchedulerLoop() {
        for (;;) {
            DualJob job;
            {
                std::unique_lock<std::mutex> lk(dual_pending_mu_);
                dual_pending_cv_.wait(lk, [&]{
                    return dual_pending_.has_value() || dual_shutdown_.load();
                });
                if (dual_shutdown_.load()) {
                    return;
                }
                job = std::move(*dual_pending_);
                dual_pending_.reset();
            }
            executeDualJob(job);
        }
    }

    void executeDualJob(const DualJob& job) {
        if (arms_.size() != 2 || !arms_[0]->arm || !arms_[1]->arm) {
            publishCommandResult(dual_movej_result_pub_, static_cast<int>(onero_api::MoveResult::EXECUTION_FAILED));
            return;
        }
        auto& left = *arms_[0];
        auto& right = *arms_[1];

        double base_speed = job.speed_scale == 0.0 ? 1.0 : job.speed_scale;
        double left_duration = left.arm->estimate_movej_duration(job.left_target, base_speed);
        double right_duration = right.arm->estimate_movej_duration(job.right_target, base_speed);
        if (!std::isfinite(left_duration) || left_duration < 0.0) {
            int code = estimateFailureCode(left_duration);
            publishDualSync(2, false, false, code, "left_arm: " + codeToMessage(code));
            publishCommandResult(dual_movej_result_pub_, code, "left_arm: ");
            // 释放 BUSY
            left.busy.store(false);
            right.busy.store(false);
            return;
        }
        if (!std::isfinite(right_duration) || right_duration < 0.0) {
            int code = estimateFailureCode(right_duration);
            publishDualSync(2, false, false, code, "right_arm: " + codeToMessage(code));
            publishCommandResult(dual_movej_result_pub_, code, "right_arm: ");
            left.busy.store(false);
            right.busy.store(false);
            return;
        }

        double left_speed = base_speed;
        double right_speed = base_speed;
        const double target_duration = std::max(left_duration, right_duration);
        const double duration_delta = std::abs(left_duration - right_duration);
        if (target_duration > 1e-6 && duration_delta / target_duration >= 0.05) {
            const char* shorter = (left_duration < right_duration) ? "left_arm" : "right_arm";
            const char* matched = (left_duration < right_duration) ? "right_arm" : "left_arm";
            if (left_duration < right_duration) {
                left_speed = alignShortArmSpeed(*left.arm, job.left_target, base_speed, right_duration);
            } else {
                right_speed = alignShortArmSpeed(*right.arm, job.right_target, base_speed, left_duration);
            }
            const double adjusted_speed = (left_duration < right_duration) ? left_speed : right_speed;
            RCLCPP_INFO(this->get_logger(),
                "[dual_sync] %s speed adjusted for sync: requested=%.4f, adjusted=%.4f, "
                "reason=match %s duration, left_base_dur=%.3fs, right_base_dur=%.3fs",
                shorter, base_speed, adjusted_speed,
                matched, left_duration, right_duration);
        }

        left.status.store(1);
        right.status.store(1);
        publishDualSync(1, false, false, 0);

        StartBarrier barrier(2);
        // 各自取自己的 command_mutex_，SDK 调用串行化（不抢同一只臂的总线）
        auto left_future = std::async(std::launch::async, [&]() {
            if (!barrier.arriveAndWait(std::chrono::milliseconds(5000))) {
                return static_cast<int>(onero_api::MoveResult::TIMEOUT);
            }
            std::lock_guard<std::mutex> lk(left.command_mutex);
            return left.arm->movej(job.left_target, left_speed, 0);
        });
        auto right_future = std::async(std::launch::async, [&]() {
            if (!barrier.arriveAndWait(std::chrono::milliseconds(5000))) {
                return static_cast<int>(onero_api::MoveResult::TIMEOUT);
            }
            std::lock_guard<std::mutex> lk(right.command_mutex);
            return right.arm->movej(job.right_target, right_speed, 0);
        });

        int left_code = static_cast<int>(onero_api::MoveResult::EXECUTION_FAILED);
        int right_code = static_cast<int>(onero_api::MoveResult::EXECUTION_FAILED);
        bool left_done = false;
        bool right_done = false;

        while (!left_done || !right_done) {
            if (!left_done &&
                left_future.wait_for(std::chrono::milliseconds(20)) == std::future_status::ready) {
                left_code = left_future.get();
                left.status.store(left_code == 0 ? 0 : 2);
                left_done = true;
                if (left_code != 0 && !right_done) {
                    stopArmInline(right);
                }
            }
            if (!right_done &&
                right_future.wait_for(std::chrono::milliseconds(20)) == std::future_status::ready) {
                right_code = right_future.get();
                right.status.store(right_code == 0 ? 0 : 2);
                right_done = true;
                if (right_code != 0 && !left_done) {
                    stopArmInline(left);
                }
            }
        }

        int final_code = left_code != 0 ? left_code : right_code;
        std::string prefix;
        if (left_code != 0) prefix = "left_arm: ";
        else if (right_code != 0) prefix = "right_arm: ";
        uint8_t final_status = final_code == 0 ? 0 : 2;
        publishDualSync(final_status, true, true, final_code, final_code == 0 ? "" : prefix + codeToMessage(final_code));
        publishCommandResult(dual_movej_result_pub_, final_code, prefix);

        // 释放双臂 BUSY
        left.busy.store(false);
        right.busy.store(false);
    }

    void dualMoveJCallback(const onero_interfaces::msg::DualMoveJ::SharedPtr msg) {
        if (arms_.size() != 2 || !arms_[0]->arm || !arms_[1]->arm) {
            publishCommandResult(dual_movej_result_pub_, static_cast<int>(onero_api::MoveResult::EXECUTION_FAILED));
            return;
        }
        auto& left = *arms_[0];
        auto& right = *arms_[1];

        // 双臂 BUSY 拦截：两臂都不忙才接受
        bool expected_l = false;
        bool expected_r = false;
        if (!left.busy.compare_exchange_strong(expected_l, true)) {
            publishCommandResult(dual_movej_result_pub_,
                static_cast<int>(onero_api::MoveResult::BUSY), "left_arm: ");
            return;
        }
        if (!right.busy.compare_exchange_strong(expected_r, true)) {
            // 回滚 left
            left.busy.store(false);
            publishCommandResult(dual_movej_result_pub_,
                static_cast<int>(onero_api::MoveResult::BUSY), "right_arm: ");
            return;
        }

        left.per_arm_should_cancel.store(false);
        right.per_arm_should_cancel.store(false);
        left.arm->reset_stop_signal();
        right.arm->reset_stop_signal();

        DualJob job;
        job.left_target.assign(msg->left_joint.begin(), msg->left_joint.end());
        job.right_target.assign(msg->right_joint.begin(), msg->right_joint.end());
        job.speed_scale = msg->speed_scale;

        // 投递给调度线程；callback 立即 return，不阻塞 executor。
        {
            std::lock_guard<std::mutex> lk(dual_pending_mu_);
            dual_pending_ = std::move(job);
        }
        dual_pending_cv_.notify_one();
    }

    void dualStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data || arms_.size() != 2) {
            return;
        }
        // 立即置位 cancel flag → SDK 当前 tick 内 return INTERRUPTED；callback 不阻塞
        arms_[0]->per_arm_should_cancel.store(true);
        arms_[1]->per_arm_should_cancel.store(true);
        if (arms_[0]->arm) {
            arms_[0]->arm->cancel_trajectory();
            arms_[0]->arm->clear_trajectory_buffer();
        }
        if (arms_[1]->arm) {
            arms_[1]->arm->cancel_trajectory();
            arms_[1]->arm->clear_trajectory_buffer();
        }
        arms_[0]->status.store(0);
        arms_[1]->status.store(0);
        publishDualSync(0, true, true, 0);
        publishCommandResult(dual_stop_result_pub_, 0);
    }

    // -----------------------------------------------------------------
    // MoveIt 集成
    // -----------------------------------------------------------------
    void setupJointStatePub() {
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    }

    void setupMoveItBridge() {
        planning_arms_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/onero_arm/moveit/planning_arms_status", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(planning_arms_mutex_);
                planning_arms_status_ = normalizePlanningTarget(msg->data);
            });
        trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/onero_arm/moveit/trajectory", 10,
            std::bind(&OneroDriverNode::trajectoryCallback, this, std::placeholders::_1));
        execute_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/onero_arm/moveit/execute", 10,
            std::bind(&OneroDriverNode::executeCallback, this, std::placeholders::_1));
        cancel_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/onero_arm/moveit/cancel", 10,
            std::bind(&OneroDriverNode::cancelCallback, this, std::placeholders::_1));
        execution_result_pub_ =
            this->create_publisher<CommandResult>("/onero_arm/moveit/trajectory_execution_result", 10);
    }

    void setupStateTimer() {
        if (state_pub_rate_ <= 0.0) {
            state_pub_rate_ = 100.0;
        }
        auto period = std::chrono::duration<double>(1.0 / state_pub_rate_);
        state_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&OneroDriverNode::publishArmState, this));
    }

    // -----------------------------------------------------------------
    // 公用 result helper
    // -----------------------------------------------------------------
    void publishCommandResult(
        const rclcpp::Publisher<CommandResult>::SharedPtr& pub,
        int code,
        const std::string& prefix = "",
        const std::string& success_message = "") {
        if (!pub) return;
        CommandResult result;
        result.stamp = this->now();
        result.success = (code == static_cast<int>(onero_api::MoveResult::SUCCESS));
        result.error_code = code;
        result.error_message = result.success ? success_message : (prefix + codeToMessage(code));
        pub->publish(result);
    }

    std::string bufferedTypeToName(onero_api::BufferedTrajectoryType type) const {
        switch (type) {
            case onero_api::BufferedTrajectoryType::MOVEJ:
                return "MoveJ";
            case onero_api::BufferedTrajectoryType::MOVEL:
                return "MoveL";
            case onero_api::BufferedTrajectoryType::MOVEP:
                return "MoveP";
            case onero_api::BufferedTrajectoryType::NONE:
                break;
        }
        return "trajectory";
    }

    rclcpp::Publisher<CommandResult>::SharedPtr resultPublisherForBufferedType(
        PerArmInterface& iface,
        onero_api::BufferedTrajectoryType type) const {
        switch (type) {
            case onero_api::BufferedTrajectoryType::MOVEJ:
                return iface.movej_result_pub;
            case onero_api::BufferedTrajectoryType::MOVEL:
                return iface.movel_result_pub;
            case onero_api::BufferedTrajectoryType::MOVEP:
                return iface.movep_result_pub;
            case onero_api::BufferedTrajectoryType::NONE:
                break;
        }
        return nullptr;
    }

    void publishDualSync(uint8_t status, bool left_done, bool right_done, int code, const std::string& message = "") {
        if (!dual_sync_state_pub_) return;
        onero_interfaces::msg::DualSyncState state;
        state.stamp = this->now();
        state.status = status;
        state.left_done = left_done;
        state.right_done = right_done;
        state.error_code = code;
        state.error_message = message;
        dual_sync_state_pub_->publish(state);
    }

    // -----------------------------------------------------------------
    // 夹爪 callback：per-arm + dual。
    // SDK 的 gripper 和 arm 共用一条 CAN/串口，必须复用 iface.command_mutex
    // 避免和 movej / movel / movep 在同一总线上交错下发。
    // -----------------------------------------------------------------
    void gripperCallback(PerArmInterface& iface,
                         onero_interfaces::msg::Gripper::SharedPtr msg) {
        if (!iface.arm || !iface.arm->has_gripper()) {
            publishCommandResult(iface.gripper_result_pub,
                static_cast<int>(onero_api::MoveResult::EXECUTION_FAILED));
            return;
        }
        std::lock_guard<std::mutex> lk(iface.command_mutex);
        int code = iface.arm->gripper()->move_position(
            msg->position, gripper_max_vel_, gripper_max_acc_, gripper_max_jerk_);
        publishCommandResult(iface.gripper_result_pub, code);
    }

    void dualGripperCallback(onero_interfaces::msg::DualGripper::SharedPtr msg) {
        if (arms_.size() < 2
            || !arms_[0]->arm || !arms_[0]->arm->has_gripper()
            || !arms_[1]->arm || !arms_[1]->arm->has_gripper()) {
            publishCommandResult(dual_gripper_result_pub_,
                static_cast<int>(onero_api::MoveResult::EXECUTION_FAILED));
            return;
        }
        auto& left  = *arms_[0];
        auto& right = *arms_[1];
        int left_code = 0;
        int right_code = 0;
        // 左右是独立 CAN 总线，wall-clock 上"同时下发"已足够同步。
        std::thread tl([&] {
            std::lock_guard<std::mutex> lk(left.command_mutex);
            left_code = left.arm->gripper()->move_position(
                msg->left_position, gripper_max_vel_, gripper_max_acc_, gripper_max_jerk_);
        });
        std::thread tr([&] {
            std::lock_guard<std::mutex> lk(right.command_mutex);
            right_code = right.arm->gripper()->move_position(
                msg->right_position, gripper_max_vel_, gripper_max_acc_, gripper_max_jerk_);
        });
        tl.join();
        tr.join();
        int code = (left_code != 0) ? left_code : right_code;
        const std::string prefix =
            (left_code != 0) ? std::string("left_arm: ")
            : (right_code != 0) ? std::string("right_arm: ")
            : std::string("");
        publishCommandResult(dual_gripper_result_pub_, code, prefix);
    }

    // -----------------------------------------------------------------
    // 运动 callback：参数校验 → BUSY CAS → 投递 Job → return
    // -----------------------------------------------------------------
    bool tryAcquireBusy(PerArmInterface& iface,
                        const rclcpp::Publisher<CommandResult>::SharedPtr& result_pub) {
        bool expected = false;
        if (!iface.busy.compare_exchange_strong(expected, true)) {
            publishCommandResult(result_pub, static_cast<int>(onero_api::MoveResult::BUSY));
            return false;
        }
        iface.per_arm_should_cancel.store(false);
        if (iface.arm) {
            iface.arm->reset_stop_signal();
        }
        return true;
    }

    void enqueueJob(PerArmInterface& iface, Job job) {
        {
            std::lock_guard<std::mutex> lk(iface.pending_mu);
            iface.pending = std::move(job);
        }
        iface.pending_cv.notify_one();
        iface.status.store(1);
    }

    void movejCallback(PerArmInterface& iface, const onero_interfaces::msg::MoveJ::SharedPtr msg) {
        if (msg->joint_positions.size() != kArmDof) {
            publishCommandResult(iface.movej_result_pub,
                static_cast<int>(onero_api::MoveResult::INVALID_PARAMS));
            return;
        }
        if (!iface.arm) {
            publishCommandResult(iface.movej_result_pub,
                static_cast<int>(onero_api::MoveResult::EXECUTION_FAILED));
            return;
        }
        if (!tryAcquireBusy(iface, iface.movej_result_pub)) return;

        Job job;
        job.kind = Job::Kind::MOVEJ;
        job.joints.assign(msg->joint_positions.begin(), msg->joint_positions.end());
        job.speed_scale = msg->speed_scale;
        job.trajectory_connect = msg->trajectory_connect;
        job.result_pub = iface.movej_result_pub;
        enqueueJob(iface, std::move(job));
    }

    void enqueuePoseJob(PerArmInterface& iface,
                        Job::Kind kind,
                        const geometry_msgs::msg::Pose& pose,
                        double speed_scale,
                        uint8_t trajectory_connect,
                        const rclcpp::Publisher<CommandResult>::SharedPtr& result_pub) {
        if (!iface.arm) {
            publishCommandResult(result_pub,
                static_cast<int>(onero_api::MoveResult::EXECUTION_FAILED));
            return;
        }
        // 左右臂的话题名已区分（/onero_arm/{left_arm,right_arm}/movel 等），
        // 不再要求 frame_id 消歧义。
        if (!tryAcquireBusy(iface, result_pub)) return;

        Job job;
        job.kind = kind;
        job.pose = toApiPose(pose);
        job.speed_scale = speed_scale;
        job.trajectory_connect = trajectory_connect;
        job.result_pub = result_pub;
        enqueueJob(iface, std::move(job));
    }

    void movelCallback(PerArmInterface& iface, const onero_interfaces::msg::MoveL::SharedPtr msg) {
        enqueuePoseJob(iface, Job::Kind::MOVEL, msg->pose,
                       msg->speed_scale, msg->trajectory_connect,
                       iface.movel_result_pub);
    }

    void movepCallback(PerArmInterface& iface, const onero_interfaces::msg::MoveP::SharedPtr msg) {
        enqueuePoseJob(iface, Job::Kind::MOVEP, msg->pose,
                       msg->speed_scale, msg->trajectory_connect,
                       iface.movep_result_pub);
    }

    void stopCallback(PerArmInterface& iface, const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;
        // ★ 立即置 cancel flag → SDK 下个 tick 看到 → movej 立即 return INTERRUPTED → worker 释放 BUSY
        iface.per_arm_should_cancel.store(true);
        if (iface.arm) {
            iface.arm->cancel_trajectory();
            iface.arm->clear_trajectory_buffer();
        }
        iface.status.store(0);
        publishCommandResult(iface.stop_result_pub, 0);
    }

    int stopArmInline(PerArmInterface& iface) {
        if (!iface.arm) return static_cast<int>(onero_api::MoveResult::EXECUTION_FAILED);
        iface.per_arm_should_cancel.store(true);
        int cancel_ret = iface.arm->cancel_trajectory();
        int clear_ret = iface.arm->clear_trajectory_buffer();
        iface.status.store(0);
        return (cancel_ret == 0 && clear_ret == 0)
            ? static_cast<int>(onero_api::MoveResult::SUCCESS)
            : static_cast<int>(onero_api::MoveResult::EXECUTION_FAILED);
    }

    void clearBufferCallback(
        PerArmInterface& iface,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        if (!iface.arm) {
            response->success = false;
            response->message = "Robot not initialized";
            return;
        }
        int ret = iface.arm->clear_trajectory_buffer();
        response->success = (ret == 0);
        response->message = response->success ? "Buffer cleared" : codeToMessage(ret);
    }

    void getEndPoseCallback(
        PerArmInterface& iface,
        const std::shared_ptr<onero_interfaces::srv::EndEffectorPose::Request> request,
        std::shared_ptr<onero_interfaces::srv::EndEffectorPose::Response> response) {
        (void)request;
        if (!iface.arm) {
            response->success = false;
            response->error_code = static_cast<int>(onero_api::MoveResult::EXECUTION_FAILED);
            response->message = "arm not initialized";
            return;
        }
        fillRosPose(iface.arm->get_end_effector_pose(), response->pose);
        response->success = true;
        response->error_code = 0;
        response->message = "";
    }

    // -----------------------------------------------------------------
    // 双臂调度辅助
    // -----------------------------------------------------------------
    double alignShortArmSpeed(
        onero_api::OneroArm& arm,
        const onero_api::JointArray& target,
        double speed_scale,
        double target_duration) {
        double hi = std::clamp(speed_scale == 0.0 ? 1.0 : speed_scale, 0.01, 5.0);
        double lo = 0.01;
        double best = hi;
        bool tol_hit = false;
        constexpr double tol = 5e-3;
        for (int i = 0; i < 20; ++i) {
            double mid = 0.5 * (lo + hi);
            double duration = arm.estimate_movej_duration(target, mid);
            if (!std::isfinite(duration) || duration < 0.0) {
                break;
            }
            if (std::abs(duration - target_duration) < tol) {
                best = mid;
                tol_hit = true;
                break;
            }
            if (duration < target_duration) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        return tol_hit ? best : hi;
    }

    int estimateFailureCode(double duration) const {
        if (std::isfinite(duration) && duration < 0.0) {
            return static_cast<int>(duration);
        }
        return static_cast<int>(onero_api::MoveResult::EXECUTION_FAILED);
    }

    // -----------------------------------------------------------------
    // MoveIt 轨迹执行
    // -----------------------------------------------------------------
    bool extractTrajectory(
        const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& points,
        size_t offset,
        size_t count,
        std::vector<onero_api::TrajectoryPoint>& out) {
        out.clear();
        out.reserve(points.size());
        for (const auto& point : points) {
            if (point.positions.size() < offset + count) {
                return false;
            }
            onero_api::TrajectoryPoint api_point;
            api_point.position.assign(point.positions.begin() + offset, point.positions.begin() + offset + count);
            if (point.velocities.size() >= offset + count) {
                api_point.velocity.assign(point.velocities.begin() + offset, point.velocities.begin() + offset + count);
            } else {
                api_point.velocity.assign(count, 0.0);
            }
            if (point.accelerations.size() >= offset + count) {
                api_point.acceleration.assign(point.accelerations.begin() + offset, point.accelerations.begin() + offset + count);
            } else {
                api_point.acceleration.assign(count, 0.0);
            }
            out.push_back(api_point);
        }
        return true;
    }

    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory_buffer_ = msg->points;
        is_executing_.store(false);
        RCLCPP_DEBUG(this->get_logger(), "Buffered MoveIt trajectory with %zu points", trajectory_buffer_.size());
    }

    void executeCallback(const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        if (is_executing_.exchange(true)) {
            RCLCPP_WARN(this->get_logger(), "Trajectory is already executing");
            return;
        }
        if (execution_thread_.joinable()) {
            execution_thread_.join();
        }
        moveit_should_cancel_.store(false);
        execution_thread_ = std::thread(&OneroDriverNode::executeTrajectory, this);
    }

    void executeTrajectory() {
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> local;
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            local = trajectory_buffer_;
        }

        if (local.empty()) {
            publishCommandResult(execution_result_pub_, static_cast<int>(onero_api::MoveResult::INVALID_PARAMS));
            is_executing_.store(false);
            return;
        }

        int result = static_cast<int>(onero_api::MoveResult::SUCCESS);
        std::string target;
        {
            std::lock_guard<std::mutex> lock(planning_arms_mutex_);
            target = planning_arms_status_;
        }

        // 注意：trajectory 执行直接进 SDK，需要拿对应臂的 command_mutex_
        if (driver_mode_ != "dual") {
            std::vector<onero_api::TrajectoryPoint> trajectory;
            if (!extractTrajectory(local, 0, kArmDof, trajectory) || arms_.empty() || !arms_[0]->arm) {
                result = static_cast<int>(onero_api::MoveResult::INVALID_PARAMS);
            } else {
                std::lock_guard<std::mutex> mu(arms_[0]->command_mutex);
                arms_[0]->per_arm_should_cancel.store(false);
                arms_[0]->arm->reset_stop_signal();
                result = arms_[0]->arm->send_trajectory(trajectory);
            }
        } else {
            if (target.empty()) {
                RCLCPP_WARN_ONCE(this->get_logger(),
                    "planning_arms_status not received yet, defaulting to dual; "
                    "ensure onero_control_dual is running before planning single-arm motions");
                target = "dual";
            }
            std::vector<onero_api::TrajectoryPoint> left_trajectory;
            std::vector<onero_api::TrajectoryPoint> right_trajectory;
            const size_t point_dof = local.front().positions.size();
            if (target == "left") {
                size_t offset = 0;
                if (!extractTrajectory(local, offset, kArmDof, left_trajectory)) {
                    result = static_cast<int>(onero_api::MoveResult::INVALID_PARAMS);
                } else {
                    std::lock_guard<std::mutex> mu(arms_[0]->command_mutex);
                    arms_[0]->per_arm_should_cancel.store(false);
                    arms_[0]->arm->reset_stop_signal();
                    result = arms_[0]->arm->send_trajectory(left_trajectory);
                }
            } else if (target == "right") {
                size_t offset = point_dof >= 14 ? kArmDof : 0;
                if (!extractTrajectory(local, offset, kArmDof, right_trajectory)) {
                    result = static_cast<int>(onero_api::MoveResult::INVALID_PARAMS);
                } else {
                    std::lock_guard<std::mutex> mu(arms_[1]->command_mutex);
                    arms_[1]->per_arm_should_cancel.store(false);
                    arms_[1]->arm->reset_stop_signal();
                    result = arms_[1]->arm->send_trajectory(right_trajectory);
                }
            } else if (target == "dual") {
                if (!extractTrajectory(local, 0, kArmDof, left_trajectory) ||
                    !extractTrajectory(local, kArmDof, kArmDof, right_trajectory)) {
                    result = static_cast<int>(onero_api::MoveResult::INVALID_PARAMS);
                } else {
                    auto left_future = std::async(std::launch::async, [&]() {
                        std::lock_guard<std::mutex> mu(arms_[0]->command_mutex);
                        arms_[0]->per_arm_should_cancel.store(false);
                        arms_[0]->arm->reset_stop_signal();
                        return arms_[0]->arm->send_trajectory(left_trajectory);
                    });
                    auto right_future = std::async(std::launch::async, [&]() {
                        std::lock_guard<std::mutex> mu(arms_[1]->command_mutex);
                        arms_[1]->per_arm_should_cancel.store(false);
                        arms_[1]->arm->reset_stop_signal();
                        return arms_[1]->arm->send_trajectory(right_trajectory);
                    });
                    int left_ret = left_future.get();
                    int right_ret = right_future.get();
                    result = left_ret != 0 ? left_ret : right_ret;
                }
            } else {
                result = static_cast<int>(onero_api::MoveResult::INVALID_PARAMS);
            }
        }

        publishCommandResult(execution_result_pub_, result);
        is_executing_.store(false);
    }

    void cancelCallback(const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        moveit_should_cancel_.store(true);
        if (driver_mode_ != "dual") {
            if (!arms_.empty()) stopArmInline(*arms_[0]);
            return;
        }
        std::string target;
        {
            std::lock_guard<std::mutex> lock(planning_arms_mutex_);
            target = planning_arms_status_;
        }
        if (target == "left") {
            stopArmInline(*arms_[0]);
        } else if (target == "right") {
            stopArmInline(*arms_[1]);
        } else {
            stopArmInline(*arms_[0]);
            stopArmInline(*arms_[1]);
        }
    }

    // -----------------------------------------------------------------
    // State timer：用 OneroArm::get_arm_state_cached()，零串口 IO，零阻塞
    // -----------------------------------------------------------------
    void publishArmState() {
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = this->now();
        joint_state.header.frame_id =
            (driver_mode_ == "dual") ? robot_model_ : (arms_.front()->robot_model + "_arm");

        for (auto& arm : arms_) {
            publishSingleArmState(*arm, joint_state);
        }

        if (!joint_state.name.empty()) {
            joint_state_pub_->publish(joint_state);
        }

        // 双臂同步夹爪状态：仅当两侧夹爪都存在且 dual_gripper_state_pub_ 已绑定才发。
        if (dual_gripper_state_pub_ && arms_.size() >= 2
            && arms_[0]->arm && arms_[0]->arm->has_gripper()
            && arms_[1]->arm && arms_[1]->arm->has_gripper()) {
            try {
                auto gl = arms_[0]->arm->gripper()->status();
                auto gr = arms_[1]->arm->gripper()->status();
                onero_interfaces::msg::DualGripper dg;
                dg.left_position    = static_cast<float>(gl.position);
                dg.left_velocity    = static_cast<float>(gl.velocity);
                dg.left_force       = static_cast<float>(gl.force);
                dg.left_error_code  = static_cast<int32_t>(gl.error);
                dg.right_position   = static_cast<float>(gr.position);
                dg.right_velocity   = static_cast<float>(gr.velocity);
                dg.right_force      = static_cast<float>(gr.force);
                dg.right_error_code = static_cast<int32_t>(gr.error);
                dual_gripper_state_pub_->publish(dg);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(),
                    "Error publishing dual gripper state: %s", e.what());
            }
        }
    }

    void publishGripperState(PerArmInterface& iface) {
        if (!iface.gripper_state_pub || !iface.arm || !iface.arm->has_gripper()) return;
        try {
            auto gs = iface.arm->gripper()->status();
            onero_interfaces::msg::Gripper g;
            g.position   = static_cast<float>(gs.position);
            g.velocity   = static_cast<float>(gs.velocity);
            g.force      = static_cast<float>(gs.force);
            g.error_code = static_cast<int32_t>(gs.error);
            iface.gripper_state_pub->publish(g);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(),
                "Error publishing gripper state for %s: %s",
                iface.robot_model.c_str(), e.what());
        }
    }

    void publishSingleArmState(PerArmInterface& iface, sensor_msgs::msg::JointState& joint_state) {
        if (!iface.arm) return;

        try {
            // ★ 用 cached 接口：SDK 控制环每拍刷新缓存，state timer 只读，零 CAN IO
            onero_api::ArmStateFromMotor state_snapshot = iface.arm->get_arm_state_cached();
            const onero_api::JointArray& positions  = state_snapshot.positions;
            const onero_api::JointArray& velocities = state_snapshot.velocities;
            const onero_api::JointArray& torques    = state_snapshot.torques;

            onero_interfaces::msg::ArmState state;
            state.header.stamp = this->now();
            state.header.frame_id =
                (driver_mode_ == "dual") ? (iface.sub_ns + "/base_link") : "base_link";
            state.robot_model = iface.robot_model;
            state.status = iface.status.load();
            state.error_code = 0;
            state.error_message = "";

            // 定长 7：缺位补 0
            state.joint_positions.fill(0.0f);
            state.joint_velocities.fill(0.0f);
            state.joint_torques.fill(0.0f);
            for (int i = 0; i < kArmDof; ++i) {
                if (static_cast<int>(positions.size()) > i)  state.joint_positions[i]  = static_cast<float>(positions[i]);
                if (static_cast<int>(velocities.size()) > i) state.joint_velocities[i] = static_cast<float>(velocities[i]);
                if (static_cast<int>(torques.size()) > i)    state.joint_torques[i]    = static_cast<float>(torques[i]);
            }
            fillRosPose(iface.arm->get_end_effector_pose_cached(), state.end_effector_pose);
            iface.arm_state_pub->publish(state);

            // 夹爪状态：只有该臂启用夹爪时才发布
            if (iface.gripper_state_pub) {
                publishGripperState(iface);
            }

            // /joint_states 用 joint_names 参数（fallback 见 makePerArm）
            for (int i = 0; i < kArmDof; ++i) {
                joint_state.name.push_back(iface.joint_names[i]);
                joint_state.position.push_back(static_cast<int>(positions.size()) > i ? positions[i] : 0.0);
                joint_state.velocity.push_back(static_cast<int>(velocities.size()) > i ? velocities[i] : 0.0);
                joint_state.effort.push_back(static_cast<int>(torques.size()) > i ? torques[i] : 0.0);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error publishing state for %s: %s",
                        iface.robot_model.c_str(), e.what());
        }
    }

    // -----------------------------------------------------------------
    // 成员
    // -----------------------------------------------------------------
    std::vector<std::unique_ptr<PerArmInterface>> arms_;

    std::string driver_mode_;
    std::string robot_model_;
    std::string mount_orientation_;
    std::string device_;
    std::string device_left_;
    std::string device_right_;
    int dof_{7};
    double state_pub_rate_{100.0};
    bool simulation_mode_{false};

    // 夹爪：enabled 单/双臂分开，左右独立；max_* 双臂共用一份
    bool gripper_enabled_single_{false};
    bool gripper_enabled_left_{false};
    bool gripper_enabled_right_{false};
    double gripper_max_vel_{100.0};
    double gripper_max_acc_{250.0};
    double gripper_max_jerk_{1000.0};

    // joint_names 配置（见 loadParams）
    std::vector<std::string> joint_names_single_;
    std::vector<std::string> joint_names_left_;
    std::vector<std::string> joint_names_right_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    rclcpp::Subscription<onero_interfaces::msg::DualMoveJ>::SharedPtr dual_movej_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dual_stop_sub_;
    rclcpp::Subscription<onero_interfaces::msg::DualGripper>::SharedPtr dual_gripper_sub_;
    rclcpp::Publisher<CommandResult>::SharedPtr dual_movej_result_pub_;
    rclcpp::Publisher<CommandResult>::SharedPtr dual_stop_result_pub_;
    rclcpp::Publisher<CommandResult>::SharedPtr dual_gripper_result_pub_;
    rclcpp::Publisher<onero_interfaces::msg::DualSyncState>::SharedPtr dual_sync_state_pub_;
    rclcpp::Publisher<onero_interfaces::msg::DualGripper>::SharedPtr dual_gripper_state_pub_;

    // 双臂调度线程
    std::thread dual_scheduler_;
    std::mutex dual_pending_mu_;
    std::condition_variable dual_pending_cv_;
    std::optional<DualJob> dual_pending_;
    std::atomic<bool> dual_shutdown_{false};

    // MoveIt 集成
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planning_arms_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr execute_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
    rclcpp::Publisher<CommandResult>::SharedPtr execution_result_pub_;

    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_buffer_;
    std::mutex trajectory_mutex_;
    std::atomic<bool> is_executing_{false};
    std::atomic<bool> moveit_should_cancel_{false};
    std::thread execution_thread_;

    std::string planning_arms_status_{"dual"};
    std::mutex planning_arms_mutex_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OneroDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
