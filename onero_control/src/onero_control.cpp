// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include "onero_interfaces/msg/command_result.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>

#define DTOF 7

class MoveItPlannerNode : public rclcpp::Node
{
public:
  MoveItPlannerNode() : Node("moveit_planner_node")
  {
    // 声明参数
    this->declare_parameter<std::string>("planning_group", "arm");
    this->declare_parameter<double>("planning_time", 15.0); // 允许规划器搜索解决方案的最大时间，不能用于控制轨迹时间
    this->declare_parameter<int>("planning_attempts", 10);
    this->declare_parameter<double>("goal_tolerance", 0.0001);
    this->declare_parameter<int>("dof", DTOF);
    this->declare_parameter<double>("velocity_scaling", 0.05);       // 速度缩放因子   目前不生效，没有在moveit2的joint_limits.yaml中设置速度和加速度限制
    this->declare_parameter<double>("acceleration_scaling", 0.05);   // 加速度缩放因子 目前不生效，没有在moveit2的joint_limits.yaml中设置速度和加速度限制
    this->declare_parameter<bool>("enable_rviz_integration", true);  // 是否启用RViz集成模式

    // 获取参数
    planning_group_ = this->get_parameter("planning_group").as_string();
    planning_time_ = this->get_parameter("planning_time").as_double();
    planning_attempts_ = this->get_parameter("planning_attempts").as_int();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    dof_ = this->get_parameter("dof").as_int();
    velocity_scaling_ = this->get_parameter("velocity_scaling").as_double();
    acceleration_scaling_ = this->get_parameter("acceleration_scaling").as_double();
    enable_rviz_integration_ = this->get_parameter("enable_rviz_integration").as_bool();

    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt2 planning node...");

    // 状态发布器
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/onero_arm/moveit/planner_status", 10);

   // 发布完整轨迹给driver
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/onero_arm/moveit/trajectory", 10);
    
    // 发布执行命令
    execute_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "/onero_arm/moveit/execute", 10);
    
    // 发布取消命令
    cancel_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "/onero_arm/moveit/cancel", 10);

    if (enable_rviz_integration_)
    {
      // RViz集成模式：订阅move_group的规划结果

      // 订阅move_group发布的规划轨迹
      display_planned_path_sub_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
          "/display_planned_path", 10,
          std::bind(&MoveItPlannerNode::displayPlannedPathCallback, this, std::placeholders::_1));

      // 发布离散后的轨迹供RViz显示
      display_trajectory_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
          "/onero_arm/moveit/display_trajectory", 10);
    }
    else
    {
      // 直接接收目标关节角度

      // 通过 Float64MultiArray 直接接收关节角度
      goal_joint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/onero_arm/moveit/goal_joints", 10,
          std::bind(&MoveItPlannerNode::goalJointCallback, this, std::placeholders::_1));

      // 通过 JSON 字符串接收
      goal_json_sub_ = this->create_subscription<std_msgs::msg::String>(
          "/onero_arm/moveit/goal_json", 10,
          std::bind(&MoveItPlannerNode::goalJsonCallback, this, std::placeholders::_1));

      // 接收driver端轨迹执行的结果反馈
      execution_result_sub_ = this->create_subscription<onero_interfaces::msg::CommandResult>(
          "/onero_arm/moveit/trajectory_execution_result", 10,
          std::bind(&MoveItPlannerNode::executionResultCallback, this, std::placeholders::_1));
    }

    // 获取当前位置
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&MoveItPlannerNode::jointStateCallback, this, std::placeholders::_1));
  }

  // 初始化 MoveIt2
  void initialize()
  {
    try
    {
      rclcpp::Node::SharedPtr node_ptr = shared_from_this();
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          node_ptr, planning_group_);

      // 配置规划器
      move_group_->setPlanningTime(planning_time_);
      move_group_->setNumPlanningAttempts(planning_attempts_);
      move_group_->setGoalJointTolerance(goal_tolerance_);
      move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
      move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

      // 获取关节名称
      joint_names_ = move_group_->getJointNames();

      RCLCPP_INFO(this->get_logger(), "MoveIt2 initialized successfully!");

      if (enable_rviz_integration_)
      {
        // 创建 FollowJointTrajectory Action Server
        using namespace std::placeholders;
        trajectory_action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
            this,
            "/arm_controller/follow_joint_trajectory",
            std::bind(&MoveItPlannerNode::handleGoal, this, _1, _2),
            std::bind(&MoveItPlannerNode::handleCancel, this, _1),
            std::bind(&MoveItPlannerNode::handleAccepted, this, _1));
      }

      moveit_initialized_ = true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "MoveIt2 initialization failed: %s", e.what());
      RCLCPP_ERROR(this->get_logger(), "Please ensure the move_group node has been started");
      moveit_initialized_ = false;
    }
  }

private:
  // 关节状态回调
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    current_joint_state_ = msg;
  }

  // 目标关节角度回调（Float64MultiArray格式）
  void goalJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!moveit_initialized_)
    {
      RCLCPP_WARN(this->get_logger(), "MoveIt2 not yet initialized, please wait");
      return;
    }

    if (is_planning_)
    {
      RCLCPP_WARN(this->get_logger(), "Planning in progress, ignoring new goal");
      return;
    }

    if (msg->data.size() != static_cast<size_t>(dof_))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Target joint count mismatch! Expected %d, received %zu",
                   dof_, msg->data.size());
      return;
    }

    std::vector<double> target_joints(msg->data.begin(), msg->data.end());

    RCLCPP_INFO(this->get_logger(), "Received target joint angles:");
    for (size_t i = 0; i < target_joints.size(); ++i)
    {
      RCLCPP_INFO(this->get_logger(), "  joint%zu: %.3f rad", i + 1, target_joints[i]);
    }

    // 在新线程中执行规划和执行
    std::thread([this, target_joints]()
                { planAndExecute(target_joints); })
        .detach();
  }

  // JSON 格式目标回调
  void goalJsonCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!moveit_initialized_)
    {
      RCLCPP_WARN(this->get_logger(), "MoveIt2 not yet initialized, please wait");
      return;
    }

    if (is_planning_)
    {
      RCLCPP_WARN(this->get_logger(), "Planning in progress, ignoring new goal");
      return;
    }

    try
    {
      auto j = nlohmann::json::parse(msg->data);

      // 支持两种格式：
      // 1. {"joints": [0.0, 0.5, -1.0, 0.0, 0.5, 0.0, 0.0]}
      // 2. {"cmd": "movej", "values": [0.0, 0.5, -1.0, 0.0, 0.5, 0.0, 0.0]}

      std::vector<double> target_joints;

      if (j.contains("joints"))
      {
        target_joints = j["joints"].get<std::vector<double>>();
      }
      else if (j.contains("values"))
      {
        target_joints = j["values"].get<std::vector<double>>();
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Invalid JSON format: missing 'joints' or 'values' field");
        return;
      }

      if (target_joints.size() != static_cast<size_t>(dof_))
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Target joint count mismatch! Expected %d, received %zu",
                     dof_, target_joints.size());
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Received target joint angles via JSON");

      // 在新线程中执行规划和执行
      std::thread([this, target_joints]()
                  { planAndExecute(target_joints); })
          .detach();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "JSON parse failed: %s", e.what());
    }
  }

  // 规划并执行轨迹
  void planAndExecute(const std::vector<double> &target_joints)
  {
    is_planning_ = true;
    publishStatus("planning");

    try
    {
      RCLCPP_INFO(this->get_logger(), "Starting trajectory planning...");

      // 重新设置速度和加速度缩放因子（确保每次规划都生效）
      move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
      move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

      // 将规划起点同步为最新的机械臂状态，避免起点与真实位置不一致
      {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        if (current_joint_state_ &&
            current_joint_state_->name.size() == current_joint_state_->position.size() &&
            !current_joint_state_->name.empty())
        {
          moveit::core::RobotState start_state(move_group_->getRobotModel());
          start_state.setVariablePositions(current_joint_state_->name,
                                           current_joint_state_->position);
          move_group_->setStartState(start_state);
        }
        else
        {
          move_group_->setStartStateToCurrentState();
          RCLCPP_WARN(this->get_logger(), "Current joint state unavailable, using MoveIt2 internal state as start point");
        }
      }

      // 设置目标关节角度
      move_group_->setJointValueTarget(target_joints);

      // 执行规划
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto result = move_group_->plan(plan);

      if (result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        publishStatus("planning_failed");
        is_planning_ = false;
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Planning succeeded! Starting interpolation and execution...");

      RCLCPP_INFO(this->get_logger(), "Planned trajectory points: %zu", plan.trajectory_.joint_trajectory.points.size());

      // 获取规划的轨迹
      const auto &trajectory = plan.trajectory_.joint_trajectory;

      // 清空之前的轨迹
      discretized_positions_.clear();
      discretized_velocities_.clear();
      discretized_accelerations_.clear();
      discretized_timestamps_.clear();
      discretized_joint_names_ = trajectory.joint_names;
      
      // 获取关节映射（确保关节顺序正确）
      std::vector<int> joint_indices = getJointIndices(trajectory.joint_names);

       // 直接复制轨迹点数据（不进行插值）
      for (const auto& point : trajectory.points)
      {
        // 按照正确的关节顺序重新排列
        std::vector<double> ordered_positions(dof_);
        std::vector<double> ordered_velocities(dof_);
        std::vector<double> ordered_accelerations(dof_);

        for (size_t i = 0; i < joint_indices.size(); ++i)
        {
          if (joint_indices[i] >= 0 && joint_indices[i] < dof_)
          {
            ordered_positions[joint_indices[i]] = point.positions[i];
            ordered_velocities[joint_indices[i]] = point.velocities[i];
            ordered_accelerations[joint_indices[i]] = point.accelerations[i];
          }
        }

        discretized_positions_.push_back(ordered_positions);
        discretized_velocities_.push_back(ordered_velocities);
        discretized_accelerations_.push_back(ordered_accelerations);
        discretized_timestamps_.push_back(rclcpp::Duration(point.time_from_start).seconds());
      }

      // 构建完整轨迹消息
      auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
      trajectory_msg.header.stamp = this->now();
      trajectory_msg.joint_names = discretized_joint_names_;
      
      for (size_t i = 0; i < discretized_positions_.size(); ++i)
      {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = discretized_positions_[i];
        point.velocities = discretized_velocities_[i];
        point.accelerations = discretized_accelerations_[i];
        point.time_from_start = rclcpp::Duration::from_seconds(discretized_timestamps_[i]);
        trajectory_msg.points.push_back(point);
      }
      
      // 发布完整轨迹给driver
      trajectory_pub_->publish(trajectory_msg);
      
      RCLCPP_INFO(this->get_logger(), "Trajectory sent to driver: %zu points", discretized_positions_.size());
      
      // 等待轨迹消息被driver接收和处理
      double wait_time_ms = 50.0 + discretized_positions_.size() * 0.01;  // 基础50ms + 每个点0.01ms
      wait_time_ms = std::min(wait_time_ms, 200.0);  // 最多等待200ms
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(wait_time_ms)));
      
      RCLCPP_INFO(this->get_logger(), "Waiting %.0f ms to ensure driver received trajectory", wait_time_ms);
      
      // 发送执行命令
      auto execute_msg = std_msgs::msg::Empty();
      execute_pub_->publish(execute_msg);

      // 等待执行结果反馈
      RCLCPP_INFO(this->get_logger(), "Waiting for trajectory execution result...");
      publishStatus("executing");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error during planning or execution: %s", e.what());
      publishStatus("error");
    }

    is_planning_ = false;
  }


  // 接收轨迹执行结果回调
  void executionResultCallback(const onero_interfaces::msg::CommandResult::SharedPtr msg)
  {
    if (msg->success)
    {
      RCLCPP_INFO(this->get_logger(), "Trajectory execution succeeded!");
      publishStatus("execution_success");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed: %s", msg->error_message.c_str());
      publishStatus("execution_failed");
    }
  }


  // 获取关节索引映射
  std::vector<int> getJointIndices(const std::vector<std::string> &traj_joint_names)
  {
    std::vector<int> indices(traj_joint_names.size(), -1);

    for (size_t i = 0; i < traj_joint_names.size(); ++i)
    {
      for (size_t j = 0; j < joint_names_.size(); ++j)
      {
        if (traj_joint_names[i] == joint_names_[j])
        {
          indices[i] = static_cast<int>(j);
          break;
        }
      }
    }

    return indices;
  }

  // 发布状态信息
  void publishStatus(const std::string &status)
  {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    status_pub_->publish(msg);
  }

  // RViz集成模式：处理move_group的规划结果
  void displayPlannedPathCallback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
  {
    if (msg->trajectory.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
      return;
    }

    // 获取第一条轨迹
    const auto &trajectory = msg->trajectory[0].joint_trajectory;

    RCLCPP_INFO(this->get_logger(), "Received MoveIt2 planned trajectory, points: %zu", trajectory.points.size());
    
    if (trajectory.points.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Trajectory is empty");
      return;
    }

    // 清空之前的轨迹
      discretized_positions_.clear();
      discretized_velocities_.clear();
      discretized_accelerations_.clear();
      discretized_timestamps_.clear();
      discretized_joint_names_ = trajectory.joint_names;
      
      // 获取关节映射（确保关节顺序正确）
      std::vector<int> joint_indices = getJointIndices(trajectory.joint_names);

       // 直接复制轨迹点数据（不进行插值）
      for (const auto& point : trajectory.points)
      {
        // 按照正确的关节顺序重新排列
        std::vector<double> ordered_positions(dof_);
        std::vector<double> ordered_velocities(dof_);
        std::vector<double> ordered_accelerations(dof_);

        for (size_t i = 0; i < joint_indices.size(); ++i)
        {
          if (joint_indices[i] >= 0 && joint_indices[i] < dof_)
          {
            ordered_positions[joint_indices[i]] = point.positions[i];
            ordered_velocities[joint_indices[i]] = point.velocities[i];
            ordered_accelerations[joint_indices[i]] = point.accelerations[i];
          }
        }

        discretized_positions_.push_back(ordered_positions);
        discretized_velocities_.push_back(ordered_velocities);
        discretized_accelerations_.push_back(ordered_accelerations);
        discretized_timestamps_.push_back(rclcpp::Duration(point.time_from_start).seconds());
      }

    RCLCPP_INFO(this->get_logger(), "Trajectory data ready: %zu points. Timestamps corrected.", discretized_positions_.size());

    // 发布离散后的轨迹供RViz显示
    publishDiscretizedTrajectory(msg->model_id);

    RCLCPP_INFO(this->get_logger(), "Click 'Execute' to run the trajectory");
  }

  // 发布离散后的轨迹供RViz显示
  void publishDiscretizedTrajectory(const std::string &model_id)
  {
    if (discretized_positions_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No discretized trajectory available to publish");
      return;
    }

    // 构建 DisplayTrajectory 消息
    auto display_msg = moveit_msgs::msg::DisplayTrajectory();
    display_msg.model_id = model_id;

    // 创建 RobotTrajectory
    moveit_msgs::msg::RobotTrajectory robot_trajectory;
    robot_trajectory.joint_trajectory.joint_names = discretized_joint_names_;

    // 添加所有离散点
    for (size_t i = 0; i < discretized_positions_.size(); ++i)
    {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = discretized_positions_[i];
      point.velocities = discretized_velocities_[i];
      point.time_from_start = rclcpp::Duration::from_seconds(discretized_timestamps_[i]);

      robot_trajectory.joint_trajectory.points.push_back(point);
    }

    display_msg.trajectory.push_back(robot_trajectory);

    // 发布
    display_trajectory_pub_->publish(display_msg);
  }

  // Action Server 回调函数
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received trajectory execution request");

    if (discretized_positions_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "No executable trajectory available! Please click Plan first");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (is_executing_)
    {
      RCLCPP_WARN(this->get_logger(), "Trajectory execution in progress, rejecting new request");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory execution request accepted");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Cancellation request received; notifying driver to stop immediately");
    
    // 立即发送取消命令给driver
    auto cancel_msg = std_msgs::msg::Empty();
    cancel_pub_->publish(cancel_msg);

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
  {
    // 在新线程中执行轨迹
    std::thread{std::bind(&MoveItPlannerNode::executeDiscretizedTrajectory, this, goal_handle)}.detach();
  }

  // 执行离散后的轨迹
  void executeDiscretizedTrajectory(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
  {
    is_executing_ = true;
    publishStatus("executing");

    if (discretized_timestamps_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Discretized trajectory is empty!");
      auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
      result->error_code = control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL;
      goal_handle->abort(result);
      is_executing_ = false;
      return;
    }

    // 构建完整轨迹消息
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    trajectory_msg.header.stamp = this->now();
    trajectory_msg.joint_names = discretized_joint_names_;
    
    for (size_t i = 0; i < discretized_positions_.size(); ++i)
    {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = discretized_positions_[i];
      point.velocities = discretized_velocities_[i];
      point.accelerations = discretized_accelerations_[i];
      point.time_from_start = rclcpp::Duration::from_seconds(discretized_timestamps_[i]);
      trajectory_msg.points.push_back(point);
    }
    
    // 发布完整轨迹给driver
    trajectory_pub_->publish(trajectory_msg);
    
    RCLCPP_INFO(this->get_logger(), "Trajectory sent to driver: %zu points", discretized_positions_.size());
    
    // 等待轨迹消息被driver接收和处理（给予足够时间）
    // 对于大轨迹（如1000个点），需要更多时间
    double wait_time_ms = 50.0 + discretized_positions_.size() * 0.01;  // 基础50ms + 每个点0.01ms
    wait_time_ms = std::min(wait_time_ms, 200.0);  // 最多等待200ms
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(wait_time_ms)));
    
    RCLCPP_INFO(this->get_logger(), "Waiting %.0f ms to ensure driver received trajectory", wait_time_ms);
    
    // 发送执行命令
    auto execute_msg = std_msgs::msg::Empty();
    execute_pub_->publish(execute_msg);
    
    // 预期执行时间
    double total_execution_time = discretized_timestamps_.back();
    RCLCPP_INFO(this->get_logger(), "Expected execution time: %.2f s", total_execution_time);

    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    auto feedback = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();

    auto start_time = this->now();
    size_t feedback_counter = 0;

    // 监控执行状态（以10Hz轮询driver状态）
    rclcpp::Rate rate(10);  // 10Hz监控频率
    
    while (rclcpp::ok() && is_executing_)
    {
      // 检查是否被取消
      if (goal_handle->is_canceling())
      {
        RCLCPP_INFO(this->get_logger(), "Cancellation request detected, aborting execution...");
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED; // Or another appropriate code
        goal_handle->canceled(result);
        is_executing_ = false;
        publishStatus("canceled");
        return;
      }
      
      // 计算当前预期进度（基于时间）
      double elapsed = (this->now() - start_time).seconds();
      
      // 发布反馈（模拟进度）
      if (feedback_counter % 5 == 0)  // 每0.5秒发布一次反馈
      {
        // 根据时间估算当前应该在哪个轨迹点
        size_t estimated_index = 0;
        for (size_t i = 0; i < discretized_timestamps_.size(); ++i)
        {
          if (discretized_timestamps_[i] <= elapsed)
          {
            estimated_index = i;
          }
          else
          {
            break;
          }
        }
        
        if (estimated_index < discretized_positions_.size())
        {
          feedback->desired.positions = discretized_positions_[estimated_index];
          feedback->desired.velocities = discretized_velocities_[estimated_index];
          feedback->desired.accelerations = discretized_accelerations_[estimated_index];
          feedback->desired.time_from_start = rclcpp::Duration::from_seconds(elapsed);
          
          // 如果有实际位置，也可以填充actual字段
          std::lock_guard<std::mutex> lock(joint_state_mutex_);
          if (current_joint_state_)
          {
            feedback->actual.positions = current_joint_state_->position;
            feedback->actual.velocities = current_joint_state_->velocity;
          }
          
          goal_handle->publish_feedback(feedback);
        }
      }
      
      feedback_counter++;
      
      // 检查是否执行完成（时间超过预期执行时间+缓冲）
      if (elapsed > total_execution_time + 0.5)  // 额外0.5秒缓冲
      {
        RCLCPP_INFO(this->get_logger(), "Trajectory execution complete!");
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
        goal_handle->succeed(result);
        is_executing_ = false;
        publishStatus("success");
        return;
      }
      
      rate.sleep();
    }

  }

  // 成员变量
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_pub_;

  // 方案C：发布完整轨迹和控制命令
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr execute_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr goal_joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_json_sub_;
   rclcpp::Subscription<onero_interfaces::msg::CommandResult>::SharedPtr execution_result_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_planned_path_sub_;

  // Action Server
  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_server_;

  std::string planning_group_;
  double planning_time_;
  int planning_attempts_;
  double goal_tolerance_;
  int dof_;
  double velocity_scaling_;
  double acceleration_scaling_;
  bool enable_rviz_integration_;

  std::vector<std::string> joint_names_;
  sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
  std::mutex joint_state_mutex_;

  // 离散化后的轨迹数据
  std::vector<std::vector<double>> discretized_positions_;
  std::vector<std::vector<double>> discretized_velocities_;
  std::vector<std::vector<double>> discretized_accelerations_;
  std::vector<double> discretized_timestamps_;
  std::vector<std::string> discretized_joint_names_;

  std::atomic<bool> is_planning_{false};
  std::atomic<bool> is_executing_{false};
  std::atomic<bool> moveit_initialized_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveItPlannerNode>();

  // 使用多线程执行器以支持并发回调
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // 在后台线程中初始化 MoveIt2
  std::thread init_thread([node]()
                          {
    // 等待 robot_description 参数和 move_group 启动
    std::this_thread::sleep_for(std::chrono::seconds(5));
    node->initialize(); });
  init_thread.detach();

  RCLCPP_INFO(node->get_logger(), "MoveIt2 planning node started successfully!");
  RCLCPP_INFO(node->get_logger(), "Initializing MoveIt2 connection...");

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
