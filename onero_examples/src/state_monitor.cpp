// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "onero_interfaces/msg/arm_state.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief 机械臂状态监控示例节点
 *
 * 在单臂 driver 启动后运行：
 *   ros2 launch onero_driver a1_l_driver.launch.py
 *   ros2 run onero_examples state_monitor
 *
 * 订阅 /onero_arm/arm_state，持续打印关节位置、速度、力矩和末端位姿。
 */
class StateMonitor: public rclcpp::Node
{
  public:
    StateMonitor();                                                                                   //构造函数
    void StateMonitor_Callback(const onero_interfaces::msg::ArmState & msg);                         //结果回调函数
  
  private:
    rclcpp::Subscription<onero_interfaces::msg::ArmState>::SharedPtr subscription_;                    //声明订阅器
};


/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void StateMonitor::StateMonitor_Callback(const onero_interfaces::msg::ArmState & msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    size_t dof = msg.joint_positions.size();
    
    // 打印关节位置
    if(dof == 7)
    {
      RCLCPP_INFO (this->get_logger(),"joint state is: [%lf, %lf, %lf, %lf, %lf, %lf, %lf]\n", 
                   msg.joint_positions[0], msg.joint_positions[1], msg.joint_positions[2], 
                   msg.joint_positions[3], msg.joint_positions[4], msg.joint_positions[5], 
                   msg.joint_positions[6]);
    }
    else if(dof == 6)
    {
      RCLCPP_INFO (this->get_logger(),"joint state is: [%lf, %lf, %lf, %lf, %lf, %lf]\n", 
                   msg.joint_positions[0], msg.joint_positions[1], msg.joint_positions[2], 
                   msg.joint_positions[3], msg.joint_positions[4], msg.joint_positions[5]);
    }
    
    // 打印关节速度（如果可用）
    if(!msg.joint_velocities.empty() && msg.joint_velocities.size() == dof)
    {
        if(dof == 7)
        {
            RCLCPP_INFO (this->get_logger(),"joint velocity is: [%lf, %lf, %lf, %lf, %lf, %lf, %lf]\n", 
                         msg.joint_velocities[0], msg.joint_velocities[1], msg.joint_velocities[2], 
                         msg.joint_velocities[3], msg.joint_velocities[4], msg.joint_velocities[5], 
                         msg.joint_velocities[6]);
        }
        else if(dof == 6)
        {
            RCLCPP_INFO (this->get_logger(),"joint velocity is: [%lf, %lf, %lf, %lf, %lf, %lf]\n", 
                         msg.joint_velocities[0], msg.joint_velocities[1], msg.joint_velocities[2], 
                         msg.joint_velocities[3], msg.joint_velocities[4], msg.joint_velocities[5]);
        }
    }
    
    // 打印关节力矩（如果可用）
    if(!msg.joint_torques.empty() && msg.joint_torques.size() == dof)
    {
        if(dof == 7)
        {
            RCLCPP_INFO (this->get_logger(),"joint torque is: [%lf, %lf, %lf, %lf, %lf, %lf, %lf]\n", 
                         msg.joint_torques[0], msg.joint_torques[1], msg.joint_torques[2], 
                         msg.joint_torques[3], msg.joint_torques[4], msg.joint_torques[5], 
                         msg.joint_torques[6]);
        }
        else if(dof == 6)
        {
            RCLCPP_INFO (this->get_logger(),"joint torque is: [%lf, %lf, %lf, %lf, %lf, %lf]\n", 
                         msg.joint_torques[0], msg.joint_torques[1], msg.joint_torques[2], 
                         msg.joint_torques[3], msg.joint_torques[4], msg.joint_torques[5]);
        }
    }
    
    // 将四元数转换为欧拉角
    tf2::Quaternion q(
        msg.end_effector_pose.orientation.x,
        msg.end_effector_pose.orientation.y,
        msg.end_effector_pose.orientation.z,
        msg.end_effector_pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    RCLCPP_INFO (this->get_logger(),"pose state is: [%lf, %lf, %lf, %lf, %lf, %lf]\n", 
                 msg.end_effector_pose.position.x, msg.end_effector_pose.position.y, 
                 msg.end_effector_pose.position.z, roll, pitch, yaw);
}   
/***********************************************end**************************************************/

/***********************************构造函数，初始化发布器订阅器****************************************/
StateMonitor::StateMonitor():rclcpp::Node("state_monitor")
{
  subscription_ = this->create_subscription<onero_interfaces::msg::ArmState>("/onero_arm/arm_state", rclcpp::ParametersQoS(), std::bind(&StateMonitor::StateMonitor_Callback, this,_1));
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}
/***********************************************end**************************************************/

/******************************************************主函数*********************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateMonitor>());
  rclcpp::shutdown();
  return 0;
}
