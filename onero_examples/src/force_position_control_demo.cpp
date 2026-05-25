// SPDX-License-Identifier: MIT
// Copyright (c) 2026 OneRobotics (Shenzhen) Co., Ltd.

//
// Created by ubuntu on 24-7-11.
//
// #include <iostream>
// #include <chrono>
// #include <functional>
// #include <memory>
// #include <unistd.h>
// #include <thread>
// #include "rclcpp/rclcpp.hpp"
// #include "rm_ros_interfaces/msg/movejp.hpp"
// #include "rm_ros_interfaces/msg/movel.hpp"
// #include "rm_ros_interfaces/msg/setforceposition.hpp"
// #include "rm_ros_interfaces/msg/movec.hpp"
// #include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "onero_interfaces/msg/move_l.hpp"
#include "onero_interfaces/msg/set_force_position.hpp"
#include "onero_interfaces/msg/move_p.hpp"
#include "std_msgs/msg/bool.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
bool set_force_postion_state = false;
bool movej_p_state = false;
bool movel_state = false;
bool stop_force_postion_state = false;
bool first_run = true;
/****************************************创建类************************************/ 
class ForcePositionControlDemoSub: public rclcpp::Node
{
  public:
    ForcePositionControlDemoSub();                                                                         //构造函数
    void ForcePositionControl_demo();                                                                   //力位混合运动规划函数
    void MoveJPDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                                 //结果回调函数
    void SetForcePostionDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                        //结果回调函数
    void MoveLDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                                  //结果回调函数
    void StopForcePostionDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                       //结果回调函数
    
  private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movej_p_subscription_;                         //声明订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movej_subscription_;                         //声明订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movel_subscription_;                           //声明订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr set_force_postion_subscription_;               //声明订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_force_postion_subscription_;              //声明订阅器
       
};

class ForcePositionControlDemoPub: public rclcpp::Node
{
  public:
    ForcePositionControlDemoPub();                                                                      //构造函数
    void ForcePositionControl_demo();                                                                   //力位混合运动规划函数
    void looppub_timer_callback();                                                                      //move运动规划函数
  private:
    rclcpp::Publisher<onero_interfaces::msg::MoveL>::SharedPtr force_position_movel_pub_;
    rclcpp::Publisher<onero_interfaces::msg::MoveP>::SharedPtr movep_pub_;
    rclcpp::Publisher<onero_interfaces::msg::SetForcePosition>::SharedPtr set_force_postion_publisher_;//声明发布器
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_force_postion_publisher_;                    //声明发布器
    rclcpp::TimerBase::SharedPtr loop_pub_Timer;                                                        //定时发布器
};

/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void ForcePositionControlDemoSub::MoveJPDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    movej_p_state = true;
    if(msg->data)
    {
        RCLCPP_INFO (this->get_logger(),"Movej_p succeeded\n");
    } else {
        RCLCPP_ERROR (this->get_logger(),"Movej_p Failed\n");
    }
}   
/***********************************************end**************************************************/


/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void ForcePositionControlDemoSub::MoveLDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    movel_state = msg->data;
    if(msg->data)
    {
    RCLCPP_INFO (this->get_logger(),"ForcePositionMoveL succeeded\n");
    } else {
        RCLCPP_ERROR (this->get_logger(),"ForcePositionMoveL Failed\n");
    }
}   
/***********************************************end**************************************************/

/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void ForcePositionControlDemoSub::SetForcePostionDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    set_force_postion_state = msg->data;
    if(msg->data)
    {
        RCLCPP_INFO (this->get_logger(),"Set Force Postion succeeded\n");
    } else {
        RCLCPP_ERROR (this->get_logger(),"Set Force Postion Failed\n");
    }
}   
/***********************************************end**************************************************/

/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void ForcePositionControlDemoSub::StopForcePostionDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    stop_force_postion_state = true;
    if(msg->data)
    {
        RCLCPP_INFO (this->get_logger(),"Stop Force Postion succeeded\n");
    } else {
        RCLCPP_ERROR (this->get_logger(),"Stop Force Postion Failed\n");
    }
}   
/***********************************************end**************************************************/


/*******************************************力位混合运动函数****************************************/
void ForcePositionControlDemoPub::looppub_timer_callback()
{
  //moveJP到达指定位置 
  if(first_run ==true)
  {
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Sending MoveP command...");
    
    // 构造MoveP消息
    auto msg = onero_interfaces::msg::MoveP();
    
    // 设置目标位姿
    //目标位姿x,y,z
    msg.pose.position.x = 0.004;  
    msg.pose.position.y = 0.399;    
    msg.pose.position.z = 0.377;   
    //四元数w,x,y,z
    msg.pose.orientation.w = -0.498;
    msg.pose.orientation.x = 0.504;
    msg.pose.orientation.y = -0.503;
    msg.pose.orientation.z = -0.495;
    
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
    first_run = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));       
  }
  //开启力位混合 
  if(movej_p_state==true)   //等待moveJ_P到达
  { 
    onero_interfaces::msg::SetForcePosition forceposition_data;
    forceposition_data.direction = 2;
    forceposition_data.force = 0;
    this->set_force_postion_publisher_->publish(forceposition_data);
    movej_p_state = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  //moveL运动 
  if(set_force_postion_state==true)
  { 
           
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Sending ForcePositionMoveL command...");

    auto msg = onero_interfaces::msg::MoveL();

     //目标位姿x,y,z
    msg.pose.position.x = - 0.18;  
    msg.pose.position.y = 0.399;    
    msg.pose.position.z = 0.377;   
    //四元数w,x,y,z
    msg.pose.orientation.w = -0.498;
    msg.pose.orientation.x = 0.504;
    msg.pose.orientation.y = -0.503;
    msg.pose.orientation.z = -0.495;

    msg.speed_scale = 0.8;
    msg.trajectory_connect = 0;

    RCLCPP_INFO(this->get_logger(), "Target pose:");
    RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f] m",
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "  Orientation (quaternion): [%.3f, %.3f, %.3f, %.3f]",
                msg.pose.orientation.w, msg.pose.orientation.x,
                msg.pose.orientation.y, msg.pose.orientation.z);
    RCLCPP_INFO(this->get_logger(), "Speed Scale: %.3f", msg.speed_scale);

    force_position_movel_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "ForcePositionMoveL command sent successfully");
    RCLCPP_INFO(this->get_logger(), "Waiting for result...");
    set_force_postion_state = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  //停止力位混合 
  if(movel_state==true)                     //等待movel到达
  {
    std_msgs::msg::Bool stop_force_postion_data;
    stop_force_postion_data.data = true;
    this->stop_force_postion_publisher_->publish(stop_force_postion_data);
    movel_state = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if(stop_force_postion_state==true)
  {
    RCLCPP_INFO (this->get_logger(),"All step run over\n");
    stop_force_postion_state = false;
  }
}
/***********************************************end**************************************************/

/***********************************构造函数，初始化发布器订阅器****************************************/
ForcePositionControlDemoPub::ForcePositionControlDemoPub():rclcpp::Node("Force_Position_Control_pub_node")
{
  movep_pub_ = this->create_publisher<onero_interfaces::msg::MoveP>("/onero_driver/movep_cmd", 10);
  force_position_movel_pub_ = this->create_publisher<onero_interfaces::msg::MoveL>("/onero_driver/force_position_movel_cmd", rclcpp::ParametersQoS());
  set_force_postion_publisher_ = this->create_publisher<onero_interfaces::msg::SetForcePosition>("/onero_driver/force_position_control_cmd", rclcpp::ParametersQoS());
  stop_force_postion_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/onero_driver/stop_force_postion_cmd", rclcpp::ParametersQoS());
  loop_pub_Timer = this->create_wall_timer(std::chrono::milliseconds(100), 
        std::bind(&ForcePositionControlDemoPub::looppub_timer_callback,this));
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}
/***********************************************end**************************************************/

/***********************************构造函数，初始化发布器订阅器****************************************/
ForcePositionControlDemoSub::ForcePositionControlDemoSub():rclcpp::Node("Force_Position_Control_sub_node")
{
  movej_p_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/onero_driver/movep_result", rclcpp::ParametersQoS(), std::bind(&ForcePositionControlDemoSub::MoveJPDemo_Callback, this,_1));
  movel_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/onero_driver/force_position_movel_result", rclcpp::ParametersQoS(), std::bind(&ForcePositionControlDemoSub::MoveLDemo_Callback, this,_1));
  set_force_postion_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/onero_driver/force_position_control_result", rclcpp::ParametersQoS(), std::bind(&ForcePositionControlDemoSub::SetForcePostionDemo_Callback, this,_1));
  stop_force_postion_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/onero_driver/stop_force_position_control_result", rclcpp::ParametersQoS(), std::bind(&ForcePositionControlDemoSub::StopForcePostionDemo_Callback, this,_1));
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}
/***********************************************end**************************************************/

/******************************************************主函数*********************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node_sub = std::make_shared<ForcePositionControlDemoSub>();
  auto node_pub = std::make_shared<ForcePositionControlDemoPub>();
  executor.add_node(node_pub);
  executor.add_node(node_sub);
  
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
