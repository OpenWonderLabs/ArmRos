#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onero_interfaces.msg import CommandResult, MoveJ, MoveL, MoveP
import time
import threading
from geometry_msgs.msg import Pose, Point, Quaternion


class MoveApiAllDemo(Node):
    def __init__(self):
        super().__init__('move_api_all_demo')
        
        # 创建发布器
        self.movej_pub_ = self.create_publisher(MoveJ, '/onero_arm/movej', 10)
        self.movel_pub_ = self.create_publisher(MoveL, '/onero_arm/movel', 10)
        self.movep_pub_ = self.create_publisher(MoveP, '/onero_arm/movep', 10)
        
        # 创建结果订阅
        self.movej_result_sub_ = self.create_subscription(
            CommandResult, '/onero_arm/movej_result',
            lambda msg: self.result_callback('MoveJ', msg), 10)
        
        self.movel_result_sub_ = self.create_subscription(
            CommandResult, '/onero_arm/movel_result',
            lambda msg: self.result_callback('MoveL', msg), 10)
        
        self.movep_result_sub_ = self.create_subscription(
            CommandResult, '/onero_arm/movep_result',
            lambda msg: self.result_callback('MoveP', msg), 10)
        
        # 状态变量
        self.result_received_ = False
        self.current_api_ = ""
        
        self.get_logger().info("========================================")
        self.get_logger().info("Move API All Demo")
        self.get_logger().info("========================================")
        self.get_logger().info("This demo shows how to use all Move APIs sequentially:")
        self.get_logger().info("- MoveJ: Joint space motion")
        self.get_logger().info("- MoveL: Cartesian space motion")
        self.get_logger().info("- MoveP: Cartesian point-to-point motion")
        self.get_logger().info("MoveL/MoveP target pose is verified in A1-L and A1-R sim")
        self.get_logger().info("Each API will execute completely before the next one starts")
        self.get_logger().info("========================================")
    
    def result_callback(self, api_type, msg):
        """处理运动结果的回调函数"""
        if self.current_api_ == api_type:
            self.get_logger().info(
                f"✓ {api_type} result: {'SUCCESS' if msg.success else 'FAILED'} {msg.error_message}"
            )
            self.result_received_ = True
    
    def run(self):
        """运行演示"""
        time.sleep(1)
        
        # 第一步：MoveJ运动
        self.get_logger().info("\n[Step 1] Executing MoveJ motion...")
        self.get_logger().info("========================================")
        self.execute_movej()

        # 等待5秒确保稳定
        self.get_logger().info("Waiting 5s to settle...")
        time.sleep(15)

        # 第二步：MoveL运动
        self.get_logger().info("\n[Step 2] Executing MoveL motion...")
        self.get_logger().info("========================================")
        self.execute_movel()

        # 等待5秒确保稳定
        self.get_logger().info("Waiting 5s to settle...")
        time.sleep(15)

        # 第三步：MoveP运动
        self.get_logger().info("\n[Step 3] Executing MoveP motion...")
        self.get_logger().info("========================================")
        self.execute_movep()
        
        self.get_logger().info("\n========================================")
        self.get_logger().info("✓ Move API All Demo Completed!")
        self.get_logger().info("All Move APIs (MoveJ/MoveL/MoveP) executed sequentially.")
        self.get_logger().info("========================================")
    
    def execute_movej(self):
        """执行MoveJ运动"""
        self.get_logger().info("  Sending MoveJ command...")
        j1 = [0.20, 1.57, 0.3, 0.5, 0.0, 0.0, 0.0]
        self.current_api_ = "MoveJ"
        self.send_move_j(j1)
        self.wait_for_result(10.0)
    
    def execute_movel(self):
        """执行MoveL运动"""
        self.get_logger().info("  Sending MoveL command...")
        self.current_api_ = "MoveL"
        self.send_move_l(-0.4786, 0.2144, 0.2552)
        self.wait_for_result(15.0)
    
    def execute_movep(self):
        """执行MoveP运动"""
        self.get_logger().info("  Sending MoveP command...")
        self.current_api_ = "MoveP"
        self.send_move_p(-0.4786, 0.2144, 0.2552)
        self.wait_for_result(15.0)
    
    def send_move_j(self, joint_angles):
        """发送MoveJ命令"""
        msg = MoveJ()
        msg.joint_positions = joint_angles
        msg.speed_scale = 0.8  
        msg.trajectory_connect = 0  # 不使用轨迹连接，立即执行
        
        self.result_received_ = False
        self.movej_pub_.publish(msg)
    
    def send_move_l(self, x, y, z):
        """发送MoveL命令"""
        msg = MoveL()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = -0.6082
        msg.pose.orientation.y = -0.2328
        msg.pose.orientation.z = 0.3607
        msg.pose.orientation.w = 0.6677
        msg.speed_scale = 0.8  # 速度缩放
        msg.trajectory_connect = 0  # 不使用轨迹连接，立即执行
        
        self.result_received_ = False
        self.movel_pub_.publish(msg)
    
    def send_move_p(self, x, y, z):
        """发送MoveP命令"""
        msg = MoveP()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = -0.6082
        msg.pose.orientation.y = -0.2328
        msg.pose.orientation.z = 0.3607
        msg.pose.orientation.w = 0.6677
        msg.trajectory_connect = 0  # 不使用轨迹连接，立即执行
        msg.speed_scale = 0.8
        
        self.result_received_ = False
        self.movep_pub_.publish(msg)
    
    def wait_for_result(self, timeout_sec):
        """等待运动结果"""
        start_time = time.time()
        while not self.result_received_:
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
            
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn("Timeout waiting for result")
                break


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = MoveApiAllDemo()
    node.run()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
