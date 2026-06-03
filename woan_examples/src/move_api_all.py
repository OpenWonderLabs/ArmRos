#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from woan_interfaces.msg import MoveJ, MoveL, MoveP
from std_msgs.msg import Bool
import time
import threading
from geometry_msgs.msg import Pose, Point, Quaternion


class MoveApiAllDemo(Node):
    def __init__(self):
        super().__init__('move_api_all_demo')
        
        # 创建发布器
        self.movej_pub_ = self.create_publisher(MoveJ, '/woan_driver/movej_cmd', 10)
        self.movel_pub_ = self.create_publisher(MoveL, '/woan_driver/movel_cmd', 10)
        self.movep_pub_ = self.create_publisher(MoveP, '/woan_driver/movep_cmd', 10)
        
        # 创建结果订阅
        self.movej_result_sub_ = self.create_subscription(
            Bool, '/woan_driver/movej_result', 
            lambda msg: self.result_callback('MoveJ', msg), 10)
        
        self.movel_result_sub_ = self.create_subscription(
            Bool, '/woan_driver/movel_result', 
            lambda msg: self.result_callback('MoveL', msg), 10)
        
        self.movep_result_sub_ = self.create_subscription(
            Bool, '/woan_driver/movep_result', 
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
        self.get_logger().info("- MoveP: Pose transmission")
        self.get_logger().info("Each API will execute completely before the next one starts")
        self.get_logger().info("========================================")
    
    def result_callback(self, api_type, msg):
        """处理运动结果的回调函数"""
        if self.current_api_ == api_type:
            self.get_logger().info(f"✓ {api_type} result: {'SUCCESS' if msg.data else 'FAILED'}")
            self.result_received_ = True
    
    def run(self):
        """运行演示"""
        time.sleep(1)
        
        # 第一步：MoveJ运动
        self.get_logger().info("\n[步骤1] 执行MoveJ运动...")
        self.get_logger().info("========================================")
        self.execute_movej()
        
        # 等待5秒确保稳定
        self.get_logger().info("等待5秒确保稳定...")
        time.sleep(15)
        
        # 第二步：MoveL运动
        self.get_logger().info("\n[步骤2] 执行MoveL运动...")
        self.get_logger().info("========================================")
        self.execute_movel()
        
        # 等待5秒确保稳定
        self.get_logger().info("等待5秒确保稳定...")
        time.sleep(15)
        
        # 第三步：MoveP运动
        self.get_logger().info("\n[步骤3] 执行MoveP运动...")
        self.get_logger().info("========================================")
        self.execute_movep()
        
        self.get_logger().info("\n========================================")
        self.get_logger().info("✓ Move API All Demo Completed!")
        self.get_logger().info("All Move APIs (MoveJ/MoveL/MoveP) executed sequentially.")
        self.get_logger().info("========================================")
    
    def execute_movej(self):
        """执行MoveJ运动"""
        self.get_logger().info("  发送MoveJ命令...")
        j1 = [0.20, 1.57, 0.3, 0.5, 0.0, 0.0, 0.0]
        self.current_api_ = "MoveJ"
        self.send_move_j(j1)
        self.wait_for_result(10.0)
    
    def execute_movel(self):
        """执行MoveL运动"""
        self.get_logger().info("  发送MoveL命令...")
        self.current_api_ = "MoveL"
        self.send_move_l(0.3, 0.3, 0.4)
        self.wait_for_result(15.0)
    
    def execute_movep(self):
        """执行MoveP运动"""
        self.get_logger().info("  发送MoveP命令...")
        self.current_api_ = "MoveP"
        self.send_move_p(0.415098, 0.043999, 0.411209)
        self.wait_for_result(15.0)
    
    def send_move_j(self, joint_angles):
        """发送MoveJ命令"""
        msg = MoveJ()
        msg.joint = joint_angles
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
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
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
        msg.pose.orientation.w = 0.499939
        msg.pose.orientation.x = -0.499979
        msg.pose.orientation.y = 0.500053
        msg.pose.orientation.z = -0.500028
        # w: 0.499939, x: -0.499979, y: 0.500053, z: -0.500028
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
