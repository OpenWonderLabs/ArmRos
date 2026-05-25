#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onero_interfaces.msg import MoveJ, MoveL, MoveP
from std_msgs.msg import Bool
import time
import threading
from geometry_msgs.msg import Pose, Point, Quaternion


class TrajectoryConnectDemo(Node):
    def __init__(self):
        super().__init__('trajectory_connect_demo')
        
        # 创建发布器
        self.movej_pub_ = self.create_publisher(MoveJ, '/onero_driver/movej_cmd', 10)
        self.movel_pub_ = self.create_publisher(MoveL, '/onero_driver/movel_cmd', 10)
        self.movep_pub_ = self.create_publisher(MoveP, '/onero_driver/movep_cmd', 10)
        
        # 创建结果订阅
        self.movej_result_sub_ = self.create_subscription(
            Bool, '/onero_driver/movej_result', 
            lambda msg: self.result_callback('MoveJ', msg), 10)
        
        self.movel_result_sub_ = self.create_subscription(
            Bool, '/onero_driver/movel_result', 
            lambda msg: self.result_callback('MoveL', msg), 10)
        
        self.movep_result_sub_ = self.create_subscription(
            Bool, '/onero_driver/movep_result', 
            lambda msg: self.result_callback('MoveP', msg), 10)
        
        # 状态变量
        self.result_received_ = False
        self.current_api_ = ""
        
        self.get_logger().info("========================================")
        self.get_logger().info("Trajectory Connect Demo")
        self.get_logger().info("========================================")
        self.get_logger().info("This demo shows how to use trajectory_connect")
        self.get_logger().info("to send multiple waypoints using all Move APIs:")
        self.get_logger().info("- MoveJ: Joint space motion")
        self.get_logger().info("- MoveL: Cartesian space motion")
        self.get_logger().info("- MoveP: Pose transmission")
        self.get_logger().info("========================================")
    
    def result_callback(self, api_type, msg):
        """处理运动结果的回调函数"""
        if self.current_api_ == api_type:
            self.get_logger().info(f"✓ {api_type} result: {'SUCCESS' if msg.data else 'FAILED'}")
            self.result_received_ = True
    
    def run(self):
        """运行演示"""
        time.sleep(1)
        
        # 混合轨迹连接演示: MoveJ -> MoveL -> MoveP
        self.get_logger().info("\n[测试] 混合轨迹连接: MoveJ -> MoveL -> MoveP")
        self.get_logger().info("========================================")
        self.test_mixed_trajectory_connect()
        
        self.get_logger().info("\n========================================")
        self.get_logger().info("✓ Trajectory Connect Demo Completed!")
        self.get_logger().info("All Move APIs (MoveJ/MoveL/MoveP) tested.")
        self.get_logger().info("========================================")
    
    def test_mixed_trajectory_connect(self):
        """测试混合轨迹连接：MoveJ -> MoveL -> MoveP"""
        # 1) MoveJ：缓冲一个关节目标（trajectory_connect=1）
        self.get_logger().info("  Buffering MoveJ waypoint...")
        j1 = [0.20, 1.57, 0.3, 0.5, 0.0, 0.0, 0.0]
        self.current_api_ = "MoveJ"
        self.send_move_j(j1, 1)
        self.wait_for_result(2.0)
        time.sleep(0.2)
        
        # 2) MoveL：缓冲一个直线位姿（trajectory_connect=1）
        self.get_logger().info("  Buffering MoveL waypoint...")
        self.current_api_ = "MoveL"
        self.send_move_l(0.3, 0.3, 0.4, 1)
        self.wait_for_result(2.0)
        time.sleep(0.2)
        
        # 3) MoveP：触发执行，作为最终段（trajectory_connect=0）
        self.get_logger().info("  Triggering execution with MoveP (final waypoint)...")
        self.current_api_ = "MoveP"
        self.send_move_p(0.5, 0.3, 0.1, 0)
        self.wait_for_result(0.2)

        # j2 = [0.0, 0.2, 0.0, -0.5, 0.0, 0.0, 0.0]
        # self.current_api_ = "MoveJ"
        # self.send_move_j(j2, 1)
        # self.wait_for_result(2.0)
        # time.sleep(20)
    
    def send_move_j(self, joint_angles, trajectory_connect):
        """发送MoveJ命令"""
        msg = MoveJ()
        msg.joint = joint_angles
        msg.speed_scale = 0.8  # 慢速，安全
        msg.trajectory_connect = trajectory_connect
        
        self.result_received_ = False
        self.movej_pub_.publish(msg)
    
    def send_move_l(self, x, y, z, trajectory_connect):
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
        msg.trajectory_connect = trajectory_connect
        
        self.result_received_ = False
        self.movel_pub_.publish(msg)
    
    def send_move_p(self, x, y, z, trajectory_connect):
        """发送MoveP命令"""
        msg = MoveP()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.trajectory_connect = trajectory_connect
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
    
    node = TrajectoryConnectDemo()
    node.run()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
