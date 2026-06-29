#!/usr/bin/env python3
"""轨迹连接示例节点。

仅适用于 a1_l 单臂模式：
  ros2 launch onero_driver a1_l_driver.launch.py
  ros2 run onero_examples trajectory_connect_demo_py
注意：实体机运行前请确认机械臂采用单臂安装方式；双臂安装方式需先重新评估轨迹，避免与桌面干涉。

演示使用 trajectory_connect 缓冲 MoveJ、MoveL，并用 MoveP 触发混合轨迹执行。
"""

import rclpy
from rclpy.node import Node
from onero_interfaces.msg import CommandResult, MoveJ, MoveL, MoveP
import time
import threading


class TrajectoryConnectDemo(Node):
    def __init__(self):
        super().__init__('trajectory_connect_demo')
        
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
        self.last_result_success_ = False
        self.current_command_is_buffer_ = False
        self.current_api_ = ""
        
        self.get_logger().info("========================================")
        self.get_logger().info("Trajectory Connect Demo")
        self.get_logger().info("========================================")
        self.get_logger().info("This demo shows how to use trajectory_connect")
        self.get_logger().info("to send multiple waypoints using all Move APIs:")
        self.get_logger().info("- MoveJ: Joint space motion")
        self.get_logger().info("- MoveL: Cartesian space motion")
        self.get_logger().info("- MoveP: Cartesian point-to-point motion")
        self.get_logger().info("MoveL/MoveP target pose is verified in A1-L and A1-R sim")
        self.get_logger().info("========================================")
    
    def result_callback(self, api_type, msg):
        """处理运动结果的回调函数"""
        if self.current_api_ == api_type:
            phase = "buffer ack" if self.current_command_is_buffer_ else "execution result"
            self.get_logger().info(
                f"{'✓' if msg.success else 'X'} {api_type} {phase}: "
                f"{'SUCCESS' if msg.success else 'FAILED'} {msg.error_message}"
            )
            self.last_result_success_ = msg.success
            self.result_received_ = True
    
    def run(self):
        """运行演示"""
        time.sleep(1)
        
        # 混合轨迹连接演示: MoveJ -> MoveL -> MoveP
        self.get_logger().info("\n[Test] Trajectory blending: MoveJ -> MoveL -> MoveP")
        self.get_logger().info("========================================")
        ok = self.test_mixed_trajectory_connect()
        
        self.get_logger().info("\n========================================")
        if ok:
            self.get_logger().info("✓ Trajectory Connect Demo Completed!")
            self.get_logger().info("All Move APIs (MoveJ/MoveL/MoveP) tested.")
        else:
            self.get_logger().error("Trajectory Connect Demo failed.")
        self.get_logger().info("========================================")
    
    def test_mixed_trajectory_connect(self):
        """测试混合轨迹连接：MoveJ -> MoveL -> MoveP"""
        # 1) MoveJ：缓冲一个关节目标（trajectory_connect=1）
        self.get_logger().info("  Buffering MoveJ waypoint...")
        j1 = [0.0, 0.8, 0.0, -0.9, 0.0, 0.0, 0.0]
        self.current_api_ = "MoveJ"
        self.send_move_j(j1, 1)
        if not self.wait_for_result(2.0) or not self.last_result_success_:
            return False
        time.sleep(0.2)
        
        # 2) MoveL：缓冲一个直线位姿（trajectory_connect=1）
        self.get_logger().info("  Buffering MoveL waypoint...")
        self.current_api_ = "MoveL"
        self.send_move_l(-0.4786, 0.2144, 0.2552, 1)
        if not self.wait_for_result(2.0) or not self.last_result_success_:
            return False
        time.sleep(0.2)
        
        # 3) MoveP：触发执行，作为最终段（trajectory_connect=0）
        self.get_logger().info("  Triggering execution with MoveP (final waypoint)...")
        self.current_api_ = "MoveP"
        self.send_move_p(-0.307447, 0.260748, 0.474408, 0)
        return self.wait_for_result(60.0) and self.last_result_success_
    
    def send_move_j(self, joint_angles, trajectory_connect):
        """发送MoveJ命令"""
        msg = MoveJ()
        msg.joint_positions = joint_angles
        msg.speed_scale = 0.5  # 慢速，安全
        msg.trajectory_connect = trajectory_connect
        
        self.result_received_ = False
        self.last_result_success_ = False
        self.current_command_is_buffer_ = trajectory_connect == 1
        self.movej_pub_.publish(msg)
    
    def send_move_l(self, x, y, z, trajectory_connect):
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
        msg.trajectory_connect = trajectory_connect
        
        self.result_received_ = False
        self.last_result_success_ = False
        self.current_command_is_buffer_ = trajectory_connect == 1
        self.movel_pub_.publish(msg)
    
    def send_move_p(self, x, y, z, trajectory_connect):
        """发送MoveP命令"""
        msg = MoveP()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = -0.524384
        msg.pose.orientation.y = 0.045629
        msg.pose.orientation.z = 0.474378
        msg.pose.orientation.w = 0.705624
        msg.trajectory_connect = trajectory_connect
        msg.speed_scale = 0.3
        
        self.result_received_ = False
        self.last_result_success_ = False
        self.current_command_is_buffer_ = trajectory_connect == 1
        self.movep_pub_.publish(msg)
    
    def wait_for_result(self, timeout_sec):
        """等待运动结果"""
        start_time = time.time()
        while not self.result_received_:
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
            
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn("Timeout waiting for result")
                return False
        return True


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = TrajectoryConnectDemo()
    node.run()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
