#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class JoyController(Node):
    def __init__(self):
        super().__init__('joy_controller')

        # Publishers
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_publisher = self.create_publisher(Pose, 'body_pose', 10)
        
        # Subscriber
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Parameters
        self.declare_parameter('speed', 0.25)
        self.declare_parameter('turn', 2.0)
        self.declare_parameter('max_roll', 20.0)  # degrees
        self.declare_parameter('max_pitch', 10.0)  # degrees
        self.declare_parameter('max_yaw', 25.0)   # degrees
        self.declare_parameter('max_z', 0.5)      # meters

        self.speed = self.get_parameter('speed').value
        self.turn = self.get_parameter('turn').value
        self.max_roll = math.radians(self.get_parameter('max_roll').value)
        self.max_pitch = math.radians(self.get_parameter('max_pitch').value)
        self.max_yaw = math.radians(self.get_parameter('max_yaw').value)
        self.max_z = self.get_parameter('max_z').value

        self.get_logger().info('Joy Controller Node Started')
        self.get_logger().info(f'Speed: {self.speed}, Turn: {self.turn}')

    def joy_callback(self, data):
        # Velocity command (Twist message)
        twist = Twist()
        
        # Linear movement
        twist.linear.x = data.axes[1] * self.speed  # Left stick Y-axis (forward/backward)
        twist.linear.y = data.buttons[4] * data.axes[0] * self.speed  # Strafe when L1 pressed
        twist.linear.z = 0.0
        
        # Angular movement
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = (not data.buttons[4]) * data.axes[0] * self.turn  # Turn when L1 not pressed
        
        self.velocity_publisher.publish(twist)

        # Body pose command
        body_pose = Pose()
        
        # Position (Z-axis only for height adjustment)
        if data.axes[5] < 0:  # L2 trigger for lowering body
            body_pose.position.z = data.axes[5] * self.max_z
        else:
            body_pose.position.z = 0.0
        body_pose.position.x = 0.0
        body_pose.position.y = 0.0

        # Orientation (Roll, Pitch, Yaw)
        roll = (not data.buttons[5]) * -data.axes[3] * self.max_roll   # Right stick X when R1 not pressed
        pitch = data.axes[4] * self.max_pitch                          # Right stick Y
        yaw = data.buttons[5] * data.axes[3] * self.max_yaw           # Right stick X when R1 pressed

        # Convert to quaternion
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        body_pose.orientation.x = quaternion[1]  # Note: different order than original
        body_pose.orientation.y = quaternion[2]
        body_pose.orientation.z = quaternion[3]
        body_pose.orientation.w = quaternion[0]

        self.pose_publisher.publish(body_pose)

def main(args=None):
    rclpy.init(args=args)
    
    joy_controller = JoyController()
    
    try:
        rclpy.spin(joy_controller)
    except KeyboardInterrupt:
        pass
    finally:
        joy_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()