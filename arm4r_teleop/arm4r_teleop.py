#!/usr/bin/env python3
# credits to: https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose

import sys, select, termios, tty
import numpy as np

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
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

class Teleop(Node):
    def __init__(self):
        super().__init__('arm4r_teleop')

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.pose_publisher = self.create_publisher(Pose, 'body_pose', 1)
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.declare_parameter('gait.swing_height', 0.0)
        self.declare_parameter('gait.nominal_height', 0.0)
        self.declare_parameter('speed', 0.25)
        self.declare_parameter('turn', 2.0)

        self.swing_height = self.get_parameter('gait.swing_height').value
        self.nominal_height = self.get_parameter('gait.nominal_height').value
        self.speed = self.get_parameter('speed').value
        self.turn = self.get_parameter('turn').value

        self.msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
u    i    o
j    k    l
m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
U    I    O
J    K    L
M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
        """

        self.velocityBindings = {
                'i':(1,0,0,0),
                'o':(1,0,0,-1),
                'j':(0,0,0,1),
                'l':(0,0,0,-1),
                'u':(1,0,0,1),
                ',':(-1,0,0,0),
                '.':(-1,0,0,1),
                'm':(-1,0,0,-1),
                'O':(1,-1,0,0),
                'I':(1,0,0,0),
                'J':(0,1,0,0),
                'L':(0,-1,0,0),
                'U':(1,1,0,0),
                '<':(-1,0,0,0),
                '>':(-1,-1,0,0),
                'M':(-1,1,0,0),
                'v':(0,0,1,0),
                'n':(0,0,-1,0),
            }

        self.poseBindings = {
                'f':(-1,0,0,0),
                'h':(1,0,0,0),
                't':(0,1,0,0),
                'b':(0,-1,0,0),
                'r':(0,0,1,0),
                'y':(0,0,-1,0),
            }

        self.speedBindings={
                'q':(1.1,1.1),
                'z':(.9,.9),
                'w':(1.1,1),
                'x':(.9,1),
                'e':(1,1.1),
                'c':(1,.9),
            }

        self.settings = termios.tcgetattr(sys.stdin)
        self.poll_keys()

    def joy_callback(self, data):
        twist = Twist()
        twist.linear.x = data.axes[1] * self.speed
        twist.linear.y = data.buttons[4] * data.axes[0] * self.speed
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = (not data.buttons[4]) * data.axes[0] * self.turn
        self.velocity_publisher.publish(twist)

        body_pose = Pose()
        if data.axes[5] < 0:
            body_pose.position.z = data.axes[5] * 0.5
        else:
            body_pose.position.z = 0
        body_pose.position.x = 0
        body_pose.position.y = 0

        roll = (not data.buttons[5]) * -data.axes[3] * 0.349066
        pitch = data.axes[4] * 0.174533
        yaw = data.buttons[5] * data.axes[3] * 0.436332

        quaternion = quaternion_from_euler(roll, pitch, yaw)
        body_pose.orientation.x = quaternion[0]
        body_pose.orientation.y = quaternion[1]
        body_pose.orientation.z = quaternion[2]
        body_pose.orientation.w = quaternion[3]

        self.pose_publisher.publish(body_pose)

    def poll_keys(self):
        x = 0.0
        y = 0.0
        z = 0.0
        th = 0.0
        status = 0.0
        cmd_attempts = 0.0

        try:
            print(self.msg)
            print(self.vels(self.speed, self.turn))

            while rclpy.ok():
                key = self.getKey()
                if key in self.velocityBindings.keys():
                    x = self.velocityBindings[key][0]
                    y = self.velocityBindings[key][1]
                    z = self.velocityBindings[key][2]
                    th = self.velocityBindings[key][3]

                    if cmd_attempts > 1:
                        twist = Twist()
                        twist.linear.x = float(x * self.speed)
                        twist.linear.y = float(y * self.speed)
                        twist.linear.z = float(z * self.speed)
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = float(th * self.turn)
                        self.velocity_publisher.publish(twist)
                    cmd_attempts += 1

                elif key in self.speedBindings.keys():
                    self.speed = self.speed * self.speedBindings[key][0]
                    self.turn = self.turn * self.speedBindings[key][1]
                    print(self.vels(self.speed, self.turn))
                    if (status == 14):
                        print(self.msg)
                    status = (status + 1) % 15

                else:
                    cmd_attempts = 0
                    if (key == '\x03'):
                        break

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop()
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()