#!/usr/bin/env python3

import rclpy
#import sys
#import serial
import math
#import time
#import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
#from std_msgs.msg import String
#from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy

class Robocolumbus25TeleopNode(Node):
    maxLinearX: float       = 1.0   # Meters per second
    maxSteerAngleRad: float = 0.7   # 0.7 ~40 degrees
    wheelBase:float         = 0.490 # meters

    def __init__(self):
        super().__init__('robocolumbus25_teleop_node')

        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Teleop Started")
       
    # Get button commands from Joy message
    def joy_callback(self, msg):
        cmd_vel = Twist()

        # get controller values
        axes1 = msg.axes[1] # throttle
        axes3 = msg.axes[3] # steer

        linearX  = axes1 * self.maxLinearX
        steerAngleRad = axes3 * self.maxSteerAngleRad

        # Basic steering calculation wheel angle to angular velocity
        angularZ = math.tan(steerAngleRad) * linearX / self.wheelBase

        # self.get_logger().info(f"{axes1=:.3f} {axes3=:.3f} : {linearX=:.3f} {angularZ=:.3f} {steerAngleRad=:.3f}")

        # Publish /cmd_vel
        cmd_vel.linear.x = linearX
        cmd_vel.angular.z = angularZ
        self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)

    node = Robocolumbus25TeleopNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()
