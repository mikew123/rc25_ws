"""
This robocolumbus wheel controller ROS2 is for controlling the 
1/6 scale model jeep wheel velocity and front steering
The steering model is Ackerman type
The "/cmd_vel" topic is subcribed to and the linear.x and angular.z 
velocity commands are used to create the jeep rear wheel velocity 
and the front wheel steering angle via a serial interface
The serial interface is TBD
The wheel diameter is TBD
The wheel base is TBD
The wheel spacing is TBD
"""

# from asyncio.windows_events import NULL
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import serial
import json
import tf_transformations
import time

class WheelControllerNode(Node):
    def __init__(self):
        super().__init__('robocolumbus25_wheel_controler_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.serialTimer = self.create_timer(0.010, self.serialTimerCallback)

        # Example parameters (replace with actual values)
        self.wheel_base = 0.490  # meters
        self.encoderCountsPerMeter = 6000

        # Robot pose state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # Serial port configuration (update as needed)
        self.serial_port = "/dev/serial/by-id/usb-Waveshare_RP2040_PiZero_E6625887D37C3E30-if00"
        self.baudrate = 1000000
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Serial port {self.serial_port} opened.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        cmd = {"mode":"cv"}
        self.send_json_cmd(cmd)
        time.sleep(5)

        self.get_logger().info(f"WheelControllerNode: Started node")

    def send_json_cmd(self,cmd) :
        self.get_logger().info(f"send_json_cmd: {cmd=}")
        if self.ser and self.ser.is_open:
            try:
                json_cmd = json.dumps(cmd) + '\n'
                self.ser.write(json_cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Failed to write to serial: {e}")

    # Process received serial messages
    def serialTimerCallback(self) :
        # self.get_logger().info(f"serialTimerCallback")
        data = {}
        while self.ser.in_waiting :
            line = self.ser.readline().decode('utf-8').strip()
            #self.get_logger().info(f"Received: {line=}")
            try:
                data = json.loads(line)
                break
            except json.JSONDecodeError:
                continue

        if 'odom' in data:
            odom = data['odom']
            self.proc_odom_msg(odom)

    def proc_odom_msg(self, odom) :
        # self.get_logger().info(f"proc_odom_msg {odom=}")

        # TODO: how to cast stamp as unsigned 32 for roll over
        # But may not be needed since it is a very large number and long time
        stamp = int(odom['stamp'])
        enc = int(odom['enc'])
        linX = float(odom['linx'])
        angZ = float(odom['angz'])
    
        # Odometry calculation
        # TODO: use time stamp
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Simple differential drive odometry update
        # TODO: use encoder counts
        delta_x = linX * math.cos(self.yaw) * dt
        delta_y = linX * math.sin(self.yaw) * dt
        delta_yaw = angZ * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw

        # Prepare odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
    
        # Convert yaw to quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom_msg.twist.twist.linear.x = linX
        odom_msg.twist.twist.angular.z = angZ

        # self.get_logger().info(f"proc_odom_msg {odom_msg=}")

        self.odom_publisher.publish(odom_msg)

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f"cmd_vel_callback {msg=}")

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # # Compute rear wheel velocity (m/s)
        # wheel_velocity = linear_x

        # # Compute front wheel steering angle (Ackermann)
        # if angular_z != 0 and linear_x != 0:
        #     turning_radius = linear_x / angular_z
        #     steering_angle = math.atan(self.wheel_base / turning_radius)
        # else:
        #     steering_angle = 0.0

        # self.get_logger().info(
        #     f"Rear wheel velocity: {wheel_velocity:.3f} m/s, "
        #     f"Front steering angle: {math.degrees(steering_angle):.3f} deg"
        # )

        # # Send commands over serial interface as JSON
        # cmd = {"cv":[wheel_velocity, steering_angle]}

        cmd = {"cv":[linear_x, angular_z]}
        self.send_json_cmd(cmd)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()