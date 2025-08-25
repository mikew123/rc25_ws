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

from sensor_msgs.msg import BatteryState

import math
import serial
import json
import tf_transformations
import time
import numpy as np

class WheelControllerNode(Node):
    #PID coefficients
    coeffA = 0.2
    coeffB = 0.04
    # Diff coefficients to help get ESC to speed faster
    coeffDA = 0.15
    coeffDB = 0.075

    last_time = 0

    # Example parameters (replace with actual values)
    wheel_base = 0.490  # meters
    encoderCountsPerMeter = 6000

    # Robot pose state
    x = 0.0
    y = 0.0
    yaw = 0.0
    last_time = 0.0 # seconds
    last_stampMs = np.uint32(0)
    last_enc = np.int32(0)
    last_angRad = 0.0

    # Serial port configuration (update as needed)
    serial_port = "/dev/serial/by-id/usb-Waveshare_RP2040_PiZero_E6625887D37C3E30-if00"
    baudrate = 1000000

    # Odometry from encoder or velocity
    odom_encoder = True

    def __init__(self):
        super().__init__('robocolumbus25_wheel_controler_node')

        self.last_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.wheel_odom_publisher = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.battery_status_msg_publisher = self.create_publisher(BatteryState, 'battery_status', 10)
        self.serialTimer = self.create_timer(0.010, self.serialTimerCallback)

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Serial port {self.serial_port} opened.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        cmd = {"pid":[self.coeffA,self.coeffB,self.coeffDA,self.coeffDB]}
        self.send_json_cmd(cmd)
        cmd = {"mode":"cv"}
        self.send_json_cmd(cmd)

        self.ser.flush()
        time.sleep(5)  # engine controller seems to act weird if no delay

        self.get_logger().info(f"WheelControllerNode: Started node")

    def send_json_cmd(self,cmd) :
        # self.get_logger().info(f"send_json_cmd: {cmd=}")
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
            self.proc_wheel_odom_msg(odom)

        if 'vbat' in data:
            vbat = data['vbat']
            self.processBatteryInfo(vbat)
        
    def processBatteryInfo(self, vbat) -> None :
        # TODO: add current and possibly cell voltages
        bmsg = BatteryState()
        bmsg.header.stamp = self.get_clock().now().to_msg()
        bmsg.header.frame_id = "base_link"
        bmsg.voltage = float(vbat)
        bmsg.present = True
        self.battery_status_msg_publisher.publish(bmsg)

    # Process odom info from wheels to get /wheel_odom with velocity and pose
    def proc_wheel_odom_msg(self, odom) :
        #self.get_logger().info(f"proc_wheel_odom_msg {odom=}")
        current_time = self.get_clock().now()

        # TODO: how to cast stamp as unsigned 32 for roll over
        # But may not be needed since it is a very large number and long time
        stampMs = np.uint32(odom['stamp']) # Milliseconds
        enc = np.int32(odom['enc'])
        linX = float(odom['linx'])
        angRad = -1* float(odom['steer'])
    
        if self.odom_encoder :
            dt = (stampMs - self.last_stampMs) * 1e-3 # converted to seconds
            self.last_stampMs = stampMs
            dEnc = enc - self.last_enc
            self.last_enc = enc

            if dt>0.1 or dt<=0.0 :
                return
   
            dx = (dEnc/self.encoderCountsPerMeter)
            linX = dx / dt

            # convert steering angle to steering angle velocity
            if (linX==0 or angRad==0) :
                angZ = 0.0
            else :
                angZ = math.tan(angRad)*linX/self.wheel_base

            delta_x = dx * math.cos(self.yaw)
            delta_y = dx * math.sin(self.yaw)
            delta_yaw = angZ * dt

        else :
            # convert steering angle to steering angle velocity
            if (linX==0 or angRad==0) :
                angZ = 0.0
            else :
                angZ = math.tan(angRad)*linX/self.wheel_base

            # Odometry calculation
            # TODO: use time stamp from engine controller
            # floats dont wrap but can lose accuracy
            dt = (current_time - self.last_time).nanoseconds * 1e-9 # converted to seconds
            self.last_time = current_time
            if(dt>1.0) : return # delta time too large, probably just start up

            # Simple differential drive odometry update
            delta_x = linX * math.cos(self.yaw) * dt
            delta_y = linX * math.sin(self.yaw) * dt
            delta_yaw = angZ * dt

        # Update pose with delta values
        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw
        # handle angle wrap
        if self.yaw > math.pi : self.yaw -= 2 * math.pi
        if self.yaw <-math.pi : self.yaw += 2 * math.pi

        # Prepare odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist.linear.x = linX
        odom_msg.twist.twist.angular.z = angZ
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
    
        # Convert yaw to quaternion
        (x,y,z,w) = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation = Quaternion(x=x,y=y,z=z,w=w)

        odom_msg.twist.twist.linear.x = linX
        odom_msg.twist.twist.angular.z = angZ

        # self.get_logger().info(f"proc_wheel_odom_msg {angZ=:.3f} {odom_msg=}")

        self.wheel_odom_publisher.publish(odom_msg)

    def cmd_vel_callback(self, msg):
        # self.get_logger().info(f"cmd_vel_callback {msg=}")

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Compute rear wheel velocity (m/s)
        wheel_velocity = linear_x

        # Compute front wheel steering angle (Ackermann)
        if angular_z != 0 and linear_x != 0:
            turning_radius = linear_x / angular_z
            steering_angle = -1* math.atan(self.wheel_base / turning_radius)
        else:
            steering_angle = 0.0

        # self.get_logger().info(
        #     f"Rear wheel velocity: {wheel_velocity:.3f} m/s, "
        #     f"Front steering angle: {math.degrees(steering_angle):.3f} deg"
        # )

        # # Send commands over serial interface as JSON
        # cmd = {"cv":[wheel_velocity, angular_z]}
        if(wheel_velocity!=0.0):
            cmd = {"wd":1000,"cv":[wheel_velocity, steering_angle]}
        else :
            cmd = {"wd":1,"cv":[0, 0]}
        self.send_json_cmd(cmd)

    def destroy_node(self):
        self.get_logger().info("destroy_node")
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            # stop the motor
            cmd = {"wd":100,"cv":[0, 0]}
            self.send_json_cmd(cmd)
            # change mode to bypass
            cmd = {"mode":"bypass"}
            self.send_json_cmd(cmd)
            # wait for message to be sent before closing
            self.ser.flush()
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelControllerNode()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        from rclpy.impl import rcutils_logger
        logger = rcutils_logger.RcutilsLogger(name="node")
        logger.info('Received Keyboard Interrupt (^C). Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()