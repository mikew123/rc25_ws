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
from sympy import Ellipse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String

import math
import serial
import json
import tf_transformations
import time
import numpy as np

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class WheelControllerNode(Node):
    #PID coefficients
    coeffA = 0.2
    coeffB = 0.04
    # Diff coefficients to help get ESC to speed faster
    coeffDA = 0.15
    coeffDB = 0.075

    # Example parameters (replace with actual values)
    wheel_base = 0.490  # meters
    encoderCountsPerMeter = 6000

    # Robot pose state
    x = 0.0
    y = 0.0
    yaw = 0.0
    last_stampMs = np.uint32(0)
    last_enc = np.int32(0)
    last_steering_angle = 0.0

    last_cv_time_sec = 0.0 # seconds
    last_cv_msg = Twist()

    # Serial port configuration (update as needed)
    serial_port = ["/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6625887D37C3E30-if00"
                   ,"/dev/serial/by-id/usb-Waveshare_RP2040_PiZero_E6625887D37C3E30-if00"]
    serial_port_idx = 0
    baudrate = 1000000

    # Odometry from encoder or velocity
    odom_encoder = True
    
    def __init__(self):
        super().__init__('robocolumbus25_wheel_controler_node')

        self.cb_group = MutuallyExclusiveCallbackGroup()

        self.openSerialPort()

        self.last_time_sec = self.get_clock().now().nanoseconds * 1e-9

        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel'
                                        , self.cmd_vel_callback, 10)

        self.wheel_odom_publisher = self.create_publisher(Odometry, 'wheel_odom', 10)

        # Timer is used for serial port, has own call back group is serial processing takes time
        self.serialTimer = self.create_timer(0.010, self.serialTimerCallback
                                       , callback_group=self.cb_group)

        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "json_msg"
                                        , self.json_msg_callback, 10)

        cmd = {"pid":[self.coeffA,self.coeffB,self.coeffDA,self.coeffDB]}
        self.sendJsonCmd(cmd)
        cmd = {"mode":"cv"}
        self.sendJsonCmd(cmd)

        self.ser.flush()
        time.sleep(5)  # engine controller seems to act weird if no delay

        # # Send zero velocity command to unstick battery status - worked!
        # cmd = {"cv":[0,0]}
        # self.sendJsonCmd(cmd)

        self.tts("Wheel Controller Node Started")
        self.get_logger().info(f"WheelControllerNode: Started node")

    def openSerialPort(self) :
        # Open serial port to tof sensors controller over USB
        # continuously try to open it
        serialOpen = False
        while not serialOpen :
            try :
                self.ser = serial.Serial(self.serial_port[self.serial_port_idx], self.baudrate, timeout=1)
                self.get_logger().info(f"openSerialPort: Serial port {self.serial_port[self.serial_port_idx]} opened.")
                serialOpen = True

            except serial.SerialException as e :
                self.get_logger().info(f"openSerialPort: Failed to open serial port: {e}")
                self.get_logger().info("openSerialPort: Try opening serial port again")
                self.serial_port_idx +=1
                if self.serial_port_idx>1 : self.serial_port_idx=0

    # get data from serial port, returns a line of text
    def getSerialData(self) -> str :
        # Check if a line has been received on the serial port
        err:bool=False
        try :
            if self.ser.in_waiting > 0 :
                received_data:str = self.ser.readline().decode().strip()
                # self.get_logger().info(f"getSerialData: {received_data=}")
                return received_data # Exit while 1 loop
            else :
                return None
            
        except Exception as ex :
            self.get_logger().info(f"getSerialData: serial read failure : {ex}")
            err=True

        if err :
            try :
                self.ser.close()    
                # self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
                self.openSerialPort()
            except serial.SerialException as e:
                self.get_logger().info(f"getSerialData: Failed to open serial port: {e}")

    def json_msg_callback(self, msg:String) -> None :
        #self.get_logger().info(f"json_msg_callback: {msg=}")
        pass

    def tts(self, tts) -> None:
        json_msg = {"speaker":{"tts":tts}}
        self.sendJsonMsg(json_msg)

    def sendJsonMsg(self, json_msg) -> None :
        str = json.dumps(json_msg)
        msg = String(data=str)
        self.json_msg_publisher.publish(msg)

    def sendJsonCmd(self,cmd) :
        # self.get_logger().info(f"sendJsonCmd: {cmd=}")
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
        # while self.ser.in_waiting :
        #     line = self.ser.readline().decode('utf-8').strip()
        #     #self.get_logger().info(f"serialTimerCallback: {line=}")

        # keep trying until a valid JSON formtaed line is read
        while 1 :
            line:str = self.getSerialData()
            if line == None : return
            
            try:
                data = json.loads(line)
                break
            except json.JSONDecodeError:
                continue

        if 'odom' in data:
            odom = data['odom']
            self.proc_wheel_odom_msg(odom)

        if 'status' in data:
            status = data['status']
            self.processEngineStatusMsg(status)
        
    def processEngineStatusMsg(self, status) -> None :
        json_msg = {"engine":{"status":status}}
        self.sendJsonMsg(json_msg)

    # Process odom info from wheels to get /wheel_odom with velocity and pose
    def proc_wheel_odom_msg(self, odom) :
        #self.get_logger().info(f"proc_wheel_odom_msg {odom=}")
        current_time = self.get_clock().now()
        current_time_sec = current_time.nanoseconds * 1e-9

        # TODO: how to cast stamp as unsigned 32 for roll over
        # But may not be needed since it is a very large number and long time
        stampMs = np.uint32(odom['stamp']) # Milliseconds
        enc = np.int32(odom['enc']) # counts
        linX = float(odom['linx']) # meters/second
        steer = -1* float(odom['steer']) # radians
    

        if self.odom_encoder == True :
            dt = (stampMs - self.last_stampMs) * 1e-3 # converted to seconds
            self.last_stampMs = stampMs
            dEnc = enc - self.last_enc
            self.last_enc = enc

            if dt>0.1 or dt<=0.0 :
                return
   
            dist = (dEnc/self.encoderCountsPerMeter)
            # recompute forward velocity based on encoder counts
            ddt = dist/dt # meters/second

            # convert steering angle to angle velocity at rear diff
            # TODO: handle low resulution around zero velocities??
            az = ddt*math.tan(steer)/self.wheel_base

            # Why scale angular velocity Z for robot to match odom and nav2 to work?
            az /= 2.4
            
            # Update pose angle at rear diff
            self.yaw += az * dt # angular rad/sec * dt
            # handle angle wrap
            if self.yaw > math.pi : self.yaw -= 2 * math.pi
            if self.yaw <-math.pi : self.yaw += 2 * math.pi

            # compute change in x and y
            # and update pose location at rear diff
            self.x += dist * math.cos(self.yaw)
            self.y += dist * math.sin(self.yaw)

            # if dEnc != 0 :
            #     self.get_logger().info(f"proc_wheel_odom_msg {dt=:.3f} {dEnc=} {steer=:.3f}"\
            #         + f"{ddt=:.3f} {az=:.3f} {self.x=:.3f} {self.y=:.3f} {self.yaw=:.3f}")

            # Prepare odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_footprint"
            # Update pose 
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            # Update velocities
            odom_msg.twist.twist.linear.x = ddt
            odom_msg.twist.twist.angular.z = az
        
            # Convert yaw to quaternion
            (x,y,z,w) = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
            odom_msg.pose.pose.orientation = Quaternion(x=x,y=y,z=z,w=w)

            # self.get_logger().info(f"proc_wheel_odom_msg {az=:.3f} {odom_msg=}")

            self.wheel_odom_publisher.publish(odom_msg)

        # else :
        
        #     dt = current_time_sec - self.last_cv_time_sec
        #     if dt<1.0 and dt>0.0 : return # cmd_vel publishes wheel_odom

        #     # No cmd_vel messages that would send odom_msg
        #     # make sure velocity values are 0 but keep pose
        #     odom_msg = Odometry()       
        #     odom_msg.header.stamp = current_time.to_msg()
        #     odom_msg.header.frame_id = "odom"
        #     odom_msg.child_frame_id = "base_link"
        #     odom_msg.pose.pose.position.x = self.x
        #     odom_msg.pose.pose.position.y = self.y
        #     odom_msg.pose.pose.position.z = 0.0
        #     # Convert yaw to quaternion
        #     (x,y,z,w) = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        #     odom_msg.pose.pose.orientation = Quaternion(x=x,y=y,z=z,w=w)

        #     self.wheel_odom_publisher.publish(odom_msg)


    def cmd_vel_callback(self, msg: Twist) -> None:
        # self.get_logger().info(f"cmd_vel_callback {msg=}")

        current_time = self.get_clock().now()
        current_time_sec = current_time.nanoseconds * 1e-9

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Why scale angular velocity Z for robot to match odom and nav2 to work?
        angular_z *= 2.4

        # Compute rear wheel velocity (m/s)
        wheel_velocity = linear_x

        # Compute front wheel steering angle (Ackermann)
        if math.fabs(linear_x) > 0.01:
            steering_angle = -1* math.atan(self.wheel_base * angular_z / linear_x)
        else :
            steering_angle = self.last_steering_angle

        self.last_steering_angle = steering_angle

        # # Send commands over serial interface as JSON
        if(wheel_velocity!=0.0):
            cmd = {"wd":1000,"cv":[wheel_velocity, steering_angle]}
        else : # force stop (how it affects PID?)
            cmd = {"wd":1000,"cv":[wheel_velocity, steering_angle]}
        self.sendJsonCmd(cmd)

        # if self.odom_encoder == False :
        #     dt = current_time_sec - self.last_time_sec
        #     lx = msg.linear.x
        #     az = msg.angular.z

        #     if(dt<0.0 or dt>1.0) : 
        #         self.last_cmd_vel = msg
        #         self.last_time_sec = current_time_sec
        #         # init with no movement
        #         self.wheel_odom_publisher.publish(Odometry())
        #         return

        #     # Simple differential drive odometry update calcs

        #     delta_x = (lx*dt) * math.cos(self.yaw)
        #     delta_y = (lx*dt) * math.sin(self.yaw)
        #     self.x += delta_x
        #     self.y += delta_y

        #     dz = az * dt
        #     self.yaw += dz
        #     # handle angle wrap
        #     if self.yaw > math.pi : self.yaw -= 2 * math.pi
        #     if self.yaw <-math.pi : self.yaw += 2 * math.pi

        #     # Prepare odometry message
        #     odom_msg = Odometry()
        #     odom_msg.header.stamp = current_time.to_msg()
        #     odom_msg.header.frame_id = "odom"
        #     odom_msg.child_frame_id = "base_link"
        #     odom_msg.twist.twist.linear.x = lx
        #     odom_msg.twist.twist.angular.z = az
        #     odom_msg.pose.pose.position.x = self.x
        #     odom_msg.pose.pose.position.y = self.y
        #     odom_msg.pose.pose.position.z = 0.0
        
        #     # Convert yaw to quaternion
        #     (x,y,z,w) = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        #     odom_msg.pose.pose.orientation = Quaternion(x=x,y=y,z=z,w=w)

        #     #self.get_logger().info(f"cmd_vel odom: {dt=:.3f} {lx=:.3f} {az=:.3f} {self.x=:.3f} {self.y=:.3f} {self.yaw=:.3f}")

        #     self.wheel_odom_publisher.publish(odom_msg)

        self.last_cv_msg = msg
        self.last_cv_time_sec = current_time_sec
            
    def destroy_node(self):
        self.get_logger().info("destroy_node")
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            # stop the motor
            cmd = {"wd":100,"cv":[0, 0]}
            self.sendJsonCmd(cmd)
            # change mode to bypass
            cmd = {"mode":"bypass"}
            self.sendJsonCmd(cmd)
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
        # rclpy.spin(node)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()    
    except KeyboardInterrupt:
        from rclpy.impl import rcutils_logger
        logger = rcutils_logger.RcutilsLogger(name="node")
        logger.info('Received Keyboard Interrupt (^C). Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()