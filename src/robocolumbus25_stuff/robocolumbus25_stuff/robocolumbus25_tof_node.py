from ctypes.wintypes import PMSG
import rclpy
import json
import serial
import math
import time
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32MultiArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import tf_transformations

class TofNode(Node):
    '''
    This processes the serial port from the RP2040 for the IMU and GPS sensors
    in the cabin
    '''

    timerRateHz = 100.0; # Rate to check serial port for messages
#    serial_port = "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6635C469F25492A-if00"
    baudrate = 1000000

    laccJsonPacket = None
    rvelJsonPacket = None
    rvecJsonPacket = None

    gpsCntr    = 0
    gpsCnt     = 10
    gpsLatYAcc = 0.0
    gpsLonXAcc = 0.0

    
    def __init__(self):
        super().__init__('tof_node')

        # Open serial port to tof sensors controller over USB
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Serial port {self.serial_port} opened.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None
            exit(1)
            
        # publish a topic for each TOF sensor fc=front_center etc
        self.tof_fc_publisher = self.create_publisher(PointCloud2, 'tof_fc', 10)
      
        # timer to check serial port
        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)
        
        # # configure interface
        # self.send_json_cmd({"cfg":{"imu":True, "gps":True, "cmp":False}})

        self.get_logger().info(f"TofNode Started")

    def send_json_cmd(self,cmd) :
        # self.get_logger().info(f"send_json_cmd: {cmd=}")
        if self.ser and self.ser.is_open:
            try:
                json_cmd = json.dumps(cmd) + '\n'
                self.ser.write(json_cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Failed to write to serial: {e}")
    
    # check serial port at timerRateHz and parse out messages to publish
    def timer_callback(self):
        # Check if a line has been received on the serial port
        if self.ser.in_waiting > 0:
            try :
                received_data = self.ser.readline().decode().strip()
                #self.get_logger().info(f"Received engine json: {received_data}")
            except Exception as ex:
                self.get_logger().error(f"TOF sensors serial read failure : {ex}")
                return

            try :
                unknown = True
                packet = json.loads(received_data)
                if "tof_fc" in packet :
                    self.tof_fc_Publish(packet.get("tof_fc"))
                    unknown = False
                if unknown :
                    self.get_logger().info(f"TOF sensors serial json unknown : {received_data}")
                    return  
            except Exception as ex:
                self.get_logger().error(f"TOF sensors serial json Exception {ex} : {received_data}")
                return

    # 8x8 point cloud for each sensor FOV 45degx45deg
    # calculate x,y,z for each point
    def tof_fc_Publish(self, tof) :
        self.get_logger().info(f"tof_fc_Publish: {tof=}")
        msg = PointCloud2()

def main(args=None):
    rclpy.init(args=args)

    node = TofNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()