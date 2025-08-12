import rclpy
import json
import serial
import math
import time
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose

class ImuGpsNode(Node):
    '''
    This processes the serial port from the RP2040 for the IMU and GPS sensors
    in the cabin
    '''

    timerRateHz = 110.0; # Rate to check serial port for messages

    # Serial port configuration (update as needed)
    serial_port = "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6635C469F25492A-if00"
    baudrate = 1000000

    laccJsonPacket = None
    rvelJsonPacket = None
    rvecJsonPacket = None

    gpsCntr    = 0
    gpsCnt     = 10
    gpsLatYAcc = 0.0
    gpsLonXAcc = 0.0

    
    def __init__(self):
        super().__init__('imu_gps_node')

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Serial port {self.serial_port} opened.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None
            exit(1)
            
        self.imu_test_publisher = self.create_publisher(String, 'imu_test', 10)
        self.imu_msg_publisher = self.create_publisher(Imu, 'imu_msg', 10)
        self.gps_nav_publisher = self.create_publisher(NavSatFix, 'gps_nav', 10)
        self.gps_pose_publisher = self.create_publisher(Pose, 'gps_pose', 10)
        self.cmp_azi_publisher = self.create_publisher(Int32, 'cmp_azi', 10)

        self.gps_nav_subscription = self.create_subscription(NavSatFix,"gps_nav", self.gps_nav_subscription_callback, 10)
        

        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)
        
        # configure interface
        self.send_json_cmd({"cfg":{"imu":True, "gps":True, "cmp":False}})

        self.get_logger().info(f"ImuGpsNode Started")

    def send_json_cmd(self,cmd) :
        # self.get_logger().info(f"send_json_cmd: {cmd=}")
        if self.ser and self.ser.is_open:
            try:
                json_cmd = json.dumps(cmd) + '\n'
                self.ser.write(json_cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Failed to write to serial: {e}")

    # # check serial ports to find the one that matches the id
    # def determineSerialPort(self, id) :
    #     for a in range(10):
    #         for serial_port in self.serialPorts :
    #             try:
    #                 self.get_logger().info(f"Try {serial_port}")
    #                 node_serial_port = serial.Serial(serial_port, 1000000, timeout=0.01)
    #                 node_serial_port.write("{\"id\":0}".encode())
    #                 time.sleep(1)
    #                 # read port multiple times checking for "id"
    #                 for i in range(10):
    #                     received_data = node_serial_port.readline().decode().strip()
    #                     #self.get_logger().info(f"Received engine json: {received_data}")
    #                     try :
    #                         packet = json.loads(received_data)
    #                         if "id" in packet:
    #                             if packet.get("id") == id :
    #                                 self.get_logger().info(f"Found {id} serial port: {serial_port}")
    #                                 return node_serial_port
    #                     except Exception as ex:
    #                         if received_data != "" :
    #                             self.get_logger().info(f"Bad json.loads packet {received_data}: {ex}")
    #             except Exception as ex:
    #                 self.get_logger().info(f"Serial port {serial_port} did not open: {ex}")
    #     return None
    
    # check serial port at timerRateHz and parse out messages to publish
    def timer_callback(self):
        # Check if a line has been received on the serial port
        if self.ser.in_waiting > 0:
            try :
                received_data = self.ser.readline().decode().strip()
                #self.get_logger().info(f"Received engine json: {received_data}")
            except Exception as ex:
                self.get_logger().error(f"IMU GPS serial read failure : {ex}")
                return

            try :
                unknown = True
                packet = json.loads(received_data)
                if "imu" in packet :
                    self.imuPublish(packet.get("imu"))
                    unknown = False
                if "gps" in packet :
                    self.gpsPublish(packet.get("gps"))
                    unknown = False
                if "cmp" in packet :
                    self.cmpPublish(packet.get("cmp"))
                    unknown = False
                if unknown :
                    self.get_logger().info(f"IMU GPS serial json unknown : {received_data}")
                    return  
            except Exception as ex:
                self.get_logger().error(f"IMU GPS serial json failure {ex} : {received_data}")
                return
            
    def gps_nav_subscription_callback(self, msg: NavSatFix) -> None:
        
        lon = msg.longitude
        lat = msg.latitude
        
        latY = (lat/360)*40007863
        #lonX = ((lon/360)*(math.cos((lat/360)*2*math.pi)))*40075017
        lonX = ((lon/360)*(1.0))*40075017
        
        # Offset relative lat lon to zero at start
        if self.gpsCntr < self.gpsCnt :
            self.gpsCntr +=1
            self.gpsLatYAcc += latY
            self.gpsLonXAcc += lonX
            
        latY = latY - self.gpsLatYAcc/self.gpsCntr
        lonX = lonX - self.gpsLonXAcc/self.gpsCntr
        
        pmsg = Pose()
        pmsg.position.x = lonX
        pmsg.position.y = latY
        
        self.gps_pose_publisher.publish(pmsg)
        
        #self.get_logger().info(f"gps_msg_subscription_callback : {latY=} {lonX=}")
        
    def cmpPublish(self, cmpJsonPacket) -> None:        
        #self.get_logger().info(f"cmpPublish : {cmpJsonPacket=}")
        msg = Int32()
        msg.data = cmpJsonPacket.get("azi")
        self.cmp_azi_publisher.publish(msg)
        
    def gpsPublish(self, gpsJsonPacket) -> None:        
        #self.get_logger().info(f"gpsPublish : {gpsJsonPacket=}")

        msg = NavSatFix();

        # NOTE: should we use the imu timestamp to get better accuracy?
        msg.header.stamp = self.get_clock().now().to_msg()
        # The IMU is located close to the rear differential 
        # and aligned XY so no offest is needed
        msg.header.frame_id = "base_link"

        # TODO: status fields
        # put num sat in view into status field service
        msg.status.service = gpsJsonPacket.get("siv")

        msg.latitude  = 1e-7*gpsJsonPacket.get("lat")
        msg.longitude = 1e-7*gpsJsonPacket.get("lon")
        msg.altitude  = 1e-3*gpsJsonPacket.get("alt") # mm to Meters

        self.gps_nav_publisher.publish(msg)

    def imuPublish(self, imuJsonPacket) :        
        #self.get_logger().info(f"imuPublish : {imuJsonPacket=}")

        # TODO: collect lacc, rvel, rvec packets with same seq# and publish imu message
        try :
            if "lacc" in imuJsonPacket :
                self.laccJsonPacket = imuJsonPacket.get("lacc")
            elif "rvel" in imuJsonPacket :
                self.rvelJsonPacket = imuJsonPacket.get("rvel")
            elif "rvec" in imuJsonPacket :
                self.rvecJsonPacket = imuJsonPacket.get("rvec")
            else :
                self.get_logger().error("")

            if self.laccJsonPacket!=None and  self.rvelJsonPacket!=None and  self.rvecJsonPacket!=None :
                laccSeq = self.laccJsonPacket.get("seq")
                rvelSeq = self.rvelJsonPacket.get("seq")
                rvecSeq = self.rvecJsonPacket.get("seq")
                if laccSeq==rvelSeq and rvelSeq==rvecSeq :
                    #self.get_logger().info(f"{laccSeq=} {rvelSeq=} {rvecSeq=}")
                    msg = Imu()

                    # NOTE: should we use the imu timestamp to get better accuracy?
                    msg.header.stamp = self.get_clock().now().to_msg()
                    # The IMU is located at the centroid of the rear differential 
                    # and aligned XY so no offest is needed
                    msg.header.frame_id = "base_link"

                    #self.get_logger().info(f"imuPublish : {self.rvecJsonPacket=}")
                    msg.orientation.x = float(self.rvecJsonPacket.get("i"))
                    msg.orientation.y = float(self.rvecJsonPacket.get("j"))
                    msg.orientation.z = float(self.rvecJsonPacket.get("k"))
                    msg.orientation.w = float(self.rvecJsonPacket.get("real"))

                    #self.get_logger().info(f"imuPublish : {self.rvelJsonPacket=}")
                    msg.angular_velocity.x =  float(self.rvelJsonPacket.get("x"))
                    msg.angular_velocity.y =  float(self.rvelJsonPacket.get("y"))
                    msg.angular_velocity.z =  float(self.rvelJsonPacket.get("z"))

                    #self.get_logger().info(f"imuPublish : {self.laccJsonPacket=}")
                    msg.linear_acceleration.x = float(self.laccJsonPacket.get("x"))
                    msg.linear_acceleration.y = float(self.laccJsonPacket.get("y"))
                    msg.linear_acceleration.z = float(self.laccJsonPacket.get("z"))

                    self.imu_msg_publisher.publish(msg)

                    #DEBUG print out

        except Exception as ex:
            self.get_logger().error(f"imuPublish json exception : {ex}")
            return


        # imuJsonStr = json.dumps(imuJsonPacket)
        # msg = String()
        # msg.data = imuJsonStr
        # self.imu_test_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = ImuGpsNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()