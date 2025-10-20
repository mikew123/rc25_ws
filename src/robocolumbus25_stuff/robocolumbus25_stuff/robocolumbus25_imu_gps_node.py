from ctypes.wintypes import PMSG

from numpy import int32
import rclpy
import json
import serial
import math
import time
import tf_transformations

from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32, Float32MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rc25_interfaces.msg import ImuCal

class ImuGpsNode(Node):
    '''
    This processes the serial port from the RP2040 for the IMU and GPS sensors
    in the cabin
    '''

    timerRateHz = 100.0; # Rate to check serial port for messages
    serial_port = ["/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6635C469F25492A-if00"
                   ,"/dev/serial/by-id/usb-Waveshare_RP2040_PiZero_E6635C469F25492A-if00"]
    serial_port_idx:int = 0

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

        self.cb_group = MutuallyExclusiveCallbackGroup()

        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "json_msg"
                                        , self.json_msg_callback, 10)
            
        self.openSerialPort()

        self.imu_test_publisher = self.create_publisher(String, 'imu_test', 10)
        self.imu_msg_publisher = self.create_publisher(Imu, 'imu', 10)
        self.imu_cal_publisher = self.create_publisher(ImuCal, 'imu/cal', 10)
        self.gps_nav_publisher = self.create_publisher(NavSatFix, 'gps_nav', 10)
        self.gps_pose_publisher = self.create_publisher(Pose, 'gps_pose', 10)
        self.cmp_azi_publisher = self.create_publisher(Float32, 'cmp_azi', 10)

        self.gps_nav_subscription = self.create_subscription(NavSatFix,"gps_nav"
                                        , self.gps_nav_subscription_callback, 10)

        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback
                                       , callback_group=self.cb_group)
        
        # configure interface
        self.send_json_cmd({"cfg":{"imu":True, "gps":True, "cmp":False}})

        time.sleep(2) # wait for json_msg_publisher to be ready!!??
        self.tts("IMU and GPS Node Started")               
        self.get_logger().info(f"ImuGpsNode Started")


    def tts(self, tts) -> None:
        """
        Send text to speaker 
        """
        json_msg = {"speaker":{"tts":tts}}
        self.sendJsonMsg(json_msg)

    def sendJsonMsg(self, json_msg) -> None :
        str = json.dumps(json_msg)
        msg = String(data=str)
        self.json_msg_publisher.publish(msg)

    def json_msg_callback(self,msg) :
        pass

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
                self.get_logger().info("openSerialPort: Try opening serial port again using other port name")
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
        # if self.ser.in_waiting > 0:
        #     try :
        #         received_data = self.ser.readline().decode().strip()
        #         #self.get_logger().info(f"Received engine json: {received_data}")
        #     except Exception as ex:
        #         self.get_logger().error(f"IMU GPS serial read failure : {ex}")
        #         return

        received_data = self.getSerialData()
        if received_data == None : return

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
            self.get_logger().error(f"IMU GPS serial json Exception {ex} : {received_data}")
            return

    # GPS pose
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
        
    # Process Compass data
    def cmpPublish(self, cmpJsonPacket) -> None:        
        #self.get_logger().info(f"cmpPublish : {cmpJsonPacket=}")
        # msg = Int32()
        # msg.data = cmpJsonPacket.get("azi")
        # self.cmp_azi_publisher.publish(msg)
        pass

    # Process GPS data
    def gpsPublish(self, gpsJsonPacket) -> None:        
        # self.get_logger().info(f"gpsPublish : {gpsJsonPacket=}")

        msg = NavSatFix();

        # NOTE: should we use the imu timestamp to get better accuracy?
        msg.header.stamp = self.get_clock().now().to_msg()
        # The IMU is located close to the rear differential 
        # and aligned XY so no offest is needed
        msg.header.frame_id = "gps_link"

        # TODO: status fields
        # put num sat in view into status field service
        status = gpsJsonPacket.get("status")
        siv = gpsJsonPacket.get("siv")
        # 5 satelites is pretty stable
        if (status==True) and (siv>=5):
            # Satelite fix is OK
            msg.status.status = 0
            # setting servive to satelites in view since we dont know which sat are used
            msg.status.service = siv
            msg.latitude  = 1e-7*gpsJsonPacket.get("lat")
            msg.longitude = 1e-7*gpsJsonPacket.get("lon")
            msg.altitude  = 1e-3*gpsJsonPacket.get("alt") # mm to Meters
        else :
            # Not a valid satelite fix
            msg.status.status = -1
            msg.status.service = siv
            msg.latitude  = math.nan
            msg.longitude = math.nan
            msg.altitude  = math.nan

        self.gps_nav_publisher.publish(msg)

    # Process IMU data
    imuCalRvel = -1
    imuCalRvec = -1
    imuCalLacc = -1

    def imuPublish(self, imuJsonPacket) :        
        # self.get_logger().info(f"imuPublish : {imuJsonPacket=}")

        # TODO: collect lacc, rvel, rvec packets with same seq# and publish imu message
        try :

            if "lacc" in imuJsonPacket :
                self.laccJsonPacket = imuJsonPacket.get("lacc")
            elif "rvel" in imuJsonPacket :
                self.rvelJsonPacket = imuJsonPacket.get("rvel")
            elif "rvec" in imuJsonPacket :
                self.rvecJsonPacket = imuJsonPacket.get("rvec")
            else :
                self.get_logger().error("IMU json does not have lacc, rvel or rvec")

            # process all 3 imu data sets when next set is available
            if self.laccJsonPacket!=None and  self.rvelJsonPacket!=None and  self.rvecJsonPacket!=None :
                # laccSeq = self.laccJsonPacket.get("seq")
                # rvelSeq = self.rvelJsonPacket.get("seq")
                # rvecSeq = self.rvecJsonPacket.get("seq")

                # seq numbers are not clustered properly to have the same seq# for each data type
                # This seems to work better when this is the only node running
                if True : #laccSeq==rvelSeq and rvelSeq==rvecSeq :
                    #self.get_logger().info(f"{laccSeq=} {rvelSeq=} {rvecSeq=}")
                    msg = Imu()

                    # NOTE: should we use the imu timestamp to get better accuracy?
                    msg.header.stamp = self.get_clock().now().to_msg()
                    # The IMU is located at the centroid of the rear differential 
                    # and aligned XY so no offest is needed
                    msg.header.frame_id = "imu_link"

                    #self.get_logger().info(f"imuPublish : {self.rvecJsonPacket=}")
                    x = float(self.rvecJsonPacket.get("i"))
                    y = float(self.rvecJsonPacket.get("j"))
                    z = float(self.rvecJsonPacket.get("k"))
                    w = float(self.rvecJsonPacket.get("real"))

                    # adjust yaw by -90 degrees: left-multiply q by q_rot = [0,0,s,c]
                    # use plain Python floats (faster than numpy for single scalar ops)
                    s = -0.7071067811865476  # -sqrt(2)/2
                    c =  0.7071067811865476  #  sqrt(2)/2

                    qx = x #c * x - s * y
                    qy = y #c * y + s * x
                    qz = z #c * z + s * w
                    qw = w #c * w - s * z

                    msg.orientation.x = qx
                    msg.orientation.y = qy
                    msg.orientation.z = qz
                    msg.orientation.w = qw

                    #self.get_logger().info(f"imuPublish : {self.rvelJsonPacket=}")
                    msg.angular_velocity.x =  float(self.rvelJsonPacket.get("x"))
                    msg.angular_velocity.y =  float(self.rvelJsonPacket.get("y"))
                    msg.angular_velocity.z =  float(self.rvelJsonPacket.get("z"))

                    #self.get_logger().info(f"imuPublish : {self.laccJsonPacket=}")
                    msg.linear_acceleration.x = float(self.laccJsonPacket.get("x"))
                    msg.linear_acceleration.y = float(self.laccJsonPacket.get("y"))
                    msg.linear_acceleration.z = float(self.laccJsonPacket.get("z"))

                    self.imu_msg_publisher.publish(msg)

                    # get calibration statuses
                    imuCalRvel = int(self.rvelJsonPacket.get("stat"))
                    imuCalRvec = int(self.rvecJsonPacket.get("stat"))
                    imuCalLacc = int(self.laccJsonPacket.get("stat"))

                    imuCalMsg = ImuCal()
                    imuCalMsg.rvel = imuCalRvel
                    imuCalMsg.rvec = imuCalRvec
                    imuCalMsg.lacc = imuCalLacc
                    self.imu_cal_publisher.publish(imuCalMsg)

                    if self.imuCalRvel != imuCalRvel :
                        self.tts(f"IMU rotation velocity cal {imuCalRvel}")
                        self.imuCalRvel = imuCalRvel
                    if self.imuCalRvec != imuCalRvec :
                        self.tts(f"IMU rotation vector cal {imuCalRvec}")
                        self.imuCalRvec = imuCalRvec
                    if self.imuCalLacc != imuCalLacc :
                        self.tts(f"IMU linear accelleration cal {imuCalLacc}")
                        self.imuCalLacc = imuCalLacc

                    # wait for 3 new packets
                    self.rvecJsonPacket = None
                    self.rvelJsonPacket = None
                    self.laccJsonPacket = None


                    # Publish the yaw headings from IMU orientation
                    (_,_,rad) =  tf_transformations.euler_from_quaternion([
                                            msg.orientation.x,
                                            msg.orientation.y,
                                            msg.orientation.z,
                                            msg.orientation.w])
                    yaw = Float32()
                    yaw.data = rad
                    self.cmp_azi_publisher.publish(yaw)

        except Exception as ex:
            self.get_logger().error(f"imuPublish json exception : {ex}")
            return


        # imuJsonStr = json.dumps(imuJsonPacket)
        # msg = String()
        # msg.data = imuJsonStr
        # self.imu_test_publisher.publish(msg)
    
    def destroy_node(self):
        self.get_logger().info("destroy_node")
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = ImuGpsNode()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()

    try :
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