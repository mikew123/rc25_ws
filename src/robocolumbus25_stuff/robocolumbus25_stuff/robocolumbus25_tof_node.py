#from ctypes.wintypes import PMSG
import rclpy
import json
import serial
import math
#import time
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

#import tf_transformations
import numpy as np

class TofNode(Node):
    '''
    This processes the serial port from the RP2040 for the fornt and rear
    TOF sensors
    '''

    timerRateHz = 100.0; # Rate to check serial port for messages
    serial_port = "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_45533065790A3B5A-if00"
    baudrate = 1000000


    
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
        self.tof_fc_pcd_publisher = self.create_publisher(PointCloud2, 'tof_fc', 10)
        self.tof_fl_pcd_publisher = self.create_publisher(PointCloud2, 'tof_fl', 10)
        self.tof_fr_pcd_publisher = self.create_publisher(PointCloud2, 'tof_fr', 10)
      
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
                    self.tof_Publish("tof_fc", packet)
                    unknown = False
                if "tof_fl" in packet :
                    self.tof_Publish("tof_fl", packet)
                    unknown = False
                if "tof_fr" in packet :
                    self.tof_Publish("tof_fr", packet)
                    unknown = False

                if unknown :
                    self.get_logger().info(f"TOF sensors serial json unknown : {received_data}")
                    return  
            except Exception as ex:
                self.get_logger().error(f"TOF sensors serial json Exception {ex} : {received_data}")
                return

    # 8x8 point cloud for each sensor FOV 45degx45deg
    # calculate x,y,z for each point
    # TODO: Optimize math with numpy
    def tof_Publish(self, tof_ab, packet) -> None:
        # self.get_logger().info(f"tof_Publish: {tof_ab=} {packet=}")

        tof = packet.get(tof_ab)
        if "dist" in tof :
            dist = tof.get("dist")
        else :
            self.get_logger().error(f"tof_Publish: no dist data")
            return
        
        # self.get_logger().info(f"tof_Publish: {dist=}")
        # return

        fov = 45.0
        fovPt = fov/8 # FOV for each 8x8 sensor point
        fovPtRad = fovPt*(math.pi/180) #scaled to Radians

        # There is a curvature in the distances of the sensors that needs to be corrected
        # Remove the curve by scaling each sensor with a inverted sin() curve over FOV
        # NOTE: This could be pre-computed outside the function since it is constant
        tofCurveCor = []
        for n in range(0,8) :
            theta:float = (n-4+0.5)*fovPtRad + math.pi/2
            s:float = math.sin(theta)
            if n == 0 : s0 = s
            tofCurveCor.append(s0/s)            

        # pointcloud is a list of tupples(x,y,z)
        xyz0:list = []
        for m in range(0,8) : # Rows bottom to top
            theta_m = (m-4+0.5)*fovPtRad
            d_m = dist[m]
            for n in range(0, 8) : # Collumns left to right
                theta_n = (n-4+0.5)*fovPtRad
                d = d_m[n]
                if d==-1: 
                    # Bad data - set as infinate number (use NaN instead?)
                    xx0 = math.inf
                    yy0 = math.inf
                    zz0 = math.inf
                else :
                    Wx =  int(d*math.cos(theta_n)*tofCurveCor[n])
                    Wy = -int(d*math.sin(theta_n)*tofCurveCor[n])
                    Wz =  int(d*math.sin(theta_m)*tofCurveCor[n])
                    # Convert mm to meters
                    xx0 = np.float32(Wx/1000.0)
                    yy0 = np.float32(Wy/1000.0)
                    zz0 = np.float32(Wz/1000.0)
                
                xyz0.append((xx0,yy0,zz0))
        
        # self.get_logger().info(f"tof_Publish: {xyz0=}")

        if tof_ab == "tof_fc" :
            pcd = self.point_cloud(xyz0, 'tof_fc_link')
            self.tof_fc_pcd_publisher.publish(pcd)
        if tof_ab == "tof_fl" :
            pcd = self.point_cloud(xyz0, 'tof_fl_link')
            self.tof_fl_pcd_publisher.publish(pcd)
        if tof_ab == "tof_fr" :
            pcd = self.point_cloud(xyz0, 'tof_fr_link')
            self.tof_fr_pcd_publisher.publish(pcd)

    def point_cloud(self, points_xy:list[tuple[np.float32]], parent_frame:str="map") -> PointCloud2:
        """
            Input list of tuples (x,y,z)
            Returns a point cloud to publish - Rviz can display it
        """
        points = np.asarray(points_xy)

        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate
        fields = [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        #self.get_logger().info(f"{itemsize = } {fields = } {points = } {data = }")

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = Header(
            frame_id=parent_frame,
            stamp = self.get_clock().now().to_msg(),
            )

        return PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False, #Pi4
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of two float32s.
            row_step=(itemsize * 3 * points.shape[0]), 
            data=data
        )

def main(args=None):
    rclpy.init(args=args)

    node = TofNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()