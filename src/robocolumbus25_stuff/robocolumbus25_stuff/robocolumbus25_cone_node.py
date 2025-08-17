from ctypes.wintypes import PMSG
import rclpy
import json
import serial
import math
import time
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose, PointStamped
from geometry_msgs.msg import Quaternion
import tf_transformations
from vision_msgs.msg import Detection3DArray

class ConeNode(Node):
    '''
    Locates cones etc
    '''
    
    def __init__(self):
        super().__init__('cone_node')
            
        self.cone_point_publisher = self.create_publisher(PointStamped, 'cone_point', 10)

        self.cone_det_subscription = self.create_subscription(Detection3DArray,"color/spatial_detections", self.cone_det_subscription_callback, 10)
                
        self.get_logger().info(f"ConeNode Started")

    # Cone detection from camera AI
    # TODO: manage multiple detections!!!! like an orange shoe or shirt
    def cone_det_subscription_callback(self, msg: Detection3DArray) -> None:
        #self.get_logger().info(f"{msg=}")
        detections = msg.detections
        for detection in detections :
            #header = detection.header
            results = detection.results
            for result in results :
                pose = result.pose.pose
                position = pose.position
                x = position.x
                y = position.y
                z = position.z
                #self.get_logger().info(f"msg: {x=:.3f} {y=:.3f} {z=:.3f}")

                pmsg = PointStamped()
                pmsg.header.frame_id="oak-d_frame"
                pmsg.point.x = z # Forward meters
                pmsg.point.y = x # Side meters
                pmsg.point.z = 0.0 # No elevation
                self.cone_point_publisher.publish(pmsg)


def main(args=None):
    rclpy.init(args=args)

    node = ConeNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()