from ctypes.wintypes import PMSG
import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
import tf_transformations
from vision_msgs.msg import Detection3DArray

class NavNode(Node):
    '''
    Navigate to cones
    '''

    # median 5 filter memory
    # list of tupples [5X(x,y,z)]
    m5_filter:list = [(0.0,0.0,0.0)]*5
    
    def __init__(self):
        super().__init__('nav_node')
            
        #self.cone_point_publisher = self.create_publisher(PointStamped, 'cone_point', 10)

        self.cone_point_subscription = self.create_subscription(PointStamped, 'cone_point', self.cone_point_subscription_callback, 10)
                
        self.get_logger().info(f"NavNode Started")

    # Cone detection from camera AI
    def cone_point_subscription_callback(self, msg: PointStamped) -> None:
        #self.get_logger().info(f"{msg=}")
        x = msg.point.x
        y = msg.point.y
        
        # Calc bearing to cone relative to camera 
        robotX = 0
        robotY = 0
        z = math.atan2(y,x)

        self.get_logger().info(f"{x=:.3f} {y=:.3f} {z=:.3f}")


def main(args=None):
    rclpy.init(args=args)

    node = NavNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()