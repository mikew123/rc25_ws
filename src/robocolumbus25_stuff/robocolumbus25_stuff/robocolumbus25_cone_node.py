from ctypes.wintypes import PMSG

from sympy import Point
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PointStamped
from geometry_msgs.msg import Quaternion
import tf_transformations
from vision_msgs.msg import Detection3DArray

class ConeNode(Node):
    '''
    Cone locations etc
    '''

    # median 5 filter memory
    # list of tupples [5X(x,y,z)]
    m5_filter:list = [(0.0,0.0,0.0)]*5

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
        num_detections = 0
        # TODO: filter out for best cone detection
        for detection in detections :
            num_detections +=1
            #header = detection.header
            results = detection.results
            for result in results :
                pose = result.pose.pose
                position = pose.position
                x = position.x
                y = position.y
                z = position.z
                #self.get_logger().info(f"msg: {x=:.3f} {y=:.3f} {z=:.3f}")

                # Use median-5 filter to remove points with distance spikes
                m5 = self.m5_filter
                # add new sample to filter list memory
                m5 = [(x,y,z),m5[0],m5[1],m5[2],m5[3]]
                self.m5_filter = m5
                # sort tupple[2] z is distance (m5 is not modified)
                m5s = sorted(m5, key=lambda xyz: xyz[2])
                # pick the median sample
                (xm,ym,zm) = m5s[2]
                #(xm,ym,zm) = m5[2] # DEBUG: unfiltered
                #(xm,ym,zm) = (x,y,z)
                #self.get_logger().info(f"{m5=} {m5s=}")
                
                # Publish the cone location point x,y,z
                pmsg = PointStamped()
                pmsg.header.frame_id="oak-d_frame"
                pmsg.point.x = zm # Forward meters
                pmsg.point.y = -xm # Side meters (-x fixes rviz location mapping)
                pmsg.point.z = ym # Elevation center of cone (on level ground)
                self.cone_point_publisher.publish(pmsg)

        if num_detections == 0 :
            # publish invalid cone location at 0,0 
            pmsg = PointStamped()
            pmsg.header.frame_id="oak-d_frame"
            self.cone_point_publisher.publish(pmsg)

def main(args=None):
    rclpy.init(args=args)

    node = ConeNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()