import rclpy
import math

from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection3DArray

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ConeNode(Node):
    '''
    Cone locations etc
    '''

    # median 5 filter memory for tof_foc data
    # list of tupples [5X(x,y,z)]
    m5_filter:list = [(0.0,0.0,0.0)]*5

    def __init__(self):
        super().__init__('cone_node')
            
        self.cone_point_publisher = self.create_publisher(PointStamped, 'cone_point', 10)

        self.cone_det_cam_subscription = self.create_subscription(Detection3DArray,"color/spatial_detections", 
                                            self.cone_det_cam_subscription_callback, 10)
                
        self.get_logger().info(f"ConeNode Started")


    # Cone detection from camera AI
    # TODO: manage multiple detections!!!! like an orange shoe or shirt
    def cone_det_cam_subscription_callback(self, msg: Detection3DArray) -> None:
        #self.get_logger().info(f"{msg=}")

        stamp = self.get_clock().now().to_msg()
        pmsg = PointStamped()
        pmsg.header.frame_id="oak-d_frame"
        pmsg.header.stamp = stamp
        detections = msg.detections
        num_detections = 0

        # TODO: filter out for best cone detection when multiple occur
        for detection in detections :
            num_detections +=1
            #header = detection.header
            results = detection.results
            bbox = detection.bbox
            bboxSize = bbox.size
            for result in results :
                pose = result.pose.pose
                position = pose.position
                x = position.x
                y = position.y
                z = position.z
                if z==0 : continue

                # use bounding box to "filter" non-cones
                bx0 = bboxSize.x
                by0 = bboxSize.y
                bx1 = 33*(2.8/z) # z is distance
                by1 = 59*(2.8/z)
                xx = math.fabs(1.0 - bx0/bx1)
                yy = math.fabs(1.0 - by0/by1)
                if z>1 : pct = 0.2
                else : pct = 0.5
                if (xx < pct) and (yy < pct) :
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
                    
                    # Publish the cone location point x,y,z relative to camera
                    pmsg.point.x = zm # Forward meters
                    pmsg.point.y = -xm # Side meters (-x fixes rviz location mapping)
                    pmsg.point.z = ym # Elevation center of cone (on level ground)
                    self.cone_point_publisher.publish(pmsg)
                    # self.get_logger().info(f"cone_callback: {stamp=} x={zm:.3f} y={-xm:.3f} z={ym:.3f} {bboxSize=}")

        if num_detections == 0 :
            # publish invalid cone location at 0,0 
            self.cone_point_publisher.publish(pmsg)
            pass
  
    def destroy_node(self):
        self.get_logger().info("destroy_node")
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = ConeNode()

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