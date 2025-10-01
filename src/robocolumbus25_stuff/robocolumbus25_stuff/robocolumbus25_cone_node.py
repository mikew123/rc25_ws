import rclpy
import math
import json
import time

from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection3DArray
from std_msgs.msg import String

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ConeNode(Node):
    '''
    Cone locations etc
    '''

    # median 5 filter memory for tof_foc data
    # list of tupples [7X(x,y,z)]
    m7_filter:list = [(0.0,0.0,0.0)]*7

    def __init__(self):
        super().__init__('cone_node')
            

        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "json_msg"
                                        , self.json_msg_callback, 10)

        self.cone_point_publisher = self.create_publisher(PointStamped, 'cone_point', 10)

        self.cone_det_cam_subscription = self.create_subscription(Detection3DArray,"color/spatial_detections", 
                                            self.cone_det_cam_subscription_callback, 10)

        time.sleep(2) # wait for json_msg_publisher to be ready!!??
        self.tts("Cone Node Started")               
        self.get_logger().info(f"ConeNode Started")

    def tts(self, tts) -> None:
        """
        Send text to speaker 
        """
        json_msg = {"speaker":{"tts":tts}}
        self.sendJsonMsg(json_msg)

    def sendJsonMsg(self, json_msg) -> None :
        #self.get_logger().info(f"{json_msg=}")
        str = json.dumps(json_msg)
        msg = String(data=str)
        self.json_msg_publisher.publish(msg)

    def json_msg_callback(self,msg) :
        pass

    cameraMsgDetected = False
    cameraConeDetected = False
    coneCounter = 0

    # Cone detection from camera AI
    # TODO: manage multiple detections!!!! like an orange shoe or shirt
    def cone_det_cam_subscription_callback(self, msg: Detection3DArray) -> None:
        #self.get_logger().info(f"{msg=}")

        if not self.cameraMsgDetected :
            self.tts("Camera is active")
            self.cameraMsgDetected = True

        stamp = self.get_clock().now().to_msg()
        pmsg = PointStamped()
        pmsg.header.frame_id="oak-d_frame"
        pmsg.header.stamp = stamp
        detections = msg.detections
        num_detections = 0

        # TODO: filter out for best cone detection when multiple occur
        for detection in detections :
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
                if z>1 : pct = 0.3
                else : pct = 0.6
                if (xx < pct) and (yy < pct) :
                    # Cone validated using bbox shape and size
                    num_detections +=1
                    # Use median-5 filter to remove points with distance spikes
                    m7 = self.m7_filter
                    # add new sample to filter list memory
                    m7 = [(x,y,z),m7[0],m7[1],m7[2],m7[3],m7[4],m7[5]]
                    self.m7_filter = m7
                    # sort tupple[2] z is distance (m7 is not modified)
                    m7s = sorted(m7, key=lambda xyz: xyz[2])
                    # pick the median sample
                    (xm,ym,zm) = m7s[3]
                    #(xm,ym,zm) = m7[3] # DEBUG: unfiltered
                    #(xm,ym,zm) = (x,y,z)
                    #self.get_logger().info(f"{m7=} {m7s=}")
                    
                    # Publish the cone location point x,y,z relative to camera
                    pmsg.point.x = zm # Forward meters
                    pmsg.point.y = -xm # Side meters (-x fixes rviz location mapping)
                    pmsg.point.z = ym # Elevation center of cone (on level ground)
                    self.cone_point_publisher.publish(pmsg)
                    # self.get_logger().info(f"cone_callback: {stamp=} x={zm:.3f} y={-xm:.3f} z={ym:.3f} {bboxSize=}")

                    break # exit detections loop after 1st validated cone

        if self.cameraConeDetected :
            # change to not detected after N consecutive no cone
            if num_detections == 0 :
                self.coneCounter +=1
                if self.coneCounter > 10 :
                    self.cameraConeDetected = False
                    self.tts("Camera lost cone detection")
            else :
                # reset when cone is detected
                self.coneCounter = 0
        else : # Cone not detected
            if num_detections>0 and pmsg.point.x>0:
                self.cameraConeDetected = True
                x = pmsg.point.x
                y = pmsg.point.y
                self.tts(f"Camera cone detected at {x=:.2f} {y=:.2f}")
            else :
                # pulish 0,0,0 cone location - invalid
                pmsg.point.x = 0.0
                pmsg.point.y = 0.0
                pmsg.point.z = 0.0
                self.cone_point_publisher.publish(pmsg)

        # if num_detections == 0 :
        #     # publish invalid cone location at 0,0 
        #     self.cone_point_publisher.publish(pmsg)
        #     if self.cameraConeDetected :
        #         # self.tts("Lost Camera AI cone detection")
        #         self.cameraConeDetected = False
        # else :
        #     if not self.cameraConeDetected :
        #         x = pmsg.point.x
        #         y = pmsg.point.y
        #         # self.tts(f"Camera AI cone has been detected at {x=} {y=}")
        #         self.cameraConeDetected = True

  
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