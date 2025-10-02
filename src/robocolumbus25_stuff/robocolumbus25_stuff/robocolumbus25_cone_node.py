import rclpy
import math
import json
import time

from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
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
        self.cone_pose_publisher = self.create_publisher(PoseStamped, 'cone_pose_cam', 10)

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
    cameraConeDetection = False
    coneCounter = 0
    coneLastDetXYZ = (0.0,0.0,0.0)

    # Cone detection from camera AI
    # TODO: manage multiple detections!!!! like an orange shoe or shirt
    def cone_det_cam_subscription_callback(self, msg: Detection3DArray) -> None:
        # self.get_logger().info(f"{msg=}")

        if not self.cameraMsgDetected :
            # speak once at startup
            self.tts("Camera is active")
            self.cameraMsgDetected = True

        # default cone at 0,0,0 - Invalid
        x = 0.0
        y = 0.0
        z = 0.0

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
                cx = position.x
                cy = position.y
                cz = position.z
                if cz==0 : continue

                # use bounding box to "filter" non-cones
                bx0 = bboxSize.x
                by0 = bboxSize.y
                bx1 = 33*(2.8/cz) # z is distance
                by1 = 59*(2.8/cz)
                xx = math.fabs(1.0 - bx0/bx1)
                yy = math.fabs(1.0 - by0/by1)
                if z>1.2 : 
                    pctx = 0.35 # height when more than 1.2 meter
                    pcty = 0.25 # width
                else : 
                    pctx = 0.55 # top or bottom of cone may be out of FOV
                    pcty = 0.55 # width
                if (xx < pctx) and (yy < pcty) :
                    # Cone validated using bbox shape and size
                    num_detections +=1
                    # Use median-7 filter to remove points with distance spikes
                    m7 = self.m7_filter
                    # add new sample to filter list memory
                    m7 = [(cx,cy,cz),m7[0],m7[1],m7[2],m7[3],m7[4],m7[5]]
                    self.m7_filter = m7
                    # sort tupple[2] z is distance (m7 is not modified)
                    m7s = sorted(m7, key=lambda xyz: xyz[2])

                    # pick the middle sample after sorting
                    (mx,my,mz) = m7s[3]

                    # translate to ROS coordinates
                    (x,y,z) = (mz,-mx,my) # (x,y) and z=distance

                    # self.get_logger().info(f"cone detect: {x=:.2f}, {y=:.2f}, {z=:.2f}")
                    break # exit detections loop after 1st validated cone

        if self.cameraConeDetection :
            # change to not detected after N consecutive no cone
            if num_detections == 0 :
                self.coneCounter +=1
                if self.coneCounter > 10 :
                    # 10 consecutive cone not detected
                    self.cameraConeDetection = False
                    self.tts("Cam no cone detect")

                else :
                    # Send last detected cone coordinates
                    (x,y,z) = self.coneLastDetXYZ
                
            else : # cone detection
                # reset counter when cone is detected
                self.coneCounter = 0
                self.coneLastDetXYZ = (x,y,z)

        else : # Cone not detected
            if num_detections>0 and x>0:
                self.coneCounter +=1
                if self.coneCounter > 4 :
                    # 4 consecutive cones detected
                    self.cameraConeDetection = True
                    self.tts(f"Cam cone detect {x=:.2f} {y=:.2f}")
                    self.coneLastDetXYZ =(x,y,z)

            else :
                # reset cone counter when cone not detected
                self.coneCounter = 0

        self.pointPublish("Camera",x,y,z)

  
    def pointPublish(self, sensor:String, x:float,y:float,z:float) -> None :
        """
        Publish a PointStamped msg topic
        The sensor name chooses the reference frame and publisher to use
        """
        # self.get_logger().info(f"pointPublish: {sensor}, {x=:.2f}, {y=:.2f}, {z=:.2f}")
        stamp = self.get_clock().now().to_msg()
        pmsg = PointStamped()
        pmsg.header.stamp = stamp
        pmsg.point.x = x # Forward meters from sensor
        pmsg.point.y = y # Side meters from sensor
        pmsg.point.z = z # Elevation of center of the cone from sensor level

        if sensor == "Camera" :
            pmsg.header.frame_id="oak-d_frame"
            self.cone_point_publisher.publish(pmsg)



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