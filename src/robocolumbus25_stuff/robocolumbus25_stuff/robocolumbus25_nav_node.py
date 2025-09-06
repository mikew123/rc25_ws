from ctypes.wintypes import PMSG
import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
import tf_transformations
from vision_msgs.msg import Detection3DArray

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import Duration
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rc25_interfaces.msg import Float32X8

class NavNode(Node):
    '''
    Navigate to cones
    '''

    # parameters

    # Timer for state machine
    timerRateHz:float = 10.0

    cd_timer:float = 0.0
    cd_state:int = 0
    cd_last_state = -1

    # use /cone_point to get location for navigator to drive to
    cd_stop_dist = 1.0
    cd_drive_t = 5.0

    # use /cone_point to get closer to cone using /cmd_vel
    cd_closer_dist = 0.5
    cd_closer_lvel = 0.25
    cd_closer_avel = 0.05

    # use /tof_fc_mid to "touch" cone using /cmd_vel
    cd_touch_dist = 0.035
    cd_touch_vel = 0.05

    # wait while touching cone for observation
    cd_touch_wait = 2.0

    # backup after touching cone using /cmd_vel
    cd_backup_dist = 1.5
    cd_backup_vel = 0.25


    # global variables 

    # for cone navigation
    # using /cone_point 
    cone_at_x:float = 0.0
    cone_at_y:float = 0.0
    cone_det_time:int = 0
    # using /tof_fc_mid (NOTE: distance from front center TOF sensor)
    tof_fc_dist:float = 0.0

    # median 5 filter memory
    # list of tupples [5X(x,y,z)]
    m5_filter:list = [(0.0,0.0,0.0)]*5
    


    def  __init__(self, nav: BasicNavigator):
        super().__init__('robocolumbus25_nav_node')
        self.nav = nav

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cone_point_subscription = self.create_subscription(PointStamped, 'cone_point', self.cone_point_subscription_callback, 10)
        self.tof_fc_mid_subscription = self.create_subscription(Float32X8, 'tof_fc_mid', self.tof_fc_mid_subscription_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)

        self.get_logger().info(f"NavNode Started")

    # Cone detection from camera AI
    def cone_point_subscription_callback(self, msg: PointStamped) -> None:
        #self.get_logger().info(f"{msg=}")
        self.cone_at_x = msg.point.x
        self.cone_at_y = msg.point.y
        self.cone_det_time = time.time_ns() * 1e-9 # seconds

    # Cone distance when close using TOF sensors
    def tof_fc_mid_subscription_callback(self, msg: Float32X8) -> None:
        #self.get_logger().info(f"{msg=}")
        davg:float = 0.0
        dcnt:int = 0
        for d in msg.data :
            if not math.isinf(d) :
                davg += d
                dcnt += 1
        if dcnt!=0 : davg /= dcnt
        else : davg = math.inf
        self.tof_fc_dist = davg

    # Timer based state machine for cone navigation
    def timer_callback(self):
        self.cone_sm()


    def cone_sm(self) :
        func = "cone_sm:"

        # self.get_logger().info(f"timer_callback: waitUntilNav2Active")
        # self.nav.waitUntilNav2Active()

        x = self.cone_at_x
        y = self.cone_at_y
        t = self.cone_det_time

        if (time.time_ns() * 1e-9) - t > 1.0 :
            # cone detection is stale
            x = 0
            y = 0
        
        state_change:bool = False
        if self.cd_state != self.cd_last_state : state_change = True

        state:int = self.cd_state
        next_state:int = state

        if state_change :
            self.cd_timer = time.time_ns()*1e-9

        if state == 0 :
            if state_change :
                self.get_logger().info(f"{func} wait for cone detection {state=}")
            
            if x!=0 and y!=0 :
                self.get_logger().info(f"{func} cone detected at {x=:.3f} {y=:.3f}  {state=}")
                next_state = 1

        elif state == 1 :
            if state_change :
                self.get_logger().info(f"{func} navigate with BasicNavigator close to cone {state=}")
                
            if x!=0 and y!=0 :
                dist = self.cd_stop_dist
                t = self.cd_drive_t
                d = float(self.gotoConeXY(x,y,dist,t))
                if d != -1.0 :
                    self.get_logger().info(f"{func} at cone {d=:.3f}  {state=}")
                    if d <= dist + 0.1 :
                        self.get_logger().info(f"{func} close to cone  {state=}")
                        next_state = 2
                else :
                    self.get_logger().info(f"{func} gotoConeXY failed  {state=}")
                    next_state = 0
            else :
                self.get_logger().info(f"{func} lost cone {state=}")
                next_state = 0

        elif state == 2 :
            if state_change :
                self.get_logger().info(f"{func} drive closer to cone using cmd_vel and cone_point {state=}")
            
            msg = Twist()
            if x==0 and y==0 :
                self.get_logger().info(f"{func} lost cone {state=}")
                next_state = 0
            elif x > self.cd_closer_dist :
                msg.linear.x =  self.cd_closer_lvel
                # Y is used to drive to cone head on
                if y > 0.02 :    msg.angular.z =  self.cd_closer_avel
                elif y < -0.02 : msg.angular.z = -self.cd_closer_avel
                else : msg.angular.z = 0.0
            else : 
                self.get_logger().info(f"{func} closer {x=:.3f} {y=:.3f}")
                next_state = 3
            self.cmd_vel_publisher.publish(msg)

        elif state == 3 :
            if state_change :
                self.get_logger().info(f"{func} drive slowly to \"touch\" cone using cmd_vel and tof sensors {state=}")
            
            msg = Twist()
            d = self.tof_fc_dist
            if((not math.isinf(d)) and (d > self.cd_touch_dist)) :
                msg.linear.x = self.cd_touch_vel
            else : next_state = 4
            self.cmd_vel_publisher.publish(msg)

        elif state == 4 :
            if state_change :
                self.get_logger().info(f"{func} wait a short time {state=}")
            
            if (time.time_ns()*1e-9 - self.cd_timer) > self.cd_touch_wait :
                next_state = 5

        elif state == 5 :
            if state_change :
                self.get_logger().info(f"{func} backup {state=}")
            
            msg = Twist()
            t = self.cd_backup_dist/self.cd_backup_vel
            if (time.time_ns()*1e-9 - self.cd_timer) < t :
                msg.linear.x = -self.cd_backup_vel
            else : next_state = 6
            self.cmd_vel_publisher.publish(msg)

        elif state == 6 :
            if state_change :
                self.get_logger().info(f"{func} STOP {state=}")
            pass


        self.cd_last_state = state
        self.cd_state = next_state

        
    # Returns distance to the cone, -1 if it did not succeed
    def gotoConeXY(self, cx:int, cy:int, stop_dist:float, t:float = 5) -> float:
        """
        Go to the cone location, but stop 0.2m short
        Rotates to point to the can before moving to it
        gets new can position every 1 second as it approaches it
        Returns distance to the can, -1 if it did not succeed
        """

        d=-1.0

        if cx!=0 and cy!=0 :
            self.get_logger().info(f"gotoConeXY:b cone at {cx=:.3f} {cy=:.3f}")
            # get current pose to determine the angle offset
            (tf_OK, current_pose) = self.getCurrentPose()
            if not tf_OK : return -1.0
            rx = current_pose.pose.position.x
            ry = current_pose.pose.position.y
            (_,_,ra) =  tf_transformations.euler_from_quaternion([
                                    current_pose.pose.orientation.x,
                                    current_pose.pose.orientation.y,
                                    current_pose.pose.orientation.z,
                                    current_pose.pose.orientation.w])
            self.get_logger().info(f"gotoConeXY:b robot at {rx=:.3f} {ry=:.3f}, {ra=:.3f}")

            dx = float(cx) - rx
            dy = float(cy) - ry
            # Calc angle to target XY coordinate
            a = math.atan2(dy,dx)
            d = math.sqrt(dx*dx + dy*dy)
            
            if d < stop_dist : return d

            # adjust the distance stop at the cost map boundary
            sd = d - stop_dist
            x = rx + sd*math.cos(a)
            y = ry + sd*math.sin(a)
            
            goto_pose = self.createPose(x,y,a)
            self.get_logger().info(f"gotoConeXY: goto {x=:.3f} {y=:.3f} {sd=:.3f} {a=:.3f} {t=:.3f}")

            # drive toward the cone for a short time before getting new position estimate
            status = self.gotoPose(goto_pose, t)
            # if status != TaskResult.SUCCEEDED : d = -1

        else :
            self.get_logger().info(f"gotoXY: No cone detected")
            d=-1.0

        return float(d)


    def gotoPose(self, pose, t):
        """
        Go to the pose within in the time limit
        """

        self.nav.goToPose(pose)
        # (result, feedback) = self.waitTaskComplete(t)
        (result, _) = self.waitTaskComplete(t)
       
        return result


    def getCurrentPose(self) -> tuple:
        """
        get map->tof_fc_link (front center tof sensor) transform
        returns (tf_OK, pose) pose: PoseStamped
        """
        return self.getPoseFromTF('tof_fc_link')


    def getPoseFromTF(self, target_frame:str) -> tuple:
        """
        get map->'target_frame' transform
        returns (tf_OK, pose) pose: PoseStamped
        """
        # try getting pose a few times
        cnt = 0
        tf_OK = False
        while tf_OK == False and cnt < 5 :
            try:
                tf = self.tf_buffer.lookup_transform (
                    'map',
                    target_frame,
                    #self.nav.get_clock().now().to_msg(),
                    rclpy.time.Time(), # default 0
                    timeout=rclpy.duration.Duration(seconds=0.1) #0.0)
                    )
                tf_OK = True

            except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().info(f'getPoseFromTF: Could not find transform map->{target_frame}: {ex}')
                tf_OK = False
                cnt += 1
                
        if tf_OK == False or cnt >= 5 :
            self.get_logger().info(f'getPoseFromTF: Failed to find transform map->{target_frame} after {cnt} tries {tf_OK=}')
            return (False,None) 
        
        # translate wall points to align with map coordinates
        if tf_OK :
            # get x, y, theta from TF
            x:float = tf.transform.translation.x
            y:float = tf.transform.translation.y
            q:float = tf.transform.rotation
            # convert quaterion to euler, discard xx and yy
            (xx,yy,a) = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            pose = self.createPose(x,y,a)
        else :
            pose = None
            
        return (tf_OK,pose)
        

    def createPose(self,x,y,a) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        (pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w) = tf_transformations.quaternion_from_euler(0.0,0.0,float(a))
        # self.get_logger().info(pose)
        return pose


    def waitTaskComplete(self,t) :
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            # self.get_logger().info(f"{feedback=}")
            # spin does not provide time in feedback
            try :
                nt = feedback.navigation_time.sec
                if nt > t :
                    self.get_logger().info(f"waitTaskComplete: Canceling task {nt=} > {t=}")
                    self.nav.cancelTask()
            except :
                pass
                
        feedback = self.nav.getFeedback()
        result = self.nav.getResult()
        # self.get_logger().info(f"{feedback=} {result=}")
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('waitTaskComplete: Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('waitTaskComplete: Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('waitTaskComplete: Goal failed!')
        else :
            self.get_logger().info(f"waitTaskComplete: nav.getResult() {result=}")

        return (result, feedback)


def main(args=None):
    rclpy.init(args=args)

    nav = BasicNavigator()
    node = NavNode(nav)
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()