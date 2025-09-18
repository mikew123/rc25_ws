import rclpy
import math
import time
import tf_transformations
import json
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import Duration
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rc25_interfaces.msg import Float32X8

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class NavNode(Node):
    '''
    Navigate to cones
    '''

    # parameters

    # Timer for state machine
    smTimerRateHz:float = 10.0

    tc_state = -1
    tc_next_state = 0

    cd_timer:float = 0.0
    cd_state:int = 0
    cd_last_state = -1

    cd_sub_state = -1

    # state=1 use /cone_point to get location for navigator to drive to
    # Distance from detected cone to end navigation
    cd_stop_dist = 1.5
    # nav this aamout of time before getting new cone fix
    cd_nav_time = 2.5

    # state=2 use /Lidar +-22.5 FOV to get closer to cone using /cmd_vel
    cd_closer_dist = 0.5
    cd_closer_lvel = 0.25
    cd_closer_avel = 2*cd_closer_lvel

    # state=3 use /tof_fc_mid to "touch" cone using /cmd_vel
    cd_touch_dist = 0.040
    cd_touch_lin_vel = 0.05
    cd_touch_ang_vel = 0.05

    # wait while touching cone for observation
    cd_touch_wait = 2.0

    # backup after touching cone using /cmd_vel
    cd_backup_dist = 1.5
    cd_backup_vel = 0.25

    # Field of view pointing forward from Lidar 36 degree scan data 
    fovRad = 0.758 # 45 deg, +-22.5 degrees


    # GLOBAL variables 

    # for cone navigation
    # using /cone_point 
    cone_at_x_cam:float = 0.0
    cone_at_y_cam:float = 0.0
    cone_at_d_cam:float = 0.0
    cone_at_a_cam:float = 0.0
    cone_det_time_cam:int = 0
    cone_det_time_out_cam:int = 2.0

    # using Lidar
    # cone_at_x_lidar:float = 0.0
    # cone_at_y_lidar:float = 0.0
    # cone_at_d_lidar:float = 0.0
    # cone_at_a_lidar:float = 0.0

    # using /tof_fc_mid (NOTE: distance from front center TOF sensor)
    cone_at_x_tof_fc:float = 0.0
    cone_at_y_tof_fc:float = 0.0
    cone_at_d_tof_fc:float = 0.0
    cone_at_a_tof_fc:float = 0.0
    cone_dist_idx_min_tof_fc:int = -1

    # median 5 filter memory
    # list of tupples [5X(x,y,z)]
    m5_filter:list = [(0.0,0.0,0.0)]*5
    
    # True when killSw is released to stop the robot motion
    killSw:bool = False
    cd_killSwChange:bool = False

    def  __init__(self, nav: BasicNavigator):
        super().__init__('robocolumbus25_nav_node')
        self.nav = nav

        self.cb_group = MutuallyExclusiveCallbackGroup()

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cone_point_subscription = self.create_subscription(PointStamped, 'cone_point'
                                            , self.cone_point_subscription_callback, 10)
        self.tof_fc_mid_subscription = self.create_subscription(Float32X8, 'tof_fc_mid'
                                            , self.tof_fc_mid_subscription_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan,"scan" 
                                            , self.lidar_subscription_callback, 10)

        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "json_msg"
                                            , self.json_msg_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # wait for navigator before turning on timer
        # This acts oddly and never returns - do I need to be lifecycle? can it be in init?
        # self.nav.waitUntilNav2Active()

        # The state machine timer runs a state machine, some functions take a lot of time (>50ms)
        # It gets its own callback group, all others use the default group
        self.sm_timer = self.create_timer((1.0/self.smTimerRateHz), self.sm_timer_callback
                                       , callback_group=self.cb_group)

        self.get_logger().info(f"NavNode Started")

    def sendJsonMsg(self, json_msg) -> None :
        str = json.dumps(json_msg)
        msg = String(data=str)
        self.json_msg_publisher.publish(msg)

    def json_msg_callback(self, msg:String) -> None :
        #self.get_logger().info(f"json_msg_callback: {msg=}")
        data = json.loads(msg.data)

        if 'kill' in data:
            kill:bool = data['kill']
            self.processKillSwStatus(kill)

    def processKillSwStatus(self, kill:bool) -> None :
        if(kill != self.killSw) :
            self.cd_killSwChange = True
            self.get_logger().info(f"json_msg_callback: kill switch is {kill}")
        self.killSw = kill

    # Timer based state machine for cone navigation
    def sm_timer_callback(self):

        # need a main state machine
        next_state = self.tc_next_state
        if self.tc_state!= next_state :
            self.get_logger().info(f"sm_timer_callback: state change to {next_state}")
        state = next_state
        self.tc_state = state

        if state == 0 :
            # wait for navigator
            # This does not work, it waits forever and send out messages like setting inital pose
            # initialPose = self.createPose(0,0,0,"map")
            # self.nav.setInitialPose(initialPose)
            # self.nav.waitUntilNav2Active()

            # Rviz 2DGoalPose can be used to move robot until a cone is detected
            # get cone xy from camera detect
            x:float = self.cone_at_x_cam
            y:float = self.cone_at_y_cam
            t:int = self.cone_det_time_cam
            to:int = self.cone_det_time_out_cam
            dt:int = (time.time_ns()*1e-9) - t
            # self.get_logger().info(f"{func} {dt=} {t=} {x=} {y=} ")
            if dt>to and t>0:
                self.get_logger().info(f"sm_timer: cone detection is stale {x=} {y=} {dt=}")
                x = 0
                y = 0
            if x!=0 :
                # cancel navigation initiated by rviz
                self.nav.cancelTask()
                next_state = 1

        # initialize at start point and wait for start command

        # goto GPS coordinate

        elif state == 1 :
            # find cone and "touch" it
            self.cd_sm()
            pass

        # set next GPS coordinate 

        self.tc_next_state = next_state

    # Executes in timer callback group, sensor callbacks are in parallel
    def cd_sm(self) :
        func = "cone_sm:"

        
        state_change:bool = False
        if self.cd_state != self.cd_last_state : 
            state_change = True

        ks = self.killSw
        ksc = self.cd_killSwChange
        if ksc : self.cd_killSwChange = False

        state:int = self.cd_state
        next_state:int = state

        if state_change :
            self.cd_timer = time.time_ns()*1e-9

        if state == 0 :
            next_state = self.wait_for_cone_det(func,state,state_change,ks,ksc)

        elif state == 1 :
            next_state = self.nav_to_cone(func,state,state_change,ks,ksc)
                
        elif state == 2 :
            next_state = self.get_closer_to_cone(func,state,state_change,ks,ksc)

        elif state == 3 :
            next_state = self.touch_cone(func,state,state_change,ks,ksc)

        elif state == 4 :
            next_state = self.wait_after_touch(func,state,state_change,ks,ksc)

        elif state == 5 :
            next_state = self.backup_after_touch(func,state,state_change,ks,ksc)

        elif state == 6 :
            next_state = self.stop_after_touch(func,state,state_change,ks,ksc)

        self.cd_last_state = state
        self.cd_state = next_state

    # state 0 - wait for camera cone detection
    def wait_for_cone_det(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} wait for cone detection {state=}")

        # get cone xy from camera detect
        x:float = self.cone_at_x_cam
        y:float = self.cone_at_y_cam
        t:int = self.cone_det_time_cam
        to:int = self.cone_det_time_out_cam
        dt:int = (time.time_ns()*1e-9) - t
        # self.get_logger().info(f"{func} {dt=} {t=} {x=} {y=} ")
        if dt>to and t>0:
            self.get_logger().info(f"{func} cone detection is stale {x=} {y=} {dt=}")
            x = 0
            y = 0

        killSwitchActive:bool  = ks
        next_state:int = state

        if x!=0 :
            self.get_logger().info(f"{func} cone detected at {x=:.3f} {y=:.3f}  {state=} {killSwitchActive=}")
            if not killSwitchActive : 
                next_state = 1
        return next_state

    #state 1 - use camera to locate cone and get close
    cd_sub_timer = 0
    def nav_to_cone(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        cur_time = time.time_ns()*1e-9
        if state_change :
            self.get_logger().info(f"{func} navigate with BasicNavigator close to cone {state=}")
            self.cd_sub_state = 0

        # get cone xy from camera detect
        x:float = self.cone_at_x_cam
        y:float = self.cone_at_y_cam
        t:int = self.cone_det_time_cam
        to:int = self.cone_det_time_out_cam
        dt:int = (time.time_ns()*1e-9) - t
        # self.get_logger().info(f"{func} {dt=} {t=} {x=} {y=} ")
        if dt>to and t>0:
            self.get_logger().info(f"{func} cone detection is stale {x=} {y=} {dt=}")
            x = 0
            y = 0

        killSwitchActive:bool = ks
        killSwitchChange = ksc
        next_state = state

        if x!=0 :
            dist = self.cd_stop_dist
            t = self.cd_nav_time

            # non-blocking navigation
            if self.cd_sub_state == 0 :
                if not killSwitchActive :
                    # issue a navigation command
                    # x,y is relative to tof_fc sensor
                    self.gotoConeXY(x,y,dist,t) # non-blocking
                    self.cd_sub_timer = cur_time
                    self.cd_sub_state = 1

            elif self.cd_sub_state == 1 :
                if killSwitchActive :
                    # Cancel goal and wait when kill switch status changes to active
                    if killSwitchChange :
                        self.nav.cancelTask()
                        self.cd_sub_state = 0
                else : # Execute when kill switch is not active
                    # check for nav complete or navigate time finished and try again
                    if (cur_time - self.cd_sub_timer) < t :
                        if self.nav.isTaskComplete() :
                            self.nav.cancelTask()
                            # x,y is relative to tof_fc sensor
                            d = math.sqrt(x*x + y*y)

                            if d <= dist + 0.1 :
                                self.get_logger().info(f"{func} nav done - close to cone {d=:.3f} {state=}")
                                next_state = 2
                            else :
                                self.get_logger().info(f"{func} nav again closer to cone {d=:.3f} {state=}")
                                self.cd_sub_state = 0
                    else :
                        self.get_logger().info(f"{func} Get new cone placement and navigate some more {state=}")
                        self.nav.cancelTask()
                        self.cd_sub_state = 0

        else :
            self.get_logger().info(f"{func} lost cone {state=}")
            self.nav.cancelTask()
            next_state = 0

        return next_state

    #state 2 - use Lidar to get closer to cone
    def get_closer_to_cone(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} drive closer to cone using cmd_vel and cone_point {state=}")
            self.nav.cancelTask()

        if (time.time_ns()*1e-9 - self.cd_timer) < 2.0 : return state

        # # get cone xy from Lidar data
        # x:float = self.cone_at_x_lidar
        # y:float = self.cone_at_y_lidar
        # a:float = self.cone_at_a_lidar

        x:float = self.cone_at_x_cam
        y:float = self.cone_at_y_cam
        a:float = self.cone_at_a_cam
        d:float = self.cone_at_d_cam

        x -= 0.200 # Crude transformation, camera is 0.200 behind tof sensor
        d -= 0.200 # Crude transformation, camera is 0.200 behind tof sensor

        killSwitchActive:bool = ks
        next_state = state
        msg = Twist()

        if killSwitchActive :
            # Stop motors and wait for kill switch not active
            # Motors are stopped by leaving velocities to msg defaults as 0
            pass
        elif d == 0 :
            self.get_logger().info(f"{func} lost cone {state=}")
            next_state = 0
        elif d > 2*self.cd_stop_dist :
            self.get_logger().info(f"{func} cone is too far {d=} {state=}")
            next_state = 5 # back up
        elif d > self.cd_closer_dist :
            msg.linear.x =  self.cd_closer_lvel
            # Y is used to trun while driving towards cone head on
            # if y > 0.02 :    msg.angular.z =  self.cd_closer_avel
            # elif y < -0.02 : msg.angular.z = -self.cd_closer_avel
            # else : msg.angular.z = 0.0
            # use angle to turn towards the cone
            msg.angular.z =  (a/0.393)*self.cd_closer_avel
            pass
        else : 
            self.get_logger().info(f"{func} at closer distance {x=:.3f} {y=:.3f}  {a=:.3f} {d=:.3f} {state=}")
            next_state = 3

        # msg.linear.x = 0.02
        # msg.angular.z = -0.1

        #self.get_logger().info(f"{func} {x=:.3f} {y=:.3f}  {d=:.3f} lx={msg.linear.x:.3f} az={msg.angular.z:.3f}")
        self.cmd_vel_publisher.publish(msg)
        # self.get_logger().info(f"{func} {state=} {msg=}")
        
        return next_state

    #state 3 - use TOF sensors to "touch" cone
    def touch_cone(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} drive slowly to \"touch\" cone using cmd_vel and tof sensors {state=}")
        
        killSwitchActive:bool = ks
        next_state = state
        msg = Twist()
        d = self.cone_at_d_tof_fc
        a = self.cone_at_a_tof_fc
        i = self.cone_dist_idx_min_tof_fc

        if killSwitchActive :
            # Stop motors and wait for kill switch not active
            # Motors are stopped by leaving velocities as msg default = 0
            pass
        elif math.isinf(d) :
            self.get_logger().info(f"{func} lost cone {state=}")
            next_state = 0
        elif d > 1.5*self.cd_closer_dist :
            self.get_logger().info(f"{func} cone is too far {d=} {state=}")
            next_state = 5 # back up
        elif d > self.cd_touch_dist :
            msg.linear.x = self.cd_touch_lin_vel
            # turn towards cone center
            if   i <= 2 : msg.angular.z =  self.cd_touch_ang_vel
            elif i >= 5 : msg.angular.z = -self.cd_touch_ang_vel
        else : 
            self.get_logger().info(f"{func} touched {d=:.3f} {state=}")
            next_state = 4

        self.cmd_vel_publisher.publish(msg)

        return next_state

    #state 4 - wait a short time for judge to see the "touch"
    def wait_after_touch(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} wait a short time {state=}")
        
        killSwitchActive:bool = ks
        next_state = state

        if killSwitchActive :
            # wait until kill switch is not active
            pass
        elif (time.time_ns()*1e-9 - self.cd_timer) > self.cd_touch_wait :
            next_state = 5
        
        return next_state

    # #state 5 - backup after "touch"
    # def backup_after_touch(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
    #     if state_change :
    #         self.get_logger().info(f"{func} backup {state=}")
            
    #     killSwitchActive:bool = ks
    #     next_state = state
    #     t = self.cd_backup_dist/self.cd_backup_vel
    #     msg = Twist()

    #     if killSwitchActive :
    #         # Stop motors and wait for kill switch not active
    #         # Motors are stopped by leaving velocities to msg default as 0
    #         pass
    #     elif (time.time_ns()*1e-9 - self.cd_timer) < t :
    #         msg.linear.x = -self.cd_backup_vel
    #     else : 
    #         next_state = 6

    #     self.cmd_vel_publisher.publish(msg)

    #     return next_state

    #state 5 - backup after "touch"
    def backup_after_touch(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} backup {state=}")
            self.cd_sub_state = 0

        cur_time = time.time_ns()*1e-9            
        killSwitchActive:bool = ks
        killSwitchChange = ksc
        next_state = state
        dist = self.cd_backup_dist
        vel = self.cd_backup_vel
        t = 1.5*self.cd_backup_dist/self.cd_backup_vel

        # if killSwitchActive :
        #     # restart timer
        #     # TODO: Should timer be frozen?
        #     self.cd_timer = time.time_ns()*1e-9

        if self.cd_sub_state == 0 :
            if not killSwitchActive :
                # issue a backup navigation command with obstical avoidance
                self.nav.backup(backup_dist=dist, backup_speed=vel
                    , time_allowance=10) #, disable_collision_checks=True) # non-blocking
                self.cd_sub_timer = cur_time
                self.cd_sub_state = 1

        elif self.cd_sub_state == 1 :
            if killSwitchActive :
                # Cancel goal and wait when kill switch status changes to active
                if killSwitchChange :
                    self.nav.cancelTask()
                    self.cd_sub_state = 0
            else : # Execute when kill switch is not active
                # check for nav backup is complete or navigate time finished
                if (cur_time - self.cd_sub_timer) < t :
                    if self.nav.isTaskComplete() :
                        next_state = 6
                else :
                    self.nav.cancelTask()
                    next_state = 6

        return next_state

    #state 6 - STOP
    def stop_after_touch(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} STOP {state=}")
        
        next_state = state

        return next_state
    
    def get_cone_dist_from_robot(self, cx:float, cy:float) -> tuple:
        """
        return (cone_dist, cone_angle, robot_x, robot_y, robot_angle).
        dist,angle relative to oak-d_frame.
        robot x,y,angle relative to.
        cx,cy is the cone xy relative to oak-d_frame.
        """
        # get current robot pose at tof_fc to determine the angle offset
        # (tf_OK, current_pose) = self.getCurrentPose()
        (tf_OK, current_pose) = self.getPoseFromTF('map','tof_fc_link')
        if not tf_OK : return -1.0
        rx = current_pose.pose.position.x
        ry = current_pose.pose.position.y
        (_,_,ra) =  tf_transformations.euler_from_quaternion([
                                current_pose.pose.orientation.x,
                                current_pose.pose.orientation.y,
                                current_pose.pose.orientation.z,
                                current_pose.pose.orientation.w])

        # calc distance from robot(tof_fc) to cone
        dx = float(cx)
        dy = float(cy)
        # Calc angle to target cone XY coordinate
        a = math.atan2(dy,dx)
        # calc distance to target cone
        d = math.sqrt(dx*dx + dy*dy)

        return (d, a, rx, ry, ra)

    def gotoConeXY(self, cx:float, cy:float, stop_dist:float, t) -> float:
        """
        The navigation is sent and is non-blocking, caller needs to check when finished.
        Go to the cone location, but stop at a given distance to the cone.
        The coordinate cx,cy is relative to tof_fc sensor.
        The stop_dist is the distance from the cone to stop.
        The t is the time to drive before returning .
        Returns distance to the cone, -1 if it did not succeed.
        """

        if cx!=0 and cy!=0 :
            # self.get_logger().info(f"gotoConeXY: cone at {cx=:.3f} {cy=:.3f}")
            
            (cd, ca, rx, ry, ra) = self.get_cone_dist_from_robot(cx, cy)

            self.get_logger().info(f"gotoConeXY: robot at {rx=:.3f} {ry=:.3f}, {ra=:.3f} cone at {cx=:.3f} {cy=:.3f} {cd=:.3f} {ca=:.3f}")

            d = cd
            if d < stop_dist : return d

            # calc angle from "map" to cone to drive with heading direct to cone
            a = ra + ca
            # adjust the distance stop at the cost map boundary
            sd = d - stop_dist
            # calc goto coordinates x,y for "map"->cone
            x = rx + sd*math.cos(a)
            y = ry + sd*math.sin(a)
            
            goto_pose = self.createPose(x,y,a,"map")

            self.get_logger().info(f"gotoConeXY: goto {x=:.3f} {y=:.3f} {sd=:.3f} {a=:.3f} {t=:.3f}")

            # non-blocking
            self.nav.goToPose(goto_pose)

        else :
            self.get_logger().info(f"gotoXY: No cone detected")
            d=-1.0

        return float(d)



    # Cone detection from camera AI relative to camera "oak-d_frame"
    def cone_point_subscription_callback(self, msg: PointStamped) -> None:
        # if msg.point.x == 0 : self.get_logger().info(f"{msg=}")
        x = msg.point.x
        y = msg.point.y
        t = time.time_ns() * 1e-9 # seconds
        a = math.atan2(y,x)
        d = math.sqrt(x*x + y*y)

        # transform cone xy to "/tof_fc_link" which is considered the front of the robot
        self.cone_at_x_cam = x
        self.cone_at_y_cam = y
        self.cone_at_d_cam = d
        self.cone_at_a_cam = a
        self.cone_det_time_cam = t

        self.get_logger().info(f"cone_callback: {x=:.3f} {y=:.3f} {a=:.3f} {d=:.3f} ")


    # Cone distance and angle relative to front TOF sensors
    tof_fc_dist_max = 1.5
    tof_fov = 0.785 

    def tof_fc_mid_subscription_callback(self, msg: Float32X8) -> None:
        self.get_logger().info(f"{msg=}")

        dmin:float = math.inf
        dmin_idx:int = -1
        idx:int = 0

        dist_max = self.tof_fc_dist_max
        fov = self.tof_fov 
        pt_angle = fov/8

        for d in msg.data :
            if not math.isinf(d) :
                if d < dmin : 
                    dmin = d
                    dmin_idx = idx
            idx += 1
            # self.get_logger().info(f"tof_fc_callback: {d=:.3f} {dmin=:.3f} {dmin_idx=}")

        # TODO: validate cone detect using width and maybe shape

        if dmin >= dist_max : dmin = math.inf

        d = dmin
        a = fov/2 - pt_angle/2 - dmin_idx*pt_angle
        x = d * math.cos(a)
        y = d * math.sin(a)

        self.cone_at_x_tof_fc = x
        self.cone_at_y_tof_fc = y
        self.cone_at_d_tof_fc = d
        self.cone_at_a_tof_fc = a
        self.cone_dist_idx_min_tof_fc = dmin_idx

        self.get_logger().info(f"tof_fc_callback: {x=:.3f} {y=:.3f} {a=:.3f} {d=:.3f} ")


    # Cone detection from Lidar LaserScan data
    def lidar_subscription_callback(self, msg: LaserScan) -> None:
        #self.get_logger().info(f"cone_det_cam_subscription_callback: {msg=}")

   
        angle_min       = np.float32(msg.angle_min)
        angle_max       = np.float32(msg.angle_max)
        angle_increment = np.float32(msg.angle_increment)
        coneRadius      = np.float32(0.05) # 100mm diameter at Lidar scan height     
        # self.get_logger().info(f"lidar_callback: {angle_min=:.3f} {angle_max:.3f} {angle_increment=:.3f}")

        # get Lidar scan data within determined field of view for cone detection
        len = np.int32((angle_max-angle_min)/angle_increment)
        pfov = np.int32(self.fovRad/angle_increment)
        dmin = np.int32(len/2 - pfov/2)
        dmax = np.int32(len/2 + pfov/2)
        coneRanges =  np.float32(msg.ranges[dmin:dmax])
        # self.get_logger().info(f"{coneRanges=}")

        diffJump = np.float32(0.4)
        coneRayMax = np.float32(2.5)
        rayCountScale = np.float32(1.20) # 20 percent
        rayInfCount = np.int32(10) # allow 10 infinities which reduce ray counts
        rayMinCnt = np.int32(10) # minimum number of ray counts in cone detect

        begin = np.int32(0)
        end = np.int32(0)
        lastRay = np.float32(coneRanges[0])
        rayNum = np.int32(1)

        # NOTE: 1st ray cant be start of cone
        for ray in coneRanges[1:] :
            # if np.math.isinf(ray) : ray = 100 
            if begin==0 :
                # look for dist jump hi to lo to indicate possible start
                if (ray<coneRayMax) and ((lastRay-ray)>diffJump) :
                    # possible start of cone detect
                    begin = rayNum

            elif (lastRay<coneRayMax) :
                # look for dist jump lo to hi to indicate possible end
                if((ray-lastRay)>diffJump) :
                    # possible end of cone detect
                    end = rayNum-1
                    if False : #(end-begin) < rayMinCnt :
                        # number of rays too small for a cone, look for another cone
                        begin = np.int32(0)
                        end = np.int32(0)

                    else :
                        coneRays = coneRanges[begin:end]
                        coneRayMin = np.min(coneRays)
                        # find ray number at center of minimums
                        idx = begin
                        cnt = np.int32(0)
                        coneRayIdxAtMin = np.int32(0)
                        for ray in coneRays :
                            if ray == coneRayMin :
                                coneRayIdxAtMin += idx
                                cnt +=1
                            idx +=1
                        coneRayIdxAtMin = np.int32(coneRayIdxAtMin/cnt)
                        # validate num rays vs distance
                        numRaysMeas = (end-begin)+1+rayInfCount
                        numRaysCalc = np.arctan2(coneRadius,coneRayMin+coneRadius) # dist to cone center
                        numRaysCalc *= 2/angle_increment
                        numRaysDiff = np.abs(numRaysMeas-numRaysCalc)

                        if True : #(numRaysDiff)  < 100 : #((numRaysCalc*rayCountScale)) :
                            # validated cone detection
                            break
                        else :
                            # not valid cone, look for another
                            begin = np.int32(0)
                            end = np.int32(0)

            else :
                # cone ray distance was too far, look for another cone
                begin = np.int32(0)
                end = np.int32(0)

            lastRay = ray
            rayNum +=1
 
        if (begin==0) or (end==0) :
            # no cone detected
            #self.get_logger().info(f"lidar_callback: No cone detected {coneRanges=}")
            # default for no cone detected
            self.cone_at_x_lidar = 0
            self.cone_at_y_lidar = 0
            self.cone_at_a_lidar = 0
            return
        
        # find angle of detected cone in FOV in front of robot where cone is searched
        # middle of coneRanges[] data is 0 degrees
        a = np.float32(angle_increment*(coneRayIdxAtMin-(np.size(coneRanges)/2)))

        # convert to x,y coordinates relative to lidar "/oak-d_frame"
        d = np.float32(coneRayMin)
        x = d*np.cos(a)
        y = d*np.sin(a)
        
        coneRays -= d # normalize data, zero is closest
        np.set_printoptions(precision=3, suppress=True)
        # self.get_logger().info(f"lidar_callback: {x=} {y=} {d=}  {a=} {pfov=} {coneRayIdxAtMin=} {coneRays=}")
        np.set_printoptions(precision=3, suppress=True)
        # transform coordinates to "/tof_fc_link" which is considered front of robot
        self.cone_at_x_lidar = x - 0.400 # crude transformation not TF, Lidar is 0.400 behind tof sensor
        self.cone_at_y_lidar = y
        self.cone_at_d_lidar = d
        self.cone_at_a_lidar = a
        
        self.get_logger().info(f"lidar_callback: {x=} {y=} {a=} {d=}")

    def gotoPoseBlocking(self, pose, t):
        """
        Go to the pose within in the time limit
        """

        self.nav.goToPose(pose)

        (result, _) = self.waitTaskComplete(t)
       
        return result


    def getCurrentPose(self) -> tuple:
        """
        get map->tof_fc_link (front center tof sensor) transform
        returns (tf_OK, pose) pose: PoseStamped
        """
        return self.getPoseFromTF('map', 'tof_fc_link')


    def getPoseFromTF(self, child_frame:str, target_frame:str) -> tuple:
        """
        get child_frame->target_frame transform
        returns (tf_OK, pose) pose: PoseStamped
        """
        # try getting pose a few times
        cnt = 0
        tf_OK = False
        while tf_OK == False and cnt < 5 :
            try:
                tf = self.tf_buffer.lookup_transform (
                    child_frame,
                    target_frame,
                    #self.nav.get_clock().now().to_msg(),
                    rclpy.time.Time(), # default 0
                    timeout=rclpy.duration.Duration(seconds=0.1) #0.0)
                    )
                tf_OK = True

            except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().info(f'getPoseFromTF: Could not find transform {child_frame}->{target_frame}: {ex}')
                tf_OK = False
                cnt += 1
                
        if tf_OK == False or cnt >= 5 :
            self.get_logger().info(f'getPoseFromTF: Failed to find transform {child_frame}->{target_frame} after {cnt} tries {tf_OK=}')
            return (False,None) 
        
        # translate wall points to align with map coordinates
        if tf_OK :
            # get x, y, theta from TF
            x:float = tf.transform.translation.x
            y:float = tf.transform.translation.y
            q:float = tf.transform.rotation
            # convert quaterion to euler, discard xx and yy
            (xx,yy,a) = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            pose = self.createPose(x,y,a,child_frame)
        else :
            pose = None
            
        return (tf_OK,pose)
        

    def createPose(self,x:float,y:float,a:float,frame_id:str) -> PoseStamped:
        stamp = self.nav.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        # TODO: use a given stamp time?
        pose.header.stamp = stamp
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

    def destroy_node(self):
        self.get_logger().info("destroy_node")
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    nav = BasicNavigator()
    node = NavNode(nav)

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