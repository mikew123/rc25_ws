# RC25 navigation node

import rclpy
import math
import time
import tf_transformations
import json
import numpy as np
import utm

import yaml
from pathlib import Path
from pprint import pprint, pformat

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, Pose
from geographic_msgs.msg import GeoPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import Duration
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rc25_interfaces.msg import Float32X8, TofDist

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
from rclpy.client import Client

class NavNode(Node):
    '''
    Navigate to cones
    '''

    # parameters

    # Timer for state machine
    smTimerRateHz:float = 10.0


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
    cd_closer_avel = cd_closer_lvel

    # state=3 use /tof_fc_mid to "touch" cone using /cmd_vel
    cd_touch_dist = 0.045
    cd_touch_lin_vel = 0.05
    cd_touch_ang_vel = cd_touch_lin_vel

    # wait while touching cone for observation
    cd_touch_wait = 2.0

    # backup after touching cone using /cmd_vel
    cd_backup_dist = 1.5
    cd_backup_vel = 0.25

    # # Field of view pointing forward from Lidar 36 degree scan data 
    # fovRad = 0.758 # 45 deg, +-22.5 degrees

    tof_fc_dist_max = 1.5
    tof_fov = 0.785 

    # GLOBAL variables 

    # for cone navigation and approach (state=1,2)
    # using /cone_point_cam
    cone_at_x_cam:float = 0.0
    cone_at_y_cam:float = 0.0
    cone_at_d_cam:float = 0.0
    cone_at_a_cam:float = 0.0
    cone_det_time_cam:int = 0
    cone_det_time_out_cam:int = 2.0

    # for touch (state=3)
    # using /cone_point_lidar
    cone_at_x_lidar:float = 0.0
    cone_at_y_lidar:float = 0.0
    cone_at_d_lidar:float = 0.0
    cone_at_a_lidar:float = 0.0
    cone_det_time_lidar:int = 0
    cone_det_time_out_lidar:int = 2.0

    # using /tof_fc_mid (NOTE: distance from front center TOF sensor)
    cone_at_x_tof_fc:float = 0.0
    cone_at_y_tof_fc:float = 0.0
    cone_at_d_tof_fc:float = 0.0
    cone_at_a_tof_fc:float = 0.0

    # median 5 filter memory
    # list of tupples [5X(x,y,z)]
    m5_filter:list = [(0.0,0.0,0.0)]*5
    
    # True when killSw is released to stop the robot motion
    killSw:bool = False
    cd_killSwChange:bool = False
    killSwEn:bool = False

    # YAML file with cone locations
    waypointsFileName = "~/sambashare/nav_files/RoboColumbus.yml"
    waypointsFile:dict = None

    wpConfig          = None
    wpGps:bool        = False
    wpCompass:bool    = False
    wpSetPose:dict    = None
    wpSetDatum:dict   = None
    wpWaypoints:dict  = None
    wpWaypoint:dict   = None
    wpWaypointNum:int = 1 # First waypoint number
    wpCone:bool       = False

    sm_last_state = -1

    # EFK filter node sensor configurations
    #    [x_pos   , y_pos    , z_pos,
    #     roll    , pitch    , yaw,
    #     x_vel   , y_vel    , z_vel,
    #     roll_vel, pitch_vel, yaw_vel,
    #     x_accel , y_accel  , z_accel]
    # Enables GPS signals for map->odom tf
    efk_global_odom1_config_gpsEn = [
        True,  True,  False, # lat,lon are used for x_pos,y_pos
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, False 
                ]
    # Disables GPS signals for map->odom tf
    efk_global_odom1_config_gpsDis = [
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, False 
                ]
    # Enables IMU yaw (compass) from efk_local
    efk_global_odom0_config_yawEn = [
        False, False, False,
        False, False, True, # compass yaw from efk_local is used
        True,  False, False,
        False, False, True,
        False, False, False
        ]
    # Disables IMU yaw (compass) from efk_local
    efk_global_odom0_config_yawDis = [
        False, False, False,
        False, False, False,
        True,  False, False,
        False, False, True,
        False, False, False
        ]
    # Enables IMU yaw (compass)
    efk_local_imu0_config_yawEn = [
        False, False, False,
        False, False, True, # Compass IMU yaw is used
        False, False, False,
        False, False, True,
        False, False, False 
        ]
    # Disables IMU yaw (compass)
    efk_local_imu0_config_yawDis = [
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, True,
        False, False, False 
        ]


    def  __init__(self, nav: BasicNavigator):
        super().__init__('robocolumbus25_nav_node')
        self.nav = nav

        self.cb_group = MutuallyExclusiveCallbackGroup()

        self.readWaypointsFile(self.waypointsFileName)

        self.set_pose_msg_publisher = self.create_publisher(PoseWithCovarianceStamped, 'set_pose', 10)

        self.navsat_transform_server_set_param_svc = self.create_client(SetParameters, '/navsat_transform/set_parameters')
        while not self.navsat_transform_server_set_param_svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/navsat_transform/set_parameters service not available, waiting again...')

        self.controller_server_set_param_svc = self.create_client(SetParameters, '/controller_server/set_parameters')
        while not self.controller_server_set_param_svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/controller_server/set_parameters service not available, waiting again...')

        self.efk_global_set_param_svc = self.create_client(SetParameters, '/efk_global/set_parameters')
        while not self.efk_global_set_param_svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/efk_global/set_parameters service not available, waiting again...')

        self.efk_local_set_param_svc = self.create_client(SetParameters, '/efk_local/set_parameters')
        while not self.efk_local_set_param_svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/efk_local/set_parameters service not available, waiting again...')

        # #DEBUG
        # self.send_set_param_request(self.controller_server_set_param_svc,
        #             "goal_checker.xy_goal_tolerance", 0.1)
            
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cone_point_cam_subscription = self.create_subscription(PointStamped, '/cone_point_cam'
                                            , self.cone_point_cam_subscription_callback, 10)
        self.cone_point_lidar_subscription = self.create_subscription(PointStamped, '/cone_point_lidar'
                                            , self.cone_point_lidar_subscription_callback, 10)

        self.tof_fc_mid_subscription = self.create_subscription(Float32X8, '/tof_fc_mid'
                                            , self.tof_fc_mid_subscription_callback, 10)
        self.tof_dist_subscription = self.create_subscription(TofDist, '/tof_dist'
                                            , self.tof_dist_subscription_callback, 10)

        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "/json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "/json_msg"
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

        time.sleep(2) # wait for json_msg_publisher to be ready!!??
        self.tts("RC25 Navigation Node Started")
        self.get_logger().info(f"NavNode Started")

    def readWaypointsFile(self, file) :
        '''
        Executed in init
        Read yaml file with waypoint locations and if gps is enable for localization
        Parse for config, datum/pose and waypoints
        Each waypoint has a selection to look for cone or not (intermediate waypoint)
        Examples :
        # waypoint with cone location in ROS map related meters x,y
        # dead recogning mode - compass and gps are not used
            config :
                compass : false
                gps : false
            set_pose :
                x : 0.0
                y : 0.0
                deg : 180
            waypoints :
                1 :
                    cone : true
                    x : 10.0
                    y : 0.0

        # waypoint location in absolute lat,lon
        # gps is used for localization, compass for direction
            config :
                gps : true
            waypoints :
                1 :
                    cone : false
                    lat : 12345678
                    lon : 567890
        '''

        self.get_logger().info(f"readWaypointsFile: {file=}")

        path = Path(file).expanduser()
        if not path.exists():
            self.get_logger().info(f"readWaypointsFile: file not found: {path}")
            return
        try:
            with path.open("r", encoding="utf-8") as f:
                doc = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().info(f"readWaypointsFile: Bad format: {e}")
            return

        if not doc:
            self.get_logger().info("readWaypointsFile: file is empty")
            return

        self.get_logger().info("readWaypointsFile: \n"+pformat(doc))

        # Parse file
        if "config" in doc :
            config = doc["config"]
            if "gps" in config :
                self.wpGps = config["gps"] 
            elif "compass" in config :
                self.wpCompass = config["compass"]
        if "set_pose" in doc :
            self.wpSetPose = doc["set_pose"]
        if "set_datum" in doc :
            self.wpSetDatum = doc["set_datum"]
        if "waypoints" in doc :
            self.wpWaypoints = doc["waypoints"]


    def setupNav(self) -> bool :
        '''
        Executed when nav button pressed
        Set up initial pose and/or datum using info in waypoints file

        If gps == True 
            The set_pose information in the file is ignored
            The lat,lon datum and x,y pose are set
            If there is no set_datum in the file then the current gps lat/lon signal is used
            If there is no set_pose the pose is 0,0 yaw=compass
            If set_pose is lat,lon then it is converted to map x,y relative to datum lat,lon
            (The datum orientation is set within navsat_transform from compass)
        If gps == False or doesnt exist
            The datum is not set
            If there is a set_pose it sets the pose using x,y (ignores any lat,lon)
                If there is no set_pose x/y then pose x,y = 0,0 origin
                If there is no set_pose deg or yaw 
                    If compass == True pose yaw = compass yaw
                        otherwise pose yaw=0

        returns True if setup OK
        '''

        # get pose and datum from waypoints file
        wpDatum:dict = self.wpSetDatum
        wpPose:dict  = self.wpSetPose

        self.get_logger().info(f"setupNav: {wpPose=} {wpDatum=}")
        if self.wpGps == True :
            # must use the compass while using gps
            self.wpCompass = True
        else :
            # No gps so datum is not used
            wpDatum = None

        if self.wpGps == True :
            if wpDatum != None :
                if not (("lat" in wpDatum) and ("lon" in wpDatum)) :
                    # get datum lat,lon current location from gps signal
                    pass
            else : # no datum data in waypoint file
                # get datum lat,lon current location from gps signal
                pass

            # create pose x,y,yaw from datum lat/lon and compass
            #TODO: get compass yaw, should we use a datum yaw if given?
            x, y, rad = 0.0, 0.0, 0.0 # defaults

            if wpPose != None :
                if ("x" in wpPose) and ("y" in wpPose) :
                    x = wpPose["x"]
                    y = wpPose["y"]
                elif ("lat" in wpPose) and ("lon" in wpPose) :
                    lat = wpPose["lat"]
                    lon = wpPose["lon"]
                    # convert lat,lon to map x,y
                    (peast, pnorth, pn, ps) = utm.from_latlon(lat, lon)
                    (deast, dnorth, dn, ds) = utm.from_latlon(wpDatum("lat"), wpDatum("lon"))
                    if pn == dn :
                        x = peast = deast
                        y = pnorth - dnorth

                # get yaw to use in pose
                if "rad" in wpPose :
                    rad = wpPose["rad"]
                elif "deg" in wpPose :
                    rad = wpPose["deg"]/180.0 * math.pi

            # alternate sources of pose yaw
            if rad == 0.0 :
                if "rad" in wpDatum :
                    rad = wpDatum["rad"]
                elif "deg" in wpDatum :
                    rad = wpDatum["deg"]/180.0 * math.pi
                elif self.wpCompass == True :
                    # TODO: get yaw from compass
                    rad = 0.0

            wpPose = {"x":x,"y":y,"rad":rad}

        if wpPose == None :
            wpPose = {"x":0.0,"y":0.0,"rad":0.0}

        # if pose orientation in degrees convert to radians
        if (not ("rad" in wpPose)) and ("deg" in wpPose):
            wpPose["rad"] = wpPose["deg"]/180.0 * math.pi

        datum = None
        if wpDatum != None :
            datum = [
                wpDatum["lat"],
                wpDatum["lon"],
                0.0 # altitude? yaw ??
            ] 
            self.send_set_param_request(self.navsat_transform_server_set_param_svc,
                                    "datum",  datum)           

        if wpPose != None :
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id="map"
            pose.pose.pose = self.createPose(wpPose["x"],wpPose["y"],wpPose["rad"])
            self.set_pose_msg_publisher.publish(pose)

        self.get_logger().info(f"setupNav: {pose=} {datum=}")
        return True

    def send_set_param_request(self, svc: Client, name, value) -> None:
        """
        Set a parameter using the given param service
        command line example:
        cli -> ros2 param set /local_costmap/local_costmap obstacle_layer.enabled False
        """
    
        while not svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('send_set_param_request: service not available, waiting again...')

        req = SetParameters.Request()
        self.callback_set_param_done = False

        param = Parameter()
        param.name = name
        param.value.type = None
        if isinstance(value, bool) :
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = value
        elif isinstance(value, float) :
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = value
        elif isinstance(value, list) :
            if isinstance(value[0], bool) :
                param.value.type = ParameterType.PARAMETER_BOOL_ARRAY
                param.value.bool_array_value = value
            elif isinstance(value[0], float) :
                param.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                param.value.double_array_value = value
                
        if param.value.type == None :
            self.get_logger().info(f"send_set_param_request: unsupported variable type {value=}")
            return

        req.parameters.append(param)

        self.get_logger().info(f"send_set_param_request: Sending {name=} {value=}")
        future = svc.call_async(req)

        # delay a tiny bit since checking future complete does not seem to work!!!
        time.sleep(0.1)

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

    def json_msg_callback(self, msg:String) -> None :
        #self.get_logger().info(f"json_msg_callback: {msg=}")
        data = json.loads(msg.data)

        if 'kill' in data :
            kill:bool = data['kill']
            self.processKillSwStatus(kill)

        if 'nav' in data :
            nav = data['nav']
            self.processNavMsg(nav)

    def getNextWaypoint(self) -> bool :
        '''
        Get the next waypoint from the read waypoints file
        returns True if there is a new waypoint
        '''
        waypoints = self.wpWaypoints
        n = self.wpWaypointNum
        if not (n in waypoints) :
            self.wpWaypoint = False
            return False
        
        self.wpWaypoint = waypoints[n]
        self.wpWaypointNum +=1
        return True

    buttonDo = False
    buttonKill = False
    buttonDoTrig = False
    buttonKillTrig = False

    def processNavMsg(self, nav:dict) -> None :
        # self.get_logger().info(f"processNavMsg: {nav=}")

        if "buttonDo" in nav :
            buttonDo = nav["buttonDo"]
            if (self.buttonDo==False) and (buttonDo==True):
                self.buttonDoTrig = True
            self.buttonDo = buttonDo

        if "buttonKill" in nav :
            buttonKill = nav["buttonKill"]
            if (self.buttonKill==False) and (buttonKill==True):
                self.buttonKillTrig = True
            self.buttonKill = buttonKill
            self.processKillButton()

        if "imu_cal_status" in nav :
            self.imuCalStatus = nav["imu_cal_status"]


    def processKillSwStatus(self, kill:bool) -> None :
        if(kill != self.killSw) :
            self.cd_killSwChange = True
            self.get_logger().info(f"processKillSwStatus: kill switch is {kill}")
        self.killSw = kill

    def processKillButton(self) -> None:
        '''
        Button on teleop game controller used for Kill
        As long as the button is not pressed, the kill sw is disabled
        When button first pressed the kill switch is enabled and kill=False
        After enabled
            While button is released kill=True
            While button is pressed kill=False
        '''
        if self.buttonKillTrig == False :
            return
        killB:bool = self.buttonKill
        if killB == True :
            self.killSwEn = True
            self.get_logger().info(f"processKillButton: teleop kill switch is enabled")
            self.tts("Kill switch is enabled")
        if self.killSwEn == True :
            self.killSw = not killB

    # Timer based state machine for cone navigation
    T_INIT_WAIT, T_CAL_IMU, T_WAIT_GO, T_GET_WP, T_NAV_WP, T_NAV_WP_AGAIN, T_GOTO_CONE, T_DONE = range(8)
    tc_state = -1
    tc_next_state = T_INIT_WAIT

    def smTimerNav2Config(self, state:int) -> None :
        '''
        Configure nav2 parameters for nav to waypoint or goto cone
        '''
        
        if state == self.T_NAV_WP :
            # configure navigation nodes parameters
            # change goal tolerance to 1M for nav to waypoint
            self.send_set_param_request(self.controller_server_set_param_svc,
                        "goal_checker.xy_goal_tolerance", 1.0)

            # Set which sensors are fused in EFK modules
            if self.wpGps == True :
                # configure for both compass and gps
                self.send_set_param_request(self.efk_global_set_param_svc, 
                            'publish_tf', True)
                self.send_set_param_request(self.efk_global_set_param_svc, # GPS
                            'odom1_config', self.efk_global_odom1_config_gpsEn)
                self.send_set_param_request(self.efk_global_set_param_svc, # Compass
                            'odom0_config', self.efk_global_odom0_config_yawEn)
                self.send_set_param_request(self.efk_local_set_param_svc, # Compass
                            'imu0_config', self.efk_local_imu0_config_yawEn)

            if self.wpCompass == True:
                self.send_set_param_request(self.efk_global_set_param_svc, # Compass
                            'imu0_config', self.efk_local_imu0_config_yawEn)
                self.send_set_param_request(self.efk_local_set_param_svc, # Compass
                            'imu0_config', self.efk_local_imu0_config_yawEn)
                    
        elif state == self.T_GOTO_CONE :
            # configure navigation nodes parameters for goto cone
            # change goal tolerance to 0.25M when approaching cone
            self.send_set_param_request(self.controller_server_set_param_svc,
                        "goal_checker.xy_goal_tolerance", 0.25)
            # Set which sensors are fused in EFK modules
            self.send_set_param_request(self.efk_global_set_param_svc, 
                        'publish_tf', False)
            self.send_set_param_request(self.efk_global_set_param_svc, # GPS
                        'odom1_config', self.efk_global_odom1_config_gpsDis)
            self.send_set_param_request(self.efk_global_set_param_svc, # Compass
                        'odom0_config', self.efk_global_odom0_config_yawDis)
            self.send_set_param_request(self.efk_local_set_param_svc, # Compass
                        'imu0_config', self.efk_local_imu0_config_yawDis)

    def sm_timer_callback(self):

        # need a main state machine
        next_state = self.tc_next_state

        stateChange = False
        if self.tc_state!= next_state :
            stateChange = True
            self.get_logger().info(f"sm_timer_callback: state change to {next_state}")
            self.tts(f"Nav timer {next_state=}")

        state = next_state
        self.tc_state = state

        if state == self.T_INIT_WAIT :
            self.tts("wait for nav 2")
            time.sleep(20)

            # calibrate compass if used
            if (self.wpCompass == True) or (self.wpGps == True) :
                next_state = self.T_CAL_IMU
            else :
                next_state = self.T_WAIT_GO
                    
        if state == self.T_CAL_IMU :
            # Calibrate IMU when compass is used
            status = self.calImu()
            if status :
                #  calibration complete
                next_state = self.T_WAIT_GO

        if state == self.T_WAIT_GO :
            # Wait for go command from button
            status = self.waitGo()
            if status :           
                self.setupNav()
                next_state = self.T_GET_WP

        elif state == self.T_GET_WP :        
            if self.getNextWaypoint() :
                next_state = self.T_NAV_WP
            else :
                next_state = self.T_DONE
        
        elif state == self.T_NAV_WP_AGAIN :
            next_state = self.T_NAV_WP

        elif state == self.T_NAV_WP :
            if stateChange :
                self.get_logger().info(f"T_NAV_WP: next {self.wpWaypoint=}")
                self.tts(f"Goto waypoint x {self.wpWaypoint["x"]:.1f}, y {self.wpWaypoint["y"]:.1f}")
                self.smTimerNav2Config(state)
            
                # execute goto waypoint once
                x = 0.0 # default
                y = 0.0
                self.wpCone = False # default
                if "cone" in self.wpWaypoint :
                    self.wpCone = self.wpWaypoint["cone"]

                if ("x" in self.wpWaypoint) and ("y" in self.wpWaypoint) :
                    # map based XY location
                    x = self.wpWaypoint["x"]
                    y = self.wpWaypoint["y"]
                    
                    #TODO: what angle?
                    a = 0.0

                    self.get_logger().info(f"Go to way point location {x=} {y=}")
                    self.tts(f"Go to waypoint location {x=} {y=}")

                    goto_pose = self.createPoseStamped(x,y,a,"map")
                    self.nav.goToPose(goto_pose)

                else :
                    # invalid waypoint, get next
                    next_state = self.T_GET_WP

            #TODO: timeout + kill switch
            if self.nav.isTaskComplete() :
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED :
                    self.tts("Navigate to way point location is successfull")
                    if self.wpCone :
                        # Find cone close to the way point
                        next_state = self.T_GOTO_CONE
                    else :
                        # No cone is at the way point
                        next_state = self.T_GET_WP
                else :
                    # try again
                    self.tts("Attempt to navigate to way point location again")
                    # next_state = self.T_WAIT_REQ
                    next_state = self.T_NAV_WP_AGAIN
            else :
                result = self.nav.getFeedback()
                #self.get_logger().info(f"gotoPose {result=}")
                

        elif state == self.T_GOTO_CONE :
            if stateChange :
                self.tts(f"Go to the cone")
                self.smTimerNav2Config(state)

            # find cone and "touch" it
            done = self.cd_sm()
            if done : 
                #go request next cone location
                self.get_logger().info(f"tc: done - touch cone succeeded")
                self.tts("cone was touched")
                next_state = self.T_GET_WP

        elif state == self.T_DONE :
            if stateChange :
                self.get_logger().info(f"T_DONE: Finished")
                self.tts(f"Finished")
            

        self.tc_next_state = next_state

    CAL_IMU_WAIT_BUTT, CAL_IMU, CAL_IMU_DONE = range(3)
    calImuState = -1
    next_calImuState = CAL_IMU_WAIT_BUTT

    def calImu(self) -> bool :
        '''
        Calibrate the IMU by moving robot
        Wait for cal button to be pressed
        Return True when IMU calibration is complete (status 3)
        '''
        #TODO: avoid obsticals
        # check for calibration status 3 (complete)
        # if self.imuCalStatus==3 : return True

        state = self.next_calImuState
        stateChange = False
        next_state = state # default
        returnVal = False

        if state != self.calImuState :
            stateChange = True
        self.calImuState = state

        if state == self.CAL_IMU_WAIT_BUTT :
            if stateChange :
                self.buttonDoTrig = False
                #TODO: message periodically till pressed
                self.tts(f"Press button to start IMU calbration")

            if self.buttonDoTrig == True :
                next_state = self.CAL_IMU

        elif state == self.CAL_IMU :
            if stateChange :
                self.tts(f"IMU is calibrating")

            status = self.drivePattern(stateChange, 0, 0.5, 2.0, 0.5)
            if status==True :
                next_state = self.CAL_IMU_DONE

        elif state == self.CAL_IMU_DONE :
            if stateChange :
                self.tts(f"IMU calbration is complete")
            
            next_state = self.CAL_IMU_WAIT_BUTT
            returnVal = True

        else :
            #TODO: invalid state ???
            next_state = self.CAL_IMU_WAIT_BUTT
            returnVal = True

        self.next_calImuState = next_state
        
        return returnVal

    NAV_GO_WAIT_BUTT, NAV_GO_BUTT_PRESSED = range(2)
    navGoState = -1
    next_navGoState = NAV_GO_WAIT_BUTT

    def waitGo(self) -> bool :
        '''
        Wait to start navigation 
        Wait for navigate button to be pressed
        Return True when button is pressed
        '''

        state = self.next_navGoState
        stateChange = False
        next_state = state # default
        returnVal = False

        if state != self.navGoState :
            stateChange = True
        self.navGoState = state

        if state == self.NAV_GO_WAIT_BUTT :
            if stateChange :
                self.buttonDoTrig = False
                #TODO: message periodically till pressed
                self.tts(f"Press button to start navigation")

            if self.buttonDoTrig==True :
                next_state = self.NAV_GO_BUTT_PRESSED

        if state == self.NAV_GO_BUTT_PRESSED :
            if stateChange :
                self.tts(f"Navigation is starting")

            if self.buttonDoTrig==True :
                next_state = self.NAV_GO_WAIT_BUTT
                returnVal = True

        self.next_navGoState = next_state

        return returnVal
    
    DP_FWD_RIGHT, DP_REV_LEFT, DP_PAUSE = range(3)
    dpState = -1
    next_dpState = DP_FWD_RIGHT
    dpStopTime = 0.0
    dpPauseNextState = -1
    dpCount = 0

    def drivePattern(self, init:bool, numMoves:int=0, speed:float=0.5, driveT:float=2.0, pauseT:float=0.5) -> bool :
        '''
        Drive in a "star" like pattern fwd-left/rev-right or similar
            init resets drive pattern count
            numMoves>0 is the number of FWD_RIGHT,REV_LEFT movements (cone search)
            numMoves=0 moves until the IMU is calibrated AND cal button is released 
            speed (m/s) is the speed while moving (NOTE: speed<0 reverses FWD/REV)
            driveT (sec) is the time while moving
            pauseT (sec) is the time paused between movements
            return=True when finished
        '''

        # /cmd_vel message to drive robot
        msg = Twist()
        
        state = self.next_dpState
        stateChange = False
        next_state = state # default
        returnVal = False

        if state != self.dpState :
            stateChange = True
        self.dpState = state

        if init :
            self.dpCount = 0

        if state == self.DP_FWD_RIGHT :
            if stateChange==True :
                self.dpStopTime = time.monotonic() + math.fabs(driveT)

            msg.linear.x  = speed
            msg.angular.z = -2*speed

            if time.monotonic() >= self.dpStopTime :
                self.dpPauseNextState = self.DP_REV_LEFT
                next_state = self.DP_PAUSE

        elif state == self.DP_REV_LEFT :
            if stateChange==True :
                self.dpStopTime = time.monotonic() + math.fabs(driveT)

            msg.linear.x  = -speed
            msg.angular.z = -2*speed

            if time.monotonic() >= self.dpStopTime :
                self.dpPauseNextState = self.DP_FWD_RIGHT
                next_state = self.DP_PAUSE

        elif state == self.DP_PAUSE :
            if stateChange==True :
                self.dpCount +=1
                self.dpStopTime = time.monotonic() + math.fabs(pauseT)

            msg.linear.x  = 0.0
            msg.angular.z = 0.0

            if time.monotonic() >= self.dpStopTime :
                next_state = self.dpPauseNextState

        # determine when to stop driving pattern
        if numMoves>0 :
            if self.dpCount>numMoves:
                msg.linear.x  = 0.0
                msg.angular.z = 0.0
                returnVal = True
        else :
            # continue calibration pattern until both calibrated and button released
            if (self.buttonDo==False) and (self.imuCalStatus==3) :
                msg.linear.x  = 0.0
                msg.angular.z = 0.0
                returnVal = True

        # drive robot command
        self.cmd_vel_publisher.publish(msg)

        self.next_dpState = next_state

        return returnVal

    # Executes in timer callback group, sensor callbacks are in parallel
    def cd_sm(self) -> bool:
        func = "cone_sm:"
        done = False

        state_change:bool = False
        if self.cd_state != self.cd_last_state : 
            state_change = True
            # self.nav.cancelTask() # cancel the running task when state changes

        ks = self.killSw
        ksc = self.cd_killSwChange
        if ksc : self.cd_killSwChange = False

        state:int = self.cd_state
        next_state:int = state

        if state_change :
            self.cd_timer = time.time_ns()*1e-9

        if state == 0 :
            next_state = self.search_for_cone(func,state,state_change,ks,ksc)

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
            # touch cone success - get ready for next cone
            done = True
            next_state = 0

        self.cd_last_state = state
        self.cd_state = next_state

        return done

    wcScanInit = False

    # state 0 - search for cone using camera  detection    
    def search_for_cone(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} wait for cone detection {state=}")
            self.tts("State 0")
            self.wcScanInit = True
            # self.nav.cancelTask()

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
            # clear out globals
            self.cone_det_time_cam = 0.0
            self.cone_at_x_cam = 0.0
            self.cone_at_y_cam = 0.0

        killSwitchActive:bool  = ks
        next_state:int = state

        if x!=0 and y<0.10 and y>-0.10:
            # A cone has been detected
            self.get_logger().info(f"{func} cone detected at {x=:.3f} {y=:.3f}  {state=} {killSwitchActive=}")
            #stop movement
            msg = Twist()
            self.cmd_vel_publisher.publish(msg)
            self.wcScanInit = True

            if not killSwitchActive : 
                next_state = 1
        else :
            # slowly scan for cone
            if not killSwitchActive :
                # if self.wcScanInit == False :
                #     self.tts("Scaning for a cone")

                status = self.drivePattern(self.wcScanInit, 10, 0.25, 5.0, 1.0)
                self.wcScanInit = False
                if status:
                    # drive pattern completed - init drive pattern on the next cycle
                    #TODO: move before scanning again - add more states?
                    self.wcScanInit = True
                
        return next_state

    #state 1 - use camera to locate cone and get close
    cd_sub_timer = 0
    def nav_to_cone(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        cur_time = time.time_ns()*1e-9
        if state_change :
            self.get_logger().info(f"{func} navigate with BasicNavigator close to cone {state=}")
            self.tts("State 1")
            self.cd_sub_state = 0
            # self.nav.cancelTask()

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
                        self.cancelNav2Task()
                        self.cd_sub_state = 0
                else : # Execute when kill switch is not active
                    # check for nav complete or navigate time finished and try again
                    if (cur_time - self.cd_sub_timer) < t :
                        if self.nav.isTaskComplete() :
                            # self.nav.cancelTask()
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
                        # self.nav.cancelTask()
                        self.cancelNav2Task()
                        self.cd_sub_state = 0

        else :
            self.get_logger().info(f"{func} lost cone {state=}")
            # self.nav.cancelTask()
            self.cancelNav2Task()
            next_state = 0

        return next_state

    def cancelNav2Task(self) :
        if not self.nav.isTaskComplete() :
            self.nav.cancelTask()

    #state 2 - get closer to cone
    def get_closer_to_cone(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} drive closer to cone using cmd_vel and camera {state=}")
            self.tts("State 2")

        if (time.time_ns()*1e-9 - self.cd_timer) < 2.0 : return state

        a:float = self.cone_at_a_cam
        d:float = self.cone_at_d_cam
        d -= 0.200 # Crude transformation, camera is 0.200 behind tof sensor

        # get TOF obstacle detections
        fl_ob_dist = self.tof_fl_obstacle_dist
        fr_ob_dist = self.tof_fr_obstacle_dist

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
            # use angle to turn towards the cone
            msg.angular.z =  (a/0.393)*self.cd_closer_avel
            # steer away from obstacle detected using TOF sensors
            if fl_ob_dist < 0.3 :
                msg.angular.z += 4*(fl_ob_dist - 0.3) * self.cd_closer_avel
            if fr_ob_dist < 0.3 :
                msg.angular.z -= 4*(fr_ob_dist - 0.3) * self.cd_closer_avel
            
        else : 
            self.get_logger().info(f"{func} at the closer distance {d=:.3f} {a=:.3f} {state=}")
            next_state = 3

        #self.get_logger().info(f"{func} {d=:.3f} lx={msg.linear.x:.3f} az={msg.angular.z:.3f}")
        self.cmd_vel_publisher.publish(msg)
        # self.get_logger().info(f"{func} {state=} {msg=}")
        
        return next_state


    #state 3 - "touch" cone
    def touch_cone(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} drive slowly to \"touch\" cone using cmd_vel and lidar sensor {state=}")
            self.tts("State 3")
            # self.nav.cancelTask()

        # self.nav.cancelTask()

        killSwitchActive:bool = ks
        next_state = state
        msg = Twist()

        d = self.cone_at_d_lidar
        a = self.cone_at_a_lidar

        # convert to xy relative to TOF/bumper
        x = d*math.cos(a)
        y = d*math.sin(a)

        x -= 0.400 # Lidar sensor is X mm back from TOF sensors
        x -= 0.055 # Cone surface is X mm further at Lidar level

        # get TOF obstacle detections
        fc_ob_dist = self.tof_fc_obstacle_dist
        fl_ob_dist = self.tof_fl_obstacle_dist
        fr_ob_dist = self.tof_fr_obstacle_dist

        # use TOF distance to help determine when touched
        d_tof = np.min((fc_ob_dist,fl_ob_dist,fr_ob_dist))

        if killSwitchActive :
            # Stop motors and wait for kill switch not active
            # Motors are stopped by leaving velocities as msg default = 0
            pass
        elif math.isinf(x) :
            self.get_logger().info(f"{func} lost cone {state=}")
            next_state = 0
        elif x > 1.5*self.cd_closer_dist :
            self.get_logger().info(f"{func} cone is too far {d=:.3f} {d_tof=:.3f} {a=:.3f} {x=:.3f} {y=:.3f} {state=}")
            self.tts(f"State 3: The cone is too far at distance {d:.3f} meters")
            next_state = 0 # restart by looking for the cone
        elif (x > self.cd_touch_dist) and (d_tof > self.cd_touch_dist) :
            self.get_logger().info(f"{func} approaching cone to touch {d=:.3f} {d_tof=:.3f} {a=:.3f} {x=:.3f} {y=:.3f} {fc_ob_dist=:.3f} {fl_ob_dist=:.3f} {fr_ob_dist=:.3f} {state=}")
            msg.linear.x = (x/0.2)*self.cd_touch_lin_vel + 0.010
            # turn towards cone center
            # msg.angular.z =  (a/0.393)*msg.linear.x #self.cd_touch_ang_vel
            msg.angular.z =  (8*y)*msg.linear.x
            # steer away from obstacle detected using TOF sensors
            if fl_ob_dist < 0.2 :
                msg.angular.z += 4*(fl_ob_dist - 0.2) * msg.linear.x
            if fr_ob_dist < 0.2 :
                msg.angular.z -= 4*(fr_ob_dist - 0.2) * msg.linear.x
        else : 
            self.get_logger().info(f"{func} touched {d=:.3f} {d_tof=:.3f} {d_tof=:.3f} {a=:.3f} {x=:.3f} {y=:.3f} {fc_ob_dist=:.3f} {fl_ob_dist=:.3f} {fr_ob_dist=:.3f} {state=}")
            self.tts("State 3: The cone was touched")
            next_state = 4

        self.cmd_vel_publisher.publish(msg)

        return next_state


    #state 4 - wait a short time for judge to see the "touch"
    def wait_after_touch(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} wait a short time {state=}")
            self.tts("State 4")
            # self.nav.cancelTask()
        
        killSwitchActive:bool = ks
        next_state = state

        if killSwitchActive :
            # wait until kill switch is not active
            pass
        elif (time.time_ns()*1e-9 - self.cd_timer) > self.cd_touch_wait :
            next_state = 5
        
        return next_state


    #state 5 - backup after "touch"
    def backup_after_touch(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} backup {state=}")
            self.tts("State 5: Backup")
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
                    , time_allowance=10) #, disable_collision_checks=False) # non-blocking
                self.cd_sub_timer = cur_time
                self.cd_sub_state = 1

        elif self.cd_sub_state == 1 :
            if killSwitchActive :
                # Cancel goal and wait when kill switch status changes to active
                if killSwitchChange :
                    # self.nav.cancelTask()
                    self.cd_sub_state = 0
            else : # Execute when kill switch is not active
                # check for nav backup is complete or navigate time finished
                if (cur_time - self.cd_sub_timer) < t :
                    if self.nav.isTaskComplete() :
                        next_state = 6
                else :
                    next_state = 6

        return next_state

    #state 6 - STOP
    def stop_after_touch(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} STOP {state=}")
            # self.nav.cancelTask()
            self.tts("State 6: Stopping now")

        next_state = state
        
        return next_state
    

    #***************************************************************************

    def get_cone_dist_from_robot(self, cx:float, cy:float) -> tuple:
        """
        return (cone_dist, cone_angle, robot_x, robot_y, robot_angle).
        dist,angle relative to oak-d_frame.
        robot x,y,angle relative to.
        cx,cy is the cone xy relative to oak-d_frame.
        """
        # get current robot pose at tof_fc to determine the angle offset
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
            
            goto_pose = self.createPoseStamped(x,y,a,"map")

            self.get_logger().info(f"gotoConeXY: goto {x=:.3f} {y=:.3f} {sd=:.3f} {a=:.3f} {t=:.3f}")

            # non-blocking
            self.nav.goToPose(goto_pose)

        else :
            self.get_logger().info(f"gotoXY: No cone detected")
            d=-1.0

        return float(d)



    # Cone detection from camera AI relative to camera "oak-d_frame"
    def cone_point_cam_subscription_callback(self, msg: PointStamped) -> None:
        # if msg.point.x == 0 : self.get_logger().info(f"{msg=}")
        x = msg.point.x
        y = msg.point.y
        # TODO: use header time
        t = time.time_ns() * 1e-9 # seconds
        a = math.atan2(y,x)
        d = math.sqrt(x*x + y*y)

        # transform cone xy to "/tof_fc_link" which is considered the front of the robot
        self.cone_at_x_cam = x
        self.cone_at_y_cam = y
        self.cone_at_d_cam = d
        self.cone_at_a_cam = a
        self.cone_det_time_cam = t

        # self.get_logger().info(f"cone_callback: {x=:.3f} {y=:.3f} {a=:.3f} {d=:.3f} ")

    # Cone detection from lidar scan relative to lidar "lidar_link"
    def cone_point_lidar_subscription_callback(self, msg: PointStamped) -> None:

        # if msg.point.x == 0 : self.get_logger().info(f"{msg=}")
        x = msg.point.x
        y = msg.point.y
        # TODO: use header time
        t = time.time_ns() * 1e-9 # seconds
        a = math.atan2(y,x)
        d = math.sqrt(x*x + y*y)

        # transform cone xy to "/tof_fc_link" which is considered the front of the robot
        self.cone_at_x_lidar = x
        self.cone_at_y_lidar = y
        self.cone_at_d_lidar = d
        self.cone_at_a_lidar = a
        self.cone_det_time_lidar = t

        # self.get_logger().info(f"cone_callback: {x=:.3f} {y=:.3f} {a=:.3f} {d=:.3f} ")

    tof_dist_obstacle_max = 0.300
    tof_fc_obstacle_dist:np.float32 = np.inf
    tof_fl_obstacle_dist:np.float32 = np.inf
    tof_fr_obstacle_dist:np.float32 = np.inf
    # tof_fc_obstacle_angle:np.float32 = 0 #TODO: do we need the angle?

    # Get TOF sensor data for obstacle detection
    def tof_dist_subscription_callback(self,msg:TofDist) -> None :
        tof = msg.tof
        # array shape as 8x8
        dist = np.float32(msg.dist).reshape(8,8)/np.float32(1000) # meters
        # replace -1mm with inf
        dist = np.where(dist>=0, dist, np.inf)

        # find minimum dist for obstacle detection
        # TODO: Do we need to not use the lower rows to not see ground clutter?
        # TODO: Should we have different max based on row?
        # TODO: Do we need to calc distance based on row angle?
        dist_min = np.min(dist)
        if dist_min > self.tof_dist_obstacle_max : dist_min = np.inf

        # self.get_logger().info(f"tof_dist_callback: {tof} {dist_min=:.3f}")

        if tof == "tof_fc" :
            self.tof_fc_obstacle_dist = dist_min
        if tof == "tof_fl" :
            self.tof_fl_obstacle_dist = dist_min
        if tof == "tof_fr" :
            self.tof_fr_obstacle_dist = dist_min


    # Cone distance and angle relative to front TOF sensors
    def tof_fc_mid_subscription_callback(self, msg: Float32X8) -> None:
        # self.get_logger().info(f"{msg=}")

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

        # self.get_logger().info(f"tof_fc_callback: {x=:.3f} {y=:.3f} {a=:.3f} {d=:.3f} ")
        

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
            pose = self.createPoseStamped(x,y,a,child_frame)
        else :
            pose = None
            
        return (tf_OK,pose)
        

    def createPose(self, x:float, y:float, yaw:float) -> Pose :
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
        ) = tf_transformations.quaternion_from_euler(0.0,0.0,float(yaw))
        
        return pose

    def createPoseStamped(self,x:float,y:float,a:float,frame_id:str) -> PoseStamped:
        stamp = self.nav.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        # TODO: use a given stamp time?
        pose.header.stamp = stamp
        pose.pose = self.createPose(x,y,a)
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
                    # self.nav.cancelTask()
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
    # wait never returns
    # nav.waitUntilNav2Active()

    node = NavNode(nav)

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