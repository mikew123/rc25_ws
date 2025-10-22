import rclpy
import os
import json
import time
import yaml

from pathlib import Path
from pprint import pprint, pformat

from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import BatteryState


class ControllerNode(Node):
    '''
    Controls the robot navigation etc
    Uses files in ~/sambashare
    Processes the json_msgs engine statuses
    '''
    # YAML file with cone locations
    waypointsFile = "~/sambashare/nav_files/RoboColumbus.yml"
    waypoints = None

    smTimerRateHz = 1.0
    sm_next_state = 0
    nextWaypointNum = 1
    requestNextWaypoint = False
    gpsLocalization = False

    bat_timer = 0
    bat_per = 10.0
    lastVbat = 100.0

    def __init__(self):
        super().__init__('controller_node')
            
        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "json_msg"
                                        , self.json_msg_callback, 10)
                
        self.battery_status_msg_publisher = self.create_publisher(BatteryState, 'battery_status', 10)

        self.readWaypointsFile(self.waypointsFile)

        self.sm_timer = self.create_timer((1.0/self.smTimerRateHz), self.sm_timer_callback)

        self.tts("Controller Node Started")
        self.get_logger().info(f"ControllerNode Started")

    def readWaypointsFile(self, file) :
        '''
        Read yaml file with waypoint locations and if gps is enable for localization
        save as dict self.waypoints
        Each waypoint has se;ection to look for cone or not (intermediate waypoint)
        Examples :
        # waypoint with cone location in ROS map related meters x,y
        # gps is not used for localization
            config :
            gps_localization : false
            waypoints :
            1 :
                cone : true
                x : 10.0
                y : 0.0
        # waypoint location in absolute lat,lon
        # gps is used for localization
            config :
            gps_localization : true
            waypoints :
            1 :
                cone : false
                lat : 12345678
                lon : 567890
        '''

        path = Path(file).expanduser()
        if not path.exists():
            self.get_logger().info(f"YAML file not found: {path}")
            return
        try:
            with path.open("r", encoding="utf-8") as f:
                doc = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().info(f"Failed to read/parse YAML: {e}")
            return

        if not doc:
            self.get_logger().info("YAML file is empty")
            return

        self.waypoints = doc

        self.get_logger().info("read waypoints File : \n"+pformat(self.waypoints))

        self.gpsLocalization = self.waypoints["config"]["gps_localization"]
        self.get_logger().info(f"GPS localization enabled = {self.gpsLocalization}")


    def sm_timer_callback(self) -> None :
        if self.waypoints == None : return

        numWaypoints = self.waypoints["config"]["num_waypoints"]

        state = self.sm_next_state
        next_state = state

        if state == 0 :
            if self.requestNextWaypoint :
                self.requestNextWaypoint = False
                self.get_logger().info(f"Getting requested next waypoint {self.nextWaypointNum}")
                self.tts(f"Getting requested way point {self.nextWaypointNum}")
                status = self.sendWaypointToNav(self.nextWaypointNum)
                if (status == True) and (self.nextWaypointNum < numWaypoints ):
                    self.nextWaypointNum +=1
                    next_state = 0
                else :
                    self.tts("Sent all waypoint locations")
                    next_state = 1

        elif state == 1:
            # DONE
            pass

        self.sm_next_state = next_state

    def sendWaypointToNav(self, n:int) -> bool :
        try :
            data = self.waypoints
            waypoints = data["waypoints"]
            waypointN = waypoints[n]
        except :
            self.get_logger().info(f"Yaml does not have the requested waypoint {n}: {self.gotoConeFile=}")
            self.tts(f"Could not find waypoint {n} location in the file")
            return False

        self.get_logger().info(f"Sending {waypointN=}")
        json_msg = {"nav":{"waypoint":waypointN}}
        self.sendJsonMsg(json_msg)
        self.tts(f"Sent way point {n} to navigation node")
        return True

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
        """
        Processes
        {"engine":{"status": {}}} messages
        """
        #self.get_logger().info(f"json_msg_callback: {msg=}")
        data = json.loads(msg.data)

        if 'engine' in data : 
            engine = data["engine"]
            if "status" in engine:
                status = engine['status']
                self.processEngineStatus(status)

        if 'nav' in data :
            nav = data['nav']
            if "request_waypoint" in nav :
                request_waypoint = nav["request_waypoint"]
                self.requestNextWaypoint = request_waypoint

    def processEngineStatus(self,status:String) -> None :
            # self.get_logger().info(f"processEngineStatus: {status=}")
            if "mode" in status : self.processMode(status["mode"])
            if "rca"  in status : self.processRca(status["rca"])
            if "kse"  in status : self.processKse(status["kse"])
            if "ksl"  in status : self.processKsl(status["ksl"])
            if "vbat" in status : self.processVbat(status["vbat"])

    # timer for battery report when not LOW
    def processVbat(self, vbat) -> None :
        # round to 1 decimal place
        vbat = round(vbat, 1)

        if vbat > 0 and vbat <= 11.1 :
            self.get_logger().warning(f"Battery low {vbat=:.3f}")
        elif (time.time_ns()*1e-9 - self.bat_timer) > self.bat_per :
            self.get_logger().info(f"Battery {vbat=:.3f}")
            self.bat_timer = time.time_ns()*1e-9

        # Send /battery_state topic message
        # TODO: add current and possibly cell voltages
        bmsg = BatteryState()
        bmsg.header.stamp = self.get_clock().now().to_msg()
        bmsg.header.frame_id = "base_link"
        bmsg.voltage = float(vbat)
        bmsg.present = True
        self.battery_status_msg_publisher.publish(bmsg)

        # Send battery status to the speaker
        if ((vbat>11.1 and vbat<(self.lastVbat - 0.1)) or vbat<self.lastVbat) :
            if vbat == 0.0 :
                self.tts("No battery")
            elif vbat>11.1 :
                self.tts(f"Battery {vbat}")
            elif vbat>10.9 :
                self.tts(f"Battery low: {vbat}")
            elif vbat > 10.8 :
                self.tts(f"Warning! Battery {vbat}")
            else :
                self.tts(f"DANGER! Battery {vbat}")
        
            self.lastVbat = vbat

    modeState:String = ""
    def processMode(self, mode:String) -> None :
        change = False
        if self.modeState != mode :
            self.modeState = mode
            change = True

        if change :
            if mode=="bypass" : self.tts(f"engine is controlled by remote") 
            if mode=="cv"     : self.tts(f"engine is controlled by computer") 

    rcaState:bool = False
    def processRca(self, rca:bool) -> None :
        change = False
        if self.rcaState != rca :
            self.rcaState = rca
            change = True

        if change :
            self.tts(f"Remote control active {rca}") 

    kseState:bool = False
    def processKse(self, kse:bool) -> None :
        change = False
        if self.kseState != kse :
            self.kseState = kse
            change = True

        if change :
            self.tts(f"Kill switch enabled {kse}") 

    kslState:bool = False
    def processKsl(self, ksl:bool) -> None :
        change = False
        if self.kslState != ksl :
            self.kslState = ksl
            change = True

        if change :
            self.tts(f"Kill switch latched {ksl}") 

    ksaState:bool = False
    def processKsa(self, ksa:bool) -> None :
        change = False
        if self.ksaState != ksa :
            self.ksaState = ksa
            change = True

        if change :
            self.tts(f"Kill switch active {ksa}") 


    def destroy_node(self):
        self.get_logger().info("destroy_node")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
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