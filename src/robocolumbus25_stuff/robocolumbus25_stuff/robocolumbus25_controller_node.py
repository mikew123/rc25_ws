import rclpy
import os
import json
import time

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

    gotoConeFile = None
    smTimerRateHz = 1.0

    def __init__(self):
        super().__init__('controller_node')
            
        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "json_msg"
                                        , self.json_msg_callback, 10)
                
        self.battery_status_msg_publisher = self.create_publisher(BatteryState, 'battery_status', 10)

        try :
            f = open("../sambashare/nav_files/navto_cone_coordinates.json")
            self.gotoConeFile = f.read()
            f.close()
        except :
            self.get_logger().info(f"No file found - navto_cone_coordinates.json")
            pass

        self.sm_timer = self.create_timer((1.0/self.smTimerRateHz), self.sm_timer_callback)


        self.tts("Controller Node Started")
        self.get_logger().info(f"ControllerNode Started")

    sm_next_state = 0
    nextConeNum = 1
    requestNextCone = False

    def sm_timer_callback(self) -> None :
        state = self.sm_next_state
        next_state = state

        if state == 0 :
            if self.requestNextCone :
                self.requestNextCone = False
                status = self.sendConeNavCoordinates(self.nextConeNum)
                if status == True :
                    self.nextConeNum +=1
                    next_state = 0
                else :
                    next_state = 1

        elif state == 1:
            # DONE
            self.tts("Successfully went to each cone")
            pass

        self.sm_next_state = next_state

    def sendConeNavCoordinates(self, n:int) -> bool :
        try :
            self.get_logger().info(f"{self.gotoConeFile=}")
            data = json.loads(self.gotoConeFile)
            gotoxy = data["gotoxy"]
            coneN = f"cone{n}"
            cone = gotoxy[coneN]
            x = cone["x"]
            y = cone["y"]
            self.get_logger().info(f"gotoxy {coneN} {x=} {y=}")
            json_msg = {"nav":{"gotoxy":{"n":n, "x":x, "y":y}}}
            self.sendJsonMsg(json_msg)
            return True
        except :
            self.get_logger().info(f"Not valid {self.gotoConeFile=}")
            return False

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
            if "requestcone" in nav :
                requestcone = nav["requestcone"]
                self.requestNextCone = requestcone

    def processEngineStatus(self,status:String) -> None :
            # self.get_logger().info(f"processEngineStatus: {status=}")
            if "mode" in status : self.processMode(status["mode"])
            if "rca"  in status : self.processRca(status["rca"])
            if "kse"  in status : self.processKse(status["kse"])
            if "ksl"  in status : self.processKsl(status["ksl"])
            if "vbat" in status : self.processVbat(status["vbat"])

    # timer for battery report when not LOW
    bat_timer = 0
    bat_per = 10.0
    lastVbat = 100.0
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