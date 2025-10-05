import rclpy
import os
import json
import time

from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

class ControllerNode(Node):
    '''
    Controls the robot navigation etc
    Uses files in ~/sambashare
    Processes the json_msgs engine statuses
    '''

    def __init__(self):
        super().__init__('controller_node')
            
        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "json_msg"
                                        , self.json_msg_callback, 10)
                
        self.tts("Controller Node Started")
        self.get_logger().info(f"ControllerNode Started")

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

    def processEngineStatus(self,status:String) -> None :
            self.get_logger().info(f"processEngineStatus: {status=}")
            if "mode" in status : self.processMode(status["mode"])
            if "rca" in status : self.processRca(status["rca"])
            if "kse" in status : self.processKse(status["kse"])
            if "ksl" in status : self.processKsl(status["ksl"])
            if "ksa" in status : self.processKsa(status["ksa"])

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