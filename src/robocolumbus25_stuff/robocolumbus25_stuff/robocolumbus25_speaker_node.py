import rclpy
import os
import json
import time

from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

class SpeakerNode(Node):
    '''
    Uses the USB speaker
    Text to speech for robot status info etc
    '''

    volume=200

    def __init__(self):
        super().__init__('speaker_node')
            
        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "json_msg"
                                        , self.json_msg_callback, 10)
                
        time.sleep(2) # wait for json_msg_publisher to be ready!!??
        self.tts("Speaker Node Started")
        self.get_logger().info(f"SpeakerNode Started")

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
        {"speaker":{"tts": string}} # Text to speach
        """
        #self.get_logger().info(f"json_msg_callback: {msg=}")
        data = json.loads(msg.data)

        if 'speaker' in data:
            speaker = data['speaker']
        else : return
        
        if "tts" in speaker:
            tts = speaker['tts']
            self.get_logger().info(f"json_msg_callback: {tts=}")

            a = self.volume
            os.system(f'espeak -a {a} "{tts}"')

        

    def destroy_node(self):
        self.get_logger().info("destroy_node")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
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