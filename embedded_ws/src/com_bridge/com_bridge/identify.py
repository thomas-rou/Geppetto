# com_bridge/com_serv.py

from embedded_ws.common.common_methods import set_mission_status
import rclpy
from rclpy.node import Node
from common_msgs.msg import IdentifyRobot
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os
import subprocess
import json

class IdentifyNode(Node):
    def __init__(self):
        super().__init__('identify_node')
        self.get_logger().info(f"Server Launched waiting for messages in {os.getenv('ROBOT')}")
        self.identification_subscription = self.create_subscription(
            IdentifyRobot,
            f"{os.getenv('ROBOT')}/identify_command",
            self.identification_callback,
            10
        )

    def identification_callback(self, msg):
        self.get_logger().info(f'identification')
        command = ["mpg123", "sounds/tp_pas_heure.mp3"]
        subprocess.run(command)

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ComServNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
