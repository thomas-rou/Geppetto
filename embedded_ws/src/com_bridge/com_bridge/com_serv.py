# com_bridge/com_serv.py

import rclpy
from rclpy.node import Node
from common_msgs.msg import StartMission
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json

class ComServNode(Node):
    def __init__(self):
        super().__init__('com_serv')

        # Abonnement au topic pour les messages de mission
        self.mission_subscription = self.create_subscription(
            StartMission,
            'start_mission_command',
            self.start_mission_callback,
            10
        )

        self.start_mission_order = self.create_publisher(Twist, 'cmd_vel', 10)
        # msg_to_send = String()
        # msg_to_send.data = 'Ok'
        # self.update.publish(msg_to_send)

    def start_mission_callback(self, msg):
        json_data = {
            "command": msg.command,
            "orientation": msg.mission_details.orientation,
            "position": {
                "x": msg.mission_details.position.x,
                "y": msg.mission_details.position.y
            },
            "timestamp": msg.timestamp
        }
        json_str = json.dumps(json_data)
        self.get_logger().info(f'Sending message: {json_str}')

        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 1.8
        
        self.start_mission_order.publish(twist_msg)


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
