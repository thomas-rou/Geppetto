# com_bridge/com_serv.py

import rclpy
from rclpy.node import Node
from common_msgs.msg import StartMission
from std_msgs.msg import String
from com_bridge.socket_manager import SocketManager
import json

class ComServNode(Node):
    def __init__(self):
        super().__init__('com_serv')
        self.socket_manager = SocketManager()

        # Abonnement au topic pour les messages de mission
        self.mission_subscription = self.create_subscription(
            StartMission,
            'start_mission_command',
            self.start_mission_callback,
            10
        )

    def server_info_callback(self, msg):
        server_info = msg.data.split(',')
        if len(server_info) == 2:
            server_ip = server_info[0]
            port = int(server_info[1])
            self.socket_manager.connect(server_ip, port)
            self.get_logger().info(f'Connected to server at {server_ip}:{port}')
            # print(f'Connected to server at {server_ip}:{port}')

    def start_mission_callback(self, msg):
        # Convertir le message ROS en JSON
        # self.get_logger().info(msg)
        # print(type(msg))
        json_data = {
            "command": msg.command,
            "orientation": msg.MissionDetails.orientation,
            "position": {
                "x": msg.MissionDetails.position.x,
                "y": msg.MissionDetails.position.y
            },
            "timestamp": msg.timestamp
        }
        json_str = json.dumps(json_data)
        self.get_logger().info(f'Sending message: {json_str}')

        # Envoyer le JSON au serveur distant
        # self.socket_manager.send(json_str)

    def destroy_node(self):
        self.socket_manager.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ComServNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
