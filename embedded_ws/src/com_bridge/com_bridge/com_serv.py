# com_bridge/com_serv.py

import rclpy
from rclpy.node import Node
from common_msgs.msg import StartMission, StopMission, IdentifyRobot
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os
import subprocess
import json

class ComServNode(Node):
    def __init__(self):
        super().__init__('com_serv')

        self.identification_subscription = self.create_subscription(
            IdentifyRobot,
            f'{os.get_env('ROBOT')}/identify_command',
            self.identification_callback,
            10
        )

        self.start_mission_subscription = self.create_subscription(
            StartMission,
            'start_mission_command',
            self.start_mission_callback,
            10
        )

        self.stop_mission_subscription = self.create_subscription(
            StopMission,
            'stop_mission_command',
            self.stop_mission_callback,
            10
        )

        self.mission_mouvements = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer_period = 0.5  # Timer period in seconds
        self.timer = None
        self.timer_active = False

    def stop_mission_callback(self, msg):
        json_data = {
            "command": msg.command,
            "timestamp": msg.timestamp
        }
        json_str = json.dumps(json_data)
        if self.timer_active:
            self.destroy_timer(self.timer)  # Stop the timer
            self.timer_active = False
            self.get_logger().info('Mission stopped')

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

        if not self.timer_active:
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
            self.timer_active = True
        json_str = json.dumps(json_data)
        self.get_logger().info(f'Starting Mission: {json_str}')
    
    def identification_callback(self, msg):
        command = ["mpg123", "sounds/tp_pas_heure.mp3"]
        subprocess.run(command)
    
    def timer_callback(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 1.8
        
        self.mission_mouvements.publish(twist_msg)


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
