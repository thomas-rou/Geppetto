import rclpy
from rclpy.node import Node
from common_msgs.msg import StartMission, StopMission, IdentifyRobot
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os
import subprocess
import json
from com_bridge.common_methods import set_mission_status
from com_bridge.common_enums import Robot_Status

class ComServNode(Node):
    def __init__(self):
        super().__init__('com_serv')
        self.get_logger().info(f"Server Launched waiting for messages in {os.getenv('ROBOT')}")
        self.identification_subscription = self.create_subscription(
            IdentifyRobot,
            f"{os.getenv('ROBOT')}/identify_command",
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
        set_mission_status(Robot_Status.WAITING)
        json_str = json.dumps(json_data)
        self.get_logger().info('Mission stopped')
        if self.timer_active:
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0
            self.mission_mouvements.publish(twist_msg)
            self.destroy_timer(self.timer)  # Stop the timer
            self.timer_active = False

    def start_mission_callback(self, msg):
        json_data = {
            "command": msg.command,
            "orientation": msg.mission_details.orientation1,
            "position": {
                "x": msg.mission_details.position1.x,
                "y": msg.mission_details.position1.y
            },
            "timestamp": msg.timestamp
        }
        set_mission_status(Robot_Status.MISSION_ON_GOING)
        if not self.timer_active:
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
            self.timer_active = True
        json_str = json.dumps(json_data)
        self.get_logger().info(f'Starting Mission: {json_str}')
    
    def identification_callback(self, msg):
        self.get_logger().info(f'identification')
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
