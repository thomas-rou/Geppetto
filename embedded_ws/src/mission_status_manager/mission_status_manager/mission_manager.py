import rclpy
from rclpy.node import Node
from limo_msgs.msg import LimoStatus
from common_msgs.msg import MissionStatus
BATTERY_CAPACITY = 12.0

import os

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.get_logger().info(f"Mission manager Launched waiting for messages in {os.getenv('ROBOT')}")
        self.mission_status_publisher = self.create_publisher(MissionStatus,f"{os.getenv('ROBOT')}/mission_status", 10)

        self.battery_subscription = self.create_subscription(
            LimoStatus,
            '/limo_status',
            self.status_publication_callback,
            10
        )

    def status_publication_callback(self, battery_data):
        try:
            battery_level = round((battery_data.battery_voltage/BATTERY_CAPACITY)*100)
            mission_status = MissionStatus()
            mission_status.robot_status = os.getenv('MISSION_STATUS')
            mission_status.battery_level = battery_level
            self.mission_status_publisher.publish(mission_status)
        except Exception as e:
            self.get_logger().info("Failed to publish mission status: "+str(e))

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        