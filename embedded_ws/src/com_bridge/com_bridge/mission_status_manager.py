import os
import rclpy
from rclpy.node import Node
from limo_msgs.msg import LimoStatus
from common_msgs.msg import MissionStatus
from com_bridge.common_methods import set_mission_status, get_mission_status
from com_bridge.common_enums import RobotStatus, GlobalConst
BATTERY_CAPACITY = 12.0
BATTERY_THRESHOLD = 30
MAX_BATTERY_LEVEL = 100

import os

class MissionStatusManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.get_logger().info(f"Mission manager Launched waiting for messages in {os.getenv('ROBOT')}")
        self.mission_status_publisher = self.create_publisher(MissionStatus,f"{os.getenv('ROBOT')}/mission_status", GlobalConst.QUEUE_SIZE)

        self.battery_subscription = self.create_subscription(
            LimoStatus,
            '/limo_status',
            self.status_publication_callback,
            GlobalConst.QUEUE_SIZE
        )

    def status_publication_callback(self, battery_data):
        try:
            battery_level = round((battery_data.battery_voltage/BATTERY_CAPACITY)*MAX_BATTERY_LEVEL)
            mission_status = MissionStatus()
            mission_status.robot_id = os.getenv('ROBOT')[-1]
            mission_status.battery_level = battery_level
            mission_status.robot_status = get_mission_status()
            if mission_status.battery_level <= BATTERY_THRESHOLD and mission_status.robot_status != RobotStatus.LOW_BATTERY:
                mission_status.robot_status = RobotStatus.LOW_BATTERY
                set_mission_status(RobotStatus.LOW_BATTERY)
                # TODO: call low battery callback here
            self.mission_status_publisher.publish(mission_status)
        except Exception as e:
            self.get_logger().info("Failed to publish mission status: "+str(e))

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MissionStatusManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        