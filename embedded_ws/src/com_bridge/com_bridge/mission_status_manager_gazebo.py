import os
import rclpy
from rclpy.node import Node
from common_msgs.msg import MissionStatus
from com_bridge.common_methods import set_mission_status, get_mission_status
from com_bridge.common_enums import RobotStatus

TIMER_PERIOD = 1.0
BATTERY_THRESHOLD = 0

class MissionStatusManagerGazebo(Node):
    def __init__(self):
        super().__init__("mission_manager_status_gazebo")
        self.battery_level = 100
        self.get_logger().info(
            f"Mission manager Launched waiting for messages in {os.getenv('ROBOT')}"
        )
        self.mission_status_publisher = self.create_publisher(
            MissionStatus, f"{os.getenv('ROBOT')}/mission_status", 10
        )
        self.timer = self.create_timer(TIMER_PERIOD, self.publish_mission_status)

    def decrease_battery_level(self):
        if self.battery_level > BATTERY_THRESHOLD:
            self.battery_level -= 0.1
        round(self.battery_level)

    def publish_mission_status(self):
        try:
            self.decrease_battery_level()
            mission_status = MissionStatus()
            # TODO: get robot id from gazebo
            mission_status.robot_id = ""
            mission_status.battery_level = int(self.battery_level)
            mission_status.robot_status = get_mission_status()
            if (
                mission_status.battery_level <= 30
                and mission_status.robot_status != RobotStatus.LOW_BATTERY
            ):
                mission_status.robot_status = RobotStatus.LOW_BATTERY
                set_mission_status(RobotStatus.LOW_BATTERY)
                # TODO: call low battery callback here
            self.mission_status_publisher.publish(mission_status)
        except Exception as e:
            self.get_logger().info("Failed to publish mission status: " + str(e))


def main(args=None):
    rclpy.init(args=args)
    node = MissionStatusManagerGazebo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
