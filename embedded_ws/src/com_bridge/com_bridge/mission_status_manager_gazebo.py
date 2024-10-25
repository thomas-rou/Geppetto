import rclpy
from rclpy.node import Node
from common_msgs.msg import MissionStatus
from com_bridge.common_methods import (
    set_mission_status,
    get_mission_status,
)
from com_bridge.common_enums import RobotStatus, GlobalConst

TIMER_PERIOD = 1.0
BATTERY_THRESHOLD = 30.0
DECREASE_BATTERY_LEVEL = 0.1


class MissionStatusManagerGazebo(Node):
    def __init__(self):
        super().__init__("mission_manager_status_gazebo")
        self.declare_parameter("robot_id", "Unknown")

        self.battery_level = 100
        self.robot_id = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )
        self.get_logger().info(
            f"Mission manager Launched for robot {self.robot_id}. Waiting for messages from Gazebo"
        )

        self.mission_status_publisher = self.create_publisher(
            MissionStatus, f"{self.robot_id}/mission_status", GlobalConst.QUEUE_SIZE
        )
        self.timer = self.create_timer(TIMER_PERIOD, self.publish_mission_status)

    def decrease_battery_level(self):
        self.battery_level -= DECREASE_BATTERY_LEVEL
        round(self.battery_level)

    def publish_mission_status(self):
        try:
            self.decrease_battery_level()
            mission_status = MissionStatus()
            mission_status.robot_id = self.robot_id
            mission_status.battery_level = int(self.battery_level)
            mission_status.robot_status = get_mission_status()
            if (
                mission_status.battery_level <= BATTERY_THRESHOLD
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
