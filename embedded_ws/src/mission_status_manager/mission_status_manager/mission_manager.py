import rclpy
from rclpy.node import Node
from limo_msgs.msg import LimoStatus
from std_msgs.msg import String, Float32
# from common_msgs.msg import StartMission, StopMission, IdentifyRobot
import os

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.get_logger().info(f"Mission manager Launched waiting for messages in {os.getenv('ROBOT')}")
        self.mission_status_publisher = self.create_publisher(String,f"{os.getenv('ROBOT')}/mission_status", 10)

        self.battery_subscription = self.create_subscription(
            LimoStatus,
            '/limo_status',
            self.status_publication_callback,
            10
        )

    def status_publication_callback(self, battery_data):
        try:
            battery_level = battery_data.voltage
            battery_msg = Float32()
            battery_msg.data = battery_level
            msg = String()
            msg.data = os.getenv('MISSION_STATUS')
            self.get_logger().info(f"Publishing: {msg.data}")
            self.mission_status_publisher.publish(msg)
        except:
            self.get_logger().info("Failed to publish mission status")

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
        