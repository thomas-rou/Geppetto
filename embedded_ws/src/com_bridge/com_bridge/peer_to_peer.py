import math
import rclpy
import asyncio
from rclpy.node import Node
from geometry_msgs.msg import Pose
from com_bridge.common_methods import get_robot_name, get_other_robot_name, get_robot_ip
from com_bridge.common_enums import Network
from com_bridge.websocket_subscriber import WebSocketSubscriber  

def calculate_cartesian_distance(pose):
    return math.sqrt(pose["position"]["x"] ** 2 + pose["position"]["y"] ** 2 + pose["position"]["z"] ** 2)

class P2PNode(Node):
    def __init__(self):
        super().__init__(f'P2PNode')
        self.robot_name = get_robot_name()
        self.other_robot_name = get_other_robot_name()
        
        self.local_pose_subscriber = self.create_subscription(
            Pose,
            f'/{self.robot_name}/pose',
            self.local_pose_callback,
            10
        )

        self.websocket_subscriber = WebSocketSubscriber()


    async def subscribe_to_other_robot_pose(self):
        """
        Subscribe to the other robot's pose topic using WebSocketSubscriber.
        """
        await self.websocket_subscriber.subscribe_to_topic(
            f'/{self.other_robot_name}/pose',
            'geometry_msgs/msg/Pose',
            self.other_robot_pose_callback
        )

    def compare_distances(self):
        """
        Compare distances of the local robot and the other robot.
        Trigger actions based on whether the local robot is the nearest or farthest.
        """
        if self.local_distance is None or self.other_distance is None:
            self.get_logger().info(f"Did not get one of distances: local {self.local_distance} \
            other : {self.other_distance}")
            return

        if self.local_distance > self.other_distance:
            self.get_logger().info("Local robot is the farthest. Showing farthest icon.")
            self.on_farthest_icon()
        else:
            self.get_logger().info("Local robot is the nearest. Showing nearest icon.")
            self.on_nearest_icon()

    def local_pose_callback(self, msg):
        """
        Handle pose updates from the local robot.
        """
        self.local_distance = calculate_cartesian_distance(msg)
        self.get_logger().info(f"Local robot distance: {self.local_distance}")
        self.compare_distances()

    def other_robot_pose_callback(self, msg):
        """
        Handle pose updates from the other robot.
        """
        self.other_distance = calculate_cartesian_distance(msg)
        self.get_logger().info(f"Other robot distance: {self.other_distance}")
        self.compare_distances()

    def on_farthest_icon(self):
        self.get_logger().info("Changing icon for the farthest robot.")

    def on_nearest_icon(self):
        self.get_logger().info("Changing icon for the nearest robot.")

    async def shutdown(self):
        """
        Close WebSocket connection on shutdown.
        """
        await self.websocket_subscriber.close()

async def main_async():
    rclpy.init()
    node = P2PNode()
    try:
        await node.subscribe_to_other_robot_pose()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        await node.shutdown()
        rclpy.shutdown()

def main():
    asyncio.run(main_async())


if __name__ == '__main__':
    main()
