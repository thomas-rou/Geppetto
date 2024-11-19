import math
import rclpy
import asyncio
from rclpy.node import Node
from dotmap import DotMap
from geometry_msgs.msg import Pose
from com_bridge.common_methods import get_robot_name, get_other_robot_name, get_robot_ip
from com_bridge.common_enums import Network
from com_bridge.websocket_subscriber import WebSocketSubscriber  

def calculate_cartesian_distance(pose):
    return math.sqrt(pose.position.x ** 2 + pose.position.y ** 2 + pose.position.z ** 2)

class P2PNode(Node):
    def __init__(self):
        super().__init__(f'P2PNode')
        self.robot_name = get_robot_name()
        self.other_robot_name = get_other_robot_name()
        self_robot_topic_pose = f'{self.robot_name}/pose'
        
        self.local_pose_subscriber = self.create_subscription(
            Pose,
            self_robot_topic_pose,
            self.local_pose_callback,
            10
        )

        self.get_logger().info(f"Subscribed succesfully to : {self_robot_topic_pose}")

        self.websocket_subscriber = WebSocketSubscriber()
        self.local_distance = None 
        self.other_distance = None

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
        msg = DotMap(msg)
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

async def spin_ros_node(node: P2PNode, sleep_interval: float = 0.1):
    """
    Asynchronously spin a ROS 2 node, processing its callbacks.
    :param node: The ROS 2 node to spin.
    :param sleep_interval: Time to sleep between spin iterations, in seconds.
    """
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)  # Process ROS events
        await asyncio.sleep(sleep_interval)  # Yield control to the asyncio event loop


async def main_async():
    rclpy.init()
    node = P2PNode()
    try:
        loop = asyncio.get_event_loop()
        loop.create_task(spin_ros_node(node))
        loop.create_task(node.subscribe_to_other_robot_pose())
        await asyncio.sleep(float('inf'))
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        await node.shutdown()
        rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == '__main__':
    main()


def main():
    asyncio.run(main_async())


if __name__ == '__main__':
    main()
