from com_bridge.common_methods import get_robot_name
from com_bridge.common_enums import GlobalConst, LogType, RobotName
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from com_bridge.log import LoggerNode
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, Quaternion, Pose
from nav_msgs.msg import Path
from common_msgs.msg import PathLength
import numpy as np

TIMER_PERIOD = 0.5

class RobotPose(Node):

    def __init__(self):
        super().__init__("robot_pose")
        self.logger = LoggerNode()

        self.declare_parameter("robot_id", "")
        self.robot_id = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )
        self.robot_name = get_robot_name() or self.robot_id

        self.last_odometry_msg, self.prev_odometry_msg = None, None

        odom_topic = f"/{self.robot_id}/odom" if self.robot_name == RobotName.GAZEBO else "/odom"
        path_topic = f"/{self.robot_id}/plan" if self.robot_name == RobotName.GAZEBO else "/path"
        self.robot_odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            GlobalConst.QUEUE_SIZE
        )

        self.distance_traveled = 0
        self.distance_traveled_subscription = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            GlobalConst.QUEUE_SIZE,
        )

        robot_pose_topic = f"{self.robot_id}/pose" if self.robot_name == RobotName.GAZEBO else f"{self.robot_name}/pose"
        distance_traveled_topic = f"{self.robot_id}/path_length" if self.robot_name == RobotName.GAZEBO else f"{self.robot_name}/path_length"

        self.robot_pose_publisher = self.create_publisher(Pose, robot_pose_topic, GlobalConst.QUEUE_SIZE)
        self.distance_traveled_publisher = self.create_publisher(PathLength, distance_traveled_topic, GlobalConst.QUEUE_SIZE)

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        self.logger.log_message(LogType.INFO, f"Robot position logger launched for {self.robot_name} and subscribed to {odom_topic}")

    def odom_callback(self, msg: Odometry) -> None:
        self.last_odometry_msg = msg

    def path_callback(self, msg: Path) -> None:
        for i in range(len(msg.poses) - 1):
            position_a = msg.poses[i].pose.position
            position_b = msg.poses[i + 1].pose.position
            self.distance_traveled += np.sqrt(
                np.power((position_b.x - position_a.x), 2) + np.power((position_b.y - position_a.y), 2)
            )

    def timer_callback(self) -> None:
        if self.should_publish():
            self.publish_pose()
            self.publish_distance_traveled()

    def publish_pose(self) -> None:
        position = self.last_odometry_msg.pose.pose.position
        orientation = self.last_odometry_msg.pose.pose.orientation

        pose_msg = Pose()
        pose_msg.position = Point(x=position.x, y=position.y, z=position.z)
        pose_msg.orientation = Quaternion(
            x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w
        )

        self.robot_pose_publisher.publish(pose_msg)
        self.prev_odometry_msg = self.last_odometry_msg

    def publish_distance_traveled(self) -> None:
        distance_traveled_msg = PathLength()
        distance_traveled_msg.path_length = self.distance_traveled
        self.distance_traveled_publisher.publish(distance_traveled_msg)
        self.logger.log_message(LogType.INFO, f"Distance traveled by {self.robot_id} is {self.distance_traveled}")

    def should_publish(self) -> bool:
        return self.last_odometry_msg is not None


def main(args=None):
    rclpy.init(args=args)
    robot_pose = RobotPose()
    executor = MultiThreadedExecutor()
    executor.add_node(robot_pose)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        robot_pose.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()