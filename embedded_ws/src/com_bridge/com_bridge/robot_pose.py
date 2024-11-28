from com_bridge.common_methods import get_robot_name
from com_bridge.common_enums import GlobalConst, LogType, RobotName
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from com_bridge.log import LoggerNode
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, Quaternion, Pose

TIMER_PERIOD = 0.5

class RobotPose(Node):

    def __init__(self):
        super().__init__("sensor_logger")
        self.logger = LoggerNode()

        self.declare_parameter("robot_id", "")
        self.robot_id = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )
        self.robot_name = get_robot_name() or self.robot_id

        self.last_odometry_msg, self.prev_odometry_msg = None, None

        odom_topic = f"/{self.robot_id}/odom" if self.robot_name == RobotName.GAZEBO else "/odom"
        self.robot_odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            GlobalConst.QUEUE_SIZE
        )

        topic_name = f"{self.robot_id}/pose" if self.robot_name == RobotName.GAZEBO else f"{self.robot_name}/pose"
        self.robot_pose_publisher = self.create_publisher(Pose, topic_name, GlobalConst.QUEUE_SIZE)

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        self.logger.log_message(LogType.INFO, f"Robot position logger launched for {self.robot_name} and subscribed to {odom_topic}")

    def odom_callback(self, msg: Odometry) -> None:
        self.last_odometry_msg = msg

    def timer_callback(self) -> None:
        if self.should_publish():
            self.publish_pose()

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

    def should_publish(self) -> bool:
        return self.last_odometry_msg is not None


def main(args=None):
    rclpy.init(args=args)
    sensor_logger = RobotPose()
    executor = MultiThreadedExecutor()
    executor.add_node(sensor_logger)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        sensor_logger.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()