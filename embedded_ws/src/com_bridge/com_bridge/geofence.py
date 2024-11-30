from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Header
from com_bridge.common_enums import GlobalConst, LogType
from com_bridge.log import LoggerNode
from common_msgs.msg import GeofenceBounds
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
from time import sleep

TIME_TO_GATHER_TOUGHTS = 3
GLOBAL_FRAME_ID = "world"


class GeofenceState(Enum):
    INSIDE = 0
    RETURN_TO_GEOFENCE = 1
    RETURNING = 2


class GeofenceNode(Node):
    def __init__(self) -> None:
        super().__init__("geofence_node")

        self.declare_parameter("robot_id", "unknown")
        self.robot_id = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )

        self.get_logger().info(
            f"Geofence node initialised for {self.robot_id}. Waiting for bounds!"
        )
        self.logger.log_message(
            LogType.INFO,
            f"Geofence node initialised for {self.robot_id}. Waiting for bounds!",
        )

        self.logger = LoggerNode()
        self.navigator = BasicNavigator(namespace=self.robot_id)

        self.is_geofence_active = False

        self.state = GeofenceState.INSIDE

        self.exploration_publisher = self.create_publisher(
            Bool, f"/{self.robot_id}/explore/resume", GlobalConst.QUEUE_SIZE
        )

        self.geofence_subscription = self.create_subscription(
            GeofenceBounds,
            "/geofence/bounds",
            self.geofence_bounds_callback,
            GlobalConst.QUEUE_SIZE,
        )

        self.toggle_subscription = self.create_subscription(
            Bool,
            "/geofence/resume",
            self.geofence_toggle_callback,
            GlobalConst.QUEUE_SIZE,
        )

        self.pose_subscription = self.create_subscription(
            Pose,
            f"/{self.robot_id}/pose",
            self.pose_callback,
            GlobalConst.QUEUE_SIZE,
        )

    def geofence_bounds_callback(self, msg: GeofenceBounds) -> None:
        self.geofence_x_min = msg.x_min
        self.geofence_x_max = msg.x_max
        self.geofence_y_min = msg.y_min
        self.geofence_y_max = msg.y_max

        self.get_logger().info(
            f"Updated geofence: x_min={self.geofence_x_min}, x_max={self.geofence_x_max}, y_min={self.geofence_y_min}, y_max={self.geofence_y_max}"
        )
        self.logger.log_message(
            LogType.INFO,
            f"Updated geofence: x_min={self.geofence_x_min}, x_max={self.geofence_x_max}, y_min={self.geofence_y_min}, y_max={self.geofence_y_max}",
        )
        self.activate_geofence()

    def geofence_toggle_callback(self, msg: Bool) -> None:
        if msg.data:
            self.activate_geofence()
        else:
            self.deactivate_geofence()

    def pose_callback(self, msg: Pose) -> None:
        self.x = msg.position.x
        self.y = msg.position.y
        self.check_state()

    def activate_geofence(self) -> None:
        self.is_geofence_active = True
        self.get_logger().info("Geofence is up and running!")
        self.logger.log_message(
            LogType.INFO,
            "Geofence is up and running!",
        )

    def deactivate_geofence(self) -> None:
        self.is_geofence_active = False
        self.get_logger().info("Geofence has been taken down!")
        self.logger.log_message(
            LogType.INFO,
            "Geofence has been taken down!",
        )

    def check_state(self) -> None:
        match self.state:
            case GeofenceState.INSIDE:
                # Check if robot is outside the geofence
                if not self.is_geofence_active:
                    return

                if self.is_outside_geofence(self.x, self.y):
                    self.get_logger().info("Robot is outside the geofence!")
                    self.logger.log_message(
                        LogType.INFO,
                        "Robot is outside the geofence!",
                    )
                    self.state = GeofenceState.RETURN_TO_GEOFENCE

            case GeofenceState.RETURN_TO_GEOFENCE:
                self.pause_exploration()
                self.return_to_geofence()
                self.state = GeofenceState.RETURNING

            case GeofenceState.RETURNING:
                # Check if robot is inside the geofence
                if not self.is_outside_geofence(self.x, self.y):
                    self.navigator.cancelTask()
                    self.get_logger().info("Robot is back inside the geofence!")
                    self.logger.log_message(
                        LogType.INFO,
                        "Robot is back inside the geofence!",
                    )
                    self.resume_exploration()
                    self.state = GeofenceState.INSIDE

    def is_outside_geofence(self, x: float, y: float) -> Bool:
        return (
            x < self.geofence_x_min
            or x > self.geofence_x_max
            or y < self.geofence_y_min
            or y > self.geofence_y_max
        )

    def pause_exploration(self) -> None:
        self.get_logger().info("Pausing exploration...")
        self.logger.log_message(
            LogType.INFO,
            "Pausing exploration...",
        )
        msg = Bool()
        msg.data = False
        self.exploration_publisher.publish(msg)

    def resume_exploration(self) -> None:
        sleep(TIME_TO_GATHER_TOUGHTS)
        self.get_logger().info("Resuming exploration...")
        self.logger.log_message(
            LogType.INFO,
            "Resuming exploration...",
        )
        msg = Bool()
        msg.data = True
        self.exploration_publisher.publish(msg)

    def return_to_geofence(self) -> None:
        sleep(TIME_TO_GATHER_TOUGHTS)
        pose = self.create_pose_stamped(*self.calculate_geofence_midpoint())

        self.navigator.goToPose(pose)
        self.get_logger().info(f"Returning inside the geofence")
        self.logger.log_message(
            LogType.INFO,
            "Returning inside the geofence",
        )

    def create_pose_stamped(self, target_x: float, target_y: float) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.stamp = Time().to_msg()
        pose_stamped.header.frame_id = GLOBAL_FRAME_ID

        pose_stamped.pose = Pose()
        pose_stamped.pose.position = Point(x=target_x, y=target_y, z=0.0)
        pose_stamped.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return pose_stamped

    def calculate_geofence_midpoint(self) -> tuple[float, float]:
        midpoint_x = (self.geofence_x_min + self.geofence_x_max) / 2
        midpoint_y = (self.geofence_y_min + self.geofence_y_max) / 2
        return midpoint_x, midpoint_y


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
