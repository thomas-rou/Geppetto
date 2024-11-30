from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Header
from com_bridge.common_enums import GlobalConst
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
from time import sleep

TIME_TO_GATHER_TOUGHTS = 3
GLOBAL_FRAME_ID = "world"


class GeoFenceState(Enum):
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

        self.navigator = BasicNavigator(namespace=self.robot_id)
        self.get_logger().info("Geofence is up and running!")

        self.geofence_x_min = -1.5
        self.geofence_x_max = 1.5
        self.geofence_y_min = -1.5
        self.geofence_y_max = 1.5

        self.get_logger().info(
            f"Geofence box is ({self.geofence_x_min}, {self.geofence_y_min}) ({self.geofence_x_max}, {self.geofence_y_max})"
        )

        self.state = GeoFenceState.INSIDE

        self.exploration_publisher = self.create_publisher(
            Bool, f"/{self.robot_id}/explore/resume", GlobalConst.QUEUE_SIZE
        )

        self.pose_subscription = self.create_subscription(
            Pose,
            f"/{self.robot_id}/pose",
            self.pose_callback,
            GlobalConst.QUEUE_SIZE,
        )

    def pose_callback(self, msg: Pose) -> None:
        self.x = msg.position.x
        self.y = msg.position.y
        self.check_state()

    def check_state(self) -> None:
        match self.state:
            case GeoFenceState.INSIDE:
                # Check if robot is outside the geofence
                if self.is_outside_geofence(self.x, self.y):
                    self.get_logger().info("Robot is outside the geofence!")
                    self.state = GeoFenceState.RETURN_TO_GEOFENCE

            case GeoFenceState.RETURN_TO_GEOFENCE:
                self.pause_exploration()
                self.return_to_geofence()
                self.state = GeoFenceState.RETURNING

            case GeoFenceState.RETURNING:
                # Check if robot is inside the geofence
                if not self.is_outside_geofence(self.x, self.y):
                    self.navigator.cancelTask()
                    self.get_logger().info("Robot is back inside the geofence!")
                    self.resume_exploration()
                    self.state = GeoFenceState.INSIDE

    def is_outside_geofence(self, x: float, y: float) -> Bool:
        return (
            x < self.geofence_x_min
            or x > self.geofence_x_max
            or y < self.geofence_y_min
            or y > self.geofence_y_max
        )

    def pause_exploration(self) -> None:
        self.get_logger().info("Pausing exploration...")
        msg = Bool()
        msg.data = False
        self.exploration_publisher.publish(msg)

    def resume_exploration(self) -> None:
        sleep(TIME_TO_GATHER_TOUGHTS)
        self.get_logger().info("Resuming exploration...")
        msg = Bool()
        msg.data = True
        self.exploration_publisher.publish(msg)

    def return_to_geofence(self) -> None:
        sleep(TIME_TO_GATHER_TOUGHTS)
        pose = self.create_pose_stamped(*self.calculate_geofence_midpoint())

        self.navigator.goToPose(pose)
        self.get_logger().info(f"Returning to the geofence")

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
