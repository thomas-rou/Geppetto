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


class GeoFenceState(Enum):
    INSIDE = 0
    RETURN_TO_GEOFENCE = 1
    RETURNING = 2


class GeofenceNode(Node):
    def __init__(self):
        super().__init__("geofence_node")
        self.navigator = BasicNavigator(namespace="limo1")
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
            Bool, "limo1/explore/resume", GlobalConst.QUEUE_SIZE
        )

        self.pose_subscription = self.create_subscription(
            Pose,
            "/limo1/pose",
            self.pose_callback,
            GlobalConst.QUEUE_SIZE,
        )

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            "/limo1/goal_pose",
            self.goal_callback,
            GlobalConst.QUEUE_SIZE,
        )

    def goal_callback(self, msg: PoseStamped):
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        if self.is_outside_geofence(goal_x, goal_y):
            self.get_logger().warn("Goal is outside the geofence. Intercepting...")

            safe_goal_x, safe_goal_y = self.get_nearest_point_inside_geofence(
                goal_x, goal_y
            )

            safe_goal = self.create_pose_stamped(safe_goal_x, safe_goal_y)

            self.navigator.goToPose(safe_goal)
            self.get_logger().info(
                f"Redirecting robot to a safe goal at ({safe_goal_x}, {safe_goal_y}) inside the geofence."
            )

    def pose_callback(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.check_state()

    def check_state(self):
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

    def is_outside_geofence(self, x: float, y: float):
        return (
            x < self.geofence_x_min
            or x > self.geofence_x_max
            or y < self.geofence_y_min
            or y > self.geofence_y_max
        )

    def pause_exploration(self):
        self.get_logger().info("Pausing exploration...")
        msg = Bool()
        msg.data = False
        self.exploration_publisher.publish(msg)

    def resume_exploration(self):
        sleep(TIME_TO_GATHER_TOUGHTS)
        self.get_logger().info("Resuming exploration...")
        msg = Bool()
        msg.data = True
        self.exploration_publisher.publish(msg)

    def return_to_geofence(self):
        sleep(TIME_TO_GATHER_TOUGHTS)
        pose = self.create_pose_stamped(0.0, 0.0)

        self.navigator.goToPose(pose)
        self.get_logger().info(f"Returning to the geofence")

    def create_pose_stamped(self, target_x: float, target_y: float):
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.stamp = Time().to_msg()
        pose_stamped.header.frame_id = "world"

        pose_stamped.pose = Pose()
        pose_stamped.pose.position = Point(x=target_x, y=target_y, z=0.0)
        pose_stamped.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return pose_stamped

    def get_nearest_point_inside_geofence(self, goal_x: float, goal_y: float):
        clamped_x = max(self.geofence_x_min, min(goal_x, self.geofence_x_max))
        clamped_y = max(self.geofence_y_min, min(goal_y, self.geofence_y_max))

        return clamped_x, clamped_y


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
