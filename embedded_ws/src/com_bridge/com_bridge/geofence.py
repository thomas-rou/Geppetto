from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
from time import sleep
import math


class GeoFenceState(Enum):
    INSIDE = 0
    RETURN_TO_GEOFENCE = 1
    RETURNING = 2


class GeofenceNode(Node):
    def __init__(self):
        super().__init__("geofence_node")
        self.navigator = BasicNavigator(namespace="limo1")
        self.get_logger().info("Geofence is up and running!")

        # Define your geofence (Example: 5x5 m area centered at (0, 0))
        self.geofence_x_min = -2.5
        self.geofence_x_max = 2.5
        self.geofence_y_min = -2.5
        self.geofence_y_max = 2.5

        self.state = GeoFenceState.INSIDE

        # Subscribe to the robot's pose
        self.pose_subscription = self.create_subscription(
            Pose,
            "/limo1/pose",  # Adjust topic to match your robot
            self.pose_callback,
            10,
        )

        # # Exploration pause service (modify with explore_lite)
        # self.create_service(PauseExploration, 'pause_exploration', self.pause_exploration_callback)

    def pose_callback(self, msg):
        # Get robot's current position
        self.x = msg.position.x
        self.y = msg.position.y

        self.check_state()

    def check_state(self):
        match self.state:
            case GeoFenceState.INSIDE:
                # Check if robot is outside the geofence
                if self.is_outside_geofence():
                    self.get_logger().info("Robot is outside the geofence!")
                    self.state = GeoFenceState.RETURN_TO_GEOFENCE

            case GeoFenceState.RETURN_TO_GEOFENCE:
                self.pause_exploration()
                self.return_to_geofence()
                self.state = GeoFenceState.RETURNING

            case GeoFenceState.RETURNING:
                # Check if robot is inside the geofence
                if not self.is_outside_geofence():
                    self.get_logger().info("Robot is back inside the geofence!")
                    self.resume_exploration()
                    self.state = GeoFenceState.INSIDE

    def is_outside_geofence(self):
        return (
            self.x < self.geofence_x_min
            or self.x > self.geofence_x_max
            or self.y < self.geofence_y_min
            or self.y > self.geofence_y_max
        )

    def pause_exploration(self):
        # Here, call a service or action to pause exploration (this depends on your implementation)
        self.get_logger().info("Pausing exploration...")

    def resume_exploration(self):
        # Here, call a service or action to pause exploration (this depends on your implementation)
        self.get_logger().info("Resuming exploration...")

    def return_to_geofence(self):
        # Command the robot to move back inside the geofence, e.g., to a predefined point
        pose = self.create_pose_stamped(0, 0)

        self.navigator.goToPose(pose)
        self.get_logger().info(f"Returning to the geofence at INITIAL POSE")

    def create_pose_stamped(self, target_x, target_y):
        # Create PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.stamp = Time().to_msg()  # Get the current time
        pose_stamped.header.frame_id = (
            "world"  # Or any other frame you are using (e.g., 'odom')
        )

        # Create the pose
        pose_stamped.pose = Pose()
        pose_stamped.pose.position = Point(
            x=target_x, y=target_y, z=0.0
        )  # Set position
        pose_stamped.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0
        )  # Set orientation (no rotation)
        return pose_stamped


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
