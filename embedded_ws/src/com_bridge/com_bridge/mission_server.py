import rclpy
import subprocess
import math
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Header
from common_msgs.msg import StartMission, StopMission, ReturnBase
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from random import uniform
from com_bridge.common_enums import RobotName
from com_bridge.common_methods import (
    clear_logs,
    get_robot_id,
    set_mission_status,
    get_mission_status,
    get_robot_name,
)
from com_bridge.common_enums import GlobalConst, LogType, RobotStatus
from com_bridge.log import LoggerNode
import os
import time
from time import sleep
import subprocess
from rclpy.parameter import Parameter
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from action_msgs.srv import CancelGoal
from nav2_simple_commander.robot_navigator import BasicNavigator


CALLBACK_PERIOD = 2.0


class MissionServerGazebo(Node):
    def __init__(self):
        super().__init__("mission_server")
        self.declare_parameter("robot_id", "")
        self.robot_id = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )
        self.declare_parameter("y", 0.0)
        self.y = self.get_parameter("y").get_parameter_value().double_value
        self.logger = LoggerNode()
        self.logger.log_message(
            LogType.INFO,
            f"Server Launched waiting for messages in {os.getenv('ROBOT')}",
        )

        self.navigator = BasicNavigator(namespace=self.robot_id)

        self.returning_home = False
        self.initial_pos = None
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.start_mission_publisher = self.create_publisher(
            Bool, f"{self.robot_id}/explore/resume", GlobalConst.QUEUE_SIZE
        )

        # Subscription pour démarrer et arrêter les missions
        self.start_mission_subscription = self.create_subscription(
            StartMission,
            "start_mission_command",
            self.new_missions_callback,
            GlobalConst.QUEUE_SIZE,
        )

        self.stop_mission_subscription = self.create_subscription(
            StopMission,
            "stop_mission_command",
            self.stop_mission_callback,
            GlobalConst.QUEUE_SIZE,
        )

        self.return_base_subscription = self.create_subscription(
            ReturnBase,
            "return_to_base",
            self.return_to_base_callback,
            GlobalConst.QUEUE_SIZE,
        )

    @property
    def mission_active(self):
        if self._mission_status == RobotStatus.LOW_BATTERY:
            return False
        return self._mission_status == RobotStatus.MISSION_ON_GOING

    @property
    def _mission_status(self):
        return get_mission_status()

    @_mission_status.setter
    def _mission_status(self, status):
        set_mission_status(status)

    def new_missions_callback(self, msg: StartMission):

        try:
            if self.mission_active:
                self.logger.log_message(
                    LogType.INFO,
                    "A goal is already being executed. Rejecting new goal request.",
                )
                return
            clear_logs()

            self._mission_status = RobotStatus.MISSION_ON_GOING
            self.logger.log_message(
                LogType.INFO, f"Received new mission for {self.robot_id}"
            )
            msg = Bool()
            msg.data = True
            if get_robot_name() == "gazebo":
                self.start_mission_publisher.publish(msg)

        except Exception as e:
            self.logger.log_message(LogType.INFO, f"Failed to start mission: {e}")

    def navigate_to_home(self):
        try:
            msg = Bool()
            msg.data = False
            if get_robot_name() == "gazebo":
                self.start_mission_publisher.publish(msg)

            sleep(3)
            pose = self.create_pose_stamped(0.0, self.y)

            self.navigator.goToPose(pose)
            self.logger.log_message(
                LogType.INFO,
                f"Returning {self.robot_id} to base at (0, {self.y})",
            )

        except Exception as e:
            self.logger.log_message(LogType.ERROR, f"Failed to navigate to base: {e}")

    def return_to_base_callback(self, msg: ReturnBase):
        self.stop_robot()
        self.returning_home = True
        time.sleep(1)
        self.navigate_to_home()
        self._mission_status = RobotStatus.WAITING

    def stop_mission_callback(self, msg: StopMission):
        try:
            if self.mission_active:
                self.stop_robot()
                self._mission_status = RobotStatus.WAITING
                self.logger.log_message(LogType.INFO, "Current mission canceled.")
            else:
                self.logger.log_message(LogType.INFO, "No active mission to cancel.")
            self._mission_status = RobotStatus.WAITING
        except Exception as e:
            self.logger.log_message(LogType.INFO, f"Failed to cancel mission: {e}")

    def stop_robot(self):
        if self.mission_active:
            msg = Bool()
            msg.data = False
            if get_robot_name() == "gazebo":
                self.start_mission_publisher.publish(msg)

    def create_pose_stamped(self, target_x: float, target_y: float) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.stamp = Time().to_msg()
        pose_stamped.header.frame_id = "world"

        pose_stamped.pose = Pose()
        pose_stamped.pose.position = Point(x=target_x, y=target_y, z=0.0)
        pose_stamped.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return pose_stamped


def main(args=None):
    rclpy.init(args=args)
    mission_server = MissionServerGazebo()
    executor = MultiThreadedExecutor()
    executor.add_node(mission_server)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        mission_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
