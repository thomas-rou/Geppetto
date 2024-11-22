import rclpy
import subprocess
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from common_msgs.msg import StartMission, StopMission
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from com_bridge.common_methods import (
    clear_logs,
    get_robot_id,
    set_mission_status,
    get_mission_status,
    get_robot_name
)
from com_bridge.common_enums import GlobalConst, LogType, RobotStatus
from com_bridge.log import LoggerNode
import os

CALLBACK_PERIOD = 2.0


class MissionServerGazebo(Node):
    def __init__(self):
        super().__init__("mission_server")
        self.declare_parameter("robot_id", "")
        self.robot_id = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )
        self.logger = LoggerNode()
        self.logger.log_message(
            LogType.INFO,
            f"Server Launched waiting for messages in {os.getenv('ROBOT')}",
        )

        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.start_mission_publisher = self.create_publisher(Bool, 'explore/resume', GlobalConst.QUEUE_SIZE)

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
        self.mission_mouvements = self.create_publisher(
            Twist, "cmd_vel", GlobalConst.QUEUE_SIZE
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
            robot_id = self.robot_id[-1] if self.robot_id else get_robot_id()
            self.logger.log_message(
                LogType.INFO, f"Received new mission for robot {robot_id}"
            )
            msg = Bool()
            msg.data = True
            if get_robot_name() == "gazebo":
                command = ["ros2", "launch", "ros_gz_example_bringup", "explore.launch.py"]
                subprocess.Popen(command)
                command = ["ros2", "launch", "ros_gz_example_bringup", "explore.launch2.py"]
                subprocess.Popen(command)
                
            else:
                self.start_mission_publisher.publish(msg)
                command = ["ros2", "launch", "explore_lite", "explore.launch.py"]
                subprocess.Popen(command)
            
        except Exception as e:
            self.logger.log_message(LogType.INFO, f"Failed to start mission: {e}")

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
                self.start_mission_publisher_limo1.publish(msg)
                self.start_mission_publisher_limo2.publish(msg)
            else:
                self.start_mission_publisher.publish(msg)
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0
            self.mission_mouvements.publish(twist_msg)


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
