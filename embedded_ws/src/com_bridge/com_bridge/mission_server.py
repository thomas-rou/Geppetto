import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from common_msgs.msg import StartMission, StopMission
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from random import uniform
from com_bridge.common_methods import (
    clear_logs,
    get_robot_id,
    set_mission_status,
    get_mission_status,
)
from com_bridge.common_enums import GlobalConst, LogType, RobotStatus
from com_bridge.log import LoggerNode
import os


class MissionServer(Node):
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

    def send_goal(self):
        if not self.action_client.server_is_ready():
            self.get_logger().info("Action server not ready yet...")
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        x_range = [0.0, 50.0]
        y_range = [0.0, 50.0]
        goal_msg.pose.position.x = uniform(x_range[0], x_range[1])
        goal_msg.pose.position.y = uniform(y_range[0], y_range[1])
        
        

        goal = NavigateToPose.Goal()
        goal.pose = goal_msg

        self.get_logger().info(
            f"Sending goal: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}"
        )
        self.action_client.send_goal_async(goal)

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
            self.logger.log_message(LogType.INFO, f"Received new mission for robot {robot_id}")
            self.timer = self.create_timer(2.0, self.send_goal)
        except Exception as e:
            self.logger.log_message(LogType.INFO, f"Failed to start mission: {e}")

    def stop_mission_callback(self, msg: StopMission):
        try:
            if self.mission_active:
                self.destroy_timer(self.timer)
                self.stop_robot()
                self._mission_status = RobotStatus.WAITING
            else:
                self.logger.log_message(LogType.INFO, "No active mission to cancel.")
            self._mission_status = RobotStatus.WAITING
        except Exception as e:
            self.logger.log_message(LogType.INFO, f"Failed to cancel mission: {e}")

    def stop_robot(self):
        if self.mission_active:
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
    mission_server = MissionServer()
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
