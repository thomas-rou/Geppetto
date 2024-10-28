import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from common_msgs.msg import StartMission, StopMission
from rclpy.executors import MultiThreadedExecutor
from random import uniform, choice
from typing import Tuple
from com_bridge.common_methods import clear_logs, get_robot_id, set_mission_status, get_mission_status
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
        self.logger.log_message(LogType.INFO, f"Server Launched waiting for messages in {os.getenv('ROBOT')}")

        # Subscription pour démarrer et arrêter les missions
        self.start_mission_subscription = self.create_subscription(
            StartMission, "start_mission_command", self.new_missions_callback, GlobalConst.QUEUE_SIZE
        )

        self.stop_mission_subscription = self.create_subscription(
            StopMission, "stop_mission_command", self.stop_mission_callback, GlobalConst.QUEUE_SIZE
        )

        # TODO: feedback msg publisher

        self.mission_mouvements = self.create_publisher(Twist, "cmd_vel", GlobalConst.QUEUE_SIZE)
        self._timer = None
        self._feedback_timer = None

        self._current_x = 0.0
        self._current_y = 0.0
        self._current_orientation = 0.0

        # Variables pour le mouvement aléatoire
        self._direction_persistence = 0
        self._current_direction = (0, 0, 0)

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
                self.logger.log_message(LogType.INFO, 
                    "A goal is already being executed. Rejecting new goal request."
                )
                return
            clear_logs()
            self._mission_status = RobotStatus.MISSION_ON_GOING
            robot_id = self.robot_id[-1] if self.robot_id else get_robot_id()
            self.logger.log_message(LogType.INFO, f"Received new mission for robot {robot_id}")
            position = getattr(msg.mission_details, f'position{robot_id}')
            orientation = getattr(msg.mission_details, f'orientation{robot_id}')
            self._current_x = position.x
            self._current_y = position.y
            self._current_orientation = orientation

            self.logger.log_message(LogType.INFO, 
                f"Accepting new mission. Starting at position: ({self._current_x}, {self._current_y}) with orientation: {self._current_orientation}"
            )

            self._timer = self.create_timer(
                0.5, self.random_move
            )  # Exécuter le mouvement toutes les 0.5 secondes
            self._feedback_timer = self.create_timer(
                1.0, self.publish_feedback
            )  # Publier le feedback à 1 Hz
        except Exception as e:
            self.logger.log_message(LogType.INFO, f"Failed to start mission: {e}")

    def stop_mission_callback(self, msg: StopMission):
        try:
            if self.mission_active:
                self.destroy_timer(self._timer)
                self.destroy_timer(self._feedback_timer)  # Arrêter le timer de feedback
                self.logger.log_message(LogType.INFO, "Cancelling the current mission.")
            else:
                self.logger.log_message(LogType.INFO, "No active mission to cancel.")
            self._mission_status = RobotStatus.WAITING
        except Exception as e:
            self.logger.log_message(LogType.INFO, f"Failed to cancel mission: {e}")

    def random_move(self):
        if self.mission_active:
            if self._direction_persistence <= 0:
                linear_speed_x = uniform(0.1, 0.5)
                linear_speed_y = uniform(-0.5, 0.5)
                angular_speed = uniform(-1.0, 1.0)
                self._current_direction = (
                    linear_speed_x,
                    linear_speed_y,
                    angular_speed,
                )
                self._direction_persistence = choice(
                    range(5, 15)
                )  # Garder cette direction entre 5 à 15 cycles
                self.publish_cmdvel(
                    (0.0, 0.0, 0.0), (0.0, 0.0, self._current_direction[2])
                )

            self._direction_persistence -= 1
            self.publish_cmdvel(
                (self._current_direction[0], self._current_direction[1], 0.0),
                (0.0, 0.0, self._current_direction[2]),
            )

            # Mettre à jour la position actuelle pour simuler le feedback
            self._current_x += self._current_direction[0] * 0.5
            self._current_y += self._current_direction[1] * 0.5

    def publish_feedback(self):
        if self.mission_active:
            # TODO
            feedback_msg = None

            # Publier les données de position et l'état de la mission
            self.logger.log_message(LogType.INFO, 
                f"Feedback: Position ({self._current_x:.2f}, {self._current_y:.2f}), Orientation: {self._current_orientation}, Status: {self._mission_status}"
            )

    def publish_cmdvel(
        self, linear: Tuple[float, float, float], angular: Tuple[float, float, float]
    ):
        twist_msg = Twist()
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = linear
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = angular
        self.mission_mouvements.publish(twist_msg)
        self.logger.log_message(LogType.INFO, 
            f"Publishing cmd_vel: linear={linear}, angular={angular}"
        )


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
