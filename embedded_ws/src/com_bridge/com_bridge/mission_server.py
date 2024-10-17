import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from common_msgs.msg import StartMission, StopMission, Feedback
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor
from random import uniform, choice
from typing import Tuple

class MissionServer(Node):
    def __init__(self):
        super().__init__('mission_server')

        # Subscription pour démarrer et arrêter les missions
        self.start_mission_subscription = self.create_subscription(
            StartMission,
            'start_mission_command',
            self.new_missions_callback,
            10
        )

        self.stop_mission_subscription = self.create_subscription(
            StopMission,
            'stop_mission_command',
            self.stop_mission_callback,
            10
        )

        # TODO: feedback msg publisher

        self.mission_mouvements = self.create_publisher(Twist, 'cmd_vel', 10)
        self.mission_active = False
        self._timer = None
        self._feedback_timer = None 

        self._current_x = 0.0
        self._current_y = 0.0
        self._current_orientation = 0.0
        self._mission_status = "Not Started"

        # Variables pour le mouvement aléatoire
        self._direction_persistence = 0    
        self._current_direction = (0, 0, 0)

    def new_missions_callback(self, msg: StartMission):
        if self.mission_active:
            self.get_logger().info('A goal is already being executed. Rejecting new goal request.')
            return
        
        self._current_x = msg.mission_details.position.x
        self._current_y = msg.mission_details.position.y
        self._current_orientation = msg.mission_details.orientation

        self.get_logger().info(f"Accepting new mission. Starting at position: ({self._current_x}, {self._current_y}) with orientation: {self._current_orientation}")
        
        self.mission_active = True
        self._timer = self.create_timer(0.5, self.random_move)  # Exécuter le mouvement toutes les 0.5 secondes
        self._feedback_timer = self.create_timer(1.0, self.publish_feedback)  # Publier le feedback à 1 Hz

    def stop_mission_callback(self, msg: StopMission):
        if self.mission_active:
            self.mission_active = False
            self.destroy_timer(self._timer)
            self.destroy_timer(self._feedback_timer)  # Arrêter le timer de feedback
            self.get_logger().info('Cancelling the current mission.')
        else:
            self.get_logger().info('No active mission to cancel.')

    def random_move(self):
        if self.mission_active:
            if self._direction_persistence <= 0:
                linear_speed_x = uniform(0.1, 0.5) 
                linear_speed_y = uniform(-0.5, 0.5)
                angular_speed = uniform(-1.0, 1.0)  
                self._current_direction = (linear_speed_x, linear_speed_y, angular_speed)
                self._direction_persistence = choice(range(5, 15))  # Garder cette direction entre 5 à 15 cycles
                self.publish_cmdvel((0.0, 0.0, 0.0), (0.0, 0.0, self._current_direction[2]))

            self._direction_persistence -= 1
            self.publish_cmdvel((self._current_direction[0], self._current_direction[1], 0.0), (0.0, 0.0, self._current_direction[2]))

            # Mettre à jour la position actuelle pour simuler le feedback
            self._current_x += self._current_direction[0] * 0.5
            self._current_y += self._current_direction[1] * 0.5

    def publish_feedback(self):
        if self.mission_active:
            # TODO
            feedback_msg = None

            # Publier les données de position et l'état de la mission
            self.get_logger().info(f"Feedback: Position ({self._current_x:.2f}, {self._current_y:.2f}), Orientation: {self._current_orientation}, Status: {self._mission_status}")

    def publish_cmdvel(self, linear: Tuple[float, float, float], angular: Tuple[float, float, float]):
        twist_msg = Twist()
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = linear
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = angular
        self.mission_mouvements.publish(twist_msg)
        self.get_logger().info(f"Publishing cmd_vel: linear={linear}, angular={angular}")

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

if __name__ == '__main__':
    main()
