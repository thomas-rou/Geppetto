import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from common_msgs.action import Mission
from rclpy.executors import MultiThreadedExecutor

class MissionServer(Node):
    def __init__(self):
        super().__init__('mission_server')
        self._action_server = ActionServer(
            self,
            Mission,
            'mission',
            execute_callback=self.start_mission_callback,
            goal_callback=self.new_missions_callback,
            cancel_callback=self.stop_mission_callback)

        self._current_goal_handle = None
        self._timer = self.create_timer(1.0, self.publish_feedback)  # Timer pour le feedback

    def new_missions_callback(self, goal_request):
        if self._current_goal_handle and self._current_goal_handle.is_active:
            self.get_logger().info('A goal is already being executed. Rejecting new goal request.')
            return GoalResponse.REJECT
        else:
            self.get_logger().info('Accepting new goal request.')
            return GoalResponse.ACCEPT

    def stop_mission_callback(self, goal_handle):
        if goal_handle == self._current_goal_handle:
            self.__class__.publish_cmdvel((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self.get_logger().info('Cancelling the current goal.')
            return CancelResponse.ACCEPT
        else:
            self.get_logger().info('No active goal to cancel or goal does not match the current goal.')
            return CancelResponse.REJECT

    def start_mission_callback(self, goal_handle):
        self.get_logger().info(f"Starting mission at position: ({goal_handle.request.initial_x}, {goal_handle.request.initial_y})")

        # Initialisation des variables de mission
        self._current_goal_handle = goal_handle
        self._current_x = goal_handle.request.initial_x
        self._current_y = goal_handle.request.initial_y
        self._mission_status = "In Progress"

        # Attendre que la mission soit terminée (exécution simulée)
        while not goal_handle.is_cancel_requested:
            rclpy.spin_once(self, timeout_sec=1)  # Attendre 1 seconde

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self._mission_status = "Canceled"
            self.get_logger().info('Mission canceled')
            return Mission.Result(status=self._mission_status)

        goal_handle.succeed()
        self._mission_status = "Completed"
        self.get_logger().info('Mission completed successfully')

        result = Mission.Result()
        result.status = self._mission_status
        return result

    def publish_feedback(self):
        # Publier le feedback si un objectif est en cours d'exécution
        if self._current_goal_handle and self._current_goal_handle.is_active:

            feedback_msg = Mission.Feedback()
            feedback_msg.current_x = self._current_x
            feedback_msg.current_y = self._current_y

            self._current_goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Feedback: Position ({self._current_x}, {self._current_y}), Status: {self._mission_status}")
    
    @staticmethod
    def publish_cmdvel(linear: Tuple[float, float, float], angular: Tuple[float, float, float]):
        twist_msg = Twist()
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = linear
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = angular
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

if __name__ == '__main__':
    main()
