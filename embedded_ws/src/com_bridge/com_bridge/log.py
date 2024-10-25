from com_bridge.common_methods import get_robot_id, get_robot_name
from com_bridge.common_enums import GlobalConst
import rclpy
from rclpy.node import Node
from common_msgs.msg import LogMessage
from datetime import datetime

class Logger(Node):

    _instance = None  # Variable de classe pour stocker l'instance unique

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(Logger, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        super().__init__("log_node")
        self.robot_id = get_robot_id()
        self.robot_name = get_robot_name()
        self.get_logger().info(
            f"Log node Launched waiting for messages in {self.robot_name}"
        )
        self.log_publisher = self.create_publisher(
            LogMessage, f"{self.robot_name}/log", GlobalConst.LOG_QUEUE_SIZE
        )

    def build_log_message(self, log_type, message) -> LogMessage:
        log_message = LogMessage()
        log_message.source = get_robot_name() if get_robot_name() else "Unknown"
        log_message.log_type = log_type
        log_message.date = datetime.now().strftime("%Y-%m-%d %Hh %Mmin %Ss")
        log_message.message = message
        return log_message

    def log_message(self, log_type, message):
        log_message = self.build_log_message(log_type, message)
        with open("/tmp/.log", "a") as f:
            f.write(log_message.source + "\t" + log_message.log_type + "\t" + log_message.date + "\t" + log_message.message + "\n")
        self.log_publisher.publish(log_message)


def main(args=None):
    rclpy.init(args=args)
    node = Logger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
