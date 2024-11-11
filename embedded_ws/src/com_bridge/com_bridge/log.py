from com_bridge.common_methods import get_robot_id, get_robot_name
from com_bridge.common_enums import GlobalConst, LogType
import rclpy
from rclpy.node import Node
from common_msgs.msg import LogMessage
from nav_msgs.msg import Odometry
from datetime import datetime

TIMER_PERIOD = 1.0

class LoggerNode(Node):

    _instance = None  # Variable de classe pour stocker l'instance unique

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(LoggerNode, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        super().__init__("log_node")
        self.robot_id = get_robot_id()
        self.robot_name = get_robot_name()
        self.log_publisher = self.create_publisher(
            LogMessage, f"/{self.robot_name}/log", GlobalConst.LOG_QUEUE_SIZE
        )
        self.robot_odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.log_odometry_callback,
            GlobalConst.QUEUE_SIZE
        )
        self.robot_odom_subscription  # prevent unused variable warning
        self.last_odometry_msg, self.last_laser_msg = None, None
        self.prev_odometry_msg, self.prev_laser_msg = None, None
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.log_message(LogType.INFO,
            f"Log node Launched waiting for messages in {self.robot_name}"
        )
        self._initialized = True

    def build_log_message(self, log_type, message) -> LogMessage:
        log_message = LogMessage()
        log_message.source = get_robot_name() if get_robot_name() else "Unknown"
        log_message.log_type = log_type
        log_message.date = datetime.now().strftime("%Y-%m-%d %Hh %Mmin %Ss")
        log_message.message = message
        return log_message

    def native_log(self, log_type, message):
        match log_type:
            case LogType.INFO:
                self.get_logger().info(message)
            case LogType.WARNING:
                self.get_logger().warn(message)
            case LogType.ERROR:
                self.get_logger().error(message)
            case LogType.DEBUG:
                self.get_logger().debug(message)

    def log_message(self, log_type, message):
        log_message = self.build_log_message(log_type, message)
        with open("/tmp/mission.log", "a") as f:
            f.write(log_message.source + "\t" + log_message.log_type + "\t" + log_message.date + "\t" + log_message.message + "\n")
        self.log_publisher.publish(log_message)
        self.native_log(log_type, message)

    def log_odometry_callback(self, msg: Odometry):
        self.last_odometry_msg = msg

    def timer_callback(self):
        if self.should_log_odometry():
            self.log_odometry()

    def should_log_odometry(self):
        return self.last_odometry_msg is not None and (
            self.prev_odometry_msg is None or self.has_odometry_changed(self.prev_odometry_msg, self.last_odometry_msg)
    )

    def log_odometry(self):
        position = self.last_odometry_msg.pose.pose.position
        orientation = self.last_odometry_msg.pose.pose.orientation
        linear_velocity = self.last_odometry_msg.twist.twist.linear
        angular_velocity = self.last_odometry_msg.twist.twist.angular
        log_message = (f"Position: x={position.x}, y={position.y}, z={position.z}; "
                    f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}; "
                    f"Linear Velocity: x={linear_velocity.x}, y={linear_velocity.y}, z={linear_velocity.z}; "
                    f"Angular Velocity: x={angular_velocity.x}, y={angular_velocity.y}, z={angular_velocity.z}")
        self.log_message(LogType.INFO, log_message)
        self.prev_odometry_msg = self.last_odometry_msg

    def has_odometry_changed(self, prev_msg: Odometry, curr_msg: Odometry) -> bool:
        return prev_msg.pose.pose.position != curr_msg.pose.pose.position or prev_msg.pose.pose.orientation != curr_msg.pose.pose.orientation or prev_msg.twist.twist != curr_msg.twist.twist


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
