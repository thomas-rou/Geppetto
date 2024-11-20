from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from com_bridge.common_enums import GlobalConst
from com_bridge.log import LoggerNode


class CurrentPositionNode(Node):

    def __init__(self):
        super().__init__('current_position_node')
        self.get_logger().info(
            "DID I START *********************************************************************************************"
        )        
        self.logger = LoggerNode()
        self.logger.log_message(LogType.INFO, "DID I START *********************************************************************************************")
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        self.firstPosSent = False
        self.initial_position = None
        self.first_pos_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            GlobalConst.QUEUE_SIZE
        )

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        if not self.firstPosSent :
            self.initial_position = msg
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            self.get_logger().info(
                f"Robot very initial position: x={position.x}, y={position.y}, z={position.z}, "
                f"And very initial orientation: w={orientation.w}, x={orientation.x}, y={orientation.y}, z={orientation.z}"
            )
            self.firstPosSent = True
        self.first_pos_publisher.publish(self.initial_position)

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = CurrentPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()