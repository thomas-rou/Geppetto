import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from com_bridge.common_enums import GlobalConst
from com_bridge.log import LoggerNode

class CurrentPositionNode(Node):

    def __init__(self):
        super().__init__('current_position_node')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            GlobalConst.QUEUE_SIZE
        )

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
            self.get_logger().info(
                "receiving current position info"
            )

def main(args=None):
    rclpy.init(args=args)
    node = CurrentPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()