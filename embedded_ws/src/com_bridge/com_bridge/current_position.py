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
        self.firstPosSent = False
        self.first_pos_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            GlobalConst.QUEUE_SIZE
        )

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        if not self.firstPosSent :
            self.get_logger().info(
                "DID I DO ANYTHING *********************************************************************************************"
            )
            self.firstPosSent = True
        self.first_pos_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CurrentPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()