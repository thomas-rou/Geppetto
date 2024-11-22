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
        self.get_logger().info(
            "DID I DO ANYTHING 1*********************************************************************************************"
        )  
        self.publish_initial_pose()


    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.get_logger().info(
            "DID I DO ANYTHING 4*********************************************************************************************"
        )
        if not self.firstPosSent :
            self.get_logger().info(
                "DID I DO ANYTHING 5*********************************************************************************************"
            )
            self.firstPosSent = True
            self.first_pos_publisher.publish(msg)

    def publish_initial_pose(self):
        self.get_logger().info(
            "DID I DO ANYTHING 2*********************************************************************************************"
        )
        initial_pose = PoseWithCovarianceStamped()
        
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'  
        
        initial_pose.pose.pose.position.x = 0.0  
        initial_pose.pose.pose.position.y = 0.0  
        initial_pose.pose.pose.position.z = 0.0 

        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.707 
        initial_pose.pose.pose.orientation.w = 0.707

        
        initial_pose.pose.covariance = [float(x) for x in [
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.1, 0, 0,
            0, 0, 0, 0, 0.1, 0,
            0, 0, 0, 0, 0, 0.1
        ]]

        self.get_logger().info(f"Covariance: {initial_pose.pose.covariance}")
        self.first_pos_publisher.publish(initial_pose)
        self.get_logger().info('INITALPOSE PUBLIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIISHED')

def main(args=None):
    rclpy.init(args=args)
    node = CurrentPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()