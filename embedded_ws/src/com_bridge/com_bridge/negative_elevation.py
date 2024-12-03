import os
import rclpy
from rclpy.node import Node
from com_bridge.log import LoggerNode
from com_bridge.common_enums import GlobalConst, LogType, RobotStatus
from sensor_msgs.msg import PointCloud2


class NegativeElevation(Node):
    def __init__(self):
        super().__init__('negative_elevation')
        self.logger = LoggerNode()
        self.logger.log_message(LogType.INFO, f"negative elevation node launched on {os.getenv('ROBOT')}")

        self.point_cloud_subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.status_elevation_callback,
            GlobalConst.QUEUE_SIZE
        )
        
    def status_elevation_callback(self, point_cloud):
        try:
            # calculate the negative elevation
            negative_elevation = 0
            
        except Exception as e:
            self.logger.log_message(LogType.INFO, "Failed to calculate negative elevation: "+str(e))
        
        
def main(args=None):
    rclpy.init(args=args)
    node = NegativeElevation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()