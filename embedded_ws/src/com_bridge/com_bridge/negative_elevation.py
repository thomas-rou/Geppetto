import os
import rclpy
from rclpy.node import Node
from com_bridge.log import LoggerNode
from com_bridge.common_enums import GlobalConst, LogType, RobotStatus
from sensor_msgs.msg import PointCloud2
import numpy as np

class NegativeElevation(Node):
    def __init__(self):
        super().__init__('negative_elevation')
        self.logger = LoggerNode()
        self.logger.log_message(LogType.INFO, f"Negative elevation node launched on {os.getenv('ROBOT')}")

        self.point_cloud_subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.status_elevation_callback,
            GlobalConst.QUEUE_SIZE
        )
        
        self.fall_threshold = -0.5 
        
    def status_elevation_callback(self, point_cloud):
        try:
            pc_data = self.extract_points_from_cloud(point_cloud)

            negative_elevation = np.min(pc_data)
            self.logger.log_message(LogType.INFO, f"Negative elevation: {negative_elevation}")

            if negative_elevation < self.fall_threshold:
                pass
                # self.logger.log_message(LogType.INFO, "Robot elevation too low! Stopping robot.")
                # self.stop_robot()

        except Exception as e:
            self.logger.log_message(LogType.INFO, "Failed to calculate negative elevation: "+str(e))

    def extract_points_from_cloud(self, point_cloud):
        pc_data = []
        for point in pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True):
            _, _, z = point 
            pc_data.append(z)

        return np.array(pc_data)

    def stop_robot(self):
        self.logger.log_message(LogType.INFO, "Published stop command to robot.")

        
def main(args=None):
    rclpy.init(args=args)
    node = NegativeElevation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
