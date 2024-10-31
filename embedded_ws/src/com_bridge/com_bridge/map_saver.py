import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        map_data = np.array(msg.data).reshape((height, width))

        image = np.zeros((height, width), dtype=np.uint8)
        image[map_data == -1] = 127  # Gray for unknown
        image[map_data == 0] = 255   # White for free space
        image[map_data == 100] = 0   # Black for occupied space

        # Save the image as PNG
        cv2.imwrite('/tmp/map.png', image)
        self.get_logger().info('Map saved as PNG')

def main(args=None):
    rclpy.init(args=args)
    map_saver = MapSaver()
    rclpy.spin(map_saver)
    map_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()