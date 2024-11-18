import rclpy
import os
import time
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from rclpy.parameter import Parameter
from common_msgs.msg import ReturnBase
from com_bridge.common_enums import GlobalConst

class ReturnBase(Node):
    def __init__(self):
        super().__init__('return_base')

        self.returning_to_base = False
        self.return_to_base_publisher = self.create_publisher(
            Bool, 'return_to_base', GlobalConst.QUEUE_SIZE
        )
        self.create_subscription(Bool, f"{os.getenv('ROBOT')}/low_battery", self.low_battery_callback, GlobalConst.QUEUE_SIZE)
        
def low_battery_callback(self, msg: Bool):
    if msg.data and not self.returning_to_base:
        self.returning_to_base = True
        self.get_logger().warn('Battery below threshold, returning to base...')
        self.returnToBase()

def returnToBase(self):
    try:
        if not self.return_to_base_publisher:
            raise RuntimeError("Publisher not initialized.")
        msg = Bool()
        msg.data = True
        self.return_to_base_publisher.publish(msg)
        self.get_logger().info('Return to base signal published.')
    except RuntimeError as e:
        self.get_logger().error(f'Publisher issue: {e}')
    except Exception as e:
        self.get_logger().error(f'Failed to initiate return to base: {e}')


def main(args=None):
    rclpy.init(args=args)
    return_base_node = ReturnBase()
    rclpy.spin(return_base_node)  
    return_base_node.destroy_node()  
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
