import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from rclpy.parameter import Parameter
from common_msgs.msg import ReturnBase
from com_bridge.common_enums import GlobalConst
from com_bridge.mission_server import MissionServerGazebo  # Import MissionServerGazebo

class ReturnBase(Node):
    def __init__(self):
        super().__init__('return_base')
        self.mission_server = MissionServerGazebo() 
        #Demnder Loic que fait cette ligne bizz
        self.create_subscription(Bool, f"{os.getenv('ROBOT')}/low_battery", self.low_battery_callback, 10)

        self.battery_subscription = self.create_subscription(
            ReturnBase,
            'return_to_base',
            self.returnToBase,
            GlobalConst.QUEUE_SIZE
        )


    def low_battery_callback(self, msg: Bool):
        if msg.data:  
            self.get_logger().warn('Battery below threshold, returning to base...')
            self.returnToBase()

    def returnToBase(self):
        try:
            self.mission_server.set_return_base(True)
            time.sleep(0.5)
            msg = Bool()
            self.mission_server.stop_mission_callback(msg)

            self.get_logger().info('Return to base initialized, returning to base.')

        except Exception as e:
            self.get_logger().error(f'Failed to initiate return to base: {e}')

def main(args=None):
    rclpy.init(args=args)
    return_base_node = ReturnBase()
    rclpy.spin(return_base_node)  # Spin to keep the node running and check battery status
    return_base_node.destroy_node()  # Clean up when shutting down
    rclpy.shutdown()  # Shutdown ROS2 system

if __name__ == '__main__':
    main()
