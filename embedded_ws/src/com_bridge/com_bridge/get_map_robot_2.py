from com_bridge.common_enums import GlobalConst, LogType
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from com_bridge.log import LoggerNode
import os, json, websocket
ROBOT_2_IP = "lm1170"
RECONNEXION_TIME_INTERVAL = 5.0

class GetMapRobot2(Node):
    def __init__(self):
        super().__init__('get_map_robot_2')
        self.logger = LoggerNode()
        self.map2_publisher = self.create_publisher(OccupancyGrid, f"{os.getenv('ROBOT')}/map", GlobalConst.QUEUE_SIZE)
        self.logger.log_message(LogType.INFO, f"GetMapRobot2 Launched waiting for messages in {os.getenv('ROBOT')}")
        self.timer = self.create_timer(RECONNEXION_TIME_INTERVAL, self.subscribe_to_robot_2_topics)
        self.ws = websocket.connect(f'self.ws://{ROBOT_2_IP}')
    
    async def subscribe_to_robot_2_topics(self):
        topic_name = '/map'
        topic_type = OccupancyGrid
        try:
            if self.ws is None or self.ws.closed:
                self.ws = await websocket.connect(f'self.ws://{ROBOT_2_IP}')
            else:
                self.destroy_timer(self.timer)
                return

            subscribe_message = {
                'op': 'subscribe',
                'topic': topic_name,
                'type': topic_type,
            }
            await self.ws.send(json.dumps(subscribe_message))
            self.logger.log(f'Subscription to topic {topic_name} of robot 2')

            async for message in self.ws:
                try:
                    message = json.loads(message)
                    message_data = message.get('data')
                    if message.get('topic') == topic_name:
                        self.map2_publisher.publish(message_data)
                except json.JSONDecodeError as error:
                    self.logger.error(f'Error processing message from topic {topic_name}: {error}')
        except Exception as error:
            self.logger.error(f'Subscription to robot 2 failed with error: {error}')

def main(args=None):
    rclpy.init(args=args)
    node = GetMapRobot2()
    rclpy.spin(node)
    rclpy.shutdown()