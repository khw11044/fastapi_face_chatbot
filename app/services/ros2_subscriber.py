import threading
import queue
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio

from app.services.llm_service import LLMService

# ROS2 메시지를 저장할 큐 (FastAPI와 공유)
ros2_message_queue = queue.Queue()
bot_message_queue = queue.Queue()
llm_service = LLMService()

class LLMInputSubscriber(Node):
    def __init__(self):
        super().__init__('llm_input_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/edie8/llm/input',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # 메시지를 큐에 저장
        ros2_message_queue.put(msg.data)
        self.get_logger().info(f'Received: "{msg.data}"')
        # LLM agent에 입력 (비동기) 및 응답을 bot_message_queue에 저장
        async def handle_llm():
            response = await llm_service.generate_response(msg.data, session_id="ros2")
            bot_message_queue.put(response)
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = None
        if loop and loop.is_running():
            loop.create_task(handle_llm())
        else:
            asyncio.run(handle_llm())

def ros2_spin_thread():
    rclpy.init()
    node = LLMInputSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

def start_ros2_subscriber():
    thread = threading.Thread(target=ros2_spin_thread, daemon=True)
    thread.start()
