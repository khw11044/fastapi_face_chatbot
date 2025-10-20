import threading
import queue
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ROS2 메시지를 저장할 큐 (FastAPI와 공유)
ros2_message_queue = queue.Queue()

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
