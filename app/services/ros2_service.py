import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading


class ROS2PublisherService:
    """
    ROS2 Publisher 서비스
    STT로 인식된 텍스트를 '/edie8/llm/input' 토픽으로 발행
    """
    def __init__(self):
        self.node = None
        self.publisher = None
        self.spin_thread = None
        self.initialized = False
        
    def initialize(self):
        """ROS2 노드 및 Publisher 초기화"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = Node('fastapi_edie_node')
            self.publisher = self.node.create_publisher(
                String,
                '/edie8/llm/input',
                10
            )
            
            # 별도 스레드에서 spin 실행
            self.spin_thread = threading.Thread(
                target=rclpy.spin,
                args=(self.node,),
                daemon=True
            )
            self.spin_thread.start()
            
            self.initialized = True
            print("✅ ROS2 Publisher initialized: /edie8/llm/input")
            
        except Exception as e:
            print(f"❌ ROS2 initialization failed: {e}")
            self.initialized = False
    
    def publish_message(self, text: str):
        """
        텍스트를 ROS2 토픽으로 발행
        
        Args:
            text (str): 발행할 텍스트
        """
        if not self.initialized or self.publisher is None:
            print("⚠️ ROS2 not initialized, skipping publish")
            return False
        
        try:
            msg = String()
            msg.data = text
            self.publisher.publish(msg)
            
            if self.node:
                self.node.get_logger().info(f'📢 Published to /edie8/llm/input: "{text}"')
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to publish message: {e}")
            return False
    
    def shutdown(self):
        """ROS2 노드 종료"""
        if self.node:
            try:
                self.node.destroy_node()
                print("✅ ROS2 node destroyed")
            except Exception as e:
                print(f"⚠️ Error destroying node: {e}")
        
        if rclpy.ok():
            try:
                rclpy.shutdown()
                print("✅ ROS2 shutdown complete")
            except Exception as e:
                print(f"⚠️ Error shutting down ROS2: {e}")


# 싱글톤 인스턴스
ros2_publisher = ROS2PublisherService()
