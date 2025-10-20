import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading


class ROS2PublisherService:
    """
    ROS2 Publisher 서비스
    STT로 인식된 텍스트를 '/edie8/llm/input' 토픽으로 발행
    LLM Agent 응답을 '/edie8/llm/output' 토픽으로 발행
    """
    def __init__(self):
        self.node = None
        self.input_publisher = None
        self.output_publisher = None
        self.spin_thread = None
        self.initialized = False
        
    def initialize(self):
        """ROS2 노드 및 Publisher들 초기화"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = Node('fastapi_edie_node')
            
            # Input publisher 초기화 (STT → LLM)
            self.input_publisher = self.node.create_publisher(
                String,
                '/edie8/llm/input',
                10
            )
            
            # Output publisher 초기화 (LLM → 다른 노드들)
            self.output_publisher = self.node.create_publisher(
                String,
                '/edie8/llm/output',
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
            print("✅ ROS2 Publishers initialized:")
            print("   - /edie8/llm/input (STT → LLM)")
            print("   - /edie8/llm/output (LLM → Others)")
            
        except Exception as e:
            print(f"❌ ROS2 initialization failed: {e}")
            self.initialized = False
    
    def publish_message(self, text: str):
        """
        STT 텍스트를 input 토픽으로 발행
        
        Args:
            text (str): 발행할 텍스트
        """
        if not self.initialized or self.input_publisher is None:
            print("⚠️ ROS2 not initialized, skipping input publish")
            return False
        
        try:
            msg = String()
            msg.data = text
            self.input_publisher.publish(msg)
            
            if self.node:
                self.node.get_logger().info(f'📢 Published to /edie8/llm/input: "{text}"')
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to publish input message: {e}")
            return False
    
    def publish_llm_response(self, response_text: str):
        """
        LLM Agent 응답을 output 토픽으로 발행
        
        Args:
            response_text (str): LLM Agent가 생성한 응답 텍스트
        """
        if not self.initialized or self.output_publisher is None:
            print("⚠️ ROS2 not initialized, skipping output publish")
            return False
        
        try:
            msg = String()
            msg.data = response_text
            self.output_publisher.publish(msg)
            
            if self.node:
                self.node.get_logger().info(f'📤 Published to /edie8/llm/output: "{response_text[:50]}..."')
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to publish output message: {e}")
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
