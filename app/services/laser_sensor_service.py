import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
from typing import Dict, List, Optional


class LaserSensorService:
    """
    레이저 센서 값을 ROS2 토픽에서 구독하고 최신 값을 저장하는 서비스
    """
    def __init__(self):
        self.node: Optional[Node] = None
        self.spin_thread: Optional[threading.Thread] = None
        self.initialized = False
        
        # 센서 값 저장
        self.sensor_values: Dict[str, List[float]] = {
            'front_left': 0,
            'front_right': 0,
            'bottom_left': 0,
            'bottom_right': 0,
        }
        self.lock = threading.Lock()
    
    def initialize(self):
        """ROS2 노드 및 구독자 초기화"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = Node('laser_sensor_listener')
            
            # Front 레이저 구독
            self.node.create_subscription(
                Float32MultiArray,
                '/edie8/sensor/front/laser',
                self._front_laser_callback,
                10
            )
            
            # Bottom 레이저 구독
            self.node.create_subscription(
                Float32MultiArray,
                '/edie8/sensor/bottom/laser_values',
                self._bottom_laser_callback,
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
            print("✅ Laser Sensor Service initialized")
            
        except Exception as e:
            print(f"❌ Laser Sensor Service initialization failed: {e}")
            self.initialized = False
    
    def _front_laser_callback(self, msg: Float32MultiArray):
        """Front 레이저 콜백"""
        try:
            with self.lock:
                if len(msg.data) >= 2:
                    self.sensor_values['front_left'] = float(msg.data[0])
                    self.sensor_values['front_right'] = float(msg.data[1])
        except Exception as e:
            print(f"❌ Front laser callback error: {e}")
    
    def _bottom_laser_callback(self, msg: Float32MultiArray):
        """Bottom 레이저 콜백"""
        try:
            with self.lock:
                if len(msg.data) >= 2:
                    self.sensor_values['bottom_left'] = float(msg.data[0])
                    self.sensor_values['bottom_right'] = float(msg.data[1])
        except Exception as e:
            print(f"❌ Bottom laser callback error: {e}")
    
    def get_sensor_values(self) -> Dict[str, float]:
        """현재 센서 값 반환"""
        with self.lock:
            return self.sensor_values.copy()
    
    def shutdown(self):
        """서비스 종료"""
        if self.node:
            try:
                self.node.destroy_node()
                print("✅ Laser Sensor Node destroyed")
            except Exception as e:
                print(f"⚠️ Error destroying sensor node: {e}")
        
        if rclpy.ok():
            try:
                rclpy.shutdown()
                print("✅ ROS2 sensor shutdown complete")
            except Exception as e:
                print(f"⚠️ Error shutting down ROS2 sensor: {e}")


# 싱글톤 인스턴스
laser_sensor_service = LaserSensorService()
