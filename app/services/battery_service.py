import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import asyncio
from typing import Optional
import threading


class BatteryService:
    def __init__(self):
        self.battery_percentage: float = 0.0
        self.battery_voltage: float = 0.0
        self._lock = threading.Lock()
        
        # 전압 범위
        self.MIN_VOLTAGE = 12.8
        self.MAX_VOLTAGE = 16.8
        
    def voltage_to_percentage(self, voltage: float) -> float:
        """전압을 백분율로 변환 (12.8V ~ 16.8V → 0% ~ 100%)"""
        percentage = ((voltage - self.MIN_VOLTAGE) / (self.MAX_VOLTAGE - self.MIN_VOLTAGE)) * 100
        return max(0.0, min(100.0, percentage))
    
    def update_battery(self, voltage: float):
        """배터리 전압 업데이트"""
        with self._lock:
            self.battery_voltage = voltage
            self.battery_percentage = self.voltage_to_percentage(voltage)
    
    def get_battery_percentage(self) -> float:
        """현재 배터리 백분율 반환"""
        with self._lock:
            return self.battery_percentage
    
    def get_battery_voltage(self) -> float:
        """현재 배터리 전압 반환"""
        with self._lock:
            return self.battery_voltage


class BatterySubscriber(Node):
    def __init__(self, battery_service: BatteryService):
        super().__init__('battery_subscriber')
        self.battery_service = battery_service
        
        # /edie8/battery/voltage 토픽 구독
        self.subscription = self.create_subscription(
            Float32,
            '/edie8/battery/voltage',
            self.battery_callback,
            10
        )
        self.get_logger().info('Battery subscriber initialized')
    
    def battery_callback(self, msg: Float32):
        """배터리 전압 수신 콜백"""
        voltage = msg.data
        self.battery_service.update_battery(voltage)
        # self.get_logger().info(f'Battery: {voltage:.2f}V ({self.battery_service.get_battery_percentage():.1f}%)')


# 전역 인스턴스
battery_service = BatteryService()
battery_node: Optional[BatterySubscriber] = None


def init_battery_service():
    """배터리 서비스 초기화"""
    global battery_node
    
    if not rclpy.ok():
        rclpy.init()
    
    battery_node = BatterySubscriber(battery_service)
    
    # 별도 스레드에서 ROS2 spin
    def spin_thread():
        rclpy.spin(battery_node)
    
    thread = threading.Thread(target=spin_thread, daemon=True)
    thread.start()
    
    return battery_service


def get_battery_service() -> BatteryService:
    """배터리 서비스 인스턴스 반환"""
    return battery_service
