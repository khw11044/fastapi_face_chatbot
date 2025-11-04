import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8, Float64MultiArray, Float32MultiArray, Int16MultiArray, Float32, Bool
from sensor_msgs.msg import Image, RegionOfInterest
from rclpy.qos import QoSProfile, ReliabilityPolicy
import threading
import cv2
import numpy as np
from cv_bridge import CvBridge
from typing import Dict, Deque
from collections import deque, Counter


class ROS2PublisherService:
    """
    ROS2 Publisher 서비스
    STT로 인식된 텍스트를 '/edie8/llm/input' 토픽으로 발행
    LLM Agent 응답을 '/edie8/llm/output' 토픽으로 발행
    이미지 토픽 구독 및 스트리밍
    """
    def __init__(self):
        self.node = None
        self.input_publisher = None
        self.output_publisher = None
        self.emotion_publisher = None
        self.left_ear_publisher = None
        self.right_ear_publisher = None
        self.image_subscriber = None
        self.spin_thread = None
        self.initialized = False
        
        # 이미지 관련
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # 센서 값 관련
        self.sensor_values: Dict[str, float] = {
            'front_left': 0,
            'front_right': 0,
            'bottom_left': 0,
            'bottom_right': 0,
        }
        self.sensor_lock = threading.Lock()
        
        # FSR 센서 값 관련
        self.fsr_values: Dict[str, int] = {
            'left_hip': 0,
            'middle_hip': 0,
            'right_hip': 0,
            'left_back': 0,
            'middle_back': 0,
            'right_back': 0,
            'head': 0,
            'left_cheek': 0,
            'right_cheek': 0,
            'left_temple': 0,
            'right_temple': 0,
            'right_temple_dup': 0,
        }
        self.fsr_lock = threading.Lock()
        
        # 배터리 값 관련
        self.battery_voltage: float = 0.0
        self.battery_percentage: float = 0.0
        self.battery_lock = threading.Lock()
        self.MIN_VOLTAGE = 10.8
        self.MAX_VOLTAGE = 16.8
        
        # 데시벨 값 관련
        self.decibel_value: float = 0.0
        self.decibel_lock = threading.Lock()
        
        # 감정 action_index 히스토리 (최근 100개)
        self.emotion_history = deque(maxlen=100)
        self.emotion_lock = threading.Lock()
        
        # 최신 감정 action_index
        self.latest_action_index = 0
        self.latest_emotion_lock = threading.Lock()

        # ROI(Closest Human) 관련
        self.latest_roi = None
        self.roi_lock = threading.Lock()

        # 사용자 감정 히스토리 (최대 100개, Unknown 제외)
        self.user_emotion_history: Deque[str] = deque(maxlen=100)
        self.user_emotion_lock = threading.Lock()
        self.valid_emotions = {"Anger", "Happiness", "Sadness", "Surprise", "Neutral"}
        self.latest_user_emotion: str = None
        
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
            
            # Emotion publisher (BEST_EFFORT)
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=10
            )
            self.emotion_publisher = self.node.create_publisher(
                UInt8,
                '/edie8/emotion/action_index',
                qos_profile
            )
            
            # Record start publisher
            self.record_start_publisher = self.node.create_publisher(
                Bool,
                '/edie8/sound/record_start',
                10
            )

            # Ear publishers (BEST_EFFORT)
            self.left_ear_publisher = self.node.create_publisher(
                Float64MultiArray,
                '/edie8_l_ear_position_controller/commands',
                qos_profile
            )
            self.right_ear_publisher = self.node.create_publisher(
                Float64MultiArray,
                '/edie8_r_ear_position_controller/commands',
                qos_profile
            )

            # Image subscriber (BEST_EFFORT)
            self.image_subscriber = self.node.create_subscription(
                Image,
                '/edie8/vision/image_raw',
                self._image_callback,
                qos_profile
            )
            
            # Closest Human ROI subscriber
            self.closest_human_roi_subscriber = self.node.create_subscription(
                RegionOfInterest,
                '/edie8/vision/closest_human_roi',
                self._closest_human_roi_callback,
                10
            )

            # 사용자 감정(표정) 상태 구독 (std_msgs/String)
            qos_profile_user_emotion = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=10
            )
            self.user_emotion_state_subscriber = self.node.create_subscription(
                String,
                '/edie8/vision/emotion_state',
                self._user_emotion_state_callback,
                qos_profile_user_emotion
            )

            # Laser sensor subscribers
            self.node.create_subscription(
                Int16MultiArray,
                '/edie8/sensor/front/laser',
                self._front_laser_callback,
                10
            )
            
            self.node.create_subscription(
                Int16MultiArray,
                '/edie8/sensor/bottom/laser_values',
                self._bottom_laser_callback,
                10
            )
            
            # FSR sensor subscriber
            self.node.create_subscription(
                Int16MultiArray,
                '/edie8/sensor/fsr',
                self._fsr_callback,
                10
            )
            
            # Battery voltage subscriber
            self.node.create_subscription(
                Float32,
                '/edie8/battery/voltage',
                self._battery_callback,
                10
            )
            
            # Emotion action_index subscriber (BEST_EFFORT)
            self.node.create_subscription(
                UInt8,
                '/edie8/emotion/action_index',
                self._emotion_action_callback,
                qos_profile
            )
            
            # Decibel subscriber
            self.node.create_subscription(
                Float32,
                '/edie8/sound/decibel',
                self._decibel_callback,
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
            print("✅ ROS2 Publishers/Subscribers initialized:")
            print("   - /edie8/llm/input (STT → LLM)")
            print("   - /edie8/llm/output (LLM → Others)")
            print("   - /edie8/emotion/action_index (Emotion, BEST_EFFORT)")
            print("   - /edie8/vision/image_raw (Image Subscriber, BEST_EFFORT)")
            print("   - /edie8/vision/emotion_state (User Emotion State Subscriber, user_emotion_state)")
            print("   - /edie8/sensor/fsr")
            print("   - /edie8/sensor/front/laser (Front Laser)")
            print("   - /edie8/sensor/bottom/laser_values (Bottom Laser)")
            print("   - /edie8/battery/voltage (Battery)")
            print("   - /edie8/emotion/action_index (Emotion Stats)")
            print("   - /edie8/sound/record_start (Record Toggle)")
            print("   - /edie8/sound/decibel (Decibel)")
            
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
    
    def publish_emotion_action(self, action_index: int):
        """
        감정 action_index를 emotion 토픽으로 퍼블리시 (BEST_EFFORT)
        Args:
            action_index (int): 1~11, 감정 인덱스
        Returns:
            bool: 성공 여부
        """
        if not self.initialized or self.emotion_publisher is None:
            print("⚠️ ROS2 not initialized, skipping emotion publish")
            return False
        try:
            msg = UInt8()
            msg.data = action_index
            self.emotion_publisher.publish(msg)
            if self.node:
                self.node.get_logger().info(f'😃 Published to /edie8/emotion/action_index: {action_index}')
            return True
        except Exception as e:
            print(f"❌ Failed to publish emotion action: {e}")
            return False

    def publish_ear_position(self, left: float, right: float) -> bool:
        """
        좌/우 귀 위치를 각각의 토픽으로 퍼블리시 (BEST_EFFORT)
        Args:
            left (float): 왼쪽 귀 위치 (0.0~0.9)
            right (float): 오른쪽 귀 위치 (0.0~0.9)
        Returns:
            bool: 성공 여부
        """
        if not self.initialized or self.left_ear_publisher is None or self.right_ear_publisher is None:
            print("⚠️ ROS2 not initialized, skipping ear publish")
            return False
        try:
            left_msg = Float64MultiArray()
            left_msg.data = [left]
            self.left_ear_publisher.publish(left_msg)
            if self.node:
                self.node.get_logger().info(f'👂 Published to /edie8_l_ear_position_controller/commands: {left}')
            right_msg = Float64MultiArray()
            right_msg.data = [right]
            self.right_ear_publisher.publish(right_msg)
            if self.node:
                self.node.get_logger().info(f'👂 Published to /edie8_r_ear_position_controller/commands: {right}')
            return True
        except Exception as e:
            print(f"❌ Failed to publish ear positions: {e}")
            return False
    
    def publish_record_start(self, is_recording: bool) -> bool:
        """
        녹음 시작/중지 신호를 퍼블리시
        Args:
            is_recording (bool): True = 녹음 시작, False = 녹음 중지
        Returns:
            bool: 성공 여부
        """
        if not self.initialized or self.record_start_publisher is None:
            print("⚠️ ROS2 not initialized, skipping record start publish")
            return False
        try:
            msg = Bool()
            msg.data = is_recording
            self.record_start_publisher.publish(msg)
            if self.node:
                status = "시작" if is_recording else "중지"
                self.node.get_logger().info(f'🎤 Published to /edie8/sound/record_start: {status}')
            return True
        except Exception as e:
            print(f"❌ Failed to publish record start: {e}")
            return False

    def _image_callback(self, msg: Image):
        """
        이미지 토픽 콜백 함수
        Args:
            msg (Image): ROS2 Image 메시지
        """
        try:
            # ROS Image → OpenCV Image 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 최신 프레임 업데이트 (스레드 안전)
            with self.frame_lock:
                self.latest_frame = cv_image
                
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'❌ Image conversion failed: {e}')
    
    def _front_laser_callback(self, msg: Int16MultiArray):
        """Front 레이저 콜백"""
        try:
            with self.sensor_lock:
                if len(msg.data) >= 2:
                    self.sensor_values['front_left'] = int(msg.data[0])
                    self.sensor_values['front_right'] = int(msg.data[1])
                    # print(f"📊 [Front Laser] left={self.sensor_values['front_left']}, right={self.sensor_values['front_right']}")
        except Exception as e:
            print(f"❌ Front laser callback error: {e}")
    
    def _bottom_laser_callback(self, msg: Int16MultiArray):
        """Bottom 레이저 콜백"""
        try:
            with self.sensor_lock:
                if len(msg.data) >= 2:
                    self.sensor_values['bottom_left'] = int(msg.data[0])
                    self.sensor_values['bottom_right'] = int(msg.data[1])
                    # print(f"📊 [Bottom Laser] left={self.sensor_values['bottom_left']}, right={self.sensor_values['bottom_right']}")
        except Exception as e:
            print(f"❌ Bottom laser callback error: {e}")
    
    def get_sensor_values(self) -> Dict[str, int]:
        """현재 센서 값 반환"""
        with self.sensor_lock:
            return self.sensor_values.copy()
    
    def get_fsr_values(self) -> Dict[str, int]:
        """현재 FSR 센서 값 반환"""
        with self.fsr_lock:
            return self.fsr_values.copy()
    
    def _fsr_callback(self, msg: Int16MultiArray):
        """FSR 센서 콜백"""
        try:
            with self.fsr_lock:
                if len(msg.data) >= 12:
                    self.fsr_values['left_hip'] = int(msg.data[0])
                    self.fsr_values['middle_hip'] = int(msg.data[1])
                    self.fsr_values['right_hip'] = int(msg.data[2])
                    self.fsr_values['left_back'] = int(msg.data[3])
                    self.fsr_values['middle_back'] = int(msg.data[4])
                    self.fsr_values['right_back'] = int(msg.data[5])
                    self.fsr_values['head'] = int(msg.data[6])
                    self.fsr_values['left_cheek'] = int(msg.data[7])
                    self.fsr_values['right_cheek'] = int(msg.data[8])
                    self.fsr_values['left_temple'] = int(msg.data[9])
                    self.fsr_values['right_temple'] = int(msg.data[10])
                    self.fsr_values['right_temple_dup'] = int(msg.data[11])
        except Exception as e:
            print(f"❌ FSR callback error: {e}")
    
    def _battery_callback(self, msg: Float32):
        """배터리 전압 콜백"""
        try:
            voltage = msg.data
            percentage = ((voltage - self.MIN_VOLTAGE) / (self.MAX_VOLTAGE - self.MIN_VOLTAGE)) * 100
            percentage = max(0.0, min(100.0, percentage))
            
            with self.battery_lock:
                self.battery_voltage = voltage
                self.battery_percentage = percentage
        except Exception as e:
            print(f"❌ Battery callback error: {e}")
    
    def get_battery_percentage(self) -> float:
        """현재 배터리 백분율 반환"""
        with self.battery_lock:
            return self.battery_percentage
    
    def get_battery_voltage(self) -> float:
        """현재 배터리 전압 반환"""
        with self.battery_lock:
            return self.battery_voltage
    
    def _decibel_callback(self, msg: Float32):
        """데시벨 콜백"""
        try:
            decibel = msg.data
            with self.decibel_lock:
                self.decibel_value = decibel
        except Exception as e:
            print(f"❌ Decibel callback error: {e}")
    
    def get_decibel(self) -> float:
        """현재 데시벨 값 반환"""
        with self.decibel_lock:
            return self.decibel_value
    
    def _emotion_action_callback(self, msg: UInt8):
        """감정 action_index 콜백"""
        try:
            action_index = msg.data
            with self.emotion_lock:
                self.emotion_history.append(action_index)
            with self.latest_emotion_lock:
                self.latest_action_index = action_index
            print(f"😃 Emotion action_index updated: {action_index}")
        except Exception as e:
            print(f"❌ Emotion action callback error: {e}")
    
    def clear_emotion_history(self):
        """감정 히스토리 초기화"""
        with self.emotion_lock:
            self.emotion_history.clear()
        print("🧹 Emotion history cleared")
    
    def get_latest_action_index(self) -> int:
        """최신 action_index 반환"""
        with self.latest_emotion_lock:
            return self.latest_action_index
    
    def _closest_human_roi_callback(self, msg: RegionOfInterest):
        """Closest Human ROI 콜백"""
        try:
            roi_data = {
                "x_offset": msg.x_offset,
                "y_offset": msg.y_offset,
                "width": msg.width,
                "height": msg.height,
                "do_rectify": msg.do_rectify
            }
            with self.roi_lock:
                self.latest_roi = roi_data
        except Exception as e:
            print(f"❌ Closest Human ROI callback error: {e}")

    def get_latest_roi(self):
        """최신 ROI 정보 반환 (dict 또는 None)"""
        with self.roi_lock:
            return self.latest_roi.copy() if self.latest_roi is not None else None

    def _user_emotion_state_callback(self, msg: String):
        """
        사용자 감정(표정) 상태 콜백 (user_emotion_state)
        - data: Anger, Happiness, Sadness, Surprise, Neutral, Unknown
        - Unknown은 무시
        - 단순히 최근 100개를 누적하고 최신 감정 저장
        """
        try:
            emotion = msg.data.strip()
            # self.node.get_logger().info(f"[USER_EMOTION_STATE] Received: {emotion}")
            if emotion not in self.valid_emotions:
                return
            with self.user_emotion_lock:
                self.user_emotion_history.append(emotion)
                self.latest_user_emotion = emotion
                self.node.get_logger().info(f"[USER_EMOTION_STATE] emotion={emotion}, history={list(self.user_emotion_history)[-5:]}")
        except Exception as e:
            self.node.get_logger().error(f"[USER_EMOTION_STATE] Exception: {e}")

    def get_latest_user_emotion(self):
        """최신 감정 반환"""
        with self.user_emotion_lock:
            return self.latest_user_emotion

    def get_user_emotion_history(self):
        """감정 히스토리(리스트) 반환"""
        with self.user_emotion_lock:
            return list(self.user_emotion_history)

    def clear_user_emotion_history(self):
        """감정 히스토리 초기화"""
        with self.user_emotion_lock:
            self.user_emotion_history.clear()
            self.latest_user_emotion = None

    def get_emotion_percentages(self) -> Dict[str, float]:
        """최근 100개 action_index 기준 8개 감정별 백분율 반환"""
        with self.emotion_lock:
            if len(self.emotion_history) == 0:
                return {
                    "curiosity": 0.0,     # 1
                    "sleepiness": 0.0,    # 2
                    "happiness": 0.0,     # 3, 4
                    "sadness": 0.0,       # 5
                    "surprise": 0.0,      # 6, 7
                    "disappointment": 0.0, # 8
                    "love": 0.0,          # 9
                    "dizziness": 0.0,     # 10, 11
                }
            
            # action_index → 감정 매핑
            emotion_mapping = {
                1: "curiosity",
                2: "sleepiness",
                3: "happiness",
                4: "happiness",
                5: "sadness",
                6: "surprise",
                7: "surprise",
                8: "disappointment",
                9: "love",
                10: "dizziness",
                11: "dizziness",
            }
            
            # 감정별 카운트
            emotion_counts = Counter()
            for action_idx in self.emotion_history:
                emotion = emotion_mapping.get(action_idx)
                if emotion:
                    emotion_counts[emotion] += 1
            
            # 백분율 계산
            total = len(self.emotion_history)
            emotion_percentages = {
                "curiosity": (emotion_counts.get("curiosity", 0) / total) * 100,
                "sleepiness": (emotion_counts.get("sleepiness", 0) / total) * 100,
                "happiness": (emotion_counts.get("happiness", 0) / total) * 100,
                "sadness": (emotion_counts.get("sadness", 0) / total) * 100,
                "surprise": (emotion_counts.get("surprise", 0) / total) * 100,
                "disappointment": (emotion_counts.get("disappointment", 0) / total) * 100,
                "love": (emotion_counts.get("love", 0) / total) * 100,
                "dizziness": (emotion_counts.get("dizziness", 0) / total) * 100,
            }
            
            return emotion_percentages

    def get_latest_frame(self):
        """
        최신 프레임 반환
        Returns:
            numpy.ndarray or None: 최신 OpenCV 이미지
        """
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

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
