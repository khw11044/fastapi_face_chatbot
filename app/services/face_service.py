import cv2
import mediapipe as mp
from imgbeddings import imgbeddings
import chromadb
from PIL import Image, ImageDraw, ImageFont
import numpy as np
from typing import List, Tuple, Optional
import os
import platform
import warnings
import logging

# 경고 메시지 억제
warnings.filterwarnings("ignore", category=FutureWarning)
warnings.filterwarnings("ignore", category=UserWarning)
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'  # TensorFlow 로그 억제
logging.getLogger('tensorflow').setLevel(logging.ERROR)
logging.getLogger('transformers').setLevel(logging.ERROR)

class EnhancedFaceService:
    """
    얼굴 인식 및 자동 로그인을 위한 향상된 얼굴 서비스 (imgbeddings 사용)
    """
    def __init__(self):
        print("얼굴 인식 서비스 초기화 중...")
        
        # Mediapipe Face Detection 초기화
        self.mp_face_detection = mp.solutions.face_detection
        
        # 느리게 로딩되는 컴포넌트들은 나중에 초기화
        self._face_detection = None
        self._ibed = None
        self._db = None
        self._client = None
        
        # ChromaDB 설정
        self.db_path = './faces'
        
        # 유사도 임계값 설정
        self.similarity_threshold = 0.15  # imgbeddings용으로 조정
        
        # 프레임 처리 최적화를 위한 변수들
        self.frame_count = 0
        self.previous_bboxes = []
        self.previous_labels = []
        self.current_detections = []
        
        # 한글 폰트 초기화
        self.font = self._load_korean_font()
        
        print("얼굴 인식 서비스 초기화 완료")
    
    @property
    def face_detection(self):
        """Lazy loading for face detection"""
        if self._face_detection is None:
            print("얼굴 탐지 모델 로딩 중...")
            self._face_detection = self.mp_face_detection.FaceDetection(
                model_selection=0, 
                min_detection_confidence=0.4
            )
            print("얼굴 탐지 모델 로딩 완료")
        return self._face_detection
    
    @property
    def ibed(self):
        """Lazy loading for image embeddings"""
        if self._ibed is None:
            print("imgbeddings 모델 로딩 중...")
            # 경고 억제를 위한 환경 변수 설정
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                self._ibed = imgbeddings()
            print("imgbeddings 모델 로딩 완료")
        return self._ibed
    
    @property
    def db(self):
        """Lazy loading for ChromaDB"""
        if self._db is None:
            print("벡터 데이터베이스 연결 중...")
            if self._client is None:
                self._client = chromadb.PersistentClient(self.db_path)
            
            self._db = self._client.get_or_create_collection(
                name='facedb',
                metadata={
                    "hnsw:space": 'cosine',
                },
            )
            print("벡터 데이터베이스 연결 완료")
        return self._db
    
    def _load_korean_font(self):
        """한글 지원 폰트를 로드합니다."""
        font_size = 20
        
        # 로컬 fonts 디렉토리 확인 (우선순위)
        local_fonts = []
        local_font_dir = "./fonts"
        if os.path.exists(local_font_dir):
            for font_file in os.listdir(local_font_dir):
                if font_file.endswith(('.ttf', '.ttc', '.otf')):
                    local_fonts.append(os.path.join(local_font_dir, font_file))
        
        # 운영체제별 한글 폰트 경로
        system_fonts = []
        
        if platform.system() == "Windows":
            system_fonts = [
                "C:/Windows/Fonts/malgun.ttf",  # 맑은 고딕
                "C:/Windows/Fonts/gulim.ttc",   # 굴림
                "C:/Windows/Fonts/batang.ttc",  # 바탕
                "C:/Windows/Fonts/NanumGothic.ttf",  # 나눔고딕
            ]
        elif platform.system() == "Darwin":  # macOS
            system_fonts = [
                "/Library/Fonts/AppleSDGothicNeo.ttc",  # 애플 SD 고딕 Neo
                "/System/Library/Fonts/AppleGothic.ttf",  # 애플고딕
                "/Library/Fonts/NanumGothic.ttf",  # 나눔고딕
            ]
        else:  # Linux
            system_fonts = [
                "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",  # 나눔고딕
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",  # DejaVu Sans
                "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",  # Noto Sans CJK
            ]
        
        # 로컬 폰트를 먼저 시도, 그 다음 시스템 폰트
        font_paths = local_fonts + system_fonts
        
        # 폰트 파일이 존재하는지 확인하고 로드
        for font_path in font_paths:
            if os.path.exists(font_path):
                try:
                    return ImageFont.truetype(font_path, font_size)
                except Exception as e:
                    continue
        
        # 기본 폰트로 fallback
        try:
            return ImageFont.load_default()
        except:
            return None
    
    def _draw_korean_text(self, img, text, position, color=(255, 255, 255), bg_color=None):
        """PIL을 사용하여 한글 텍스트를 이미지에 그립니다."""
        if self.font is None:
            return img
        
        # OpenCV 이미지를 PIL 이미지로 변환
        img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(img_pil)
        
        # 텍스트 크기 계산
        text_bbox = draw.textbbox((0, 0), text, font=self.font)
        text_width = text_bbox[2] - text_bbox[0]
        text_height = text_bbox[3] - text_bbox[1]
        
        x, y = position
        
        # 배경 박스 그리기 (선택사항)
        if bg_color:
            padding = 5
            bg_bbox = [
                x - padding, 
                y - text_height - padding, 
                x + text_width + padding, 
                y + padding
            ]
            draw.rectangle(bg_bbox, fill=bg_color)
        
        # 텍스트 그리기
        draw.text((x, y - text_height), text, font=self.font, fill=color)
        
        # PIL 이미지를 다시 OpenCV 이미지로 변환
        img_cv = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
        
        return img_cv
    
    def generate_new_label(self):
        """ChromaDB에 저장된 얼굴 벡터 수 기반으로 새로운 레이블 생성"""
        count = self.db.count()
        new_label = f"user_{count + 1:05d}"
        return new_label
    
    def detect_faces(self, frame):
        """
        얼굴을 탐지하고 바운딩 박스를 반환합니다.
        """
        # 프레임 카운트 증가
        self.frame_count += 1
        
        # 얼굴 탐지 (lazy loading)
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_detection.process(image_rgb)
        
        current_bboxes = []
        
        if results.detections:
            for detection in results.detections:
                bbox = detection.location_data.relative_bounding_box
                h, w, _ = frame.shape
                x1 = int(bbox.xmin * w)
                y1 = int(bbox.ymin * h)
                x2 = int((bbox.xmin + bbox.width) * w)
                y2 = int((bbox.ymin + bbox.height) * h)
                
                # 바운딩 박스 보정 (이미지 경계 내로 제한)
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(w, x2)
                y2 = min(h, y2)
                
                current_bboxes.append((x1, y1, x2, y2))
        
        return current_bboxes
    
    def get_face_embedding(self, frame, bbox):
        """얼굴 영역에서 임베딩을 추출합니다."""
        x1, y1, x2, y2 = bbox
        cropped_face = frame[y1:y2, x1:x2]
        
        if cropped_face.size == 0:
            return None
            
        try:
            cropped_face_rgb = Image.fromarray(cv2.cvtColor(cropped_face, cv2.COLOR_BGR2RGB))
            # 경고 억제
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                embedding = self.ibed.to_embeddings(cropped_face_rgb)[0]
            return embedding
        except Exception as e:
            print(f"임베딩 추출 오류: {e}")
            return None
    
    def search_face_in_db(self, embedding):
        """ChromaDB에서 유사한 얼굴을 검색합니다."""
        try:
            search_results = self.db.query(
                query_embeddings=[embedding.tolist()],
                n_results=1,
                include=["distances", "metadatas"]
            )
            
            if (search_results["distances"] and 
                len(search_results["distances"][0]) > 0 and 
                search_results["distances"][0][0] < self.similarity_threshold):
                
                user_id = search_results["metadatas"][0][0]["user_id"]
                distance = search_results["distances"][0][0]
                return user_id, distance
            else:
                return None, None
                
        except (IndexError, KeyError) as e:
            return None, None
    
    def register_new_face(self, frame, bbox, user_id):
        """새로운 얼굴을 데이터베이스에 등록합니다."""
        embedding = self.get_face_embedding(frame, bbox)
        
        if embedding is None:
            return False
            
        try:
            # 고유 ID 생성 (user_id + timestamp 기반)
            import time
            unique_id = f"{user_id}_{int(time.time())}"
            
            # ChromaDB에 새로운 얼굴 저장
            self.db.add(
                embeddings=[embedding.tolist()],
                metadatas=[{"user_id": user_id}],
                ids=[unique_id]
            )
            print(f"New face registered for user: {user_id}")
            return True
            
        except Exception as e:
            print(f"Error registering new face: {e}")
            return False
    
    def process_frame_for_login(self, frame):
        """
        로그인을 위한 프레임 처리
        얼굴을 탐지하고 데이터베이스에서 검색하여 사용자 정보를 반환
        """
        current_bboxes = self.detect_faces(frame)
        face_info = []
        
        # 얼굴이 탐지되지 않은 경우
        if not current_bboxes:
            self.current_detections = []
            return frame, []
        
        # 프레임 최적화: 매 프레임마다 DB 검색하지 않고 일정 간격으로만 검색
        if (len(current_bboxes) != len(self.previous_bboxes) or 
            self.frame_count % 30 == 0):  # 30프레임마다 또는 얼굴 수가 변경될 때
            
            labels = []
            for bbox in current_bboxes:
                embedding = self.get_face_embedding(frame, bbox)
                
                if embedding is not None:
                    user_id, distance = self.search_face_in_db(embedding)
                    if user_id:
                        labels.append(user_id)
                    else:
                        labels.append("Unknown")
                else:
                    labels.append("Unknown")
            
            self.previous_labels = labels
        else:
            labels = self.previous_labels
        
        # 얼굴 정보 수집
        for bbox, label in zip(current_bboxes, labels):
            face_info.append({
                'bbox': bbox,
                'user_id': label if label != "Unknown" else None,
                'is_known': label != "Unknown"
            })
        
        # 이전 상태 업데이트
        self.previous_bboxes = current_bboxes
        self.current_detections = face_info
        
        return frame, face_info
    
    def detect_and_draw_with_login(self, frame):
        """
        얼굴을 탐지하고 로그인 정보와 함께 바운딩 박스를 그린 프레임을 반환합니다.
        한글 텍스트 지원
        """
        processed_frame, face_info = self.process_frame_for_login(frame)
        
        # 바운딩 박스 및 라벨 그리기
        for info in face_info:
            bbox = info['bbox']
            user_id = info['user_id']
            is_known = info['is_known']
            
            x1, y1, x2, y2 = bbox
            
            # 색상 결정: 알려진 얼굴은 초록색, 모르는 얼굴은 빨간색
            color = (0, 255, 0) if is_known else (0, 0, 255)
            label = user_id if is_known else "Unknown"
            
            # 바운딩 박스 그리기
            cv2.rectangle(processed_frame, (x1, y1), (x2, y2), color, 2)
            
            # 한글 지원 라벨 그리기
            if self.font:
                # PIL을 사용하여 한글 텍스트 그리기
                text_color = (255, 255, 255) if is_known else (255, 255, 255)
                bg_color = color  # BGR을 RGB로 변환
                bg_color_rgb = (bg_color[2], bg_color[1], bg_color[0])
                
                processed_frame = self._draw_korean_text(
                    processed_frame, 
                    label, 
                    (x1, y1 - 5), 
                    color=text_color,
                    bg_color=bg_color_rgb
                )
            else:
                # 폰트가 없는 경우 기본 OpenCV 텍스트 사용 (영어만 지원)
                cv2.putText(processed_frame, label, (x1, y1 - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return processed_frame
    
    def get_auto_login_user(self):
        """
        현재 탐지된 얼굴 중에서 자동 로그인할 수 있는 사용자 ID를 반환
        """
        for detection in self.current_detections:
            if detection['is_known'] and detection['user_id']:
                return detection['user_id']
        return None
    
    def register_face_for_current_user(self, frame, user_id):
        """
        현재 탐지된 Unknown 얼굴을 지정된 사용자 ID로 등록
        """
        for detection in self.current_detections:
            if not detection['is_known']:  # Unknown 얼굴인 경우
                success = self.register_new_face(frame, detection['bbox'], user_id)
                if success:
                    # 현재 탐지 정보 업데이트
                    detection['user_id'] = user_id
                    detection['is_known'] = True
                    return True
        return False


# 싱글톤 인스턴스 생성 (하지만 실제 초기화는 lazy loading)
enhanced_face_service = EnhancedFaceService()