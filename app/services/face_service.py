import cv2
import mediapipe as mp
from imgbeddings import imgbeddings
import chromadb
from PIL import Image
import numpy as np
from typing import List, Tuple, Optional


class EnhancedFaceService:
    """
    얼굴 인식 및 자동 로그인을 위한 향상된 얼굴 서비스
    """
    def __init__(self):
        # Mediapipe Face Detection 초기화
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detection = self.mp_face_detection.FaceDetection(
            model_selection=0, 
            min_detection_confidence=0.4
        )
        
        # ChromaDB PersistentClient 초기화
        self.db_path = './faces'
        self.client = chromadb.PersistentClient(self.db_path)
        self.db = self.client.get_or_create_collection(
            name='facedb',
            metadata={
                "hnsw:space": 'cosine',
            },
        )
        
        # imgbeddings 초기화
        self.ibed = imgbeddings()
        
        # 유사도 임계값 설정
        self.similarity_threshold = 0.09
        
        # 프레임 처리 최적화를 위한 변수들
        self.frame_count = 0
        self.previous_bboxes = []
        self.previous_labels = []
        self.current_detections = []
    
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
        
        # 얼굴 탐지
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
            embedding = self.ibed.to_embeddings(cropped_face_rgb)[0]
            return embedding
        except Exception as e:
            print(f"Error generating embedding: {e}")
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
            print(f"Error searching face in DB: {e}")
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
            
            # 라벨 그리기
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            cv2.rectangle(processed_frame, 
                         (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), 
                         color, -1)
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


# 싱글톤 인스턴스 생성
enhanced_face_service = EnhancedFaceService()