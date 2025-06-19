from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
import cv2
import threading
import time
from typing import Optional

from ..services.face_service import enhanced_face_service

router = APIRouter()

class FaceRegistrationRequest(BaseModel):
    user_id: str

class CameraManager:
    def __init__(self):
        self.camera = None
        self.is_streaming = False
        self.lock = threading.Lock()
        self.frame_generator = None
        self.current_frame = None
    
    def start_camera(self, camera_index: int = 0):
        """카메라를 시작합니다."""
        with self.lock:
            if self.camera is not None:
                self.camera.release()
            
            try:
                self.camera = cv2.VideoCapture(camera_index)
                if not self.camera.isOpened():
                    raise Exception(f"카메라 {camera_index}를 열 수 없습니다.")
                
                # 카메라 설정
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.camera.set(cv2.CAP_PROP_FPS, 30)
                
                self.is_streaming = True
                print(f"카메라 {camera_index} 시작됨")
                return True
                
            except Exception as e:
                print(f"카메라 시작 오류: {e}")
                self.camera = None
                self.is_streaming = False
                return False
    
    def stop_camera(self):
        """카메라를 완전히 중지합니다."""
        with self.lock:
            print("카메라 중지 요청...")
            self.is_streaming = False
            
            # 카메라 리소스 해제
            if self.camera is not None:
                try:
                    self.camera.release()
                    self.camera = None
                    print("카메라 리소스 해제 완료")
                except Exception as e:
                    print(f"카메라 해제 중 오류: {e}")
            
            # OpenCV 윈도우 정리 (있다면)
            try:
                cv2.destroyAllWindows()
            except Exception as e:
                pass
            
            print("카메라 완전히 중지됨")
    
    def generate_frames(self):
        """프레임을 생성합니다."""
        frame_count = 0
        while self.is_streaming:
            with self.lock:
                if self.camera is None or not self.camera.isOpened():
                    print("카메라가 없거나 열리지 않음. 스트림 종료.")
                    break
                
                success, frame = self.camera.read()
                if not success:
                    print("프레임을 읽을 수 없습니다.")
                    break
                
                # 프레임 좌우 반전 (미러 효과)
                frame = cv2.flip(frame, 1)
                
                # 현재 프레임 저장 (얼굴 등록용)
                self.current_frame = frame.copy()
                
                # ✅ 향상된 얼굴 탐지 및 바운딩 박스 그리기
                try:
                    frame = enhanced_face_service.detect_and_draw_with_login(frame)
                except Exception as e:
                    print(f"얼굴 인식 오류: {e}")
                    # 얼굴 인식에 실패해도 원본 프레임은 전송

                # 프레임을 JPEG로 인코딩
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not ret:
                    print("프레임 인코딩 실패")
                    continue
                
                frame_bytes = buffer.tobytes()
                frame_count += 1
                
                # 스트리밍이 중지되었는지 다시 확인
                if not self.is_streaming:
                    print(f"총 {frame_count}개 프레임 전송 후 스트림 종료")
                    break
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            time.sleep(0.033)  # 약 30 FPS
        
        print("프레임 생성 함수 종료")

# 전역 카메라 매니저 인스턴스
camera_manager = CameraManager()

@router.get("/stream")
async def video_stream():
    """비디오 스트림을 반환합니다."""
    if not camera_manager.is_streaming:
        # 카메라가 시작되지 않았다면 시작 시도
        if not camera_manager.start_camera():
            raise HTTPException(status_code=500, detail="카메라를 시작할 수 없습니다.")
    
    return StreamingResponse(
        camera_manager.generate_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@router.post("/start")
async def start_camera(camera_index: int = 0):
    """카메라를 시작합니다."""
    try:
        success = camera_manager.start_camera(camera_index)
        if success:
            return {"message": f"카메라 {camera_index} 시작됨", "status": "success"}
        else:
            raise HTTPException(status_code=500, detail="카메라를 시작할 수 없습니다.")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/stop")
async def stop_camera():
    """카메라를 중지합니다."""
    try:
        camera_manager.stop_camera()
        return {"message": "카메라 중지됨", "status": "success"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/status")
async def camera_status():
    """카메라 상태를 반환합니다."""
    return {
        "is_streaming": camera_manager.is_streaming,
        "camera_active": camera_manager.camera is not None and camera_manager.camera.isOpened() if camera_manager.camera else False
    }

@router.get("/auto-login")
async def get_auto_login_user():
    """자동 로그인 가능한 사용자 ID를 반환합니다."""
    try:
        user_id = enhanced_face_service.get_auto_login_user()
        if user_id:
            return {"user_id": user_id, "auto_login": True}
        else:
            return {"user_id": None, "auto_login": False}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/register-face")
async def register_face(request: FaceRegistrationRequest):
    """현재 탐지된 Unknown 얼굴을 지정된 사용자로 등록합니다."""
    try:
        if not camera_manager.is_streaming or camera_manager.current_frame is None:
            raise HTTPException(status_code=400, detail="카메라가 활성화되지 않았습니다.")
        
        success = enhanced_face_service.register_face_for_current_user(
            camera_manager.current_frame, 
            request.user_id
        )
        
        if success:
            return {
                "message": f"사용자 '{request.user_id}'의 얼굴이 등록되었습니다.",
                "status": "success"
            }
        else:
            return {
                "message": "등록할 수 있는 얼굴이 탐지되지 않았습니다.",
                "status": "failed"
            }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))