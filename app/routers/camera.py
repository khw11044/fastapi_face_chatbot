from fastapi import APIRouter
from fastapi.responses import StreamingResponse
from app.services.ros2_service import ros2_publisher
import cv2
import time
import numpy as np

router = APIRouter()


def generate_mjpeg_stream():
    """
    MJPEG 스트림 생성기
    ROS2 토픽에서 받은 이미지를 JPEG로 인코딩하여 스트리밍
    """
    while True:
        # 최신 프레임 가져오기
        frame = ros2_publisher.get_latest_frame()
        
        if frame is not None:
            # JPEG 인코딩
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            
            if ret:
                # MJPEG 형식으로 프레임 전송
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        else:
            # 프레임이 없으면 빈 이미지 전송 (640x480 검은색)
            empty_img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(empty_img, 'Waiting for camera...', 
                       (120, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 
                       1, 
                       (200, 200, 200), 
                       2)
            ret, empty_buffer = cv2.imencode('.jpg', empty_img)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + empty_buffer.tobytes() + b'\r\n')
        
        # FPS 제어 (약 30 FPS)
        time.sleep(0.033)


@router.get("/stream")
async def camera_stream():
    """
    카메라 스트림 엔드포인트
    /edie8/vision/image_raw 토픽의 이미지를 MJPEG 스트림으로 제공
    """
    return StreamingResponse(
        generate_mjpeg_stream(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )
