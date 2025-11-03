from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import asyncio
import json
from ..services.ros2_service import ros2_publisher

router = APIRouter()

# action_index → 이미지 파일명 매핑
ACTION_INDEX_TO_IMAGE = {
    0: "expressionless.jpg",  # 기본값
    1: "curiosity.jpg",
    2: "sleepiness.jpg",
    3: "happiness.jpg",
    4: "happiness.jpg",
    5: "sadness.jpg",
    6: "surprise.jpg",
    7: "surprise.jpg",
    8: "disappointment.jpg",
    9: "love.jpg",
    10: "dizziness.jpg",
    11: "dizziness.jpg",
}

def get_emotion_image_path(action_index: int) -> str:
    """
    action_index → 이미지 경로 변환
    Args:
        action_index (int): 0~11
    Returns:
        str: 이미지 경로 (예: /static/face_raw/happiness.jpg)
    """
    filename = ACTION_INDEX_TO_IMAGE.get(action_index, "expressionless.jpg")
    return f"/static/face_raw/{filename}"


@router.websocket("/ws/laser-sensors")
async def websocket_laser_sensors(websocket: WebSocket):
    """
    웹소켓을 통해 실시간 레이저 센서와 FSR 값 전송
    """
    await websocket.accept()
    
    try:
        while True:
            # ros2_publisher에서 센서 값 가져오기
            sensor_values = ros2_publisher.get_sensor_values()
            fsr_values = ros2_publisher.get_fsr_values()
            
            # 웹소켓으로 전송
            await websocket.send_json({
                "front_left": sensor_values['front_left'],
                "front_right": sensor_values['front_right'],
                "bottom_left": sensor_values['bottom_left'],
                "bottom_right": sensor_values['bottom_right'],
                "left_hip": fsr_values['left_hip'],
                "middle_hip": fsr_values['middle_hip'],
                "right_hip": fsr_values['right_hip'],
                "left_back": fsr_values['left_back'],
                "middle_back": fsr_values['middle_back'],
                "right_back": fsr_values['right_back'],
                "head": fsr_values['head'],
                "left_cheek": fsr_values['left_cheek'],
                "right_cheek": fsr_values['right_cheek'],
                "left_temple": fsr_values['left_temple'],
                "right_temple": fsr_values['right_temple'],
            })
            
            # 100ms 주기로 전송
            await asyncio.sleep(0.1)
            
    except WebSocketDisconnect:
        print("❌ WebSocket client disconnected")
    except Exception as e:
        print(f"❌ WebSocket error: {e}")
        await websocket.close(code=1000)


@router.websocket("/ws/battery")
async def websocket_battery(websocket: WebSocket):
    """
    웹소켓을 통해 1분마다 배터리 상태 전송
    """
    await websocket.accept()
    
    try:
        while True:
            # 배터리 백분율 가져오기 (ros2_publisher에서)
            percentage = ros2_publisher.get_battery_percentage()
            voltage = ros2_publisher.get_battery_voltage()
            
            # 웹소켓으로 전송
            await websocket.send_json({
                "percentage": round(percentage, 1),
                "voltage": round(voltage, 2)
            })
            
            # 60초(1분) 주기로 전송
            await asyncio.sleep(60)
            
    except WebSocketDisconnect:
        print("❌ Battery WebSocket client disconnected")
    except Exception as e:
        print(f"❌ Battery WebSocket error: {e}")
        await websocket.close(code=1000)


@router.websocket("/ws/emotion-stats")
async def websocket_emotion_stats(websocket: WebSocket):
    """
    웹소켓을 통해 실시간 감정 통계 전송 (새 데이터가 들어올 때마다)
    """
    await websocket.accept()
    
    try:
        previous_data = None
        
        while True:
            # 감정 백분율 가져오기 (ros2_publisher에서)
            emotion_percentages = ros2_publisher.get_emotion_percentages()
            
            # 데이터가 변경되었을 때만 전송 (효율화)
            if emotion_percentages != previous_data:
                await websocket.send_json(emotion_percentages)
                previous_data = emotion_percentages.copy()
            
            # 0.1초마다 체크 (새 action_index가 들어오면 즉시 반영)
            await asyncio.sleep(0.1)
            
    except WebSocketDisconnect:
        print("❌ Emotion stats WebSocket client disconnected")
    except Exception as e:
        print(f"❌ Emotion stats WebSocket error: {e}")
        await websocket.close(code=1000)


@router.websocket("/ws/emotion-face")
async def websocket_emotion_face(websocket: WebSocket):
    """
    웹소켓을 통해 실시간 감정 얼굴 이미지 경로 전송
    action_index가 변경될 때마다 즉시 전송
    """
    await websocket.accept()
    
    try:
        previous_action_index = None
        
        while True:
            # ros2_publisher에서 최신 action_index 가져오기
            current_action_index = ros2_publisher.get_latest_action_index()
            
            # action_index가 변경되었을 때만 전송
            if current_action_index != previous_action_index:
                image_path = get_emotion_image_path(current_action_index)
                
                await websocket.send_json({
                    "action_index": current_action_index,
                    "image_path": image_path
                })
                
                previous_action_index = current_action_index
                print(f"😃 Sent emotion image: {image_path} (action_index: {current_action_index})")
            
            # 0.1초마다 체크
            await asyncio.sleep(0.1)
            
    except WebSocketDisconnect:
        print("❌ Emotion face WebSocket client disconnected")
    except Exception as e:
        print(f"❌ Emotion face WebSocket error: {e}")
        await websocket.close(code=1000)


@router.websocket("/ws/decibel")
async def websocket_decibel(websocket: WebSocket):
    """
    웹소켓을 통해 실시간 데시벨 값 전송
    """
    await websocket.accept()
    
    try:
        while True:
            # 데시벨 값 가져오기 (ros2_publisher에서)
            decibel = ros2_publisher.get_decibel()
            
            # 웹소켓으로 전송
            await websocket.send_json({
                "decibel": round(decibel, 1)
            })
            
            # 0.1초 주기로 전송 (실시간 시각화)
            await asyncio.sleep(0.1)
            
    except WebSocketDisconnect:
        print("❌ Decibel WebSocket client disconnected")
    except Exception as e:
        print(f"❌ Decibel WebSocket error: {e}")
        await websocket.close(code=1000)


@router.post("/record-toggle")
async def record_toggle(request: dict):
    """녹음 시작/중지 토글"""
    try:
        is_recording = request.get("is_recording", False)
        success = ros2_publisher.publish_record_start(is_recording)
        
        if success:
            status = "시작" if is_recording else "중지"
            return {
                "success": True,
                "is_recording": is_recording,
                "message": f"녹음 {status}"
            }
        else:
            return {
                "success": False,
                "message": "ROS2 토픽 발행 실패"
            }
    except Exception as e:
        return {
            "success": False,
            "message": str(e)
        }

@router.websocket("/ws/user-emotion")
async def websocket_user_emotion(websocket: WebSocket):
    """
    사용자 감정(표정) 실시간/누적 WebSocket
    단순히 ROS2 토픽에서 받은 감정을 실시간으로 표시
    """
    await websocket.accept()
    print("✅ User emotion WebSocket client connected")
    
    try:
        prev_emotion = None
        prev_histogram = None

        while True:
            # 최신 감정
            latest_emotion = ros2_publisher.get_latest_user_emotion()
            # 100개 히스토리
            history = ros2_publisher.get_user_emotion_history()
            
            # 디버그: 매 루프마다 현재 상태 출력
            # print(f"[WS DEBUG] latest_emotion={latest_emotion}, history_len={len(history)}")
            
            # 감정별 비율 계산
            emotions = ["Anger", "Happiness", "Sadness", "Surprise", "Neutral"]
            histogram = {e: 0 for e in emotions}
            total = len(history)
            for e in history:
                if e in histogram:
                    histogram[e] += 1
            if total > 0:
                for e in emotions:
                    histogram[e] = round(histogram[e] / total * 100, 1)
            
            # 변경 시에만 전송
            if latest_emotion != prev_emotion or histogram != prev_histogram:
                print(f"[WS] Sending to client: latest={latest_emotion}, histogram={histogram}")
                await websocket.send_json({
                    "latest_emotion": latest_emotion,
                    "histogram": histogram
                })
                prev_emotion = latest_emotion
                prev_histogram = histogram.copy()
            
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        print("❌ User emotion WebSocket client disconnected")
    except Exception as e:
        print(f"❌ User emotion WebSocket error: {e}")
        await websocket.close(code=1000)
