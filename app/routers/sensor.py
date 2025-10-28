from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import asyncio
import json
from ..services.ros2_service import ros2_publisher

router = APIRouter()


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
