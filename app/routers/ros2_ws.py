from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import asyncio
import json
from app.services.ros2_subscriber import ros2_message_queue, bot_message_queue

router = APIRouter()

@router.websocket("/ws/ros2")
async def ros2_websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            sent = False
            # ROS2 입력 메시지 우선 전송
            try:
                msg = await asyncio.get_event_loop().run_in_executor(
                    None, ros2_message_queue.get, True, 0.1
                )
                await websocket.send_text(json.dumps({"type": "robot", "text": msg}))
                sent = True
            except Exception:
                pass
            # LLM 응답 메시지 전송
            try:
                bot_msg = await asyncio.get_event_loop().run_in_executor(
                    None, bot_message_queue.get, True, 0.1
                )
                await websocket.send_text(json.dumps({"type": "bot", "text": bot_msg}))
                sent = True
            except Exception:
                pass
            if not sent:
                await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass
