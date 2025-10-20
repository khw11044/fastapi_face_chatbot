from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import asyncio
from app.services.ros2_subscriber import ros2_message_queue

router = APIRouter()

@router.websocket("/ws/ros2")
async def ros2_websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            # 큐에 메시지가 있으면 바로 전송, 없으면 대기
            try:
                # 0.5초마다 큐 polling
                msg = await asyncio.get_event_loop().run_in_executor(
                    None, ros2_message_queue.get, True, 0.5
                )
                await websocket.send_text(msg)
            except Exception:
                await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass
