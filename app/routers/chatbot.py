from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from ..services.llm_service import LLMService, parse_emotion_from_response, get_emotion_image_path
from ..services.ros2_service import ros2_publisher

router = APIRouter()
llm_service = LLMService()

# 요청 데이터 구조 정의
class ChatRequest(BaseModel):
    message: str
    session_id: str = "default"

@router.post("/chat")
async def chat(request: ChatRequest):
    """사용자 메시지를 받아서 AI 응답을 반환합니다."""
    try:
        print(f'💬 텍스트 입력 수신: "{request.message}" (세션: {request.session_id})')
        
        # ROS2 input 토픽으로 텍스트 메시지 발행 (STT와 동일하게)
        try:
            if ros2_publisher and ros2_publisher.initialized:
                success = ros2_publisher.publish_message(request.message)
                print(f"📢 텍스트 입력 '{request.message}'가 /edie8/llm/input 토픽으로 발행됨: {success}")
                if not success:
                    print(f"❌ 텍스트 입력 ROS2 토픽 발행 실패")
            else:
                print(f"⚠️ ROS2 서비스가 초기화되지 않았습니다")
        except Exception as ros_error:
            print(f"⚠️ ROS2 input 토픽 발행 실패: {ros_error}")
            # ROS2 실패해도 채팅은 계속 진행
        
        print(f'[사용자 ({request.session_id})] {request.message}')
        
        # AI 응답 생성
        response = await llm_service.generate_response(request.message, request.session_id)
        
        # 감정 파싱 및 이미지 경로 매핑
        emotion = parse_emotion_from_response(response)
        emotion_img = get_emotion_image_path(emotion) if emotion else None

        print(f'[AI] {response}')
        print(f'[감정] {emotion}, [이미지] {emotion_img}')
        
        return {
            "response": response,
            "emotion": emotion,
            "emotion_img": emotion_img
        }
    except Exception as e:
        print(f"Chat error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/clear")
async def clear_chat(request: dict):
    """채팅 기록을 초기화합니다."""
    try:
        session_id = request.get("session_id", "default")
        llm_service.clear_history(session_id)
        return {"message": "Chat history cleared successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/archive")
async def archive_database():
    """DB 전체를 아카이빙하고 새로 시작합니다."""
    try:
        archive_name = llm_service.archive_and_reset_database()
        
        # 감정 히스토리도 초기화
        ros2_publisher.clear_emotion_history()
        
        return {
            "message": "Database archived successfully",
            "archive_name": archive_name
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/history/{session_id}")
async def get_chat_history(session_id: str, limit: int = 10):
    """특정 세션의 채팅 기록을 가져옵니다."""
    try:
        history = llm_service.get_history(session_id, limit)
        return {"history": history}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/sessions")
async def get_all_sessions():
    """모든 세션 정보를 가져옵니다."""
    try:
        sessions = llm_service.get_all_sessions()
        return {"sessions": sessions}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
