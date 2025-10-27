import asyncio
import os

from langchain_openai import ChatOpenAI
from langchain_ollama import ChatOllama
from langchain_core.messages import HumanMessage, AIMessage
from langchain_core.runnables.history import RunnableWithMessageHistory
from langchain_community.chat_message_histories import SQLChatMessageHistory
from langchain_core.output_parsers import StrOutputParser
from langchain.agents import create_tool_calling_agent, AgentExecutor
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from typing import Dict, List
from dotenv import load_dotenv


from utils.prompts.prompt import edie_agent_prompt   # , edie_robot_system_simple_prompt_obj
from utils.databases.database import DatabaseManager
from utils.agent.toolbox import ToolBox
from utils.agent.toolbox import calculation as calculation_tool
from utils.agent.toolbox import action as action_tool
from utils.agent.toolbox import expression as expression_tool

load_dotenv()

import re

# 감정 키워드 → 이미지 파일명 매핑
EMOTION_IMAGE_MAP = {
    "curious": "curious.png",
    "delight": "delight.png",
    "dizzy": "dizzy.png",
    "expressionless": "expressionless.png",
    "love": "love.png",
    "sad": "sad.png",
    "sleepy": "sleepy.png",
    "surprise": "surprise.png",
    "disappointment": "disappointment.png",
}

emotions = {
    1: "궁금함",                # curious.png
    2: "졸림",                  # sleepy.png
    3: "웃김(즐거움)",          # delight.png
    4: "기쁨(뿌듯함)",          # delight.png
    5: "슬픔",                  # sad.png
    6: "놀람",                  # surprise.png
    7: "매우놀람",              # surprise.png
    8: "실망",                  # disappointment.png
    9: "사랑",                  # love.png
    10: "어지러움",             # dizzy.png
    11: "아주 어지러움",        # dizzy.png
    # "low-dattery": 12,
}

def parse_emotion_from_response(response: str):
    """
    '[감정] : <감정>' 패턴에서 감정 키워드 추출
    """
    match = re.search(r"\[감정\]\s*:\s*([a-zA-Z_]+)", response)
    if match:
        return match.group(1).strip().lower()
    return None

def get_emotion_image_path(emotion: str):
    """
    감정 키워드 → 이미지 파일 경로 반환 (없으면 기본값)
    """
    filename = EMOTION_IMAGE_MAP.get(emotion)
    if filename:
        return f"/static/face/{filename}"
    # fallback: 기본 neutral 이미지
    return "/static/face/expressionless.png"

def get_action_index_from_emotion(emotion: str):
    """
    감정 키워드(영문/한글) → action_index 반환 (없으면 None)
    """
    if not emotion:
        return None
    # 1. 영문 감정명 → action_index
    emotion = emotion.strip().lower()
    # 영문 매핑
    EN_EMOTION_TO_INDEX = {
        "curious": 1,
        "sleepy": 2,
        "delight": 3,  # delight는 3,4 모두 매핑 가능하나 우선 3
        "sad": 5,
        "surprise": 6,  # surprise는 6,7 모두 매핑 가능하나 우선 6
        "disappointment": 8,
        "love": 9,
        "dizzy": 10,
        "expressionless": None,  # 표정없음은 action_index 없음
    }
    if emotion in EN_EMOTION_TO_INDEX:
        return EN_EMOTION_TO_INDEX[emotion]
    # 2. 한글 감정명 → action_index
    for idx, kor in emotions.items():
        if emotion in kor or kor in emotion:
            return idx
    # 3. 기타(매핑 실패)
    return None

class LLMService:
    def __init__(self):
        # OpenAI ChatGPT 모델 초기화
        self.llm = ChatOpenAI(
            model_name="gpt-4.1-mini",
            temperature=0.1,
            openai_api_key=os.getenv("OPENAI_API_KEY")
        )
        
        # edie_qwen2.5_0.5b_q4_k_m:latest 
        # edie_qwen2.5_1.5b_q4_k_m:latest 
        # edie_qwen2.5_1.5b_q4_0:latest 
        
        # model_name = 'edie_qwen2.5_1.5b_q4_k_m:latest '
        # self.llm = ChatOllama(
        #     model=model_name,
        #     temperature=0.1
        # )
        
        # chats 디렉토리 생성
        self.chats_dir = "./chats"
        os.makedirs(self.chats_dir, exist_ok=True)
        
        # 데이터베이스 경로 설정
        self.db_path = os.path.join(self.chats_dir, "chat_history.db")
        print("self.db_path :",self.db_path)
        
        # 데이터베이스 매니저 초기화
        self.db_manager = DatabaseManager(self.db_path)
        
        # ToolBox 초기화 (자동으로 모든 도구들 로드됨)
        self.toolbox = ToolBox()
        # self.toolbox.add_packages([action_tool, expression_tool])   # calculation_tool
        # self.toolbox.add_packages([action_tool, calculation_tool])   # calculation_tool
        self.toolbox.add_packages([action_tool, expression_tool])   # calculation_tool
        self.tools = self.toolbox.get_tools()
        self.prompt = edie_agent_prompt
        
        # Agent 초기화
        self.agent = self.init_agent()
        self.executor = self.init_executor()
        
        # ROS2 서비스 연동
        try:
            from .ros2_service import ros2_publisher
            self.ros2_service = ros2_publisher
            print("✅ ROS2 서비스 연동 완료")
        except Exception as e:
            print(f"⚠️ ROS2 서비스 연동 실패: {e}")
            self.ros2_service = None
        
        # 현재 세션 ID
        self.current_session_id = 'default'
        
        print(f"✅ LLM Agent initialized with {len(self.tools)} tools")
        # for tool in self.tools:
        #     print(f"  - {tool.name}: {tool.description}")
    
    def init_agent(self):
        """간단한 LLM Agent를 초기화합니다."""
        agent = create_tool_calling_agent(self.llm, self.tools, self.prompt)
        return agent
    
    def init_executor(self):
        """AgentExecutor를 초기화합니다."""
        executor = AgentExecutor(
            agent=self.agent,
            tools=self.tools,
            verbose=True,
            return_intermediate_steps=True,
            handle_parsing_errors=True,
            max_iterations=10,
        )
        return executor
    
    def get_chat_history(self, session_id: str):
        """세션 기록을 가져오는 함수"""
        return SQLChatMessageHistory(
            table_name='chat_messages',
            session_id=session_id,
            connection=f"sqlite:///{self.db_path}",  # 새로운 데이터베이스 경로 사용
        )
    
    async def generate_response(self, user_message: str, session_id: str = "default") -> str:
        """사용자 메시지에 대한 AI 응답을 생성합니다 (Agent 사용)."""
        try:
            # 세션 ID 업데이트
            if session_id:
                self.current_session_id = session_id
            
            print(f"[대화 세션ID]: {self.current_session_id}")
            print(f"[데이터베이스 경로]: {self.db_path}")
            
            # RunnableWithMessageHistory를 사용한 대화형 Agent
            conversational_agent = RunnableWithMessageHistory(      
                self.executor,                              # AgentExecutor 사용
                self.get_chat_history,                      # 세션 기록을 가져오는 함수
                input_messages_key="input",                 # 입력 메시지의 키
                history_messages_key="chat_history",        # 기록 메시지의 키
            )
            
            # Agent 실행
            result = await asyncio.to_thread(
                conversational_agent.invoke,
                {"input": user_message},
                {"configurable": {"session_id": self.current_session_id}}
            )
            
            # Agent 결과에서 최종 응답 추출
            response = result.get("output", "응답을 생성할 수 없습니다.")
            
            # 중간 단계 로깅 (도구 사용 내역)
            intermediate_steps = result.get("intermediate_steps", [])
            if intermediate_steps:
                print(f"[도구 사용 내역]:")
                for i, (action, observation) in enumerate(intermediate_steps):
                    print(f"  {i+1}. {action.tool}: {action.tool_input}")
                    print(f"     결과: {observation}")
            else:
                # 도구 미사용 시 감정 파싱 및 자동 표현
                emotion = parse_emotion_from_response(response)
                action_index = get_action_index_from_emotion(emotion)
                expression_result = ""
                if action_index:
                    try:
                        # expression_tool.call_expression_action은 toolbox에 등록되어 있으므로 직접 호출
                        expression_result = self.toolbox.get_tool("call_expression_action").run({"action_index": action_index})
                    except Exception as e:
                        expression_result = f"감정 퍼블리시 실패: {e}"
                else:
                    expression_result = "감정 매핑 실패"
                image_path = get_emotion_image_path(emotion)
                # 응답 포맷 확장
                response = f"{response}\n[감정이미지]: {image_path}\n[표현결과]: {expression_result}"
            
            # 데이터베이스에 대화 내용 저장
            await asyncio.to_thread(
                self.db_manager.save_conversation,
                self.current_session_id,
                user_message,
                response
            )
            
            # ROS2 토픽으로 LLM 응답 발행
            if self.ros2_service:
                try:
                    success = self.ros2_service.publish_llm_response(response)
                    if success:
                        print(f"📤 LLM 응답이 ROS2 토픽으로 발행됨: /edie8/llm/output")
                    else:
                        print(f"⚠️ ROS2 토픽 발행 실패")
                except Exception as ros_error:
                    print(f"⚠️ ROS2 토픽 발행 중 오류: {ros_error}")
            
            return response
            
        except Exception as e:
            print(f"Error generating response: {e}")
            error_message = "죄송합니다. 응답을 생성하는 중에 오류가 발생했습니다."
            
            # 에러도 기록
            try:
                await asyncio.to_thread(
                    self.db_manager.save_conversation,
                    self.current_session_id,
                    user_message,
                    error_message
                )
            except Exception as db_error:
                print(f"Error saving to database: {db_error}")
            
            return error_message
    
    def clear_history(self, session_id: str = "default"):
        """특정 세션의 대화 히스토리를 초기화합니다."""
        try:
            # SQLChatMessageHistory 초기화
            chat_history = self.get_chat_history(session_id)
            chat_history.clear()
            
            # 커스텀 데이터베이스에서도 삭제
            self.db_manager.clear_session_history(session_id)
            
            print(f"Session {session_id} history cleared.")
        except Exception as e:
            print(f"Error clearing history: {e}")
    
    def get_history(self, session_id: str = "default", limit: int = 10):
        """특정 세션의 대화 히스토리를 반환합니다."""
        try:
            return self.db_manager.get_conversation_history(session_id, limit)
        except Exception as e:
            print(f"Error getting history: {e}")
            return []
    
    def get_all_sessions(self):
        """모든 세션 정보를 반환합니다."""
        try:
            return self.db_manager.get_all_sessions()
        except Exception as e:
            print(f"Error getting sessions: {e}")
            return []
    
    def get_database_info(self):
        """데이터베이스 정보를 반환합니다."""
        return self.db_manager.get_database_info()
