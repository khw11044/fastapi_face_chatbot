import asyncio
import os
from langchain_openai import ChatOpenAI
from langchain_core.messages import HumanMessage, AIMessage
from langchain_core.runnables.history import RunnableWithMessageHistory
from langchain_community.chat_message_histories import SQLChatMessageHistory
from langchain_core.output_parsers import StrOutputParser
from langchain.agents import create_tool_calling_agent, AgentExecutor
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from typing import Dict, List
from dotenv import load_dotenv


from utils.prompts.prompt import edie_agent_prompt
from utils.databases.database import DatabaseManager
from utils.agent.toolbox import ToolBox
from utils.agent.toolbox import calculation as calculation_tool
from utils.agent.toolbox import action as action_tool
from utils.agent.toolbox import expression as expression_tool

load_dotenv()

class LLMService:
    def __init__(self):
        # OpenAI ChatGPT 모델 초기화
        self.llm = ChatOpenAI(
            model_name="gpt-4o-mini",
            temperature=0.7,
            openai_api_key=os.getenv("OPENAI_API_KEY")
        )
        
        # chats 디렉토리 생성
        self.chats_dir = "./chats"
        os.makedirs(self.chats_dir, exist_ok=True)
        
        # 데이터베이스 경로 설정
        self.db_path = os.path.join(self.chats_dir, "chat_history.db")
        
        # 데이터베이스 매니저 초기화
        self.db_manager = DatabaseManager(self.db_path)
        
        # ToolBox 초기화 (자동으로 모든 도구들 로드됨)
        self.toolbox = ToolBox()
        self.toolbox.add_packages([action_tool, expression_tool])   # calculation_tool
        self.tools = self.toolbox.get_tools()
        
        # Agent 초기화
        self.agent = self.init_agent()
        self.executor = self.init_executor()
        
        # 현재 세션 ID
        self.current_session_id = 'default'
        
        print(f"✅ LLM Agent initialized with {len(self.tools)} tools")
        for tool in self.tools:
            print(f"  - {tool.name}: {tool.description}")
    
    def init_agent(self):
        """간단한 LLM Agent를 초기화합니다."""
        agent = create_tool_calling_agent(self.llm, self.tools, edie_agent_prompt)
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
            
            # 데이터베이스에 대화 내용 저장
            await asyncio.to_thread(
                self.db_manager.save_conversation,
                self.current_session_id,
                user_message,
                response
            )
            
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
