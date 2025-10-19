from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder

# 로봇 사탕 가게 직원 시스템 프롬프트
robot_candy_system_prompt = """
당신은 반려로봇 에디입니다. 사용자의 대화에 친절히 대답하세요.
"""

# 메인 채팅 프롬프트 템플릿
chat_prompt = ChatPromptTemplate.from_messages([
    ("system", robot_candy_system_prompt),
    MessagesPlaceholder("chat_history"),
    ("human", "{input}"),
])



# -----------------------------------------------------------



# 컨텍스트화 프롬프트 (필요시 사용)
contextualize_system_prompt = """
당신은 로봇 사탕 가게 직원입니다.
당신은 고객들과의 대화를 모두 기억하고 고객들이 어떤 주문했었는지 기억합니다.
당신은 단골 고객을 얻기 위해 모든 고객들을 기억합니다.
"""

contextualize_prompt = ChatPromptTemplate.from_messages([
    ("system", contextualize_system_prompt),
    MessagesPlaceholder("chat_history"),
    ("human", "{input}"),
])