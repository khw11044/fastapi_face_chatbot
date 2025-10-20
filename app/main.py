from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
from .routers import chatbot
from .routers import ros2_ws
import os

app = FastAPI()

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Static files 설정
app.mount("/static", StaticFiles(directory="static"), name="static")

# 라우터 등록
app.include_router(chatbot.router, prefix="/chatbot", tags=["Chatbot"])
app.include_router(ros2_ws.router)

# 메인 페이지 서빙
@app.get("/", response_class=HTMLResponse)
async def get_index():
    with open("static/index.html", "r", encoding="utf-8") as f:
        return HTMLResponse(content=f.read(), status_code=200)

# 앱 시작 시 초기화
from app.services.ros2_subscriber import start_ros2_subscriber

@app.on_event("startup")
async def startup_event():
    # 필요한 디렉토리들 생성
    os.makedirs("./chats", exist_ok=True)
    print("Chat database directory initialized: ./chats")
    start_ros2_subscriber()

# 앱 종료 시 정리
@app.on_event("shutdown")
async def shutdown_event():
    pass
