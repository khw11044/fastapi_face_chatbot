from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
from .routers import chatbot, speech, camera
from .services.ros2_service import ros2_publisher
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
app.include_router(speech.router, prefix="/speech", tags=["Speech"])
app.include_router(camera.router, prefix="/camera", tags=["Camera"])

# 메인 페이지 서빙
@app.get("/", response_class=HTMLResponse)
async def get_index():
    with open("static/index.html", "r", encoding="utf-8") as f:
        return HTMLResponse(content=f.read(), status_code=200)

# 터치&레이저 페이지 서빙
@app.get("/touch-laser", response_class=HTMLResponse)
async def get_touch_laser():
    with open("static/touch_laser.html", "r", encoding="utf-8") as f:
        return HTMLResponse(content=f.read(), status_code=200)

# 앱 시작 시 초기화
@app.on_event("startup")
async def startup_event():
    # 필요한 디렉토리들 생성
    os.makedirs("./chats", exist_ok=True)
    print("Chat database directory initialized: ./chats")
    
    # ROS2 Publisher 초기화
    try:
        ros2_publisher.initialize()
    except Exception as e:
        print(f"⚠️ ROS2 initialization failed: {e}")
        print("⚠️ Continuing without ROS2 support")

# 앱 종료 시 정리
@app.on_event("shutdown")
async def shutdown_event():
    # ROS2 정리
    try:
        ros2_publisher.shutdown()
    except Exception as e:
        print(f"⚠️ ROS2 shutdown error: {e}")
