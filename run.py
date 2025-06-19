#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 환경 설정을 가장 먼저 실행
import sys
import os

# 프로젝트 루트를 Python 경로에 추가
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 환경 설정 (모든 import 이전에 실행)
from config import setup_environment, print_startup_info

setup_environment()
print_startup_info()

# FastAPI 앱 import (환경 설정 후)
import uvicorn
from app.main import app

if __name__ == "__main__":
    print("🌐 서버 시작 중...")
    
    # Uvicorn 설정 최적화
    uvicorn.run(
        "app.main:app", 
        host="0.0.0.0", 
        port=8000, 
        reload=True,
        log_level="info",  # 로그 레벨 설정
        access_log=False,  # 액세스 로그 비활성화 (성능 향상)
        # workers=1,  # 개발 모드에서는 1개 워커 사용
    )