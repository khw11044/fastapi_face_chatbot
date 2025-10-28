#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 환경 설정을 가장 먼저 실행
import sys
import os
import subprocess

# ROS2 환경 자동 로드 (import 전에 실행)
ros2_setup_paths = [
    '/home/khw/ros2_ws/install/setup.bash',
    '/opt/ros/humble/setup.bash'
]

for setup_path in ros2_setup_paths:
    if os.path.exists(setup_path):
        try:
            # 현재 환경에 ROS2 setup 로드
            result = subprocess.run(
                ['bash', '-c', f'source {setup_path} && env'],
                capture_output=True,
                text=True
            )
            if result.returncode == 0:
                # 환경 변수 현재 프로세스에 적용
                for line in result.stdout.split('\n'):
                    if '=' in line:
                        key, value = line.split('=', 1)
                        os.environ[key] = value
                print(f"✅ ROS2 setup loaded from {setup_path}")
                break
        except Exception as e:
            print(f"⚠️ Failed to load ROS2 setup from {setup_path}: {e}")

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
