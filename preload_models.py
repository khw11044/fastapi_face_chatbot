def preload_simple_features():
    """간단한 특징 추출 테스트"""
    print("📥 OpenCV 기반 특징 추출 테스트 중...")
    try:
        import cv2
        import numpy as np
        
        # 더미 이미지로 테스트
        dummy_img = np.zeros((64, 64), dtype=np.uint8)
        
        # 히스토그램 계산 테스트
        hist = cv2.calcHist([dummy_img], [0], None, [256], [0, 256])
        
        # HOG 특징 테스트 (Sobel 필터)
        grad_x = cv2.Sobel(dummy_img, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(dummy_img, cv2.CV_64F, 0, 1, ksize=3)
        
        print("✅ OpenCV 특징 추출 준비 완료")
        return True
    except Exception as e:
        print(f"❌ OpenCV 특징 추출 테스트 실패: {e}")
        return False#!/usr/bin/env python3
"""
모델을 사전에 다운로드하고 캐시하는 스크립트
첫 실행 시에만 실행하면 이후 빠른 시작이 가능합니다.
"""

import os
import sys
import warnings
from config import setup_environment

# 환경 설정
setup_environment()

def preload_mediapipe():
    """MediaPipe 모델 사전 로딩"""
    print("📥 MediaPipe 얼굴 탐지 모델 다운로드 중...")
    try:
        import mediapipe as mp
        mp_face_detection = mp.solutions.face_detection
        face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.4)
        print("✅ MediaPipe 모델 캐시 완료")
        return True
    except Exception as e:
        print(f"❌ MediaPipe 로딩 실패: {e}")
        return False

def preload_imgbeddings():
    """imgbeddings 모델 사전 로딩"""
    print("📥 imgbeddings 모델 다운로드 중...")
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            from imgbeddings import imgbeddings
            ibed = imgbeddings()
            
            # 더미 이미지로 테스트
            from PIL import Image
            import numpy as np
            dummy_img = Image.fromarray(np.zeros((224, 224, 3), dtype=np.uint8))
            embedding = ibed.to_embeddings(dummy_img)
            print("✅ imgbeddings 모델 캐시 완료")
            return True
    except Exception as e:
        print(f"❌ imgbeddings 로딩 실패: {e}")
        return False

def preload_chromadb():
    """ChromaDB 초기화"""
    print("📥 ChromaDB 초기화 중...")
    try:
        import chromadb
        client = chromadb.PersistentClient("./faces")
        db = client.get_or_create_collection(
            name='facedb',
            metadata={"hnsw:space": 'cosine'},
        )
        print("✅ ChromaDB 초기화 완료")
        return True
    except Exception as e:
        print(f"❌ ChromaDB 초기화 실패: {e}")
        return False

def check_system_resources():
    """시스템 리소스 확인"""
    print("\n🔍 시스템 리소스 확인:")
    
    # GPU 확인
    try:
        import torch
        if torch.cuda.is_available():
            gpu_name = torch.cuda.get_device_name(0)
            gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1024**3
            print(f"  🎮 GPU: {gpu_name} ({gpu_memory:.1f}GB)")
        else:
            print("  💻 GPU 없음 - CPU 모드로 실행")
    except ImportError:
        print("  💻 PyTorch 없음 - CPU 모드로 실행")
    
    # 메모리 확인
    try:
        import psutil
        memory = psutil.virtual_memory()
        print(f"  🧠 RAM: {memory.total / 1024**3:.1f}GB (사용가능: {memory.available / 1024**3:.1f}GB)")
    except ImportError:
        print("  🧠 메모리 정보 확인 불가 (psutil 필요)")

def create_cache_directories():
    """필요한 캐시 디렉토리 생성"""
    print("\n📁 디렉토리 구조 생성 중...")
    
    directories = [
        "./faces",
        "./chats", 
        "./fonts",
        "./model_cache"
    ]
    
    for directory in directories:
        os.makedirs(directory, exist_ok=True)
        print(f"  📂 {directory}")
    
    print("✅ 디렉토리 구조 생성 완료")

def optimize_environment():
    """환경 최적화 설정"""
    print("\n⚙️  환경 최적화 설정:")
    
    # HuggingFace 캐시 디렉토리 설정
    cache_dir = os.path.abspath("./model_cache")
    os.environ['TRANSFORMERS_CACHE'] = cache_dir
    os.environ['HF_HOME'] = cache_dir
    print(f"  📦 모델 캐시: {cache_dir}")
    
    # CUDA 메모리 최적화
    os.environ['CUDA_LAUNCH_BLOCKING'] = '0'
    os.environ['TF_FORCE_GPU_ALLOW_GROWTH'] = 'true'
    print("  🎮 GPU 메모리 최적화 설정")
    
    # Python 최적화
    os.environ['PYTHONUNBUFFERED'] = '1'
    print("  🐍 Python 출력 버퍼링 비활성화")

def main():
    """메인 함수"""
    print("=" * 60)
    print("🚀 FastAPI 얼굴 인식 챗봇 - 모델 사전 로딩")
    print("=" * 60)
    
    # 시스템 리소스 확인
    check_system_resources()
    
    # 디렉토리 생성
    create_cache_directories()
    
    # 환경 최적화
    optimize_environment()
    
    print("\n📥 모델 다운로드 및 캐시 시작...")
    print("=" * 40)
    
    success_count = 0
    total_count = 3
    
    # 1. MediaPipe 모델 로딩
    if preload_mediapipe():
        success_count += 1
    
    # 2. imgbeddings 모델 로딩  
    if preload_imgbeddings():
        success_count += 1
    
    # 3. ChromaDB 초기화
    if preload_chromadb():
        success_count += 1
    
    print("\n" + "=" * 60)
    print(f"📊 결과: {success_count}/{total_count} 모델 로딩 완료")
    
    if success_count == total_count:
        print("🎉 모든 모델이 성공적으로 캐시되었습니다!")
        print("💡 이제 'python run.py'를 실행하면 빠르게 시작됩니다.")
    else:
        print("⚠️  일부 모델 로딩에 실패했습니다.")
        print("🔧 네트워크 연결을 확인하고 다시 시도해보세요.")
    
    print("\n📈 다음 실행 시 예상 시작 시간: 3-5초")
    print("=" * 60)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n❌ 사용자에 의해 중단되었습니다.")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n💥 예상치 못한 오류: {e}")
        sys.exit(1)