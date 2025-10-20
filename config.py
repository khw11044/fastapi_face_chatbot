import os
import warnings
import logging
import sys

def setup_environment():
    """
    애플리케이션 시작 전 환경 설정 및 경고 억제
    """
    # TensorFlow 로그 레벨 설정 (오류만 표시)
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
    
    # CUDA 관련 경고 억제
    os.environ['CUDA_VISIBLE_DEVICES'] = '0'  # GPU 사용하는 경우
    os.environ['TF_FORCE_GPU_ALLOW_GROWTH'] = 'true'
    
    # Transformers 캐시 디렉토리 설정 (선택사항)
    # os.environ['TRANSFORMERS_CACHE'] = './model_cache'
    
    # Python 경고 필터 설정
    warnings.filterwarnings('ignore', category=FutureWarning)
    warnings.filterwarnings('ignore', category=UserWarning)
    warnings.filterwarnings('ignore', category=DeprecationWarning)
    
    # 로깅 설정
    logging.getLogger('tensorflow').setLevel(logging.ERROR)
    logging.getLogger('transformers').setLevel(logging.ERROR)
    logging.getLogger('huggingface_hub').setLevel(logging.ERROR)
    logging.getLogger('chromadb').setLevel(logging.ERROR)
    
    # MediaPipe 로그 억제
    os.environ['GLOG_minloglevel'] = '3'
    
    print("✅ 환경 설정 완료 - 경고 메시지 억제됨")

def check_model_cache():
    """
    모델 캐시 상태 확인
    """
    cache_dirs = [
        os.path.expanduser("~/.cache/huggingface"),
        os.path.expanduser("~/.cache/torch"),
        "./model_cache"
    ]
    
    total_cache_size = 0
    for cache_dir in cache_dirs:
        if os.path.exists(cache_dir):
            for root, dirs, files in os.walk(cache_dir):
                total_cache_size += sum(os.path.getsize(os.path.join(root, file)) for file in files)
    
    if total_cache_size > 0:
        cache_size_mb = total_cache_size / (1024 * 1024)
        print(f"📦 모델 캐시 크기: {cache_size_mb:.1f} MB")
    else:
        print("📦 모델 캐시 없음 - 첫 실행 시 다운로드됩니다")

def print_startup_info():
    """
    시작 정보 출력
    """
    print("=" * 50)
    print("🚀 FastAPI 에디 대화 시작")
    print("=" * 50)
    
    # Python 버전 정보
    print(f"🐍 Python 버전: {sys.version.split()[0]}")
    
    # GPU 정보 (선택사항)
    try:
        import torch
        if torch.cuda.is_available():
            print(f"🎮 GPU: {torch.cuda.get_device_name(0)}")
        else:
            print("💻 CPU 모드로 실행")
    except ImportError:
        print("💻 CPU 모드로 실행")
    
    check_model_cache()
    print("-" * 50)

if __name__ == "__main__":
    setup_environment()
    print_startup_info()