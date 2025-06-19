// 카메라 스트리밍 관련 기능
class CameraStreaming {
    constructor() {
        this.cameraActive = false;
        this.initElements();
        this.bindEvents();
        this.checkCameraStatus();
    }

    initElements() {
        this.startCameraBtn = document.getElementById('start-camera-btn');
        this.stopCameraBtn = document.getElementById('stop-camera-btn');
        this.cameraStream = document.getElementById('camera-stream');
        this.cameraPlaceholder = document.getElementById('camera-placeholder');
        this.cameraError = document.getElementById('camera-error');
        this.cameraStatusText = document.getElementById('camera-status-text');
        this.cameraStatusIndicator = document.getElementById('camera-status-indicator');
    }

    bindEvents() {
        this.startCameraBtn.addEventListener('click', () => this.startCamera());
        this.stopCameraBtn.addEventListener('click', () => this.stopCamera());
        
        // 페이지를 떠날 때 카메라 리소스 정리
        window.addEventListener('beforeunload', () => {
            if (this.cameraActive) {
                this.stopCamera();
            }
        });
    }

    // 카메라 시작 함수
    async startCamera() {
        try {
            this.startCameraBtn.disabled = true;
            this.updateCameraStatus('연결 중...', 'connecting');
            
            const response = await fetch('/camera/start', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                }
            });

            if (response.ok) {
                // 카메라 스트림 시작
                this.cameraStream.src = '/camera/stream?' + new Date().getTime();
                this.cameraStream.style.display = 'block';
                this.cameraPlaceholder.style.display = 'none';
                this.cameraError.style.display = 'none';
                this.hideBlackScreen();
                
                this.cameraActive = true;
                this.updateCameraStatus('온라인', 'online');
                
                // 카메라 스트림 로드 이벤트
                this.cameraStream.onload = () => {
                    this.updateCameraStatus('스트리밍 중', 'online');
                };
                
                this.cameraStream.onerror = () => {
                    this.showCameraError('스트림 오류가 발생했습니다.');
                };
                
            } else {
                const data = await response.json();
                this.showCameraError(data.detail || '카메라를 시작할 수 없습니다.');
            }
        } catch (error) {
            console.error('카메라 시작 오류:', error);
            this.showCameraError('카메라 연결에 실패했습니다.');
        } finally {
            this.startCameraBtn.disabled = false;
        }
    }

    // 카메라 중지 함수 - 검은 화면으로 변경
    async stopCamera() {
        try {
            this.stopCameraBtn.disabled = true;
            this.updateCameraStatus('중지 중...', 'connecting');
            
            const response = await fetch('/camera/stop', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                }
            });

            if (response.ok) {
                // 카메라 스트림 완전히 중지
                this.cameraStream.src = '';
                this.cameraStream.style.display = 'none';
                
                // 모든 표시 요소 숨기기
                this.cameraPlaceholder.style.display = 'none';
                this.cameraError.style.display = 'none';
                
                // 검은 화면 생성 또는 표시
                this.showBlackScreen();
                
                this.cameraActive = false;
                this.updateCameraStatus('오프라인', 'offline');
            } else {
                const data = await response.json();
                console.error('카메라 중지 오류:', data.detail);
                this.updateCameraStatus('중지 오류', 'offline');
            }
        } catch (error) {
            console.error('카메라 중지 오류:', error);
            this.updateCameraStatus('중지 오류', 'offline');
        } finally {
            this.stopCameraBtn.disabled = false;
        }
    }

    // 검은 화면 표시 함수
    showBlackScreen() {
        // 기존 검은 화면 요소가 있으면 제거
        const existingBlackScreen = document.getElementById('camera-black-screen');
        if (existingBlackScreen) {
            existingBlackScreen.remove();
        }

        // 새로운 검은 화면 요소 생성
        const blackScreen = document.createElement('div');
        blackScreen.id = 'camera-black-screen';
        blackScreen.className = 'camera-black-screen';
        blackScreen.innerHTML = `
            <div class="black-screen-content">
                <div class="camera-off-icon">📷</div>
                <p>카메라가 중지되었습니다</p>
            </div>
        `;

        // camera-view에 추가
        const cameraView = document.querySelector('.camera-view');
        cameraView.appendChild(blackScreen);
    }

    // 검은 화면 숨기기 함수
    hideBlackScreen() {
        const blackScreen = document.getElementById('camera-black-screen');
        if (blackScreen) {
            blackScreen.style.display = 'none';
        }
    }

    // 카메라 상태 업데이트 함수
    updateCameraStatus(text, status) {
        this.cameraStatusText.textContent = text;
        this.cameraStatusIndicator.className = `status-indicator ${status}`;
    }

    // 카메라 오류 표시 함수
    showCameraError(message) {
        this.cameraStream.style.display = 'none';
        this.cameraPlaceholder.style.display = 'none';
        this.cameraError.style.display = 'flex';
        this.cameraError.querySelector('p').textContent = message;
        
        // 검은 화면 숨기기
        this.hideBlackScreen();
        
        this.cameraActive = false;
        this.updateCameraStatus('오류', 'offline');
    }

    // 카메라 상태 확인 함수
    async checkCameraStatus() {
        try {
            const response = await fetch('/camera/status');
            if (response.ok) {
                const data = await response.json();
                if (data.is_streaming && !this.cameraActive) {
                    // 서버에서는 스트리밍 중인데 클라이언트에서 비활성화된 경우
                    this.cameraStream.src = '/camera/stream?' + new Date().getTime();
                    this.cameraStream.style.display = 'block';
                    this.cameraPlaceholder.style.display = 'none';
                    this.cameraError.style.display = 'none';
                    this.hideBlackScreen();
                    
                    this.cameraActive = true;
                    this.updateCameraStatus('스트리밍 중', 'online');
                } else if (!data.is_streaming && !this.cameraActive) {
                    // 서버와 클라이언트 모두 비활성화된 경우 검은 화면 표시
                    this.showBlackScreen();
                    this.updateCameraStatus('오프라인', 'offline');
                }
            }
        } catch (error) {
            console.error('카메라 상태 확인 오류:', error);
        }
    }
}

// 카메라 스트리밍 인스턴스 생성 (전역 변수로 노출)
window.cameraStreaming = new CameraStreaming();