// 메인 스크립트 - 페이지 로드 시 모든 모듈 초기화
document.addEventListener('DOMContentLoaded', function() {
    console.log('FastAPI Face Chatbot 애플리케이션 시작');
    
    // 카메라 스트리밍과 챗봇이 이미 각각의 파일에서 초기화됨
    // window.cameraStreaming과 window.chatBot으로 접근 가능
    
    // 전역 오류 처리
    window.addEventListener('error', function(e) {
        console.error('전역 오류 발생:', e.error);
    });
    
    // 디버그 정보 출력
    console.log('카메라 스트리밍 인스턴스:', window.cameraStreaming);
    console.log('챗봇 인스턴스:', window.chatBot);
    
    // 개발자 모드에서 사용할 수 있는 유틸리티 함수들
    window.debugUtils = {
        getCameraStatus: () => window.cameraStreaming?.cameraActive,
        getChatStatus: () => window.chatBot?.currentUserId,
        restartCamera: () => {
            if (window.cameraStreaming) {
                window.cameraStreaming.stopCamera().then(() => {
                    setTimeout(() => window.cameraStreaming.startCamera(), 1000);
                });
            }
        },
        clearAllData: () => {
            if (window.chatBot) {
                window.chatBot.logoutUser();
            }
            if (window.cameraStreaming && window.cameraStreaming.cameraActive) {
                window.cameraStreaming.stopCamera();
            }
        }
    };
    
    // 배터리 WebSocket 연결
    initBatteryWebSocket();
});

// 배터리 WebSocket 관리
let batteryWebSocket = null;

function initBatteryWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/sensor/ws/battery`;
    
    batteryWebSocket = new WebSocket(wsUrl);
    
    batteryWebSocket.onopen = () => {
        console.log('✅ Battery WebSocket connected');
    };
    
    batteryWebSocket.onmessage = (event) => {
        const data = JSON.parse(event.data);
        updateBatteryUI(data.percentage, data.voltage);
    };
    
    batteryWebSocket.onerror = (error) => {
        console.error('❌ Battery WebSocket error:', error);
    };
    
    batteryWebSocket.onclose = () => {
        console.log('⚠️ Battery WebSocket disconnected. Reconnecting in 5 seconds...');
        setTimeout(initBatteryWebSocket, 5000);
    };
}

function updateBatteryUI(percentage, voltage) {
    const batteryFill = document.getElementById('battery-fill');
    const batteryText = document.getElementById('battery-text');
    
    if (batteryFill && batteryText) {
        batteryFill.style.width = `${percentage}%`;
        batteryText.textContent = `${percentage.toFixed(1)}%`;
        
        // 배터리 잔량에 따라 색상 변경
        if (percentage < 20) {
            batteryFill.style.background = 'linear-gradient(90deg, #f44336, #e57373)'; // 빨강
        } else if (percentage < 50) {
            batteryFill.style.background = 'linear-gradient(90deg, #ff9800, #ffb74d)'; // 주황
        } else {
            batteryFill.style.background = 'linear-gradient(90deg, #4CAF50, #8BC34A)'; // 초록
        }
        
        console.log(`🔋 Battery: ${percentage.toFixed(1)}% (${voltage.toFixed(2)}V)`);
    }
}
