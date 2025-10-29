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
    
    // 감정 통계 WebSocket 연결
    initEmotionStatsWebSocket();
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

// 감정 통계 WebSocket 관리
let emotionStatsWebSocket = null;

function initEmotionStatsWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/sensor/ws/emotion-stats`;
    
    emotionStatsWebSocket = new WebSocket(wsUrl);
    
    emotionStatsWebSocket.onopen = () => {
        console.log('✅ Emotion stats WebSocket connected');
    };
    
    emotionStatsWebSocket.onmessage = (event) => {
        const emotionPercentages = JSON.parse(event.data);
        updateEmotionVisualization(emotionPercentages);
    };
    
    emotionStatsWebSocket.onerror = (error) => {
        console.error('❌ Emotion stats WebSocket error:', error);
    };
    
    emotionStatsWebSocket.onclose = () => {
        console.log('⚠️ Emotion stats WebSocket disconnected. Reconnecting in 5 seconds...');
        setTimeout(initEmotionStatsWebSocket, 5000);
    };
}

function updateEmotionVisualization(percentages) {
    // 중심 좌표
    const centerX = 160;
    const centerY = 160;
    
    // 팔각형 꼭지점 (가장 큰 팔각형 기준)
    const vertices = {
        curiosity: { x: 160, y: 52 },       // 상단 (궁금함)
        surprise: { x: 83.6, y: 83.6 },     // 좌상단 (놀람)
        sleepiness: { x: 52, y: 160 },      // 좌측 (졸림)
        sadness: { x: 83.6, y: 236.4 },     // 좌하단 (슬픔)
        love: { x: 160, y: 268 },           // 하단 (사랑)
        dizziness: { x: 236.4, y: 236.4 },  // 우하단 (어지러움)
        disappointment: { x: 268, y: 160 }, // 우측 (실망)
        happiness: { x: 236.4, y: 83.6 },   // 우상단 (기쁨)
    };
    
    // 감정 → 라벨 한글 매핑
    const emotionLabels = {
        curiosity: "궁금함",
        happiness: "기쁨",
        disappointment: "실망",
        dizziness: "어지러움",
        love: "사랑",
        sadness: "슬픔",
        sleepiness: "졸림",
        surprise: "놀람"
    };
    
    // 1. 최댓값 찾기
    const maxPercentage = Math.max(...Object.values(percentages));
    const maxEmotion = Object.keys(percentages).reduce((a, b) => 
        percentages[a] > percentages[b] ? a : b
    );
    
    // 2. 각 감정별 좌표 계산 (정규화 + 스케일링)
    const points = [];
    const emotionOrder = ['curiosity', 'happiness', 'disappointment', 'dizziness', 'love', 'sadness', 'sleepiness', 'surprise'];
    
    for (const emotion of emotionOrder) {
        const vertex = vertices[emotion];
        const percentage = percentages[emotion] || 0;
        
        // 정규화 (최댓값 기준 0~100%)
        const normalizedPercentage = maxPercentage > 0 ? (percentage / maxPercentage) * 100 : 0;
        
        // 스케일링 (26% ~ 100% 범위로 매핑, 가장 작은 팔각형이 0%)
        const scaledPercentage = 26 + (normalizedPercentage * 0.74);
        
        // point = center + (vertex - center) * (scaledPercentage / 100)
        const x = centerX + (vertex.x - centerX) * (scaledPercentage / 100);
        const y = centerY + (vertex.y - centerY) * (scaledPercentage / 100);
        
        points.push(`${x.toFixed(1)},${y.toFixed(1)}`);
    }
    
    // 3. SVG polygon 업데이트
    const svg = document.querySelector('.octagon-stack svg');
    let emotionPolygon = svg.querySelector('#emotion-polygon');
    
    if (!emotionPolygon) {
        // 감정 polygon 생성 (없으면)
        emotionPolygon = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
        emotionPolygon.id = 'emotion-polygon';
        emotionPolygon.setAttribute('fill', 'rgba(26, 127, 138, 0.3)');
        emotionPolygon.setAttribute('stroke', '#ff6b6b');
        emotionPolygon.setAttribute('stroke-width', '2');
        svg.appendChild(emotionPolygon);
    }
    
    emotionPolygon.setAttribute('points', points.join(' '));
    
    // 4. 최댓값 감정 라벨 강조 (보라색 네온)
    const maxLabel = emotionLabels[maxEmotion];
    svg.querySelectorAll('text').forEach(text => {
        if (text.textContent === maxLabel) {
            // 보라색 네온 효과
            text.setAttribute('fill', '#9d4edd');
            text.setAttribute('font-weight', 'bold');
            text.style.filter = 'drop-shadow(0 0 8px #c77dff) drop-shadow(0 0 12px #e0aaff)';
        } else {
            // 기본 색상
            text.setAttribute('fill', '#222');
            text.setAttribute('font-weight', 'normal');
            text.style.filter = 'none';
        }
    });
    
    console.log(`😊 Emotion stats updated (max: ${maxEmotion} ${maxPercentage.toFixed(1)}%):`, percentages);
}
