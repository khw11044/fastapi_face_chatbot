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

    // ROI 바운딩 박스용 캔버스 및 WebSocket 초기화
    initRoiOverlay();

    // 사용자 감정 시각화 WebSocket 및 UI 초기화
    initUserEmotionVisualization();

    // 배터리 WebSocket 연결
    initBatteryWebSocket();
    
    // 감정 통계 WebSocket 연결
    initEmotionStatsWebSocket();
    
    // 데시벨 WebSocket 연결
    initDecibelWebSocket();
    
    // 녹음 토글 버튼 초기화
    initRecordToggle();
});

/** ROI 바운딩 박스 오버레이 및 WebSocket 초기화 */
function initRoiOverlay() {
    const cameraImg = document.getElementById('camera-stream');
    if (!cameraImg) {
        console.warn('카메라 이미지 요소를 찾을 수 없습니다.');
        return;
    }

    // 캔버스 생성 및 camera-viewer에 삽입
    let roiCanvas = document.getElementById('roi-canvas');
    if (!roiCanvas) {
        roiCanvas = document.createElement('canvas');
        roiCanvas.id = 'roi-canvas';
        roiCanvas.style.position = 'absolute';
        roiCanvas.style.left = '0';
        roiCanvas.style.top = '0';
        roiCanvas.style.pointerEvents = 'none';
        roiCanvas.style.zIndex = '10';
        // camera-viewer는 position:relative 여야 함
        const viewer = cameraImg.closest('.camera-viewer');
        if (viewer) {
            viewer.style.position = 'relative';
            viewer.appendChild(roiCanvas);
        } else {
            cameraImg.parentElement.appendChild(roiCanvas);
        }
    }

    // 캔버스 크기 동기화 함수
    function syncCanvasSize() {
        roiCanvas.width = cameraImg.clientWidth;
        roiCanvas.height = cameraImg.clientHeight;
        roiCanvas.style.width = cameraImg.clientWidth + 'px';
        roiCanvas.style.height = cameraImg.clientHeight + 'px';
    }

    // 이미지 로드/리사이즈 시 캔버스 크기 맞춤
    cameraImg.addEventListener('load', syncCanvasSize);
    window.addEventListener('resize', syncCanvasSize);
    setTimeout(syncCanvasSize, 500);

    // ROI WebSocket 연결
    let roiWebSocket = null;
    function connectRoiWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/camera/ws/roi`;
        roiWebSocket = new WebSocket(wsUrl);

        roiWebSocket.onopen = () => {
            console.log('✅ ROI WebSocket connected');
        };

        roiWebSocket.onmessage = (event) => {
            const roi = JSON.parse(event.data);
            drawRoiBox(roi);
        };

        roiWebSocket.onerror = (error) => {
            console.error('❌ ROI WebSocket error:', error);
        };

        roiWebSocket.onclose = () => {
            console.log('⚠️ ROI WebSocket disconnected. Reconnecting in 5 seconds...');
            setTimeout(connectRoiWebSocket, 5000);
        };
    }
    connectRoiWebSocket();

    // ROI 박스 그리기
    function drawRoiBox(roi) {
        syncCanvasSize();
        const ctx = roiCanvas.getContext('2d');
        ctx.clearRect(0, 0, roiCanvas.width, roiCanvas.height);

        if (!roi || !('x_offset' in roi) || !('y_offset' in roi) || !('width' in roi) || !('height' in roi)) {
            return;
        }

        // 이미지와 ROI 좌표가 동일 해상도라고 가정
        const scaleX = roiCanvas.width / cameraImg.naturalWidth;
        const scaleY = roiCanvas.height / cameraImg.naturalHeight;

        // naturalWidth/Height가 0이면(아직 이미지 로드 전) skip
        if (!cameraImg.naturalWidth || !cameraImg.naturalHeight) return;

        // 높이 1.25배 확대, y_offset도 위로 보정
        const newHeight = roi.height * 1.6;
        const h = newHeight * scaleY;
        const w = roi.width * scaleX;
        const x = roi.x_offset * scaleX;
        const y = (roi.y_offset - (newHeight - roi.height) / 2) * scaleY;

        ctx.save();
        ctx.strokeStyle = '#ff3b3b';
        ctx.lineWidth = 3;
        ctx.globalAlpha = 0.85;
        ctx.beginPath();
        ctx.rect(x, y, w, h);
        ctx.stroke();
        ctx.restore();
    }
}

/** 사용자 감정 시각화 WebSocket 및 UI */
function initUserEmotionVisualization() {
    let userEmotionWS = null;

    // WebSocket 연결 (user_id 없음)
    function connectUserEmotionWS() {
        if (userEmotionWS) {
            userEmotionWS.close();
            userEmotionWS = null;
        }
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/sensor/ws/user-emotion`;
        userEmotionWS = new WebSocket(wsUrl);

        userEmotionWS.onopen = () => {
            console.log('✅ User emotion WebSocket connected');
        };
        userEmotionWS.onmessage = (event) => {
            const data = JSON.parse(event.data);
            console.log('[user-emotion WS] 수신:', data);
            if (data.error) return;
            updateUserEmotionUI(data.latest_emotion, data.histogram);
        };
        userEmotionWS.onerror = (e) => {
            console.error('❌ User emotion WebSocket error:', e);
        };
        userEmotionWS.onclose = () => {
            console.log('⚠️ User emotion WebSocket disconnected. Reconnecting in 5 seconds...');
            setTimeout(connectUserEmotionWS, 5000);
        };
    }

    // 네온 효과 및 막대 그래프 갱신
    function updateUserEmotionUI(latest, histogram) {
        // 네온 효과: bar-label에만 적용
        document.querySelectorAll('.user-emotion-bar-row').forEach(row => {
            const label = row.querySelector('.bar-label');
            if (row.dataset.emotion === latest) {
                label.classList.add('active-neon');
            } else {
                label.classList.remove('active-neon');
            }
        });
        // 막대 그래프
        if (histogram) {
            Object.entries(histogram).forEach(([emotion, percent]) => {
                const row = document.querySelector(`.user-emotion-bar-row[data-emotion="${emotion}"]`);
                if (row) {
                    const fill = row.querySelector('.bar-fill');
                    const value = row.querySelector('.bar-value');
                    fill.style.width = percent + '%';
                    value.textContent = percent + '%';
                }
            });
        }
    }

    // 페이지 로드 시 즉시 연결
    connectUserEmotionWS();

    // 대화 초기화 버튼 이벤트 (감정 히스토리 초기화)
    const clearBtn = document.getElementById('clear-btn');
    if (clearBtn) {
        clearBtn.addEventListener('click', async () => {
            try {
                await fetch('/chatbot/user/emotion/clear', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({})
                });
                // UI도 초기화
                updateUserEmotionUI(null, {
                    Anger: 0, Happiness: 0, Sadness: 0, Surprise: 0, Neutral: 0
                });
            } catch (e) {
                console.error('❌ 사용자 감정 초기화 실패:', e);
            }
        });
    }
}

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

/* 사용자 감정 네온 효과용 CSS 동적 삽입 */
(function injectUserEmotionNeonCSS() {
    const style = document.createElement('style');
    style.textContent = `
    .user-emotion-bar-row {
        display: flex;
        align-items: center;
        margin: 2px 0;
    }
    .user-emotion-bar-row .bar-label {
        width: 70px;
        font-size: 13px;
        margin-right: 6px;
        transition: all 0.3s;
    }
    .user-emotion-bar-row .bar-label.active-neon {
        color: #fff;
        font-weight: bold;
        text-shadow:
            0 0 8px #00e6ff,
            0 0 16px #00e6ff,
            0 0 24px #00e6ff,
            0 0 32px #00e6ff;
    }
    .user-emotion-bar-row .bar-bg {
        flex: 1;
        height: 16px;
        background: #222;
        border-radius: 8px;
        margin-right: 6px;
        overflow: hidden;
        position: relative;
    }
    .user-emotion-bar-row .bar-fill {
        height: 100%;
        background: linear-gradient(90deg, #00e6ff, #00ffb3);
        border-radius: 8px;
        transition: width 0.3s;
    }
    .user-emotion-bar-row .bar-value {
        width: 38px;
        text-align: right;
        font-size: 12px;
        color: #00e6ff;
    }
    `;
    document.head.appendChild(style);
})();

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
    
    // 모든 감정이 0%일 때는 "curiosity"를 기본값으로 설정
    let maxEmotion;
    if (maxPercentage === 0) {
        maxEmotion = "curiosity";
    } else {
        maxEmotion = Object.keys(percentages).reduce((a, b) => 
            percentages[a] > percentages[b] ? a : b
        );
    }
    
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

// 데시벨 WebSocket 관리
let decibelWebSocket = null;

function initDecibelWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/sensor/ws/decibel`;
    
    decibelWebSocket = new WebSocket(wsUrl);
    
    decibelWebSocket.onopen = () => {
        console.log('✅ Decibel WebSocket connected');
    };
    
    decibelWebSocket.onmessage = (event) => {
        const data = JSON.parse(event.data);
        updateDecibelUI(data.decibel);
    };
    
    decibelWebSocket.onerror = (error) => {
        console.error('❌ Decibel WebSocket error:', error);
    };
    
    decibelWebSocket.onclose = () => {
        console.log('⚠️ Decibel WebSocket disconnected. Reconnecting in 5 seconds...');
        setTimeout(initDecibelWebSocket, 5000);
    };
}

function updateDecibelUI(decibel) {
    const decibelBar = document.getElementById('decibel-bar');
    const decibelText = document.getElementById('decibel-text');
    
    if (decibelBar && decibelText) {
        // 0~120 dB 범위를 0~100%로 변환
        const percentage = Math.min((decibel / 120) * 100, 100);
        decibelBar.style.width = `${percentage}%`;
        decibelText.textContent = `${decibel.toFixed(1)} dB`;
        
        // 데시벨에 따라 색상 변경
        if (decibel < 60) {
            // 초록 (조용)
            decibelBar.style.background = 'linear-gradient(90deg, #4CAF50, #8BC34A)';
        } else if (decibel < 90) {
            // 노랑 (보통)
            decibelBar.style.background = 'linear-gradient(90deg, #FFC107, #FFD54F)';
        } else {
            // 빨강 (시끄러움)
            decibelBar.style.background = 'linear-gradient(90deg, #f44336, #e57373)';
        }
        
        // console.log(`🔊 Decibel: ${decibel.toFixed(1)} dB`);
    }
}

// 녹음 토글 버튼 관리
let isRecording = false;

function initRecordToggle() {
    const recordBtn = document.getElementById('record-toggle-btn');
    
    if (!recordBtn) {
        console.warn('⚠️ Record toggle button not found');
        return;
    }
    
    recordBtn.addEventListener('click', async () => {
        // 토글
        isRecording = !isRecording;
        
        try {
            // FastAPI로 전송
            const response = await fetch('/sensor/record-toggle', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ is_recording: isRecording })
            });
            
            const data = await response.json();
            
            if (data.success) {
                // UI 업데이트
                if (isRecording) {
                    recordBtn.classList.add('recording');
                    recordBtn.textContent = '녹음 중지';
                    console.log('🎤 녹음 시작');
                } else {
                    recordBtn.classList.remove('recording');
                    recordBtn.textContent = '녹음';
                    console.log('🎤 녹음 중지');
                }
            } else {
                console.error('❌ Record toggle failed:', data.message);
                // 실패 시 상태 원복
                isRecording = !isRecording;
            }
        } catch (error) {
            console.error('❌ Record toggle error:', error);
            // 오류 시 상태 원복
            isRecording = !isRecording;
        }
    });
}
