/**
 * 레이저 센서 웹소켓 클라이언트
 * 실시간으로 센서 값을 받아 표에 표시
 */

class SensorWebSocketClient {
    constructor() {
        this.ws = null;
        this.isConnected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectDelay = 3000; // 3초
    }

    connect() {
        try {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.host}/sensor/ws/laser-sensors`;
            
            console.log(`🔌 Connecting to WebSocket: ${wsUrl}`);
            
            this.ws = new WebSocket(wsUrl);
            
            this.ws.onopen = () => this.onOpen();
            this.ws.onmessage = (event) => this.onMessage(event);
            this.ws.onerror = (event) => this.onError(event);
            this.ws.onclose = () => this.onClose();
            
        } catch (error) {
            console.error('❌ WebSocket connection error:', error);
            this.reconnect();
        }
    }

    onOpen() {
        console.log('✅ WebSocket connected');
        this.isConnected = true;
        this.reconnectAttempts = 0;
    }

    onMessage(event) {
        try {
            const data = JSON.parse(event.data);
            console.log('📊 [WebSocket Received]', data);
            this.updateSensorDisplay(data);
        } catch (error) {
            console.error('❌ Error parsing sensor data:', error);
        }
    }

    onError(event) {
        console.error('❌ WebSocket error:', event);
    }

    onClose() {
        console.log('⚠️ WebSocket disconnected');
        this.isConnected = false;
        this.reconnect();
    }

    updateSensorDisplay(sensorData) {
        try {
            // 터치맵 테이블 업데이트
            const touchMapRows = document.querySelectorAll('.touch-map-table tbody tr');
            if (touchMapRows.length >= 4) {
                // 1행 데이터 (hip과 back)
                const row1Data = touchMapRows[1].querySelectorAll('td');
                if (row1Data.length >= 6) {
                    row1Data[0].textContent = sensorData.left_hip;
                    row1Data[1].textContent = sensorData.middle_hip;
                    row1Data[2].textContent = sensorData.right_hip;
                    row1Data[3].textContent = sensorData.left_back;
                    row1Data[4].textContent = sensorData.middle_back;
                    row1Data[5].textContent = sensorData.right_back;
                }
                
                // 3행 데이터 (head와 cheek, temple)
                const row3Data = touchMapRows[3].querySelectorAll('td');
                if (row3Data.length >= 5) {
                    row3Data[0].textContent = sensorData.head;
                    row3Data[1].textContent = sensorData.left_cheek;
                    row3Data[2].textContent = sensorData.right_cheek;
                    row3Data[3].textContent = sensorData.left_temple;
                    row3Data[4].textContent = sensorData.right_temple;
                }
            }
            
            // 센서 값에 따른 색상 변경 함수
            const updateOvalColor = (elementId, sensorValue) => {
                const element = document.getElementById(elementId);
                if (element) {
                    element.style.backgroundColor = sensorValue >= 1 ? '#ff0000' : '#00ff00';
                }
            };
            
            // 모든 센서에 대한 색상 업데이트
            // Head 센서 (2개 타원)
            updateOvalColor('green-oval-head', sensorData.head);
            updateOvalColor('ellipse-oval-head', sensorData.head);
            
            // Hip 센서들
            updateOvalColor('ellipse-oval-left-hip', sensorData.left_hip);
            updateOvalColor('ellipse-oval-middle-hip', sensorData.middle_hip);
            updateOvalColor('ellipse-oval-right-hip', sensorData.right_hip);
            
            // Back 센서들
            updateOvalColor('ellipse-oval-left-back', sensorData.left_back);
            updateOvalColor('ellipse-oval-middle-back', sensorData.middle_back);
            updateOvalColor('ellipse-oval-right-back', sensorData.right_back);
            
            // Cheek/Temple 센서들 (circle의 초록색 타원들)
            updateOvalColor('green-oval-left-cheek', sensorData.left_cheek);
            updateOvalColor('green-oval-right-cheek', sensorData.right_cheek);
            updateOvalColor('green-oval-left-temple', sensorData.left_temple);
            updateOvalColor('green-oval-right-temple', sensorData.right_temple);
            
            // 레이저 테이블 업데이트
            const laserRows = document.querySelectorAll('.laser-table tbody tr');
            if (laserRows.length >= 2) {
                const laserCells = laserRows[1].querySelectorAll('td');
                if (laserCells.length >= 4) {
                    laserCells[0].textContent = Math.round(sensorData.front_left);
                    laserCells[1].textContent = Math.round(sensorData.front_right);
                    laserCells[2].textContent = Math.round(sensorData.bottom_left);
                    laserCells[3].textContent = Math.round(sensorData.bottom_right);
                }
            }
        } catch (error) {
            console.error('❌ Error updating sensor display:', error);
        }
    }

    reconnect() {
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;
            console.log(`🔄 Reconnecting... (Attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts})`);
            
            setTimeout(() => {
                this.connect();
            }, this.reconnectDelay);
        } else {
            console.error('❌ Max reconnection attempts reached');
        }
    }

    disconnect() {
        if (this.ws) {
            this.ws.close();
        }
    }
}

// 페이지 로드 시 웹소켓 클라이언트 초기화
document.addEventListener('DOMContentLoaded', () => {
    const sensorClient = new SensorWebSocketClient();
    sensorClient.connect();
});
