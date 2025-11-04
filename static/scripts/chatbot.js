// 챗봇 관련 기능
class ChatBot {
    constructor() {
        this.isLoading = false;
        this.currentUserId = null;
        this.sessionId = null;
        this.initElements();
        this.bindEvents();
        this.init();
    }

    initElements() {
        this.chatBox = document.getElementById('chat-box');
        this.userInput = document.getElementById('user-input');
        this.sendButton = document.getElementById('send-btn');
        this.clearButton = document.getElementById('clear-btn');
        
        // 사용자 ID 관련 요소들
        this.userIdInput = document.getElementById('user-id');
        this.loginButton = document.getElementById('login-btn');
        this.currentUserDiv = document.getElementById('current-user');
        this.currentUserName = document.getElementById('current-user-name');
        this.logoutButton = document.getElementById('logout-btn');
        
        // 음성 인식 관련 요소
        this.micButton = document.getElementById('mic-btn');
        this.mediaRecorder = null;
        this.audioChunks = [];
        this.isRecording = false;
        this.silenceTimer = null;

        // STT 모드 토글 요소 및 상태
        this.sttToggle = document.getElementById('stt-toggle'); // 체크박스 등
        this.isSttMode = this.sttToggle ? this.sttToggle.checked : false;
    }

    bindEvents() {
        this.sendButton.addEventListener('click', () => this.sendMessage());
        this.clearButton.addEventListener('click', () => this.clearChat());
        this.loginButton.addEventListener('click', () => this.loginUser());
        this.logoutButton.addEventListener('click', () => this.logoutUser());
        this.micButton.addEventListener('click', () => this.toggleRecording());
        
        this.userInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter' && !e.shiftKey) {
                e.preventDefault();
                this.sendMessage();
            }
        });

        this.userIdInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') {
                e.preventDefault();
                this.loginUser();
            }
        });

        // STT 모드 토글 이벤트
        if (this.sttToggle) {
            this.sttToggle.addEventListener('change', (e) => {
                this.isSttMode = e.target.checked;
                
                // STT 모드 활성화 시 텍스트 입력창 비활성화
                if (this.isSttMode) {
                    this.userInput.disabled = true;
                    this.userInput.placeholder = '🎤 STT 모드 활성화 (스페이스바로 음성 인식)';
                    this.userInput.style.backgroundColor = '#f8f9fa';
                } else {
                    // 로그인 상태에 따라 입력창 활성화
                    if (this.currentUserId) {
                        this.userInput.disabled = false;
                        this.userInput.placeholder = '메시지를 입력하세요...';
                        this.userInput.style.backgroundColor = '';
                    }
                }
            });
        }

        // 스페이스바 누르고 있는 동안만 STT (STT 모드일 때만)
        this.isSpaceRecording = false;
        window.addEventListener('keydown', async (e) => {
            if (
                this.isSttMode && // STT 모드가 켜진 경우에만
                e.code === 'Space' &&
                !this.isSpaceRecording &&
                !this.isRecording
            ) {
                // 입력창, 버튼 등 포커스가 있을 때만 동작 (원치 않는 오작동 방지)
                if (document.activeElement === this.userInput || document.activeElement === document.body) {
                    this.isSpaceRecording = true;
                    this.micButton.classList.add('stt-recording');
                    this.userInput.placeholder = '🎤 스페이스바 누르는 동안 인식 중...';
                    await this.startRecording();
                }
            }
        });
        window.addEventListener('keyup', (e) => {
            if (
                this.isSttMode && // STT 모드가 켜진 경우에만
                e.code === 'Space' &&
                this.isSpaceRecording
            ) {
                this.isSpaceRecording = false;
                this.micButton.classList.remove('stt-recording');
                this.stopRecording();
                this.userInput.placeholder = '메시지를 입력하세요...';
            }
        });
    }

    init() {
        // 초기 상태: 채팅 비활성화
        this.setChatDisabled(true);
        // 초기 포커스를 사용자 ID 입력창에
        this.userIdInput.focus();
    }

    // 사용자 로그인
    async loginUser() {
        const userId = this.userIdInput.value.trim();
        
        if (!userId) {
            alert('사용자 ID를 입력해주세요.');
            return;
        }
        
        if (userId.length < 2) {
            alert('사용자 ID는 2글자 이상이어야 합니다.');
            return;
        }

        // 로그인 처리
        this.currentUserId = userId;
        this.sessionId = `user_${userId}`;
        
        // UI 업데이트
        this.userIdInput.style.display = 'none';
        this.loginButton.style.display = 'none';
        this.currentUserDiv.style.display = 'flex';
        this.currentUserName.textContent = userId;
        
        // 채팅 활성화
        this.setChatDisabled(false);
        
        // 이전 대화 기록 불러오기
        await this.loadChatHistory();
        
        this.userInput.focus();
    }

    // 사용자 로그아웃
    logoutUser() {
        this.currentUserId = null;
        this.sessionId = null;
        
        // UI 초기화
        this.userIdInput.style.display = 'block';
        this.loginButton.style.display = 'block';
        this.userIdInput.value = '';
        this.currentUserDiv.style.display = 'none';
        
        // 채팅 비활성화 및 초기화
        this.setChatDisabled(true);
        this.chatBox.innerHTML = `
            <div class="message bot">
                안녕하세요! 저는 에디입니다. 🤖
            </div>
        `;
        
        this.userIdInput.focus();
    }

    // 채팅 활성화/비활성화
    setChatDisabled(disabled) {
        const chatInputDiv = document.querySelector('.chat-input');
        if (disabled) {
            chatInputDiv.classList.add('chat-disabled');
            this.userInput.disabled = true;
            this.sendButton.disabled = true;
        } else {
            chatInputDiv.classList.remove('chat-disabled');
            this.userInput.disabled = false;
            this.sendButton.disabled = false;
        }
    }

    // 이전 대화 기록 불러오기
    async loadChatHistory() {
        if (!this.sessionId) return;
        
        try {
            const response = await fetch(`/chatbot/history/${this.sessionId}`);
            if (response.ok) {
                const data = await response.json();
                const history = data.history;
                
                if (history && history.length > 0) {
                    // 채팅박스 초기화 (환영 메시지 제거)
                    this.chatBox.innerHTML = '';
                    
                    // 최근 10개 대화만 표시 (역순으로 정렬되어 있으므로 reverse)
                    history.reverse().forEach(([userMsg, aiMsg, timestamp]) => {
                        this.addMessage('user', userMsg);
                        this.addMessage('bot', aiMsg);
                    });
                }
            }
        } catch (error) {
            console.error('Error loading chat history:', error);
        }
    }

    // 메시지 전송 함수
    async sendMessage() {
        if (!this.currentUserId || !this.sessionId) {
            alert('먼저 로그인해주세요.');
            return;
        }
        
        const message = this.userInput.value.trim();
        
        if (!message || this.isLoading) return;
        
        // 사용자 메시지 추가
        this.addMessage('user', message);
        this.userInput.value = '';
        
        // 로딩 상태 설정
        this.setLoading(true);
        
        // 로딩 메시지 표시
        const loadingDiv = this.addLoadingMessage();
        
        try {
            const response = await fetch('/chatbot/chat', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    message: message,
                    session_id: this.sessionId
                })
            });

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data = await response.json();
            
            // 로딩 메시지 제거
            if (loadingDiv) {
                loadingDiv.remove();
            }
            
            // AI 응답 추가
            if (data.emotion_img) {
                this.addMessage('bot', data.emotion_img, true);
            } else {
                this.addMessage('bot', data.response);
            }
            
        } catch (error) {
            console.error('Error:', error);
            
            // 로딩 메시지 제거
            if (loadingDiv) {
                loadingDiv.remove();
            }
            
            this.addMessage('bot', '죄송합니다. 응답을 가져오는 중에 오류가 발생했습니다. 다시 시도해주세요.');
        } finally {
            this.setLoading(false);
        }
    }

    // 메시지 추가 함수
    addMessage(sender, text, isImage = false) {
        const messageDiv = document.createElement('div');
        messageDiv.className = `message ${sender}`;

        if (isImage) {
            // 이미지 말풍선
            const img = document.createElement('img');
            img.src = text;
            img.alt = '감정 이미지';
            img.className = 'emotion-image';
            img.onload = () => {
                // 이미지가 완전히 로드된 후 스크롤 내리기
                this.chatBox.scrollTop = this.chatBox.scrollHeight;
            };
            messageDiv.appendChild(img);

            // EDIE Emotion 탭 이미지 동기화
            if (typeof setEdieEmotionImageFromResponse === 'function') {
                setEdieEmotionImageFromResponse(`[감정이미지]: ${text}`);
                // EDIE Emotion 탭이 열려 있으면 즉시 갱신
                const edieTab = document.getElementById('tab-edie');
                if (edieTab && edieTab.style.display !== 'none' && typeof updateEdieEmotionImage === 'function') {
                    updateEdieEmotionImage();
                }
            }
        } else {
            // 줄바꿈 처리: \n을 실제 줄바꿈으로 변환
            messageDiv.textContent = text;
        }

        this.chatBox.appendChild(messageDiv);
        // 텍스트 메시지이거나, 이미지가 이미 캐시되어 바로 로드된 경우에도 스크롤 내림
        this.chatBox.scrollTop = this.chatBox.scrollHeight;
    }

    // 로딩 메시지 추가 함수
    addLoadingMessage() {
        const loadingDiv = document.createElement('div');
        loadingDiv.className = 'message bot loading-message';
        loadingDiv.innerHTML = `
            <span>AI가 응답을 생성하고 있습니다</span>
            <div class="loading"></div>
            <div class="loading"></div>
            <div class="loading"></div>
        `;
        
        this.chatBox.appendChild(loadingDiv);
        this.chatBox.scrollTop = this.chatBox.scrollHeight;
        
        return loadingDiv;
    }

    // 로딩 상태 설정 함수
    setLoading(loading) {
        this.isLoading = loading;
        this.sendButton.disabled = loading;
        this.userInput.disabled = loading;
        
        if (loading) {
            this.sendButton.textContent = '전송중...';
            this.userInput.placeholder = 'AI가 응답을 생성하고 있습니다...';
        } else {
            this.sendButton.textContent = '전송';
            this.userInput.placeholder = '메시지를 입력하세요...';
            this.userInput.focus();
        }
    }

    // 대화 기록 초기화 함수 (아카이빙)
    async clearChat() {
        if (!this.currentUserId || !this.sessionId) {
            alert('먼저 로그인해주세요.');
            return;
        }
        
        if (!confirm('현재 대화 기록을 아카이빙하고 새로 시작하시겠습니까?\n(기존 대화는 타임스탬프 파일로 저장됩니다)')) {
            return;
        }
        
        try {
            const response = await fetch('/chatbot/archive', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                }
            });

            if (response.ok) {
                const data = await response.json();
                console.log(`📦 Database archived: ${data.archive_name}`);
                
                this.chatBox.innerHTML = `
                    <div class="message bot">
                        이전 대화가 저장되었습니다. 🗂️<br>
                        새로운 대화를 시작합니다! 안녕하세요 ${this.currentUserId}님! 🤖
                    </div>
                `;
            }
        } catch (error) {
            console.error('Error archiving chat:', error);
            alert('대화 아카이빙 중 오류가 발생했습니다.');
        }
    }

    // ========== 음성 인식 기능 ==========
    
    // 녹음 토글
    async toggleRecording() {
        if (!this.currentUserId || !this.sessionId) {
            alert('먼저 로그인해주세요.');
            return;
        }
        
        if (this.isRecording) {
            this.stopRecording();
        } else {
            await this.startRecording();
        }
    }
    
    // 녹음 시작
    async startRecording() {
        try {
            // 마이크 권한 요청
            const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
            
            // MediaRecorder 초기화
            this.mediaRecorder = new MediaRecorder(stream);
            this.audioChunks = [];
            
            // 데이터 수집
            this.mediaRecorder.addEventListener('dataavailable', (event) => {
                this.audioChunks.push(event.data);
            });
            
            // 녹음 완료 시 처리
            this.mediaRecorder.addEventListener('stop', async () => {
                const audioBlob = new Blob(this.audioChunks, { type: 'audio/webm' });
                await this.sendAudioToServer(audioBlob);
                
                // 스트림 정리
                stream.getTracks().forEach(track => track.stop());
            });
            
            // 녹음 시작
            this.mediaRecorder.start();
            this.isRecording = true;
            
            // UI 업데이트
            this.micButton.classList.add('recording');
            this.userInput.placeholder = '🎤 인식 중... (2초 무음 후 자동 전송)';
            
            // 2초 타이머 시작
            this.resetSilenceTimer();
            
            console.log('인식 시작');
            
        } catch (error) {
            console.error('마이크 접근 오류:', error);
            alert('마이크에 접근할 수 없습니다. 브라우저 설정을 확인해주세요.');
        }
    }
    
    // 녹음 중지
    stopRecording() {
        if (this.mediaRecorder && this.isRecording) {
            this.mediaRecorder.stop();
            this.isRecording = false;
            
            // 타이머 정리
            if (this.silenceTimer) {
                clearTimeout(this.silenceTimer);
                this.silenceTimer = null;
            }
            
            // UI 업데이트
            this.micButton.classList.remove('recording');
            this.userInput.placeholder = '메시지를 입력하세요...';
            
            console.log('인식 중지');
        }
    }
    
    // 무음 감지 타이머 초기화 (2초)
    resetSilenceTimer() {
        if (this.silenceTimer) {
            clearTimeout(this.silenceTimer);
        }
        
        // 2초 후 자동 중지
        this.silenceTimer = setTimeout(() => {
            console.log('2초 무음 감지 - 인식 중지');
            this.stopRecording();
        }, 2000);
    }
    
    // 오디오를 서버로 전송하고 텍스트 받기
    async sendAudioToServer(audioBlob) {
        try {
            // 로딩 상태 표시
            this.userInput.placeholder = '🎤 음성 인식 중...';
            this.micButton.disabled = true;
            
            // FormData 생성
            const formData = new FormData();
            formData.append('audio', audioBlob, 'recording.webm');
            
            // 서버로 전송
            const response = await fetch('/speech/recognize', {
                method: 'POST',
                body: formData
            });

            if (response.status === 204) {
                // 트리거 워드 미포함: 안내 메시지
                this.userInput.placeholder = '에디를 부르고 말해주세요.';
                setTimeout(() => {
                    this.userInput.placeholder = '메시지를 입력하세요...';
                }, 3000);
                this.micButton.disabled = false;
                return;
            }

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data = await response.json();

            if (data.success && data.text) {
                // 인식된 텍스트를 입력창에 표시
                this.userInput.value = data.text;

                // 자동으로 메시지 전송
                await this.sendMessage();
            } else {
                // 인식 실패
                this.userInput.placeholder = '❌ ' + (data.error || '음성을 인식할 수 없습니다.');
                setTimeout(() => {
                    this.userInput.placeholder = '메시지를 입력하세요...';
                }, 3000);
            }

        } catch (error) {
            console.error('음성 인식 오류:', error);
            this.userInput.placeholder = '❌ 음성 인식 중 오류가 발생했습니다.';
            setTimeout(() => {
                this.userInput.placeholder = '메시지를 입력하세요...';
            }, 3000);
        } finally {
            this.micButton.disabled = false;
        }
    }

    // 소멸자 - 페이지 언로드 시 정리
    destroy() {
        // 녹음 중이면 중지
        if (this.isRecording) {
            this.stopRecording();
        }
    }
}

// 페이지 언로드 시 정리
window.addEventListener('beforeunload', () => {
    if (window.chatBot) {
        window.chatBot.destroy();
    }
});

// 챗봇 인스턴스 생성 (전역 변수로 노출)
window.chatBot = new ChatBot();
