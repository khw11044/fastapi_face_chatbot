// 챗봇 관련 기능
class ChatBot {
    constructor() {
        this.isLoading = false;
        this.currentUserId = null;
        this.sessionId = null;
        this.autoLoginCheckInterval = null;
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
    }

    bindEvents() {
        this.sendButton.addEventListener('click', () => this.sendMessage());
        this.clearButton.addEventListener('click', () => this.clearChat());
        this.loginButton.addEventListener('click', () => this.loginUser());
        this.logoutButton.addEventListener('click', () => this.logoutUser());
        
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
    }

    init() {
        // 초기 상태: 채팅 비활성화
        this.setChatDisabled(true);
        // 초기 포커스를 사용자 ID 입력창에
        this.userIdInput.focus();
        // 자동 로그인 체크 시작
        this.startAutoLoginCheck();
    }

    // 자동 로그인 체크 시작
    startAutoLoginCheck() {
        // 2초마다 자동 로그인 가능한 사용자 확인
        this.autoLoginCheckInterval = setInterval(async () => {
            if (!this.currentUserId) {  // 로그인하지 않은 상태에서만 체크
                await this.checkAutoLogin();
            }
        }, 2000);
    }

    // 자동 로그인 체크 중지
    stopAutoLoginCheck() {
        if (this.autoLoginCheckInterval) {
            clearInterval(this.autoLoginCheckInterval);
            this.autoLoginCheckInterval = null;
        }
    }

    // 자동 로그인 확인
    async checkAutoLogin() {
        try {
            const response = await fetch('/camera/auto-login');
            if (response.ok) {
                const data = await response.json();
                if (data.auto_login && data.user_id) {
                    await this.autoLogin(data.user_id);
                }
            }
        } catch (error) {
            console.error('자동 로그인 체크 오류:', error);
        }
    }

    // 자동 로그인 실행
    async autoLogin(userId) {
        console.log(`자동 로그인 시도: ${userId}`);
        
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
        
        // 환영 메시지가 없다면 추가
        if (this.chatBox.children.length === 0) {
            this.addMessage('bot', `다시 만나서 반가워요 ${userId}님! 저는 로봇 사탕가게 직원입니다. 오늘은 무엇을 주문하시겠어요? 🍭🤖`);
        } else {
            // 기존 대화가 있는 경우 간단한 인사만
            this.addMessage('bot', `안녕하세요 ${userId}님! 이어서 대화해볼까요? 😊`);
        }
        
        this.userInput.focus();
        
        // 자동 로그인 체크 중지
        this.stopAutoLoginCheck();
    }

    // 사용자 로그인 (수동)
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

        // 얼굴 등록 시도
        try {
            const response = await fetch('/camera/register-face', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    user_id: userId
                })
            });

            const data = await response.json();
            
            if (response.ok && data.status === 'success') {
                console.log('얼굴 등록 성공:', data.message);
                
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
                
                this.addMessage('bot', `안녕하세요 ${userId}님! 얼굴 등록이 완료되었습니다. 다음번부터는 자동으로 로그인됩니다. 저는 로봇 사탕가게 직원입니다. 빨간색 사탕(딸기), 파란 사탕(소다), 노란 사탕(레몬), 오렌지 주스를 판매합니다. 무엇을 주문하시겠어요? 🍭🤖`);
                
                this.userInput.focus();
                
                // 자동 로그인 체크 중지
                this.stopAutoLoginCheck();
                
            } else {
                // 얼굴 등록에 실패한 경우, 일반 로그인 진행
                console.log('얼굴 등록 실패, 일반 로그인 진행:', data.message);
                await this.performManualLogin(userId);
            }
            
        } catch (error) {
            console.error('얼굴 등록 오류:', error);
            // 오류 발생 시에도 일반 로그인 진행
            await this.performManualLogin(userId);
        }
    }

    // 수동 로그인 처리
    async performManualLogin(userId) {
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
        
        this.addMessage('bot', `안녕하세요 ${userId}님! 저는 로봇 사탕가게 직원입니다. 카메라 앞에서 얼굴을 보여주시면 다음번에 자동 로그인이 가능합니다. 빨간색 사탕(딸기), 파란 사탕(소다), 노간 사탕(레몬), 오렌지 주스를 판매합니다. 무엇을 주문하시겠어요? 🍭🤖`);
        
        this.userInput.focus();
        
        // 자동 로그인 체크 중지
        this.stopAutoLoginCheck();
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
                안녕하세요! 저는 로봇 사탕가게 직원입니다. 먼저 사용자 ID를 입력해주세요! 🍭🤖
            </div>
        `;
        
        this.userIdInput.focus();
        
        // 자동 로그인 체크 재시작
        this.startAutoLoginCheck();
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
            this.addMessage('bot', data.response);
            
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
    addMessage(sender, text) {
        const messageDiv = document.createElement('div');
        messageDiv.className = `message ${sender}`;
        
        // 줄바꿈 처리: \n을 실제 줄바꿈으로 변환
        messageDiv.textContent = text;
        
        this.chatBox.appendChild(messageDiv);
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

    // 대화 기록 초기화 함수
    async clearChat() {
        if (!this.currentUserId || !this.sessionId) {
            alert('먼저 로그인해주세요.');
            return;
        }
        
        if (!confirm('정말로 대화 기록을 삭제하시겠습니까?')) {
            return;
        }
        
        try {
            const response = await fetch('/chatbot/clear', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ session_id: this.sessionId })
            });

            if (response.ok) {
                this.chatBox.innerHTML = `
                    <div class="message bot">
                        안녕하세요 ${this.currentUserId}님! 저는 로봇 사탕가게 직원입니다. 빨간색 사탕(딸기), 파란 사탕(소다), 노간 사탕(레몬), 오렌지 주스를 판매합니다. 무엇을 주문하시겠어요? 🍭🤖
                    </div>
                `;
            }
        } catch (error) {
            console.error('Error clearing chat:', error);
            alert('대화 기록 삭제 중 오류가 발생했습니다.');
        }
    }

    // 소멸자 - 페이지 언로드 시 자동 로그인 체크 중지
    destroy() {
        this.stopAutoLoginCheck();
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