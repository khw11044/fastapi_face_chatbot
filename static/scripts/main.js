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
});