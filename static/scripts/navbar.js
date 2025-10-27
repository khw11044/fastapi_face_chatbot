// 네비게이션 메뉴 기능

document.addEventListener('DOMContentLoaded', () => {
    const hamburgerBtn = document.getElementById('hamburger-btn');
    const sidebar = document.getElementById('sidebar');
    const closeSidebarBtn = document.getElementById('close-sidebar');
    const sidebarOverlay = document.getElementById('sidebar-overlay');
    
    // 햄버거 메뉴 버튼 클릭
    hamburgerBtn?.addEventListener('click', () => {
        openSidebar();
    });
    
    // 닫기 버튼 클릭
    closeSidebarBtn?.addEventListener('click', () => {
        closeSidebar();
    });
    
    // 오버레이 클릭
    sidebarOverlay?.addEventListener('click', () => {
        closeSidebar();
    });
    
    // ESC 키로 사이드바 닫기
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && sidebar?.classList.contains('open')) {
            closeSidebar();
        }
    });
    
    // 사이드바 열기
    function openSidebar() {
        sidebar?.classList.add('open');
        sidebarOverlay?.classList.add('show');
        document.body.style.overflow = 'hidden'; // 배경 스크롤 방지
    }
    
    // 사이드바 닫기
    function closeSidebar() {
        sidebar?.classList.remove('open');
        sidebarOverlay?.classList.remove('show');
        document.body.style.overflow = ''; // 스크롤 복원
    }
    
    // 현재 페이지에 따라 active 클래스 추가
    const currentPath = window.location.pathname;
    const menuLinks = document.querySelectorAll('.menu-link');
    
    menuLinks.forEach(link => {
        const href = link.getAttribute('href');
        if (href === currentPath || 
            (currentPath === '/' && href === '/') ||
            (currentPath === '/touch-laser' && href === '/touch-laser')) {
            link.classList.add('active');
        }
    });
});
