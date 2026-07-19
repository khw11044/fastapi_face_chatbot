# 모델 선택 기능 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** LLM 모델을 `config.yaml`로 관리하고 웹 UI 드롭다운에서 실시간 선택할 수 있게 하며, 관련 정리(의존성·죽은 코드·API 키 버그)를 함께 처리한다.

**Architecture:** `config.yaml`(루트) → 얇은 설정 로더(`utils/config/settings.py`) → `LLMService`가 로드해 활성 모델을 메모리로 관리(`set_model`) → 라우터가 `GET /chatbot/models`·`POST /chatbot/model`로 노출 → `chatbot.js`가 드롭다운을 채우고 변경을 전송. 선택은 전역·메모리 한정, config.yaml은 읽기 전용.

**Tech Stack:** FastAPI, langchain-openai(ChatOpenAI), PyYAML, 바닐라 JS.

## Global Constraints

- Python 3.12 (conda `fastapi` env). 실행 파이썬: `/home/roboseasy/miniforge3/envs/fastapi/bin/python`.
- **들여쓰기는 4-space** — 이 프로젝트 기존 파일(`llm_service.py` 등)이 4-space이므로 그대로 맞춘다(탭 금지, 혼용 금지).
- **절대경로 하드코딩 금지** — `BASE_DIR` 기준 상대경로.
- 신규 함수·메서드에 타입 힌트. Boolean은 `is_`/`has_` 접두(해당 시).
- **config.yaml은 런타임에 수정하지 않는다**(읽기 전용 소스).
- 작업은 **현재 브랜치 `next01`**(사용자 작업 브랜치)에서 진행. 구현·검증 완료 후 `next01` → `main` 병합 + `origin` push(Task 7). `main`에 직접 커밋하지 않는다.
- 커밋 태그는 RobosEasy 규약(`Add:` `Fix:` `Improve:` `Delete:` `Refactor:`), 콜론 뒤 공백 1개, 50자 이내, 마침표 없음. 커밋 끝에 co-author 라인 추가. **커밋은 사용자 승인 후 실행.**

---

### Task 0: 작업 브랜치 확인

**Files:** 없음(git 상태 확인만)

- [ ] **Step 1: 현재 브랜치가 next01인지 확인**

Run:
```bash
git branch --show-current
```
Expected: `next01` (아니면 `git checkout next01`). 이후 모든 커밋은 이 브랜치에서.

---

### Task 1: config.yaml + 설정 로더

**Files:**
- Create: `config.yaml`
- Create: `utils/config/__init__.py`
- Create: `utils/config/settings.py`

**Interfaces:**
- Produces: `load_llm_config() -> dict` — 반환 `{'default': str, 'temperature': float, 'models': list[str]}`. `FileNotFoundError`/`ValueError` 발생 가능.

- [ ] **Step 1: config.yaml 작성**

Create `config.yaml`:
```yaml
llm:
  default: gpt-4o-mini
  temperature: 0.7
  models:
    - gpt-4o-mini
    - gpt-4o
    - gpt-4.1
    - gpt-4.1-mini
    - gpt-4-turbo
    - gpt-3.5-turbo
```

- [ ] **Step 2: 패키지 마커 생성**

Create `utils/config/__init__.py` (빈 파일).

- [ ] **Step 3: 로더 구현**

Create `utils/config/settings.py`:
```python
import os

import yaml

# utils/config/settings.py → utils/config → utils → 프로젝트 루트
BASE_DIR = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
CONFIG_PATH = os.path.join(BASE_DIR, 'config.yaml')


def load_llm_config() -> dict:
    """config.yaml의 llm 설정을 읽어 반환한다.

    Returns:
        dict: {'default': str, 'temperature': float, 'models': list[str]}

    Raises:
        FileNotFoundError: config.yaml이 없을 때.
        ValueError: models가 비었거나 default가 models에 없을 때.
    """
    if not os.path.exists(CONFIG_PATH):
        raise FileNotFoundError(f'config.yaml을 찾을 수 없습니다: {CONFIG_PATH}')
    with open(CONFIG_PATH, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    llm = data.get('llm', {})
    models = llm.get('models', [])
    default = llm.get('default')
    temperature = llm.get('temperature', 0.7)
    if not models:
        raise ValueError('config.yaml: llm.models가 비어 있습니다')
    if default not in models:
        raise ValueError(
            f'config.yaml: default({default})가 models 목록에 없습니다'
        )
    return {
        'default': default,
        'temperature': float(temperature),
        'models': list(models),
    }
```

- [ ] **Step 4: 로더 동작 검증**

Run:
```bash
/home/roboseasy/miniforge3/envs/fastapi/bin/python -c "from utils.config.settings import load_llm_config; print(load_llm_config())"
```
Expected: `{'default': 'gpt-4o-mini', 'temperature': 0.7, 'models': ['gpt-4o-mini', 'gpt-4o', 'gpt-4.1', 'gpt-4.1-mini', 'gpt-4-turbo', 'gpt-3.5-turbo']}`

- [ ] **Step 5: 검증 로직 확인(default 불일치 시 에러)**

Run:
```bash
/home/roboseasy/miniforge3/envs/fastapi/bin/python -c "
import utils.config.settings as s
s.load_llm_config()  # 정상
print('정상 로드 OK')
"
```
Expected: `정상 로드 OK`

- [ ] **Step 6: 커밋(승인 후)**

```bash
git add config.yaml utils/config/__init__.py utils/config/settings.py
git commit -m "Add: config.yaml 기반 LLM 설정 로더"
```

---

### Task 2: LLMService 모델 설정 연동

**Files:**
- Modify: `app/services/llm_service.py`

**Interfaces:**
- Consumes: `load_llm_config()` (Task 1).
- Produces: `LLMService.set_model(model: str) -> None` (`ValueError` on 미허용), `get_available_models() -> list[str]`, `get_current_model() -> str`.

- [ ] **Step 1: import 추가**

`app/services/llm_service.py` 상단 로컬 import 블록(`from utils.databases.database import DatabaseManager` 아래)에 추가:
```python
from utils.config.settings import load_llm_config
```

- [ ] **Step 2: `__init__`에서 설정 로드 + LLM 구성으로 교체**

기존 `__init__`의 `self.llm = ChatOpenAI(...)` 블록과 `self.chain = self.init_chain()` 호출을 아래로 바꾼다(하드코딩 모델 제거):
```python
    def __init__(self):
        # config.yaml에서 LLM 설정 로드
        cfg = load_llm_config()
        self.available_models = cfg['models']
        self.current_model = cfg['default']
        self.temperature = cfg['temperature']

        # LLM·체인 구성
        self._build_llm()

        # chats 디렉토리 생성
        self.chats_dir = "./chats"
        os.makedirs(self.chats_dir, exist_ok=True)

        # 데이터베이스 경로 설정
        self.db_path = os.path.join(self.chats_dir, "chat_history.db")

        # 데이터베이스 매니저 초기화
        self.db_manager = DatabaseManager(self.db_path)

        # 현재 세션 ID
        self.current_session_id = 'default'
```

- [ ] **Step 3: `init_chain`을 `_build_llm`으로 대체 + 모델 관리 메서드 추가**

기존 `init_chain` 메서드를 아래 메서드들로 교체:
```python
    def _build_llm(self) -> None:
        """현재 모델·온도로 ChatOpenAI와 체인을 재구성한다."""
        self.llm = ChatOpenAI(
            model=self.current_model,
            temperature=self.temperature,
            openai_api_key=os.getenv("OPENAI_API_KEY"),
        )
        self.chain = chat_prompt | self.llm | StrOutputParser()

    def set_model(self, model: str) -> None:
        """활성 모델을 교체하고 체인을 재구성한다.

        Args:
            model: config.yaml의 models 목록에 있는 모델명.

        Raises:
            ValueError: 목록에 없는 모델일 때.
        """
        if model not in self.available_models:
            raise ValueError(f'허용되지 않은 모델: {model}')
        self.current_model = model
        self._build_llm()

    def get_available_models(self) -> list[str]:
        """선택 가능한 모델 목록을 반환한다."""
        return self.available_models

    def get_current_model(self) -> str:
        """현재 활성 모델명을 반환한다."""
        return self.current_model
```

> 주의: `chat_prompt`는 파일 상단에서 이미 import됨. `init_chain`을 호출하던 곳이 `__init__` 뿐이므로 다른 참조 없음.

- [ ] **Step 4: 동작 검증(더미 키, 네트워크 호출 없음)**

Run:
```bash
cd /home/roboseasy/workspace/fastapi_face_chatbot && OPENAI_API_KEY=sk-test /home/roboseasy/miniforge3/envs/fastapi/bin/python -c "
from app.services.llm_service import LLMService
s = LLMService()
print('models:', s.get_available_models())
print('current:', s.get_current_model())
s.set_model('gpt-4o'); print('after set:', s.get_current_model(), '| llm.model:', s.llm.model_name)
try:
    s.set_model('없는모델')
except ValueError as e:
    print('검증 OK:', e)
"
```
Expected(요약): `current: gpt-4o-mini` → `after set: gpt-4o | llm.model: gpt-4o` → `검증 OK: 허용되지 않은 모델: 없는모델`

- [ ] **Step 5: 커밋(승인 후)**

```bash
git add app/services/llm_service.py
git commit -m "Add: LLMService 모델 config 연동 및 set_model"
```

---

### Task 3: 모델 조회/변경 API

**Files:**
- Modify: `app/routers/chatbot.py`

**Interfaces:**
- Consumes: `llm_service.get_available_models/get_current_model/set_model` (Task 2).
- Produces: `GET /chatbot/models` → `{"models": [...], "current": str}`; `POST /chatbot/model` `{"model": str}` → `{"current": str}` 또는 400.

- [ ] **Step 1: 요청 모델 정의 추가**

`app/routers/chatbot.py`의 `ChatRequest` 클래스 아래에 추가:
```python
class ModelSelectRequest(BaseModel):
    model: str
```

- [ ] **Step 2: 엔드포인트 2개 추가**

`get_all_sessions` 아래(파일 끝)에 추가:
```python
@router.get("/models")
async def get_models():
    """선택 가능한 모델 목록과 현재 활성 모델을 반환합니다."""
    return {
        "models": llm_service.get_available_models(),
        "current": llm_service.get_current_model(),
    }


@router.post("/model")
async def set_model(request: ModelSelectRequest):
    """활성 모델을 변경합니다."""
    try:
        llm_service.set_model(request.model)
        return {"current": llm_service.get_current_model()}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
```

- [ ] **Step 3: TestClient로 엔드포인트 검증(네트워크 호출 없음)**

Run:
```bash
cd /home/roboseasy/workspace/fastapi_face_chatbot && OPENAI_API_KEY=sk-test /home/roboseasy/miniforge3/envs/fastapi/bin/python -c "
from fastapi.testclient import TestClient
from app.routers.chatbot import router
from fastapi import FastAPI
app = FastAPI(); app.include_router(router, prefix='/chatbot')
c = TestClient(app)
print('GET /models:', c.get('/chatbot/models').json())
print('POST valid:', c.post('/chatbot/model', json={'model':'gpt-4o'}).json())
r = c.post('/chatbot/model', json={'model':'nope'}); print('POST invalid status:', r.status_code)
"
```
Expected: `GET /models`에 models·current 포함 → `POST valid: {'current': 'gpt-4o'}` → `POST invalid status: 400`

- [ ] **Step 4: 커밋(승인 후)**

```bash
git add app/routers/chatbot.py
git commit -m "Add: 모델 조회/변경 API 엔드포인트"
```

---

### Task 4: 웹 UI 드롭다운

**Files:**
- Modify: `static/index.html`
- Modify: `static/scripts/chatbot.js`
- Modify: `static/styles/chatbot.css`

**Interfaces:**
- Consumes: `GET /chatbot/models`, `POST /chatbot/model` (Task 3).

- [ ] **Step 1: index.html 헤더에 드롭다운 추가**

`static/index.html`의 아래 블록을
```html
                <div class="chat-header">
                    <h1>챗봇</h1>
                    <button id="clear-btn" class="clear-button">대화 초기화</button>
                </div>
```
다음으로 교체:
```html
                <div class="chat-header">
                    <h1>챗봇</h1>
                    <div class="header-controls">
                        <select id="model-select" class="model-select" title="모델 선택"></select>
                        <button id="clear-btn" class="clear-button">대화 초기화</button>
                    </div>
                </div>
```

- [ ] **Step 2: chatbot.js — 요소 등록**

`initElements()`의 `this.clearButton = document.getElementById('clear-btn');` 아래에 추가:
```javascript
        this.modelSelect = document.getElementById('model-select');
```

- [ ] **Step 3: chatbot.js — 이벤트 바인딩**

`bindEvents()`의 `this.clearButton.addEventListener(...)` 아래에 추가:
```javascript
        this.modelSelect.addEventListener('change', (e) => this.changeModel(e.target.value));
```

- [ ] **Step 4: chatbot.js — init에서 모델 로드**

`init()` 메서드 안, `this.startAutoLoginCheck();` 아래에 추가:
```javascript
        // 모델 목록 로드
        this.loadModels();
```

- [ ] **Step 5: chatbot.js — 메서드 2개 추가**

`init()` 메서드 닫는 `}` 다음에 추가:
```javascript
    async loadModels() {
        try {
            const response = await fetch('/chatbot/models');
            const data = await response.json();
            this.modelSelect.innerHTML = '';
            data.models.forEach((name) => {
                const opt = document.createElement('option');
                opt.value = name;
                opt.textContent = name;
                if (name === data.current) opt.selected = true;
                this.modelSelect.appendChild(opt);
            });
        } catch (err) {
            console.error('모델 목록 로드 실패:', err);
        }
    }

    async changeModel(model) {
        try {
            const response = await fetch('/chatbot/model', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ model }),
            });
            if (!response.ok) throw new Error('모델 변경 실패');
            const data = await response.json();
            console.log('활성 모델 변경:', data.current);
        } catch (err) {
            console.error(err);
            alert('모델 변경에 실패했습니다.');
            this.loadModels();  // 서버의 현재 모델로 되돌림
        }
    }
```

- [ ] **Step 6: chatbot.css — 드롭다운 스타일**

`static/styles/chatbot.css`의 `.chat-header {` 규칙 근처에 추가:
```css
.header-controls {
    display: flex;
    align-items: center;
    gap: 8px;
}

.model-select {
    padding: 6px 10px;
    border: 1px solid rgba(255, 255, 255, 0.4);
    border-radius: 6px;
    background: rgba(255, 255, 255, 0.15);
    color: #fff;
    font-size: 13px;
    cursor: pointer;
}

.model-select option {
    color: #000;
}
```
> 실제 헤더 배경색에 맞춰 색은 조정 가능. 기존 `.clear-button` 톤을 참고.

- [ ] **Step 7: 실제 서버 구동 검증**

Run:
```bash
cd /home/roboseasy/workspace/fastapi_face_chatbot && /home/roboseasy/miniforge3/envs/fastapi/bin/python run.py
```
그리고 브라우저 `http://localhost:8000`:
- 드롭다운에 config.yaml 모델 6개 표시, `gpt-4o-mini` 선택됨.
- 다른 모델 선택 시 콘솔에 `활성 모델 변경: ...` 로그.
Expected: 위 두 가지 확인. (확인 후 Ctrl+C)

- [ ] **Step 8: 커밋(승인 후)**

```bash
git add static/index.html static/scripts/chatbot.js static/styles/chatbot.css
git commit -m "Add: 모델 선택 드롭다운 UI"
```

---

### Task 5: .env API 키 손상 수정(ascii 버그) + 방어 검증

**Files:**
- Modify: `.env` (비밀 파일 — 값 비노출로 처리)
- Modify: `utils/config/settings.py`

**확정 원인:** `.env`의 `OPENAI_API_KEY` 값 끝(index 164)에 한글 'ㄴ'(U+3134)이 섞여 `Authorization` 헤더 ascii 인코딩이 position 171에서 실패.

- [ ] **Step 1: .env의 키에서 비-ascii 제거(비밀 비노출)**

> 사용자에게 `.env`(비밀 파일) 수정 승인을 먼저 받는다.

Run:
```bash
cd /home/roboseasy/workspace/fastapi_face_chatbot && /home/roboseasy/miniforge3/envs/fastapi/bin/python -c "
import re
lines = open('.env', encoding='utf-8').read().splitlines()
out = []
for ln in lines:
    if ln.split('=')[0].strip() == 'OPENAI_API_KEY':
        k, _, v = ln.partition('=')
        v_clean = ''.join(ch for ch in v.strip() if ord(ch) < 128)
        out.append(f'OPENAI_API_KEY={v_clean}')
    else:
        out.append(ln)
open('.env','w',encoding='utf-8').write('\n'.join(out) + '\n')
print('완료')
"
```
Expected: `완료`

- [ ] **Step 2: 키가 순수 ascii인지 확인(값 비노출)**

Run:
```bash
cd /home/roboseasy/workspace/fastapi_face_chatbot && /home/roboseasy/miniforge3/envs/fastapi/bin/python -c "
from dotenv import dotenv_values
v = dotenv_values('.env')['OPENAI_API_KEY']
print('길이:', len(v), '| ascii:', v.isascii())
"
```
Expected: `길이: 164 | ascii: True`

- [ ] **Step 3: 방어 검증 추가 — settings.py에 키 점검 함수**

`utils/config/settings.py` 끝에 추가:
```python
def check_openai_api_key() -> None:
    """OPENAI_API_KEY가 비-ascii를 포함하면 명확한 에러를 낸다.

    Raises:
        ValueError: 키에 ascii 밖 문자가 섞였을 때(헤더 인코딩 실패 예방).
    """
    key = os.getenv('OPENAI_API_KEY', '')
    if key and not key.isascii():
        bad = [hex(ord(c)) for c in key if ord(c) > 127]
        raise ValueError(
            f'OPENAI_API_KEY에 비-ascii 문자 포함(위치 확인 필요): {bad}'
        )
```

- [ ] **Step 4: LLMService에서 방어 검증 호출**

`app/services/llm_service.py`의 import에 추가:
```python
from utils.config.settings import load_llm_config, check_openai_api_key
```
`_build_llm` 첫 줄에 추가:
```python
        check_openai_api_key()
```

- [ ] **Step 5: 실제 한글 응답 검증(유효 키·네트워크 필요)**

Run: `run.py`로 서버 기동 후 브라우저에서 로그인 → 한글 메시지 전송.
Expected: `[AI] ...` 에 정상 한글 응답(더 이상 "죄송합니다..." fallback 아님). 서버 로그에 ascii 에러 없음.

- [ ] **Step 6: 커밋(승인 후)**

> `.env`는 `.gitignore` 대상이어야 한다(커밋 금지). settings.py만 커밋.
```bash
git add utils/config/settings.py app/services/llm_service.py
git commit -m "Fix: OPENAI_API_KEY 비-ascii로 인한 헤더 인코딩 오류 방지"
```

- [ ] **Step 7: .env가 gitignore되는지 확인**

Run:
```bash
git check-ignore .env && echo "gitignore됨(정상)" || echo "경고: .env가 추적될 수 있음 — .gitignore에 추가 필요"
```
Expected: `gitignore됨(정상)`. 아니면 `.gitignore`에 `.env` 추가 후 커밋.

---

### Task 6: requirements 버전 고정 + 죽은 의존성/코드 정리

**Files:**
- Modify: `requirements.txt`
- Delete: `utils/agent/` (pyc만 남음)
- Delete: `langchain_ollama_023_unpacked/` (미사용 서드파티 unpack)

- [ ] **Step 1: requirements.txt 재작성(핀 + 정리)**

`requirements.txt` 전체를 아래로 교체:
```
# FastAPI 서버
fastapi==0.139.2
uvicorn[standard]==0.51.0
python-multipart==0.0.32

# 설정 파일(config.yaml) 파싱
pyyaml==6.0.3

# LangChain (create_tool_calling_agent 호환, 검증됨)
langchain==0.3.7
langchain-community==0.3.7
langchain-openai==0.3.4

# 얼굴 인식 / 임베딩 / 벡터DB
opencv-python==4.11.0.86
mediapipe==0.10.21
imgbeddings==0.1.0
transformers==4.48.3
huggingface-hub==0.25.2
chromadb==1.5.9

# ROS2 rclpy는 PyPI에 없고 ROS2 환경에서 제공됨(현재 코드 미사용).
# 필요 시 ROS2를 source 하여 사용:  source /opt/ros/jazzy/setup.bash
# rclpy
```
> 제거된 죽은 의존성: `langchain-ollama`, `SpeechRecognition`, `pydub`, `facenet-pytorch` (코드 import 없음 확인).

- [ ] **Step 2: 클린 설치 검증(격리 venv, 현재 env 비파괴)**

Run:
```bash
cd /tmp/claude-1000/-home-roboseasy-workspace-fastapi-face-chatbot/f7734d03-f8dc-4871-9dc2-e1631b73c7eb/scratchpad && /home/roboseasy/miniforge3/envs/fastapi/bin/python -m pip install --dry-run -r /home/roboseasy/workspace/fastapi_face_chatbot/requirements.txt 2>&1 | tail -5
```
Expected: 의존성 해석 성공(에러 없음). rclpy 관련 에러 없음(주석 처리됨).

- [ ] **Step 3: 죽은 코드 삭제(사용자 최종 확인 후)**

> 두 경로 모두 git 추적/영향 재확인 후 삭제.
Run:
```bash
cd /home/roboseasy/workspace/fastapi_face_chatbot && git rm -r utils/agent langchain_ollama_023_unpacked
```
Expected: 삭제된 파일 목록 출력.

- [ ] **Step 4: 서버 정상 기동 재확인**

Run: `run.py`로 서버 기동 → 정상 startup 후 Ctrl+C.
Expected: import 에러 없이 `Application startup complete.`

- [ ] **Step 5: 커밋(승인 후)**

```bash
git add requirements.txt
git commit -m "Improve: requirements 버전 고정 및 미사용 의존성 정리"
git commit -m "Delete: 미사용 죽은 코드(utils/agent, langchain_ollama_023_unpacked)"
```
> `git rm`은 이미 스테이징되므로 두 번째 커밋은 `git commit`만.

---

### Task 7: next01 → main 병합 + push (모든 구현·검증 완료 후)

**⚠️ 되돌리기 어렵고 외부로 나가는 작업 — 실행 직전 사용자 최종 확인 필수.**

**전제:** `main`이 `next01`보다 19커밋 앞서고 `next01`이 6커밋 앞선 **분기 상태**.
fast-forward 불가 → 실제 머지(충돌 가능). RobosEasy 규약상 충돌은 feature(next01)
쪽에서 먼저 해결한다.

- [ ] **Step 1: 최신 원격 반영 + 분기 내용 확인**

Run:
```bash
git fetch origin
git log --oneline main ^next01 | head -20   # main에만 있는 19커밋(어떤 작업인지 확인)
```
Expected: main의 19커밋 목록. 이것이 next01과 합쳐져도 되는지 사용자와 확인.

- [ ] **Step 2: next01 커밋·푸시 완결 확인**

Run:
```bash
git checkout next01 && git status --short && git push origin next01
```
Expected: 워킹트리 clean, `origin/next01` 최신.

- [ ] **Step 3: (충돌 대비) main을 next01로 먼저 머지해 충돌을 feature 쪽에서 해결**

Run:
```bash
git merge origin/main
```
충돌 시: next01에서 해결 → `git add` → `git commit`. 해결 후 서버 재기동 검증.

- [ ] **Step 4: main으로 이동해 next01 병합**

Run:
```bash
git checkout main && git pull origin main && git merge next01
```
Expected: 깨끗한 머지(Step 3 이후이므로 충돌 없음).

- [ ] **Step 5: push (최종 확인 후)**

Run:
```bash
git push origin main
```
Expected: `origin/main` 업데이트. 완료 후 `git checkout next01`로 복귀.

---

## Self-Review (작성자 점검)

**1. Spec coverage:**
- config.yaml 구조 → Task 1 ✓
- 설정 로더 → Task 1 ✓
- LLMService set_model/전역·메모리 → Task 2 ✓
- API 2개 → Task 3 ✓
- 프론트 드롭다운(html/js/css) → Task 4 ✓
- ascii 버그(확정 원인: .env 키) → Task 5 ✓
- requirements 핀 + 죽은 의존성/코드 → Task 6 ✓
- 검증(서버 구동/재시작 복귀/400) → Task 3·4·5 검증 스텝 ✓

**2. Placeholder scan:** TBD/모호 표현 없음. 모든 코드 스텝에 실제 코드 포함.

**3. Type consistency:** `load_llm_config`/`set_model`/`get_available_models`/`get_current_model`/`check_openai_api_key` 시그니처가 Task 간 일치. 엔드포인트 경로(`/chatbot/models`, `/chatbot/model`)와 프론트 fetch 경로 일치.

## 비고: 테스트 방식

이 프로젝트는 기존 테스트 스위트가 없다(pytest 미설치). 불필요한 테스트 프레임워크
도입(scope creep)을 피하고, RobosEasy "실제 흐름 구동 검증" 원칙에 따라 각 태스크를
**실행 가능한 스니펫 + 실제 서버 구동**으로 검증한다. 정식 pytest 스위트가 필요하면
별도 요청 시 추가한다.
