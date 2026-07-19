# 모델 선택 기능 설계 — config.yaml 기반 관리 + 웹 UI 드롭다운

작성일: 2026-07-19
대상 프로젝트: `fastapi_face_chatbot`

## 1. 목표

LLM 모델을 코드에 하드코딩(`ChatOpenAI(model_name="gpt-4o-mini")`)하지 않고
`config.yaml`로 관리하며, 웹 UI 드롭다운에서 사용 가능한 OpenAI 챗 모델을
골라 실시간으로 바꿀 수 있게 한다.

## 2. 확정된 결정 (브레인스토밍 결과)

- **모델 범위: OpenAI 챗 모델만.** 목록은 `config.yaml`에서 사용자가 큐레이션한다.
  Ollama·동적 조회는 이번 범위에서 제외.
- **선택 적용 방식: 전역 + 메모리만.**
  - 드롭다운에서 모델을 고르면 서버 전체의 활성 모델이 즉시 교체된다(체인 재구성).
  - 이후 모든 세션의 `/chat`이 새 모델을 사용한다.
  - 서버를 재시작하면 `config.yaml`의 `default`로 복귀한다.
  - `config.yaml`은 런타임에 **수정하지 않는다**(읽기 전용 소스).

## 3. `config.yaml` 구조

프로젝트 루트(`config.py` 옆)에 신규 생성한다.

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

- 사용자가 직접 편집하는 파일. 모델 추가/삭제는 이 목록만 고치면 된다.
- `temperature`도 이 파일로 분리한다(기존 하드코딩 `0.7` 대체).
- o1/o3 계열 reasoning 모델은 `temperature`를 지원하지 않으므로 기본 목록에서
  제외한다. 추후 필요하면 파라미터 분기를 별도로 설계한다.
- `default`는 `models` 목록에 포함되어야 한다(로더에서 검증).

## 4. 백엔드 설계

### 4.1 설정 로더 (신규)

`utils/config/settings.py` — 단일 목적: `config.yaml`을 읽어 LLM 설정을 반환한다.

- 절대경로 하드코딩 금지. `BASE_DIR = os.path.dirname(os.path.abspath(__file__))`
  기준으로 프로젝트 루트의 `config.yaml`을 찾는다.
- 반환 형태: `default: str`, `temperature: float`, `models: list[str]`.
- `config.yaml`이 없거나 `default`가 `models`에 없으면 명확한 예외를 던진다.
- 타입 힌트·Google 스타일 docstring 적용.

### 4.2 `LLMService` 변경 — `app/services/llm_service.py`

- `__init__`에서 설정 로더 호출 →
  `self.available_models: list[str]`, `self.current_model: str`(=default),
  `self.temperature: float` 저장.
- `_build_llm()` 내부 헬퍼: 현재 `self.current_model`·`self.temperature`로
  `ChatOpenAI`를 만들고 `self.chain`을 재구성한다. `__init__`과 `set_model`에서 공용.
- 신규 메서드:
  - `set_model(self, model: str) -> None`: `model`이 `available_models`에 있으면
    `self.current_model` 갱신 후 `_build_llm()` 재호출. 없으면 `ValueError`.
  - `get_available_models(self) -> list[str]`
  - `get_current_model(self) -> str`
- 기존 `generate_response`/`get_chat_history` 등 나머지 로직은 유지.

### 4.3 API 엔드포인트 — `app/routers/chatbot.py`

라우터는 `/chatbot` prefix로 마운트되어 있다(프론트가 `/chatbot/chat` 호출).

- `GET /chatbot/models`
  → `{"models": [...], "current": "gpt-4o-mini"}`
- `POST /chatbot/model`  body: `{"model": "gpt-4o"}`
  → 성공 시 `{"current": "gpt-4o"}`, 검증 실패 시 `HTTP 400`.

## 5. 프론트엔드 설계

### 5.1 `static/index.html`

챗봇 헤더(`<h1>챗봇</h1>` + 초기화 버튼 영역)에 모델 선택 드롭다운을 추가한다.

```html
<div class="chat-header">
    <h1>챗봇</h1>
    <div class="header-controls">
        <select id="model-select" class="model-select"></select>
        <button id="clear-btn" class="clear-button">대화 초기화</button>
    </div>
</div>
```

### 5.2 `static/scripts/chatbot.js`

- 페이지 로드 시 `GET /chatbot/models` 호출 → `<option>`들을 채우고
  `current` 값을 선택 상태로 둔다.
- `#model-select`의 `change` 이벤트 → `POST /chatbot/model`로 선택 모델 전송.
  실패 시 이전 선택으로 되돌리고 사용자에게 알린다.

### 5.3 `static/styles/chatbot.css`

- 드롭다운을 기존 헤더 톤에 맞춰 최소한으로 스타일링. 새 색/폰트 도입 없이
  기존 변수·톤 재사용.

## 6. 데이터 흐름

```
[페이지 로드] → GET /chatbot/models → 드롭다운 채움(current 선택)
[모델 변경]   → POST /chatbot/model → LLMService.set_model() → chain 재구성
[대화]        → POST /chatbot/chat  → 현재 활성 모델로 응답
[서버 재시작] → config.yaml.default 로 복귀
```

## 7. 함께 포함되는 정리 작업

사용자 요청("리팩터링" + "requirements 버전 fix")에 따라 이번 작업에 포함한다.

### 7.1 `requirements.txt`
- 실사용 패키지를 현재 설치 버전으로 핀 고정하고 `pyyaml`을 추가한다.
- 실사용 확인된 버전:
  fastapi 0.139.2, uvicorn 0.51.0, langchain 0.3.7, langchain-community 0.3.7,
  langchain-openai 0.3.4, chromadb 1.5.9, opencv-python 4.11.0.86,
  huggingface-hub 0.25.2, transformers 4.48.3, imgbeddings 0.1.0,
  python-multipart 0.0.32, pyyaml 6.0.3.
- **죽은 의존성 제거**(코드에서 import 없음 확인): `langchain-ollama`,
  `SpeechRecognition`, `pydub`, `facenet-pytorch`.
- `rclpy`: PyPI에 없고(ROS2 제공) 현재 코드에서 미사용 → 주석 처리하고
  "ROS2 환경에서 제공됨"을 명시한다.

### 7.2 죽은 코드 삭제
- `utils/agent/` — 소스 `.py`가 없고 `__pycache__/*.pyc`만 남은 디렉토리. 삭제.
- `langchain_ollama_023_unpacked/` — 서드파티 wheel을 풀어놓은 미사용 폴더. 삭제.
- 삭제 대상은 git 추적 파일이므로 실행 전 사용자에게 최종 확인한다.

### 7.3 ascii 인코딩 버그 수정 (원인 확정)
- 증상: 한글 응답 생성 시
  `'ascii' codec can't encode character 'ㄴ'(ㄴ) in position 171` 로
  `generate_response`가 실패하고 fallback 에러 메시지가 반환된다.
- **확정된 원인: 코드 버그가 아니라 `.env`의 손상된 API 키.**
  `OPENAI_API_KEY` 값(165자) 끝(index 164)에 한글 'ㄴ'(U+3134)이 섞여 있다.
  langchain/openai가 `Authorization: Bearer <키>` 헤더를 ascii로 인코딩할 때
  position 171(= "Bearer " 7자 + 키 index 164)에서 실패한다. 요청 본문(한글)은
  정상 전송됨(더미 키 재현 시 401까지 도달, ascii 에러 없음)이 확인됨.
- 수정:
  1. `.env`의 `OPENAI_API_KEY` 값에서 비-ascii 문자를 제거한다(비밀 비노출,
     프로그램적으로 처리). 부수적으로 `KEY =` 앞뒤 공백도 정리.
  2. 방어적 조치(선택): 설정 로더/서비스 초기화 시 키가 비-ascii를 포함하면
     `ValueError`로 명확히 알린다. 다음에 같은 사고가 나면 암호 같은
     `UnicodeEncodeError` 대신 원인이 바로 드러나게 한다.
- 수용 기준: 유효 키 + 네트워크 환경에서 한글 입력에 정상 한글 응답이 오고
  DB에 저장된다.

## 8. 검증 (수용 기준)

실제 서버를 구동해 관찰한다.

1. 서버 기동 시 드롭다운에 `config.yaml`의 모델 목록이 표시되고 `default`가 선택됨.
2. 드롭다운으로 모델을 바꾼 뒤 `/chat`이 바뀐 모델로 응답함.
3. 한글 대화가 정상 동작함(ascii 버그 해소).
4. 서버 재시작 시 활성 모델이 `default`로 복귀함.
5. 잘못된 모델명으로 `POST /chatbot/model` 시 400 반환.

## 9. 이번 범위에서 제외 (Out of scope)

- Ollama·기타 provider 연동, OpenAI `/v1/models` 동적 조회.
- 세션별 모델 선택, config.yaml로의 런타임 write-back.
- reasoning 모델(o1/o3) 파라미터 분기.
- 스트리밍 응답 방식 변경.
