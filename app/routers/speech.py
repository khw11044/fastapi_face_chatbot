from fastapi import APIRouter, UploadFile, File, HTTPException
import speech_recognition as sr
from pydub import AudioSegment
import tempfile
import os

router = APIRouter()

@router.post("/recognize")
async def recognize_speech(audio: UploadFile = File(...)):
    """
    업로드된 오디오 파일을 텍스트로 변환합니다.
    """
    temp_webm_path = None
    temp_wav_path = None
    
    try:
        # 1. WebM 파일을 임시 파일로 저장
        with tempfile.NamedTemporaryFile(delete=False, suffix='.webm') as temp_webm:
            content = await audio.read()
            temp_webm.write(content)
            temp_webm_path = temp_webm.name
        
        # 2. WebM을 WAV로 변환 (pydub + ffmpeg)
        with tempfile.NamedTemporaryFile(delete=False, suffix='.wav') as temp_wav:
            temp_wav_path = temp_wav.name
        
        try:
            # pydub로 WebM 로드 후 WAV로 변환
            audio_segment = AudioSegment.from_file(temp_webm_path, format="webm")
            audio_segment.export(temp_wav_path, format="wav")
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"오디오 변환 오류: {str(e)}"
            )
        
        # 3. speech_recognition으로 음성 인식
        recognizer = sr.Recognizer()
        
        with sr.AudioFile(temp_wav_path) as source:
            # 주변 소음 조정
            recognizer.adjust_for_ambient_noise(source, duration=0.5)
            # 오디오 데이터 읽기
            audio_data = recognizer.record(source)
        
        # 4. Google Speech Recognition으로 한국어 인식
        try:
            text = recognizer.recognize_google(audio_data, language='ko-KR')
            return {
                "success": True,
                "text": text
            }
        except sr.UnknownValueError:
            # 음성을 인식할 수 없는 경우
            return {
                "success": False,
                "text": "",
                "error": "음성을 인식할 수 없습니다."
            }
        except sr.RequestError as e:
            # API 요청 오류
            raise HTTPException(
                status_code=500, 
                detail=f"음성 인식 서비스 오류: {str(e)}"
            )
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"오디오 처리 오류: {str(e)}"
        )
    
    finally:
        # 임시 파일 삭제
        if temp_webm_path and os.path.exists(temp_webm_path):
            try:
                os.remove(temp_webm_path)
            except:
                pass
        
        if temp_wav_path and os.path.exists(temp_wav_path):
            try:
                os.remove(temp_wav_path)
            except:
                pass

@router.get("/test")
async def test_speech():
    """음성 인식 서비스 테스트 엔드포인트"""
    return {
        "status": "ok",
        "message": "Speech recognition service is running"
    }
