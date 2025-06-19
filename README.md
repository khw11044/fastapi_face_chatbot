

# 가상 환경 준비 

```
conda create -n fastapi python=3.10 -y

conda activate fastapi
```

# 패키지 설치 


```
pip install -r requirements.txt
```


# 한국어 폰트 다운 

```
sudo apt-get install fonts-nanum
```

# 필요 모델 사전 로딩 

```
# 모델 사전 로딩 (첫 실행만)
python preload_models.py
```

# fastapi 웹 실행

```
python run.py
```

