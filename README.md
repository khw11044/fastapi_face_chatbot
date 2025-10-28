

# 가상 환경 준비 

```
conda create -n fastapi python=3.10 -y

conda activate fastapi
```


```
sudo apt install python3-venv

python3 -m venv venv 

source venv/bin/activate
```


# 패키지 설치 

```
sudo apt update
sudo apt install ffmpeg
```

```
pip install -r requirements.txt
```


# 한국어 폰트 다운 

```
sudo apt-get install fonts-nanum
```


# fastapi 웹 실행

```
source venv/bin/activate
python run.py
```

`http://localhost:8000/`

로 접근해야함


# TroubleShooting

```
ImportError: /home/khw/miniconda3/envs/fastapi/bin/../lib/libstdc++.so.6: version `GLIBCXX_3.4.30' not found (required by /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/_rclpy_pybind11.cpython-310-x86_64-linux-gnu.so)
The C extension '/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/_rclpy_pybind11.cpython-310-x86_64-linux-gnu.so' failed to be imported while being present on the system. Please refer to 'https://docs.ros.org/en/{distro}/Guides/Installation-Troubleshooting.html#import-failing-even-with-library-present-on-the-system' for possible solutions
```

이런 오류가 생기는 경우

1. Conda 환경의 libstdc++를 시스템 libstdc++와 교체
2. LD_LIBRARY_PATH 설정: 환경 변수 설정으로 ROS2 라이브러리 경로 조정

### 1. Conda 환경의 libstdc++를 시스템 libstdc++와 교체

```
conda install -c conda-forge libstdcxx-ng
```

### 2. LD_LIBRARY_PATH 설정

```
# ROS2 libstdc++ 호환성 문제 해결
os.environ['LD_LIBRARY_PATH'] = '/opt/ros/humble/lib:' + os.environ.get('LD_LIBRARY_PATH', '')
```
