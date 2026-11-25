# AI Worker 로봇 운용 및 데이터 수집 매뉴얼

> **⚠️ ai_worker 2.0.0 이후 (Zenoh):** 로봇이 다른 PC에서 구동 중이면 카메라 토픽이
> 들어오지 않습니다. RMW가 `rmw_zenoh_cpp`로 바뀌었고, 호스트마다 별도의 zenoh 라우터가
> 뜨는데 서로 연결되지 않기 때문입니다. 토픽 이름이나 QoS 문제가 아닙니다.
> 컨테이너 안에서 실행 전에 아래를 먼저 `source` 하세요:
>
> ```bash
> DAW
> ROBOT_IP=192.168.6.2 source /root/ros2_ws/src/ai_worker/custom/pc/network/zenoh_robot_env.sh
> cd /root/ros2_ws/src/ai_worker/custom/shared/record
> python3 record_cameras_headless.py <이름>
> ```
>
> 자세한 내용: `custom/pc/control/README.md` → "Zenoh networking"

> **GUI 버전(`record_cameras.py`) 실행 시 `Pillow is required` 오류:** Pillow은 이미
> 깔려 있지만 `ImageTk`는 별도 패키지입니다. 안내 메시지의 `pip install Pillow`로는
> 해결되지 않습니다. 컨테이너 안에서:
>
> ```bash
> apt-get install -y python3-pil.imagetk
> ```
>
> 컨테이너를 새로 만들면(이미지 업데이트 등) 다시 설치해야 합니다.

> 전체 셋업 가이드: https://ai.robotis.com/ai_worker/setup_guide_ai_worker.html

---

## 1. 로봇 전원 켜기

1. **열쇠를 2시 방향**으로 돌린다
2. **파란 전원 버튼을 3초** 누른다
   - Beep 소리 1~2번 → 머리에 파란불 점등 확인
3. **리모컨 아무 버튼**을 한 번 누른다
   - 모터 E-Stop 해제 (안 하면 모터가 신호를 받지 못해 동작 안 됨)

---

## 2. 노트북에서 로봇 네트워크 연결

### Wi-Fi 연결
| 항목 | 값 |
|------|-----|
| SSID | `AIWORKER1016_5G` 또는 `AIWORKER1016` |
| 비밀번호 | `AIWORKER1016_5G` 또는 `AIWORKER1016` |

> 가능하면 5G 대역 사용 권장 (지연 낮음)

### SSH 접속
```bash
ssh robotis@ffw-SNPR48A1016.local
# 비밀번호: pica1847!
```

---

## 3. 텔레오퍼레이션 (Leader–Follower) 실행

SSH 접속 후 아래 순서로 진행한다.

```bash
cd ai_worker && bash docker/container.sh enter
ffw_sg2_ai
```

> 종료: `Ctrl+C`

---

## 4. 로봇 조작 (조이스틱)

| 입력 | 기능 |
|------|------|
| 왼쪽 조이스틱 | 머리 컨트롤 **또는** 베이스 이동 |
| 오른쪽 조이스틱 | 몸통 상하 이동 **또는** 몸통 상하 + 베이스 회전 |
| 양쪽 조이스틱 동시 누름 | 머리 컨트롤 ↔ 베이스 이동 모드 전환 |

---

## 5. 데이터 수집 (카메라 녹화)

텔레오퍼레이션이 실행된 상태에서 **별도 터미널**로 진행한다.

### 5-1. Docker 컨테이너 진입
```bash
cd ai_worker && bash docker/container.sh enter
```

### 5-2. 녹화 스크립트 위치로 이동
```bash
cd ~/ros2_ws/src/ai_worker/custom/shared/record
```

### 5-3. 녹화 시작
```bash
python3 record_cameras_headless.py <에피소드_이름>

# 예시
python3 record_cameras_headless.py episode_001
python3 record_cameras_headless.py pick_bowl_001
```

### 5-4. 녹화 진행 흐름

```
Waiting for cameras..........
All cameras ready: ['head', 'left', 'right']

Recording  →  ~/ros2_ws/src/ai_worker/custom/shared/record/videos/episode_001
Cameras    :  ['head', 'left', 'right']
Press Enter or Ctrl+C to stop.

  0:00:08  |  head:59  left:241  right:238
```

- **카메라 3개 모두** 신호가 들어와야 녹화 시작 (10초 내)
- 실시간으로 각 카메라 수신 프레임 수 표시
- **Enter** 또는 **Ctrl+C** 로 녹화 종료

### 5-5. 녹화 종료 및 인코딩

```
Stopped — 558 frames, 8.2s
  head:  59 frames @  7.2 fps  dim=(672, 376)
  left: 241 frames @ 29.4 fps  dim=(424, 240)
  right: 258 frames @ 31.5 fps  dim=(424, 240)
Re-encoding to H.264…
  ffmpeg: head.raw  → head.mp4
  ffmpeg: left.raw  → left.mp4
  ffmpeg: right.raw → right.mp4
Saved → .../videos/episode_001
```

종료 후 자동으로 H.264 MP4로 인코딩된다.

### 5-6. 저장 위치

```
custom/shared/record/videos/
└── episode_001/
    ├── head.mp4
    ├── left.mp4
    └── right.mp4
```

### 5-7. 카메라 오류 시

```
ERROR: cameras not responding: ['head', 'right']
Please restart the robot and try again.
```

카메라 3개가 모두 응답하지 않으면 녹화를 시작하지 않는다.
→ **로봇을 재시작**한 후 다시 시도한다.

---

## 6. 문제 해결

| 증상 | 원인 | 해결 |
|------|------|------|
| 모터가 움직이지 않음 | E-Stop 미해제 | 리모컨 아무 버튼 누르기 |
| SSH 접속 안 됨 | Wi-Fi 연결 안 됨 또는 로봇 미부팅 | Wi-Fi 재연결, 로봇 전원 확인 |
| 카메라 신호 없음 | 센서 노드 미실행 또는 부팅 불완전 | 로봇 재시작 후 launch 재실행 |
| 녹화 후 영상 없음 | Docker 미진입 상태로 실행 | `DAW` 로 컨테이너 진입 후 재시도 |
