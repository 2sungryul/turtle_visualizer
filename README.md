# Turtle Tracker

ROS2 Jazzy용 turtlesim 궤적 추적 패키지입니다. OpenCV를 사용하여 거북이의 움직임을 실시간으로 시각화하고, desired path와 함께 표시합니다.

## 기능

- `/turtle1/pose` 토픽 구독 - 거북이의 현재 위치와 방향 추적
- `/desired_path` 토픽 구독 - 목표 경로 표시
- turtlesim과 동일한 크기(500x500)의 윈도우 생성
- 거북이의 궤적을 빨간색 선으로 표시
- 목표 경로를 초록색 선과 점으로 표시
- 현재 거북이 위치를 초록색 원으로 표시
- 거북이 방향을 파란색 화살표로 표시
- 키보드 단축키:
  - `c`: 궤적 초기화 (목표 경로는 유지)
  - `q`: 프로그램 종료

## 빌드 방법

```bash
cd ~/turtle_tracker_ws
colcon build --packages-select turtle_tracker
source install/setup.bash
```

## 실행 방법

### 방법 1: 개별 실행

터미널 1:
```bash
ros2 run turtlesim turtlesim_node
```

터미널 2:
```bash
ros2 run turtle_tracker turtle_tracker_node
```

터미널 3 (거북이 조종):
```bash
ros2 run turtlesim turtle_teleop_key
```

### 방법 2: Launch 파일 사용

```bash
ros2 launch turtle_tracker turtle_tracker.launch.py
```

별도 터미널에서 거북이 조종:
```bash
ros2 run turtlesim turtle_teleop_key
```

### 방법 3: path_publisher와 함께 사용

모든 노드를 한 번에 실행:
```bash
ros2 launch turtle_tracker turtle_with_path.launch.py
```

별도 터미널에서 거북이 조종:
```bash
ros2 run turtlesim turtle_teleop_key
```

또는 개별 실행:
```bash
# 터미널 1: turtlesim
ros2 run turtlesim turtlesim_node

# 터미널 2: tracker
ros2 run turtle_tracker turtle_tracker_node

# 터미널 3: path publisher
ros2 run path_publisher circular_path_publisher_node

# 터미널 4: 거북이 조종
ros2 run turtlesim turtle_teleop_key
```

## 의존성

- ROS2 Jazzy
- rclcpp
- turtlesim
- geometry_msgs
- nav_msgs
- cv_bridge
- OpenCV 4.x

## 패키지 구조

```
turtle_tracker/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── turtle_tracker_node.cpp
├── include/
│   └── turtle_tracker/
└── launch/
    └── turtle_tracker.launch.py
```

## 주요 특징

1. **좌표계 변환**: turtlesim의 좌표계(원점이 왼쪽 아래)를 OpenCV 좌표계(원점이 왼쪽 위)로 자동 변환
2. **실시간 렌더링**: 30Hz로 화면 업데이트
3. **메모리 관리**: 궤적 포인트가 10,000개를 초과하면 오래된 데이터 자동 삭제
4. **사용자 인터페이스**: 현재 저장된 포인트 수, 방향각, 경로 정보 표시 및 키보드 단축키 안내
5. **다중 경로 시각화**: 실제 거북이 궤적(빨간색)과 목표 경로(초록색) 동시 표시
6. **방향 표시**: 파란색 화살표로 거북이의 현재 방향 표시

## 작동 원리

### Pose 추적
1. `/turtle1/pose` 토픽에서 거북이의 위치(x, y)와 방향(theta) 수신
2. turtlesim 좌표(0~11.088889)를 픽셀 좌표(0~500)로 스케일링
3. 이전 위치와 현재 위치 사이에 빨간색 선 그리기
4. 현재 위치에 원 표시, 방향에 파란색 화살표 표시
5. 30Hz로 화면 업데이트

### Path 표시
1. `/desired_path` 토픽에서 nav_msgs/Path 메시지 수신
2. 경로의 각 포인트를 초록색 선으로 연결
3. 각 포인트에 작은 초록색 원 표시
4. 거북이 궤적과 함께 실시간 업데이트

## 문제 해결

- **OpenCV 윈도우가 표시되지 않는 경우**: 디스플레이 환경 확인 (X11, Wayland)
- **토픽을 받지 못하는 경우**: `ros2 topic list`로 토픽 확인
- **빌드 오류**: OpenCV 패키지 설치 확인 (`sudo apt install libopencv-dev`)
