# Pinky 부품 배송 및 RViz 미션 시스템

ROS 2 Jazzy, Nav2, RViz2를 기반으로 Pinky Pro 로봇의 지도 기반 배송 임무를 설정하고 실행하는 프로젝트입니다. 장기 목표는 두 대의 Pinky를 배정하는 소형 FMS이며, 현재 저장소에는 한 대의 로봇으로 검증한 RViz 미션 시스템과 관련 설정이 포함됩니다.

## 현재 구현된 범위

- RViz2 `Publish Point` 도구로 네 지점 지정
- `1 대기`, `2 픽업`, `3 배송 1`, `4 배송 2` Marker 표시
- 고정 연결선 없이 지점만 표시하고 Nav2의 현재 계획 경로(`/plan`) 표시
- RViz2 안에서 동작하는 Qt Mission Panel
- Nav2 `/navigate_to_pose` Action을 이용한 순차 이동
- 지도, TF, LiDAR, Costmap, Nav2 상태를 확인하는 주행 전 검사
- 목표점 Costmap 통행 가능 여부 검사
- 취소, 일시 정지(현재 Goal 취소), 재개, 임무 저장·불러오기
- 대기 및 카메라 촬영 임무 확장 구조
- ROS_DOMAIN_ID 42에서 노트북↔Pinky 양방향 DDS 통신 확인
- 실습실 지도 `sesac_gangnam_20261110`

## 네 지점과 여섯 배송 경로

| 지점 | 역할 | ArUco ID |
|---|---|---:|
| 1 | 대기 위치 | 66 |
| 2 | 픽업 위치 | 24 |
| 3 | 배송 위치 1 | 87 |
| 4 | 배송 위치 2 | 42 |

| 모드 | 이동 순서 |
|---:|---|
| 1 | `1 → 2 → 3 → 1` |
| 2 | `1 → 2 → 4 → 1` |
| 3 | `1 → 2 → 3 → 4 → 1` |
| 4 | `1 → 2 → 4 → 3 → 1` |
| 5 | `1 → 2 → 3 → 2 → 4 → 1` |
| 6 | `1 → 2 → 4 → 2 → 3 → 1` |

## ROS 2 노드

| 노드 | 구현 | 역할 |
|---|---|---|
| `pinky_mission_manager` | Python / rclpy | 지점, 임무 순서, Nav2 Action, 상태, 안전 검사, 저장 |
| `pinky_mission_panel_1` | C++ / rclcpp | RViz Panel 명령 발행 및 상태 표시 |

ArUco 위치 인식 노드는 아직 구현하지 않았습니다. 현재는 ID 매핑과 카메라·보정 요구사항만 정리되어 있습니다. 실제 `aruco_localizer` 노드는 카메라 내참수, Marker 실측 크기, 각 Marker의 `map` 좌표를 확정한 뒤 추가해야 합니다.

## 디렉터리

```text
.
├── pinky_mission/              # rclpy 미션 관리자 패키지
├── pinky_mission_panel/        # RViz2 C++ Panel 패키지
├── start_pinky_mission.sh      # 노트북 실행 스크립트
├── event_control/maps/         # 저장한 OccupancyGrid 지도
├── aruco_docking/              # ArUco 도킹·위치 인식 설계
├── task_allocation/            # 경로 및 향후 다중 로봇 배정
├── config/                     # 공통 설정 안내
├── launch/                     # 통합 실행 안내
├── tests/                      # 테스트 안내
└── docs/interfaces.md          # 팀 인터페이스 초안
```

패키지 내부에 필요한 launch, config, RViz 설정과 테스트를 함께 두었습니다. 루트의 기존 모듈 폴더에는 같은 파일을 복제하지 않고 실제 구현 위치를 안내합니다.

## 빌드

환경: Ubuntu 24.04, ROS 2 Jazzy, Nav2, RViz2, Python 3, Qt5

```bash
mkdir -p ~/dev_ws/ros/src
cd ~/dev_ws/ros/src
git clone https://github.com/brianpark1223/pinky_mini.git
cd ..

source /opt/ros/jazzy/setup.bash
colcon build --packages-select pinky_mission pinky_mission_panel
```

`colcon`은 저장소 아래의 두 ROS 패키지를 재귀적으로 찾습니다. 빌드 후에는 저장소 루트 또는 워크스페이스 루트에서 실행 스크립트를 사용할 수 있습니다.

## 실행 순서

### 1. Pinky: 하드웨어와 LiDAR

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch pinky_bringup bringup_robot.launch.xml
```

### 2. Pinky: 지도, AMCL, Nav2

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch pinky_navigation bringup_launch.xml \
  map:=/home/pinky/maps/sesac_gangnam_20261110.yaml
```

### 3. 노트북: RViz 미션 시스템

처음에는 주행 차단 상태로 확인합니다.

```bash
cd ~/dev_ws/ros/src/pinky_mini
./start_pinky_mission.sh
```

현장 정지 수단과 Nav2 상태를 확인한 뒤 실제 주행을 허용합니다.

```bash
ROS_DOMAIN_ID=42 ./start_pinky_mission.sh allow_motion:=true
```

스크립트는 중복 실행을 잠금 파일로 차단합니다.

## RViz 사용

1. 지도, TF, LaserScan, Costmap과 로봇 모델이 표시되는지 확인합니다.
2. `2D Pose Estimate`로 실제 로봇의 초기 위치와 방향을 설정합니다.
3. `Publish Point`로 대기, 픽업, 배송 1, 배송 2를 순서대로 클릭합니다.
4. Mission Panel에서 배송 모드를 선택하고 `설정 적용`을 누릅니다.
5. `Nav2`, `Map`, `Scan`, `Costmap`이 모두 정상이고 오류가 없을 때 임무를 시작합니다.

`2D Goal Pose`는 이 미션 시스템에서 사용하지 않습니다. Mission Panel이 Nav2 Action Goal을 전송합니다.

## 지도

`event_control/maps/sesac_gangnam_20261110.yaml`과 `.pgm`은 2026-11-10에 저장한 지도입니다.

- 해상도: 0.05 m/cell
- 크기: 214 × 326
- 원점: `[-7.981, -8.156, 0]`

지도는 실제 운영 환경에 맞는지 확인한 후 사용해야 합니다.

## 테스트

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 -m unittest discover -s pinky_mission/test -v
ROS_DOMAIN_ID=68 python3 pinky_mission/test/smoke_ros.py
```

검증된 항목:

- 여섯 배송 경로의 정확한 순서
- 자동 Yaw 계산
- Costmap 점유·미확인·경계 검사
- 네 번의 `/clicked_point` 입력과 Marker 발행
- 기본 주행 차단
- 저장, 되돌리기, 불러오기
- 두 ROS 패키지의 colcon 빌드

실제 로봇에서 계속 확인할 항목:

- 새 지도에서 AMCL 수렴과 장시간 주행
- Nav2 취소 후 바퀴 정지 확인
- 공용 Wi-Fi 장애 상황
- 카메라 보정과 ArUco 기반 위치 추정
- 두 대의 Pinky 통합 배정

## 안전

- `allow_motion=false`가 기본값입니다.
- 지도 클릭만으로 로봇이 움직이지 않습니다.
- 임무 실패 시 다음 지점으로 진행하지 않습니다.
- 소프트웨어 취소는 하드웨어 비상 정지를 대신하지 않습니다.
- 실차 시험 중에는 현장 감시자와 검증된 정지 수단이 필요합니다.

## 기존 FMS 확장 계획

기존 저장소의 `event_control`, `task_allocation`, `aruco_docking` 구조는 두 대의 로봇을 위한 FMS 확장 위치로 유지합니다. 픽업대 점유, 공용 대기열, Cost 기반 로봇 배정, 도킹 제어권 전환은 [팀 인터페이스 초안](docs/interfaces.md)을 기준으로 후속 구현합니다.
