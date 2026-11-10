# pinky_mission

Pinky Pro용 RViz2 미션 관리자 패키지입니다. `/clicked_point`로 네 지점을 받고, Marker와 상태를 발행하며, 안전 조건을 만족할 때 Nav2 `/navigate_to_pose` Action으로 배송 경로를 실행합니다.

## 주요 인터페이스

| 이름 | 타입 | 방향 |
|---|---|---|
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | 입력 |
| `/pinky_mission/command` | `std_msgs/msg/String` JSON | 입력 |
| `/pinky_mission/status` | `std_msgs/msg/String` JSON | 출력 |
| `/pinky_mission/markers` | `visualization_msgs/msg/MarkerArray` | 출력 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Action Client |

입력 상태는 `/map`, `/scan`, TF `map→base_footprint`, `/global_costmap/costmap_raw`, 카메라 이미지입니다.

## 안전 조건

- 기본 `allow_motion=false`
- 정확히 네 지점 필요
- Map, TF, LaserScan, Costmap, Nav2 Action Server 필요
- 모든 목표점 주변이 알려진 통행 가능 Costmap 영역이어야 함
- 이동 중 TF, LaserScan, Costmap 또는 Nav2가 끊기면 현재 Goal 취소 요청
- 취소 결과가 3초 안에 확인되지 않으면 하드웨어 정지 사용 경고

## 설정

- `config/mission_params.yaml`: 시간 제한, Topic, 저장 경로, footprint 검사 반경
- `config/aruco_markers.yaml`: ID 66/24/87/42와 네 역할의 매핑
- `config/example_mission.json`: 저장 형식 예시
- `rviz/pinky_mission.rviz`: Mission Panel과 Pinky Nav2 표시 설정

ArUco 설정에는 현재 ID와 역할만 있습니다. Marker 크기와 `map` 자세는 측정 전이므로 위치 추정에 사용하면 안 됩니다.

## 실행

```bash
ros2 launch pinky_mission mission.launch.py allow_motion:=false
```

실차 주행은 현장 안전 확인 후에만 `allow_motion:=true`로 실행합니다.

## 테스트

```bash
python3 -m unittest discover -s pinky_mission/test -v
ROS_DOMAIN_ID=68 python3 pinky_mission/test/smoke_ros.py
```
