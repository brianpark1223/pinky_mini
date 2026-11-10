# 테스트

현재 실행 가능한 테스트는 각 ROS 패키지와 함께 관리합니다.

- `pinky_mission/test/test_logic.py`: 여섯 경로, 방향, Costmap 검사
- `pinky_mission/test/smoke_ros.py`: 클릭, Marker, 주행 차단, 저장·불러오기

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m unittest discover -s pinky_mission/test -v
ROS_DOMAIN_ID=68 python3 pinky_mission/test/smoke_ros.py
```

두 대 로봇 배정과 ArUco 도킹이 구현되면 이 디렉터리에 저장소 전체 통합 테스트를 추가합니다.
