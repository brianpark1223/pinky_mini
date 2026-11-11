# 테스트

현재 테스트는 `event_control/`에 있습니다. 해당 폴더에서 실행합니다.

```bash
python3 -m unittest discover -s . -p test_sim.py
python3 test_fms.py fms_controller.py
python3 test_fsm.py parts_delivery.py
python3 test_notifier.py robot_notifier.py
mkdir -p /tmp/pinky_gui_test
QT_QPA_PLATFORM=offscreen python3 test_gui.py delivery_gui.py /tmp/pinky_gui_test
```

GUI 테스트에는 NumPy와 PyQt5 또는 PyQt6가 필요합니다.
이 테스트들은 ROS 통신을 모사하며 실제 로봇·Nav2 통합 시험을 대체하지 않습니다.
이 폴더는 향후 테스트 분리를 위한 위치입니다.

## RViz 미션 패키지 테스트

`pinky_mission` 테스트는 패키지 안에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m unittest discover -s src/pinky_mini/pinky_mission/test -v
ROS_DOMAIN_ID=68 python3 src/pinky_mini/pinky_mission/test/smoke_ros.py
```

- `test_logic.py`: 여섯 배송 경로, 자동 방향, Costmap 검사
- `smoke_ros.py`: 클릭, Marker, 기본 주행 차단, 저장·되돌리기·불러오기
