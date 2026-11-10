# 테스트

현재 테스트는 `event_control/`에 있습니다. 해당 폴더에서 실행합니다.

```bash
python3 test_fms.py fms_controller.py
python3 test_fsm.py parts_delivery.py
python3 test_notifier.py robot_notifier.py
mkdir -p /tmp/pinky_gui_test
QT_QPA_PLATFORM=offscreen python3 test_gui.py delivery_gui.py /tmp/pinky_gui_test
```

GUI 테스트에는 NumPy와 PyQt5 또는 PyQt6가 필요합니다.
이 테스트들은 ROS 통신을 모사하며 실제 로봇·Nav2 통합 시험을 대체하지 않습니다.
이 폴더는 향후 테스트 분리를 위한 위치입니다.
