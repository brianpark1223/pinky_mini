# 통합 실행

현재 실행 가능한 launch 파일은 `pinky_mission/launch/mission.launch.py`입니다. 같은 파일을 이 디렉터리에 복제하지 않습니다.

노트북에서는 루트의 `start_pinky_mission.sh`를 사용합니다. Pinky 하드웨어, 지도와 Nav2는 Pinky 공식 패키지의 `pinky_bringup`, `pinky_navigation` launch를 먼저 실행해야 합니다.

향후 두 대의 로봇, FMS, ArUco 도킹을 한 번에 실행하는 상위 launch 파일만 이 디렉터리에 추가합니다.
