# 이벤트 제어

FMS 상태 머신, 관제 GUI, 로봇 알림 코드를 관리합니다.

## 추가할 파일

- `parts_delivery.py`: FMS 컨트롤러와 상태 전이
- `delivery_gui.py`: 작업 요청·적재·수령 확인 화면
- `robot_notifier.py`: 상태별 알림과 전방 근접 경고

지도는 `maps/`, 알림음은 `sounds/`에 넣습니다.
현재는 폴더 구조만 준비되어 있으며 실행 코드는 아직 업로드되지 않았습니다.
공통 연동 규칙은 [팀 인터페이스](../docs/interfaces.md)에 정리합니다.
