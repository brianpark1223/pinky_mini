# 작업 배정

현재 구현은 [event_control/task_allocation.py](../event_control/task_allocation.py)에 있습니다.
2대 FMS가 이 파일의 `assign()`과 `pickup_order()`를 호출합니다.

- `assign()`: 요청 순서대로 픽업대까지 직선거리가 가장 짧은 가용 로봇에 배정
- `pickup_order()`: 먼저 기다리기 시작한 로봇부터 픽업대 사용

이 폴더는 향후 모듈 분리를 위한 위치입니다. 지금은 컨트롤러가 불러오는 위 파일을 수정합니다.
