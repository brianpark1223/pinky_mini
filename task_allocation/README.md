# 작업 배정 (cost 기반 우선순위)

cost 알고리즘은 이 폴더의 **`allocator.py`** 에 작성합니다. 2대 FMS 컨트롤러(`event_control/fms_controller.py`)가
`event_control/task_allocation.py`(연결부)를 거쳐 아래 두 함수를 부릅니다.

| 함수 | 하는 일 | 돌려줄 값 |
|---|---|---|
| `assign(free, queue, points)` | 지금 일을 받을 수 있는 Pinky에 대기 중인 요청을 나눠 줌 | `[(로봇이름, 작업), ...]` (로봇당 1건, 작업당 1대, `queue` 안의 작업 그대로) |
| `pickup_order(waiting, points)` | 픽업대가 비었을 때 기다리던 Pinky 중 누가 먼저 들어갈지 | 로봇 이름 (없으면 `None`) |

입력
- `free`: `{로봇이름: {'pose': (x, y) 또는 None, 'state': 'IDLE' 또는 'TO_HOME'}}`
- `queue`: `[{'id': 3, 'station': 'A'}, ...]` 요청 순서대로
- `points`: `{'STORE': (x, y, yaw), 'A': ..., 'B': ..., 'HOME': ...}`
- `waiting`: `{로봇이름: {'pose': (x, y) 또는 None, 'since': 기다리기 시작한 시각(초), 'task': 작업}}`

규칙
- 함수 이름과 입력·출력 모양만 지키면 안쪽은 자유롭게 바꿔도 됩니다.
- 이 파일이 없거나, 오류가 나거나, 잘못된 값을 돌려주면 컨트롤러는 기본 규칙(가까운 로봇 · 먼저 온 순)으로 계속 돌아갑니다.
  어떤 쪽을 쓰는지는 컨트롤러 시작 로그의 `allocator=` 에 나옵니다.
- 지금 `allocator.py`에는 기본 규칙이 그대로 들어 있습니다. 여기서부터 cost 계산식으로 바꾸면 됩니다.
- 다른 위치의 파일을 시험하려면 `PINKY_ALLOCATOR=/경로/allocator.py python3 fms_controller.py`.
