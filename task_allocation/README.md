# 작업 배정 (cost 기반 우선순위)

cost 알고리즘은 이 폴더의 **`allocator.py`** 에 있습니다. 2대 FMS 컨트롤러(`event_control/fms_controller.py`)가
`event_control/task_allocation.py`(연결부)를 거쳐 `allocator.py`의 `Allocator` 클래스를 씁니다.

## cost 식

```
Cᵢ = w_dist · clip(Dᵢ / D_max) + w_wait · clip(Wᵢ / W_max) + w_battery · (1 − Bᵢ / 100)
```

- **Dᵢ**: 새 요청을 위해 픽업대(STORE)까지 가야 하는 총 주행 거리 [m]
- **Wᵢ**: 멈춰 있는 시간 [s] (ArUco 정렬 + 픽업대 대기 + 지금 작업에서 남은 적재·수령 시간)
- **Bᵢ**: 배터리. 배터리 관리를 안 하므로 `w_battery = 0`
- `w_dist = 0.6`, `w_wait = 0.4`. `D_max`, `W_max`는 지점 좌표와 시간 설정으로 자동 계산
- cost 차이가 `tie_eps`(0.01) 이하면 동률로 보고 이름 순

로봇 상태별로 비기까지 남은 일(설계 케이스 ①~⑤)은 `Allocator._remaining()`에 있습니다.

## 컨트롤러와 연결되는 방식

| Allocator | 컨트롤러가 부르는 때 | 결과 |
|---|---|---|
| `dispatch(robots)` | 대기열에 요청이 있는 동안 0.2초마다 | FMS가 배정 가능하다고 표시한 `IDLE` 로봇 중 cost가 가장 낮은 로봇이 바로 출발 |
| `next_for(robot)` | 로봇이 [수령 완료]를 받았을 때 | 대기열 맨 앞 요청을 이어서 맡아 수납대를 거치지 않고 바로 픽업대로. 없으면 수납대로 복귀 |

`Allocator` 자체는 `TO_HOME`도 후보로 다룰 수 있지만, 현재 FMS는 복귀 goal 완료 전에는 `free=false`로 전달해 재배정을 막습니다.

연결부가 맞춰 주는 것

- **대기열은 컨트롤러 것이 기준**입니다. 부를 때마다 `alloc.queue`를 컨트롤러 대기열로 새로 채웁니다.
- 로봇 정보는 매번 `Robot(name, state, pos, task, state_elapsed_s, battery)`로 만들어 넘깁니다.
  - `DOCKING`(Allocator에 없는 상태)은 픽업대 위치의 `TO_STORE`로 넘깁니다.
  - Nav2가 아직 준비되지 않은 로봇, 작업 정보가 없는 바쁜 로봇은 `ERROR`로 넘겨서 고르지 않게 합니다.
  - 위치(AMCL)를 아직 모르면 가고 있는 지점, 그것도 없으면 수납대 위치로 봅니다.
- `Config`의 시간·속도는 컨트롤러 파라미터를 씁니다: `v` ← `avg_speed`(0.15 m/s), `t_load_s` ← `t_loading`(20 s),
  `t_receive_s` ← `t_unloading`(15 s). 도킹을 끄면(`docking:=false`) `t_align_s = 0`. 나머지(가중치, `tie_eps` 등)는 `allocator.py` 값 그대로.
- 거리 함수는 직선거리 × `detour_factor`(1.25).
- 픽업대 순서(`pickup_order`)는 `allocator.py`에 없으므로 먼저 기다린 로봇부터(기본 규칙).

## 기본 규칙으로 넘어가는 경우

- `allocator.py`가 없거나, 불러올 때 오류가 나거나, `dispatch()`가 오류를 내거나 잘못된 값(없는 로봇, 비어 있지 않은 로봇 등)을
  돌려주면 그 번에는 기본 규칙(픽업대에 가장 가까운 빈 로봇)으로 처리하고 터미널에 이유를 출력합니다.
- 어느 쪽을 쓰는지는 컨트롤러 시작 로그의 `allocator=`에 나옵니다 (`task_allocation/allocator.py (Allocator class)`).
  파일을 고치면 컨트롤러를 다시 켜야 반영됩니다.
- 다른 위치의 파일을 시험하려면 `PINKY_ALLOCATOR=/경로/allocator.py python3 fms_controller.py`.
- 예전 함수 형식(`assign(robots, queue, points, ctx)`, `pickup_order(waiting, points)`)도 그대로 동작합니다.
