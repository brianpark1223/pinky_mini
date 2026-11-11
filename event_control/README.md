# 이벤트 제어: FMS 컨트롤러 · 관제 GUI · 로봇 알림 (v5 · Pinky 2대)

ROS2 Jazzy / Python3. 의존 패키지: rclpy, nav2_msgs, action_msgs, std_msgs, std_srvs, sensor_msgs, geometry_msgs.
**Pinky 2대는 `fms_controller.py`** 로 운영합니다. `parts_delivery.py`는 로봇 1대로 시험할 때 쓰는 1대용 컨트롤러입니다.
두 컨트롤러의 `POINTS`는 예시 좌표입니다. 실물 모드 전에 실제 map 좌표로 바꾸세요(두 파일 모두).

## 파일 구성과 실행 위치

| 파일 | 메모 번호 | 실행 위치 | 역할 |
|---|---|---|---|
| `fms_controller.py` | - | 노트북 | **2대용 FMS 컨트롤러**: 로봇별 상태 머신, 로봇별 Nav2, 픽업대 점유, 수납대 자리 나눔, ArUco 도킹 연결 |
| `task_allocation.py` | - | (컨트롤러가 불러씀) | **작업 배정 연결부**: 레포의 `task_allocation/allocator.py`(cost 알고리즘)가 있으면 그걸 쓰고, 없거나 오류면 기본 규칙(가까운 로봇 · 먼저 온 순) 사용 |
| `parts_delivery.py` | 1, 4(로직), 6 | 로봇 또는 노트북 | 1대용 컨트롤러 (로봇 한 대 시험용) |
| `robot_notifier.py` | 3, 5 | **로봇** (스피커가 달린 쪽) | 대기 상태 진입 시 노래, 주행 중 전방 근접 시 삐빅 |
| `delivery_gui.py` | 4(GUI) | 노트북 | 요청/확인 버튼, 로봇·작업 상태, 카메라, 로그, RViz 실행 |
| `sounds/song.wav`, `sounds/beep.wav` | 3, 5 | 로봇 | 직접 만든 짧은 알림음 (원하는 wav로 교체 가능) |
| `test_*.py` | - | 어디서나 | ROS 없이 로직 검증 (`test_fms.py` = 2대 컨트롤러) |

지점: 픽업대 = `STORE`, 작업대1 = `A`, 작업대2 = `B`, 수납대 = `HOME` (1곳, 두 Pinky 공용)

## 지점 (모두 고정 좌표)

모든 지점은 `parts_delivery.py`의 `POINTS`에 직접 적습니다(map 좌표 x m, y m, 방향 rad). RViz로 찍어 등록하는 기능은 없앴습니다.

| 키 | 이름 | 용도 |
|---|---|---|
| `STORE` | 픽업대 | ArUco 마커 앞. 한 번에 한 대만 사용 |
| `A`, `B` | 작업대1, 작업대2 | 배달 목적지 |
| `HOME` | 수납대 | 배송을 마친 Pinky가 돌아가는 자리. 1곳을 두 Pinky가 같이 씀 (2대 컨트롤러가 자리를 나눠야 함, 아래 참고) |

좌표 확인: RViz의 Publish Point로 찍고 `ros2 topic echo /clicked_point`, 또는 로봇을 그 자리에 두고 GUI ③ 탭 위치 칸을 읽습니다.
컨트롤러가 상태 JSON에 `points`를 실어 보내면 GUI 지도는 그 값을 씁니다.

## 팀 인터페이스 (이것만 지키면 각자 수정해도 연동됨)

- 서비스 (`std_srvs/srv/Trigger`): `delivery/request_a`, `delivery/request_b`, `delivery/loaded`,
  `delivery/received`, `delivery/retry`, `delivery/reset`
- 상태 토픽 `delivery/status` (`std_msgs/String`, JSON, 0.2초 주기)

| 필드 | 사용하는 쪽 | 설명 |
|---|---|---|
| `state` | GUI, notifier | **필수.** IDLE / TO_STORE / WAIT_LOAD / TO_STATION / WAIT_RECEIVE / TO_HOME / ERROR |
| `current`, `queue` | GUI | 작업 `{id, station, ...}`. 추가 키(count 등)는 GUI에 그대로 표시됨 |
| `batch` | GUI | (선택) 한 번에 싣고 가는 작업 목록. task 로직에서 묶음 배송을 넣으면 채워주세요 |
| `target`, `error`, `nav_ready`, `dry_run`, `timeout_left` | GUI | 표시용 |
| `distance_remaining` | GUI, notifier | Nav2 feedback의 남은 거리(m). notifier가 목적지 근처에서 삐빅을 끄는 데 사용 |

상태 이름을 바꾸면 `robot_notifier.py`의 `wait_states`/`MOVING`, `delivery_gui.py`의 `STATE_STYLE`도 맞춰 주세요.

### 팀 인터페이스 · 2대 (`fms_controller.py`가 이 형식을 씀)

컨트롤러 하나가 두 Pinky를 관리하고, 상태는 `delivery/status` 하나에 로봇별로 넣습니다.
GUI는 `robots` 필드가 있으면 2대 화면, 없으면 위의 1대 형식으로 동작합니다.

```json
{
  "robots": {
    "pinky_1": {"state": "TO_STATION", "current": {"id": 3, "station": "A", "robot": "pinky_1"},
                "target": "A", "distance_remaining": 2.4, "timeout_left": 70, "error": null,
                "nav_ready": true, "home": "HOME"},
    "pinky_2": {"state": "WAIT_PICKUP", "current": {"id": 4, "station": "B", "robot": "pinky_2"},
                "target": null, "nav_ready": true, "home": "HOME"}
  },
  "queue": [{"id": 5, "station": "A"}],
  "pickup": "pinky_1",
  "dry_run": false,
  "points": {"STORE": [-5.64, 2.1, 0.0], "...": "..."}
}
```

| 항목 | 내용 |
|---|---|
| 요청 서비스 | `delivery/request_a`, `delivery/request_b` (공용, 그대로) |
| 로봇별 서비스 | `delivery/<robot>/loaded`, `delivery/<robot>/received`, `delivery/<robot>/retry`, `delivery/<robot>/reset` |
| 로봇 상태 | 1대와 같음 + `WAIT_PICKUP` (픽업대를 다른 Pinky가 쓰는 중 → 그 자리에 멈춰 대기) + `DOCKING` (픽업대 앞 ArUco 정렬 중, `docking:=true`일 때) |
| 배정 | 대기열은 하나. 비어 있는(IDLE 또는 수납대로 복귀 중인) Pinky에 `task_allocation.assign()`이 정한 대로 줌. 기본 규칙 = 요청 순서대로, 픽업대에 가장 가까운 Pinky. 맡긴 작업에는 `robot` 필드가 붙음 |
| 픽업대 | 한 번에 한 대(`pickup` = 점유한 로봇). 출발할 때 점유를 잡고 적재 후 지도 위치가 픽업대 반경 밖으로 벗어난 것이 확인될 때 풂. 기다리는 Pinky는 차례대로 진입 |
| 수납대 | 1곳을 자리 2개로 나눠 씀: 첫 번째 = `HOME`, 두 번째 = `HOME`에서 왼쪽으로 `home_slot_offset`(0.35 m). 0이면 둘 다 `HOME` |
| 오류 | 그 로봇만 `ERROR`. Nav2 실패·취소는 최종 결과를 확인하고, 픽업대·작업대 잠금은 새로운 지도 위치로 이탈이 확인될 때 풂. 결과가 불명확하면 잠금을 유지하고 현장 확인 필요 |
| 로봇별 토픽 | `/<robot>/amcl_pose`, `/<robot>/plan`, `/<robot>/delivery/dock`, `/<robot>/camera/image_raw/compressed` (Nav2를 로봇별 namespace로 띄우면 이 이름이 됨). TF는 `<robot>/base_footprint`(frame_prefix)가 있으면 사용 |

GUI는 대기열 작업마다 어느 Pinky가 맡을지, 픽업대가 겹칠 때 기다리는 시간까지 넣어 도착 시간을 예상합니다.

- **ArUco 도킹 연결** (`fms_controller.py --ros-args -p docking:=true`일 때)
  1. Nav2가 픽업대(`STORE`)에 도착하면 컨트롤러가 `/<robot>/delivery/dock_start` (`std_srvs/Trigger`)를 부르고 `DOCKING` 상태가 됩니다.
  2. 도킹 노드는 `/<robot>/delivery/dock`으로 아래 JSON을 5~10Hz로 보내고, 끝나면 `"state": "done"`, 실패하면 `"state": "failed"`를 보냅니다.
  3. `done` → `WAIT_LOAD`, `failed` 또는 `dock_timeout`(60초) → `ERROR`.
  `docking:=false`(기본)면 Nav2 도착 즉시 `WAIT_LOAD`입니다. 도킹 노드가 준비되기 전까지는 이대로 쓰면 됩니다.
- 도킹 피드백 토픽 `/<robot>/delivery/dock` (`std_msgs/String`, JSON, 카메라 프레임마다 5~10Hz) — **ArUco 도킹 노드가 발행**

| 필드 | 설명 |
|---|---|
| `distance` | 멈출 지점까지 남은 거리(m) = 마커까지 거리 − 멈춤 거리 |
| `angle_deg` | 마커가 로봇 정면에서 벗어난 각도(°, 오른쪽 +). 정렬에 돌아야 하는 양 |
| `visible` | 이번 프레임에 마커가 보였는지. `false`면 GUI는 직전 거리를 유지하고 "마커 찾는 중" 표시 |
| `state` | `running` / `done` / `failed` (컨트롤러가 도킹 완료·실패 판단에 사용). 실패 이유는 `reason`에 |

```python
# 카메라 노드 예시 (OpenCV 4.7+): 마커 한 변 marker_len(m), 카메라 행렬 K, 왜곡 dist
corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
if ids is not None:
    obj = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], np.float32) * marker_len / 2
    _, rvec, tvec = cv2.solvePnP(obj, corners[0][0], K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    x, z = float(tvec[0]), float(tvec[2])              # 카메라 기준: x 오른쪽, z 앞
    fb = {'distance': max(0.0, math.hypot(x, z) - stop_distance),
          'angle_deg': math.degrees(math.atan2(x, z)), 'visible': True}
else:
    fb = {'visible': False}
fb['state'] = 'done' if docked else 'running'          # 정렬이 끝나면 done, 포기하면 failed
pub.publish(String(data=json.dumps(fb)))
```

## 3, 5번: robot_notifier.py (로봇에서 실행)

```bash
sudo apt install alsa-utils            # aplay 없을 때
aplay sounds/song.wav                  # 스피커 출력 먼저 확인
python3 robot_notifier.py
# Pinky 2대: 로봇마다 자기 이름으로 실행 (상태의 robots[이름]만 따라감)
python3 robot_notifier.py --ros-args -p robot_name:=pinky_2
# 조정 예시
python3 robot_notifier.py --ros-args -p beep_distance:=0.5 -p sector_deg:=40.0 \
    -p song_repeat:=20.0 -p scan_topic:=/scan
```

- **노래**: WAIT_LOAD(픽업 장소 도착), WAIT_RECEIVE(작업대 도착)에 들어가면 재생하고, 계속 대기하면 `song_repeat`초마다 다시 재생합니다. 상태가 바뀌면 바로 멈춥니다.
- **삐빅**: 주행 중(TO_*)에만, LiDAR 전방 `sector_deg` 안에 `beep_distance`보다 가까운 물체가 있으면 `beep_interval`초마다 울립니다. 목표까지 `near_goal_mute` 미만이면 작업대/벽 때문에 울리지 않게 꺼 둡니다.
- LiDAR는 사람과 물체를 구분하지 못합니다. 좁은 통로에서 벽 때문에 계속 울리면 `sector_deg`와 `beep_distance`를 줄이세요.
- LiDAR 0°가 로봇 정면이 아니면 `front_offset_deg`로 보정합니다. 확인 방법: 로봇 정면에 손을 대고 로그의 "ahead" 거리가 줄어드는지 봅니다.
- 버저가 GPIO/토픽 방식이라 wav 재생이 안 되면 `Player.play()`만 그 방식으로 바꾸면 됩니다. `-p song_file:='' -p beep_file:=''`이면 소리 없이 로그만 남깁니다.

## 4번: delivery_gui.py (노트북에서 실행)

```bash
sudo apt install python3-pyqt5      # 또는 PyQt6 (pip install PyQt6) - 둘 중 있는 것을 자동으로 사용
python3 delivery_gui.py --ros-args -p station_labels:='작업대1,작업대2' \
    -p camera_topic:=/camera/image_raw/compressed -p rviz_config:=/path/to/nav.rviz
```

- **옵션 없이 실행해도 됨** (VS Code 실행 버튼 포함): `~/maps/sesac_gangnam_2.yaml` → 이 폴더의 `maps/sesac_gangnam_2.yaml` → `~/maps/sesac_gangnam.yaml` 순서로 있는 지도를 자동으로 쓰고,
  작업대 이름은 기본 "작업대1, 작업대2", 로봇 프레임은 `base_footprint` → 없으면 `base_link`를 자동 선택합니다.
- 화면은 브라우저처럼 탭 3개로 나뉩니다. 위쪽 상태 배지와 아래 로그는 어느 탭에서나 보입니다.
  - **작업 요청**: 작업대별 요청 버튼, 지금 처리 중인 작업과 대기열 (탭 제목에 대기 건수)
  - **적재 확인 · 수령 현황**: 지금 할 일 안내, 픽업대에서 누르는 [적재 완료], 수령은 상태만 표시 (적재 확인이 필요하면 탭에 주황 점)
  - **Pinky 현재 상태**: 상태·진행 단계·위치, 오류와 재시도/취소, RViz, 카메라 (오류면 탭에 빨간 점)
- **대기 안내**: 요청 버튼을 눌렀을 때 Pinky가 다른 배송 중이면 화면 위에 안내가 12초간 뜹니다.
  예) `작업대2 요청 접수 · 현재 Pinky가 작업대1 쪽으로 배달하러 가는 중이라 대기해야 합니다. (대기 순번 1번)`
  대기 순번은 컨트롤러 응답의 `(position N)`을 읽고, 없으면 대기열 길이로 계산합니다.
- **지도 (세 탭 모두)**: RViz 대신 `/map`, `/amcl_pose`, `/plan`을 직접 그린 미니 지도입니다. 지점 위치는
  `parts_delivery.py`의 POINTS를 자동으로 읽습니다(다른 곳에 있으면 `-p points:='{"STORE":[0,0,0],"A":[1,0,0]}'`).
  로봇 원 색 = 현재 상태, 주황 지점 = 목적지, 점선 = Nav2 경로. 자세한 확인은 "RViz 열기"를 쓰세요.
- **새 요청 팝업**: 상태 토픽에 새 작업 번호가 보이면(어느 GUI에서 요청했든) "작업대2에서 부품 요청이 들어왔습니다." 팝업과
  알림음을 띄웁니다. **② 적재 확인 · 수령 현황 탭을 열고 있을 때만** 뜨고, 다른 탭을 보는 동안 들어온 요청은 보관했다가
  ② 탭을 열 때 (아직 적재 전인 것만) 보여줍니다. ② 탭에서 나가면 팝업은 닫힙니다. 여러 건이면 쌓아서 보여줍니다.
  요청 전용 화면처럼 팝업이 필요 없는 GUI는 `-p popup_on_request:=false`.
- 부품 요청 버튼은 누르면 **2초 동안 잠깁니다** (더블클릭·오클릭 방지, 잠긴 동안 회색). 시간 변경: `-p request_lock_sec:=3.0`, 끄기: `0`.
  팝업에는 **어느 Pinky에 적재할지**("pinky_1에 적재해 주세요")와 그 Pinky가 언제 픽업대에 오는지가 함께 나옵니다.
  로봇 이름은 작업의 `robot` 필드 > 상태 JSON의 `robot` 필드 > `-p robot_name:=pinky_1` 순서로 정합니다.
  지금은 1대라 파라미터 값이 쓰이고, 여러 대가 되면 task 로직이 작업마다 `robot`을 채우면 됩니다.
- **도착까지 남은 시간**: ① 탭에 `2m 05s left` 형식으로 표시합니다. 지금 달리는 구간은 Nav2 남은 거리 ÷ 속도라 줄어들고,
  아직 안 달린 구간은 지점 사이 직선거리 × `detour_factor`(1.25) ÷ 속도라 적재·수령 대기 중에는 멈춰 있습니다.
  속도는 `nominal_speed`(0.15 m/s)로 시작해 구간을 하나 달릴 때마다 실제 이동 거리/시간으로 보정합니다(로그에 표시).
  Nav2는 목표 0.3m 안에 들어오면 도착으로 판정하므로, 구간마다 0.3m를 빼고 계산합니다(`-p arrive_tolerance:=0.3`,
  Nav2의 xy_goal_tolerance와 같은 값으로 맞추세요).
  대기열 작업도 "약 _m_s left"로 보이지만 사람이 싣고 내리는 시간은 빠져 있어 실제보다 짧게 나옵니다.
- **픽업대 ArUco 도킹 반영**: 픽업대로 가는 구간 = Nav2 이동 + 마커 정렬, 두 단계로 나눠 계산합니다.

  | 단계 | 남은 거리를 어디서 | 시간 |
  |---|---|---|
  | Nav2 이동 | Nav2 feedback `distance_remaining` | (남은 거리 − 0.3) ÷ 평균 속도 |
  | 마커 정렬 (도킹 중) | 카메라 `delivery/dock`의 `distance`, `angle_deg` | 거리 ÷ `dock_speed` + \|각도\| ÷ `turn_speed` |
  | 마커 정렬 (아직 시작 전) | `dock_approach` (Nav2 목표 → 멈춤 지점, 0.4 m) | 0.4 ÷ `dock_speed` |
  | 도킹 후 돌아서기 (선택) | `dock_turn_after` (예: 180) | 각도 ÷ `turn_speed` |

  - 도킹 피드백이 1초 안에 들어오는 동안은 "픽업대 마커에 맞추는 중 (0.10 m, +6°)"으로 표시하고 Nav2 거리는 무시합니다.
    피드백이 끊기면 Nav2 계산으로 돌아갑니다. ③ 탭 "남은 거리"도 도킹 중에는 마커 거리로 바뀝니다.
  - 마커 추적은 일반 이동보다 훨씬 느려서 속도를 따로 둡니다: `dock_speed` 0.05 m/s, `turn_speed` 30°/s로 시작하고,
    도킹이 끝날 때마다(적재 대기로 바뀔 때) 실제 거리/시간으로 보정합니다(로그 `[도킹]`). 이동 속도 학습에서도 도킹 시간은 뺍니다.
  - 아직 출발 전인 대기열 작업에도 픽업대마다 도킹 시간이 더해집니다. 도킹하는 지점은 `-p dock_points:=STORE`(쉼표로 여러 개).
  - 예: Nav2 남은 1.3 m, 속도 0.2 → 5 s / 도킹 0.4 m ÷ 0.05 → 8 s / 픽업대 → 작업대1 2.0 m → 8.5 s, 합계 약 22 s.
- **로봇 위치**: 기본은 TF `map → base_footprint`(초당 수십 회, 부드럽게 움직임)이고, TF가 없거나 1초 넘게
  갱신이 멈추면 `/amcl_pose`(약 25cm 이동마다 갱신)로 자동 전환합니다. 위치 타일에 `· TF` / `· AMCL` / `· 끊김`으로 표시되고,
  끊기면 지도의 로봇이 회색이 됩니다. Pinky의 기준 프레임이 다르면 `-p base_frame:=base_link`.
  확인: `ros2 run tf2_ros tf2_echo map base_footprint`
- **떨림 줄이기**: TF가 없어 `amcl_pose`로 받을 때는 그 사이를 바퀴 오도메트리(`/odom`, 로봇별 `/<robot>/odom`)로 이어서
  부드럽게 움직입니다(위치 타일 `· AMCL+odom`). 지도에 그리는 위치는 0.3초 정도 살짝 따라가게 하고, 2 cm·1° 이하 흔들림은 무시,
  0.5 m 넘게 튀면(위치 재설정) 바로 이동합니다. 조절: `-p pose_smooth:=0.5`(더 부드럽게) / `0`(끄기), `-p odom_topic:=''`(오도메트리 끄기).
  가장 좋은 건 로봇별 TF(`pinky_1/base_footprint`)를 쓰는 것입니다. TF는 AMCL과 오도메트리를 이미 합친 값이라 거의 떨리지 않습니다.
- **저장된 지도 파일**: `-p map_yaml:=~/maps/pinky.yaml`을 주면 map_server 없이도 그 지도를 바로 그립니다
  (yaml 옆의 pgm/png를 map_server와 같은 규칙으로 읽음: occupied_thresh / free_thresh / negate / origin / resolution).
  로봇과 연결돼 `/map`이 들어오면 그 지도로 바꿉니다. 로봇의 map.yaml과 이미지 파일을 노트북에 함께 복사해 두세요.
- **보기 좋은 지도 (기본)**: 지도를 받으면 한 번만 정리해서 표시용 그림을 만듭니다(0.2초, numpy만 사용).
  방 밖으로 뻗은 라이다 자국 제거, 책상 밑 미탐색 구멍 메움, 벽은 진하게·가구는 연하게, 방 영역만 확대,
  벽이 수평이 되도록 화면 회전. 좌표는 원본 그대로라 로봇·지점·경로 위치는 같습니다.
  RViz처럼 원본 칸 그대로 보려면 `-p map_style:=raw`, 회전을 끄려면 `-p map_rotate:=0` (각도 직접 지정도 가능).
- **지도와 평면도 (sesac_gangnam_2)**: `maps/`에 지도(`sesac_gangnam_2.pgm`, `.yaml`)와 평면도(`sesac_gangnam_2.floorplan.json`)가 함께 있습니다.
  평면도는 지도 yaml과 **같은 폴더, 같은 이름**이면 자동으로 붙습니다(`~/maps`에 셋 다 두거나, 이 폴더의 `maps/`를 그대로 사용).
  주행 통로만 그린 도면입니다: 벽, 칸막이, 수납장 턱, 커팅매트 선, 상자 3개. 책상 쪽은 점선. 화면은 반시계 90° 돌리고(`view_deg: 90`) 여백을 줄여(`margin_m: 0.15`) 통로를 크게 보여 줍니다.
  의자는 자주 움직여서 넣지 않았습니다.
  벽·장애물이 map 좌표(m)로 들어 있어서
  로봇 위치(TF)·지점(POINTS)·Nav2 경로가 그 위에 그대로 겹쳐집니다.
  **화면 표시 전용**입니다: GUI는 이 파일을 읽기만 하고 지도를 발행하지 않으므로 Nav2·AMCL은 원래 SLAM 지도로 동작합니다.
  끄려면 `-p floorplan:=none`, 다른 파일은 `-p floorplan:=경로`. 꼭짓점을 고치면 모양을 바꿀 수 있습니다.
- 지도가 안 나오면 `ros2 topic info /map -v`로 이름을 확인하고 `-p map_topic:=...`, 경로가 안 나오면 `-p plan_topic:=...`
- 요청 버튼은 누를 때마다 요청이 한 건씩 들어가고(중복 입력), 컨트롤러의 응답(대기열 추가/병합/거절)이 로그에 남습니다.
- **Pinky 2대**: 기본 `-p robots:=pinky_1,pinky_2`. 컨트롤러가 2대 형식으로 상태를 보내면
  상단에 로봇별 상태, ① 탭에 로봇별 도착 시간과 "→ pinky_2 예정", ② 탭에 로봇별 [적재 완료]와 수령 현황,
  ③ 탭 위쪽 버튼으로 로봇을 골라 상태·카메라·재시도/취소, 지도에 두 로봇(P1, P2)이 함께 나옵니다.
  1대 컨트롤러(`parts_delivery.py`)에 붙이면 지금처럼 1대 화면으로 동작합니다.
- **수령 완료는 작업대에서**: ① 탭의 작업대1·작업대2 칸마다 [부품 요청]과 [수령 완료]가 있습니다. [수령 완료]는 Pinky가 그 작업대에 도착해 기다릴 때만 켜지고(칸이 주황색, ① 탭에 주황 점), 누르면 그 Pinky의 received 서비스를 부릅니다.
- 적재 완료/수령 완료는 해당 상태일 때만, retry는 ERROR일 때만 눌립니다. reset은 확인 창을 거칩니다.
- 상태 토픽이 2초 이상 오지 않으면 "컨트롤러 응답 없음"으로 표시하고 확인 버튼을 잠급니다.
- 카메라 토픽 이름은 `ros2 topic list | grep image`로 확인해서 넣으세요. `compressed`로 끝나면 CompressedImage, 아니면 Image(rgb8/bgr8/mono8)로 받습니다. 무선 환경에서는 compressed가 훨씬 가볍습니다.
- RViz는 화면 안에 넣지 않고 "RViz 열기" 버튼으로 별도 창을 띄웁니다.

## v1 대비 변경점

| 항목 | v1 | v2 |
|---|---|---|
| 노드 시작 | `self.services` 대입으로 AttributeError (rclpy 읽기 전용 property와 충돌) | `self.srvs`로 이름 변경 |
| 주행 실패 후 | ERROR에서 빠져나올 수 없음 → 재시작 필요, 대기열 소실 | `/delivery/retry`, `/delivery/reset` 추가 |
| Nav2 준비 전 요청 | 즉시 ERROR | Nav2가 준비될 때까지 IDLE에서 대기 (5초마다 경고 로그) |
| Nav2 무응답 | 영원히 이동 중 상태 | `nav_timeout`(기본 90초) 초과 시 goal 취소 후 ERROR |
| 복귀 중 새 요청 | HOME 도착 후 출발 | 현재 복귀 goal 완료 후 출발 (충돌 goal 방지) |
| ERROR 중 요청 | 거절 | 대기열에 추가 (복구 후 처리) |
| 상태 토픽 | state/current/queue | + target, error, nav_ready, timeout_left |

이전 goal의 늦게 도착한 결과(타임아웃·리셋·선점 이후)는 goal 번호로 걸러서 무시합니다.

## 1. 로봇 없이 흐름 확인

```bash
source /opt/ros/jazzy/setup.bash
python3 fms_controller.py                 # Pinky 2대 모의 실행 (로봇이 움직이지 않음)
# python3 parts_delivery.py               # 1대용
```

다른 터미널(각각 ROS 환경 source):

```bash
ros2 topic echo /delivery/status          # 상태 모니터링 (또는 delivery_gui.py)

ros2 service call /delivery/request_a std_srvs/srv/Trigger '{}'          # 작업대1 요청 → 한 대 출발
ros2 service call /delivery/request_b std_srvs/srv/Trigger '{}'          # 작업대2 요청 → 다른 한 대는 WAIT_PICKUP
ros2 service call /delivery/pinky_1/loaded   std_srvs/srv/Trigger '{}'   # 픽업대에 있는 로봇 이름으로
ros2 service call /delivery/pinky_1/received std_srvs/srv/Trigger '{}'
```
(1대용 `parts_delivery.py`는 `/delivery/loaded`, `/delivery/received`처럼 로봇 이름 없이 부릅니다.)

## 2. 실제 Nav2 연동

1. `points.template.json`과 `home_slots.template.json`을 복사해 STORE, A, B, HOME 및 로봇별 HOME 주차 위치의 **실측** 지도 좌표(x m, y m, yaw rad)를 채웁니다. 두 주차 위치는 `home_min_separation`(기본 0.5 m)보다 떨어져야 합니다. 원본 예시 좌표와 기본 `home_slot_offset`은 실주행에 사용할 수 없습니다. 접근 가능한 free space, 로봇 외형과 목표 방향을 현장에서 확인합니다.
2. 각 Pinky의 SLAM/지도·Nav2를 namespace(`/pinky_1`, `/pinky_2`)로 실행하고, 로봇마다 map 좌표 위치를 확인합니다. 두 차가 같은 지도 좌표계를 사용해야 합니다. `ros2 action list -t`에서 두 `NavigateToPose` action, `ros2 topic list -t`에서 두 `amcl_pose`가 보여야 합니다.
3. 모의 실행 프로세스를 종료합니다. 같은 컨트롤러를 중복 실행하지 마세요. 실차는 안전 담당자가 물리적 정지 수단을 확보한 뒤에만 진행합니다.

```bash
python3 fms_controller.py --ros-args -p dry_run:=false \
    -p points_file:=/absolute/path/to/measured_points.json \
    -p home_slots_file:=/absolute/path/to/measured_home_slots.json -p docking:=false
# 로봇 이름 / Nav2 action 이름 / 위치 토픽이 다를 때
python3 fms_controller.py --ros-args -p dry_run:=false -p robots:=pinky_a,pinky_b \
    -p points_file:=/absolute/path/to/measured_points.json \
    -p home_slots_file:=/absolute/path/to/measured_home_slots.json \
    -p nav_action:='/{robot}/navigate_to_pose' -p pose_topic:='/{robot}/amcl_pose'
# 1대만 시험할 때
python3 parts_delivery.py --ros-args -p dry_run:=false
```

실주행 모드는 시작해도 `armed=false`로 대기합니다. 상태 토픽에서 두 로봇의 `pose_ready`, `nav_ready`와 `pickup_clear`를 먼저 확인하세요. AMCL 위치 갱신이 3초 넘게 없으면 기본 설정에서는 출발을 막거나 진행 중인 Nav2 goal 취소를 요청합니다. 정지 상태에서 AMCL 갱신이 드문 환경은 `pose_timeout`을 현장 데이터에 맞춰 조정하거나 지속적인 map 위치 토픽을 사용해야 합니다. 취소 완료와 실제 물리적 정지는 별도로 확인해야 합니다.

```bash
ros2 topic echo /delivery/status std_msgs/msg/String --once
ros2 service call /delivery/arm std_srvs/srv/Trigger '{}'
ros2 service call /delivery/disarm std_srvs/srv/Trigger '{}'  # 새 배정 중지 및 진행 중 Nav2 goal 취소 요청
```

한 작업대에는 한 차만 배정하고, 픽업대도 한 차만 사용합니다. 수령 완료 직후 작업대 잠금은 유지되며 로봇이 지도 위치상 그 반경을 벗어난 뒤 해제됩니다. Nav2 result가 확인되지 않으면 해당 로봇은 `ERROR`로 남고 재시도·초기화가 막힙니다. 현장에서 로봇 정지와 Nav2 goal 상태를 확인하고 컨트롤러를 재시작해야 합니다. `/delivery/disarm`은 물리적 비상 정지 명령이 아닙니다.

## 3. 오류가 났을 때

`/delivery/status`의 `error` 필드(2대면 `robots.<이름>.error`)에서 원인을 확인한 뒤:
GUI ③ 탭에서 로봇을 골라 [재시도]/[작업 취소]를 누르거나, 2대 컨트롤러는 `/delivery/<robot>/retry`, `/delivery/<robot>/reset`을 부릅니다. 1대용은:

```bash
# 장애물 치우기 등 원인을 해결했으면 → 같은 목적지로 다시 주행
ros2 service call /delivery/retry std_srvs/srv/Trigger '{}'

# 현재 작업을 포기하고 다음 작업으로 → 현재 작업만 버리고 대기열은 유지
ros2 service call /delivery/reset std_srvs/srv/Trigger '{}'
```

- 위치를 잃은 경우(로봇이 엉뚱한 곳을 자기 위치로 인식)에는 RViz에서 초기 위치를 다시 잡은 뒤 retry 하세요.
- reset 후 다음 작업은 보관소부터 시작합니다. 버린 작업에 부품이 실려 있었다면 사람이 직접 정리하세요.

## 범위와 제한

- 한 요청 = 목적지 작업대에 대한 배송 1건. 품목/수량/재고는 없습니다.
- 적재/하역은 사람이 하고 서비스 호출로 확인합니다.
- 자동 충전, 개별 작업 취소, 영구 작업 이력, 재시작 후 복구는 미구현입니다.
- 프로세스 종료 시 알려진 Nav2 goal 취소를 요청하지만 완료를 보장하지 않습니다. 실물 정지는 로봇 자체 정지 절차를 사용하세요.
- 공용 통로 전체의 교통 잠금, 두 로봇 간 직접 충돌 회피, 정적·동적 costmap의 목표 유효성 검사는 이 컨트롤러에 없습니다. Nav2 자체의 경로 계획·회피를 현장에서 별도 확인해야 합니다. 이 코드는 두 차가 같은 구역에 있을 때의 안전을 보증하지 않습니다.
- 이 노드 실행 중 RViz 등에서 별도 Nav2 목적지를 보내지 마세요.

## 검증 범위

ROS 없이 가짜 rclpy로 검증했습니다 (`python3 test_fms.py fms_controller.py` 등).
- `test_fms.py` (2대 컨트롤러, 외부 배정 모듈 사용·오류·중복 배정 시 기본 규칙 포함): 로봇별 서비스·Nav2·위치 구독, 요청 2건 분배, 픽업대 한 대씩(WAIT_PICKUP → 차례), 가까운 로봇 배정·위치 모르면 후순위·먼저 기다린 순, 복귀 중 새 요청, 한 대 오류여도 다른 한 대 계속, 오류 시 점유 해제/유지, 재시도·취소, 시간 초과, 남은 거리, 늦은 결과 무시, 도킹 시작·진행·실패·재시도·완료·서비스 없음, 수납대 자리 2개, 파라미터
- `test_fsm.py` 26개: 상태 머신 전체 흐름, 복구, 타임아웃, Nav2 남은 거리 반영, 고정 지점·공용 수납대
- `test_notifier.py` 21개: 2대일 때 자기 로봇 상태만 따라감, 전방 섹터 판정(측면/몸체/inf 무시, 각도 보정), 주행 중에만 삐빅, 목적지 근처 음소거, 쿨다운, 대기 상태 노래·반복·중지
- `test_gui.py` 129개 + 컨트롤러 상태 연동 7개 (PyQt5, PyQt6 각각 통과) (`QT_QPA_PLATFORM=offscreen`): 상태별 버튼 활성화, 서비스 호출, 중복 요청, 요청 버튼 2초 잠금, 대기 안내 문구, 지도 변환·지도 파일(yaml+pgm) 읽기·표시용 지도 정리·회전·평면도 파일·탭별 지도, TF/AMCL 위치 전환·오도메트리로 사이 메우기·표시 떨림 보정, 새 요청 팝업·담당 Pinky 표시, 도착 시간 계산·멈춤·속도 학습, ArUco 도킹 시간(카메라 피드백·마커 놓침·피드백 끊김·도킹 속도 학습), 위치·카메라 표시, 작업대 칸의 수령 완료, Pinky 2대(로봇별 토픽·상태·도착 시간·배정 예상·픽업대 대기·로봇별 서비스·로봇 선택), 1대 컨트롤러로 되돌아가기, `fms_controller.py`가 실제로 보낸 상태로 화면·서비스 이름 확인 (`python3 test_fms.py fms_controller.py s.json` 후 `python3 test_gui.py delivery_gui.py /tmp s.json`)

실제 ROS2/Nav2, Pinky 실물, 스피커 출력, 실제 카메라 토픽은 미검증입니다.
