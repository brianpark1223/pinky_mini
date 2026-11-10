# Pinky 부품 배송 FMS

작업대에서 부품을 요청하면 Pinky Pro 로봇 2대가 픽업대에서 부품을 싣고 작업대까지 가져다주는 **소형 부품 공급 FMS(Fleet Management System)** 입니다.
ROS2 Jazzy + Nav2 기반이며, 3명이 3일 동안 진행한 팀 프로젝트입니다.

<p align="center">
  <img src="docs/images/gui_request.png" width="80%" alt="관제 GUI - 작업 요청 화면">
</p>

## 무엇을 하나

1. 작업자가 **작업대**에서 [부품 요청]을 누릅니다.
2. FMS가 비어 있는 Pinky 중 가장 알맞은 로봇에 작업을 배정합니다.
3. Pinky가 **픽업대**로 가서 ArUco 마커를 보고 정확히 정렬해 섭니다.
4. 픽업대 담당자가 부품을 싣고 [적재 완료]를 누릅니다.
5. Pinky가 **작업대**로 배달하고, 요청한 작업자가 부품을 꺼낸 뒤 [수령 완료]를 누릅니다.
6. 다음 요청이 있으면 이어서 처리하고, 없으면 **수납대**로 돌아갑니다.

로봇이 사람을 기다리는 동안(적재·수령)에는 로봇 스피커로 알림음이 나오고, 주행 중 앞에 사람이나 물체가 가까이 오면 경고음이 울립니다.

## 팀 구성과 역할

| 역할 | 담당 내용 |
|---|---|
| **ArUco 주차 (도킹)** | 픽업대 앞 마커 인식, 거리·각도 계산, 정밀 정렬 주행, 도킹 완료 판단 |
| **Cost 기반 작업 배정** | 요청을 어느 Pinky에 줄지(거리·대기 시간 cost), 대기열 우선순위, 픽업대·수납대 사용 순서 |
| **이벤트 제어** | 로봇 상태 머신, 사람 입력(요청·적재·수령), 알림(소리·팝업), 오류·복구, 관제 GUI, 도착 시간 예측 |

## 시스템 구성

```mermaid
flowchart LR
    subgraph Laptop["관제 노트북"]
        GUI["관제 GUI<br/>delivery_gui.py"]
        FMS["FMS 컨트롤러<br/>상태 머신 · 작업 배정"]
    end
    subgraph P1["Pinky 1"]
        N1["Nav2"]
        D1["ArUco 도킹"]
        S1["알림<br/>robot_notifier.py"]
    end
    subgraph P2["Pinky 2"]
        N2["Nav2"]
        D2["ArUco 도킹"]
        S2["알림<br/>robot_notifier.py"]
    end
    GUI -- "요청 · 적재 · 수령 (서비스)" --> FMS
    FMS -- "상태 JSON (0.2초마다)" --> GUI
    FMS -- "목표 좌표" --> N1 & N2
    FMS -- "도킹 시작" --> D1 & D2
    D1 & D2 -- "거리 · 각도 · 완료" --> FMS
    FMS -- "상태" --> S1 & S2
```

- 판단은 **FMS 컨트롤러 한 곳**에서만 합니다. GUI는 사람 입력을 전달하고 상태를 그리기만 하므로, GUI를 여러 대 켜도 화면이 어긋나지 않습니다.
- 로봇마다 Nav2를 namespace(`/pinky_1`, `/pinky_2`)로 띄워 토픽을 구분합니다.

## 로봇 상태 머신

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> TO_STORE: 작업 배정
    TO_STORE --> WAIT_PICKUP: 픽업대 사용 중
    WAIT_PICKUP --> TO_STORE: 픽업대 비움
    TO_STORE --> WAIT_LOAD: 도착 + ArUco 정렬 완료
    WAIT_LOAD --> TO_STATION: 적재 완료
    TO_STATION --> WAIT_RECEIVE: 작업대 도착
    WAIT_RECEIVE --> TO_STORE: 수령 완료 (대기열 있음)
    WAIT_RECEIVE --> TO_HOME: 수령 완료 (대기열 없음)
    TO_HOME --> TO_STORE: 새 요청
    TO_HOME --> IDLE: 수납대 도착
    TO_STORE --> ERROR: 주행 실패 / 시간 초과
    TO_STATION --> ERROR: 주행 실패 / 시간 초과
    ERROR --> TO_STORE: 재시도
    ERROR --> IDLE: 작업 취소
```

| 상태 | 의미 | 로봇 알림 |
|---|---|---|
| `IDLE` | 수납대에서 대기 | - |
| `TO_STORE` | 픽업대로 이동 (도착 후 ArUco 정렬) | 전방 근접 경고음 |
| `WAIT_PICKUP` | 다른 Pinky가 픽업대를 쓰는 중이라 차례 대기 | - |
| `WAIT_LOAD` | 픽업대에서 적재 대기 | 알림음 |
| `TO_STATION` | 작업대로 배달 | 전방 근접 경고음 |
| `WAIT_RECEIVE` | 작업대에서 수령 대기 | 알림음 |
| `TO_HOME` | 수납대로 복귀 | 전방 근접 경고음 |
| `ERROR` | 그 로봇만 멈춤, 다른 로봇은 계속 동작 | - |

## 운영 규칙

| 항목 | 규칙 |
|---|---|
| 지점 | 모두 고정 좌표: 픽업대 `STORE`, 작업대1 `A`, 작업대2 `B`, 수납대 `HOME` |
| 작업 배정 | 공용 대기열 하나. 비어 있는 Pinky 중 cost가 가장 낮은 로봇이 맡음 |
| 픽업대 | 한 번에 한 대. 겹치면 늦게 온 Pinky가 대기 (`WAIT_PICKUP`) |
| 수납대 | 1곳을 두 Pinky가 공유 |
| 사람 확인 | 적재 완료는 픽업대 담당자, 수령 완료는 요청한 작업자가 작업대에서 누름 |
| 오류 | 주행 실패·시간 초과 시 그 로봇만 `ERROR`. GUI에서 재시도 또는 작업 취소 |

## 관제 GUI

탭 3개로 나뉩니다. 사용 위치에 따라 필요한 탭만 띄워 두면 됩니다.

| 탭 | 사용자 | 기능 |
|---|---|---|
| ① 작업 요청 | 작업대 작업자 | 작업대별 [부품 요청]·[수령 완료], 로봇별 도착 예상 시간, 대기열과 담당 로봇 예상 |
| ② 적재 확인 · 수령 현황 | 픽업대 담당자 | 로봇별 [적재 완료], 새 요청 팝업("pinky_2에 적재해 주세요") |
| ③ Pinky 현재 상태 | 관리자 | 로봇 선택 → 상태·진행 단계·위치·카메라, 재시도/취소 |

<p align="center">
  <img src="docs/images/gui_load.png" width="49%" alt="적재 확인 화면">
  <img src="docs/images/gui_status.png" width="49%" alt="Pinky 상태 화면">
</p>

- **실시간 지도:** 두 로봇의 위치(TF / AMCL)와 Nav2 경로를 실습실 평면도 위에 표시합니다.
- **도착 시간 예측:** Nav2 남은 거리 + ArUco 정렬 시간 + 픽업대 대기 시간으로 계산하고, 주행할 때마다 실제 속도로 보정합니다. 사람이 싣고 꺼내는 동안에는 숫자가 멈춥니다.
- **오클릭 방지:** 요청 버튼은 누른 뒤 2초 동안 잠기고, 확인 버튼은 해당 상태일 때만 켜집니다.

## 레포 구성

```
.
├── README.md              # 이 문서 (프로젝트 전체 설명)
├── docs/images/           # README 화면 캡처
└── event_control/         # 이벤트 제어: 상태 머신, 관제 GUI, 로봇 알림
    ├── README.md          # 상세 사용법, 팀 인터페이스(토픽·서비스·상태 JSON 형식)
    ├── parts_delivery.py  # FMS 컨트롤러 (현재 1대용)
    ├── delivery_gui.py    # 관제 GUI (PyQt5 / PyQt6, 1대·2대 모두 지원)
    ├── robot_notifier.py  # 로봇 알림음 · 전방 근접 경고음
    ├── maps/              # 실습실 평면도 (화면 표시용)
    ├── sounds/            # 알림음
    └── test_*.py          # ROS 없이 동작 검증
```

ArUco 도킹과 작업 배정 코드는 각 담당자가 별도 폴더로 추가할 예정입니다.
모듈 사이 약속(토픽·서비스·상태 형식)은 [`event_control/README.md`의 "팀 인터페이스"](event_control/README.md#팀-인터페이스-이것만-지키면-각자-수정해도-연동됨)에 정리되어 있습니다.

## 실행

환경: Ubuntu 24.04, ROS2 Jazzy, Nav2, Python 3, PyQt5 또는 PyQt6

```bash
# 각 Pinky: bringup + Nav2(지도 sesac_gangnam) 실행 후, 알림 노드
python3 robot_notifier.py --ros-args -p robot_name:=pinky_1   # 두 번째 로봇은 pinky_2

# 관제 노트북 (로봇과 같은 ROS_DOMAIN_ID)
python3 parts_delivery.py                               # 모의 실행 (로봇이 움직이지 않음)
python3 parts_delivery.py --ros-args -p dry_run:=false  # 실제 주행
PINKY_QT=6 python3 delivery_gui.py                      # PyQt6 사용 시
```

로봇 없이 확인하려면 `python3 test_fsm.py parts_delivery.py`, `QT_QPA_PLATFORM=offscreen python3 test_gui.py delivery_gui.py`를 실행합니다.
자세한 설정과 문제 해결은 [`event_control/README.md`](event_control/README.md)를 보세요.

## 진행 상황

| 항목 | 상태 |
|---|---|
| 1대 FMS 컨트롤러 (상태 머신, Nav2 연동, 오류 복구) | 완료 |
| 관제 GUI (1대·2대, 도착 시간 예측, 실시간 지도) | 완료 |
| 로봇 알림음 · 전방 근접 경고음 | 완료 |
| ArUco 도킹 | 진행 중 |
| Cost 기반 2대 작업 배정 컨트롤러 | 진행 중 |
| 실제 로봇 2대 통합 시험 | 예정 |

## 한계와 확장 방향

- 통로 교통 관제는 없고, 로봇끼리 마주치면 각자의 Nav2 장애물 회피에 맡깁니다.
- 대기열은 컨트롤러 메모리에만 있어 재시작하면 사라집니다.
- 배터리·충전 관리, 고장 난 로봇의 작업을 다른 로봇으로 넘기는 기능은 없습니다.
- 도착 시간에 사람이 싣고 꺼내는 시간은 포함하지 않습니다.
