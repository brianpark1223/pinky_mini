# 팀 인터페이스

실제 코드에 맞춘 명세는 [event_control/README.md의 팀 인터페이스](../event_control/README.md#팀-인터페이스-이것만-지키면-각자-수정해도-연동됨)를 기준으로 합니다.

- 공용 요청: `delivery/request_a`, `delivery/request_b`
- 2대 로봇별 확인·복구: `delivery/<robot>/loaded`, `received`, `retry`, `reset`
- 공용 상태: `delivery/status` (`std_msgs/String` JSON, `robots` 사전 포함)
- 도킹 시작: `/<robot>/delivery/dock_start` (`std_srvs/Trigger`)
- 도킹 피드백: `/<robot>/delivery/dock` (JSON의 `state`: `running` / `done` / `failed`)

ArUco 도킹 노드는 별도로 구현해야 합니다. 토픽·서비스·필드를 바꿀 때에는 코드와 위 명세를 함께 수정합니다.
