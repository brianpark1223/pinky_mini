# 테스트 보고서

최종 업데이트: 2026-11-10

## 확인 완료

- Ubuntu 24.04 / ROS 2 Jazzy에서 `pinky_mission`, `pinky_mission_panel` 빌드 성공
- Nav2, RViz2, OpenCV, cv_bridge 의존성 확인
- ROS_DOMAIN_ID 42에서 노트북↔Pinky 양방향 문자열 Topic 통신 확인
- 여섯 배송 경로, 자동 방향, Costmap 통행 가능 검사 단위 테스트 통과
- 격리 Domain 68에서 클릭, Marker, 주행 차단, 저장, 되돌리기, 불러오기 Smoke Test 통과
- RViz Mission Panel 로딩 확인
- 지도 `sesac_gangnam_20261110`: 214×326, 0.05 m/cell로 저장 및 Nav2 Map Server 로딩 가능

## 실차에서 추가 확인 필요

- 새 지도에서 AMCL 수렴
- 여섯 배송 경로 전체 실주행
- Nav2 Goal 취소 후 실제 바퀴 정지
- 네 목표점의 실제 Costmap 안전 여유
- 카메라 내참수와 ArUco Marker 실측값
- 두 대 로봇 namespace와 작업 배정

## 알려진 제한

- 일시 정지는 현재 Nav2 Goal을 취소하고, 재개 시 같은 목표로 다시 계획합니다.
- 소프트웨어 취소는 통신 장애 시 하드웨어 정지를 보장하지 않습니다.
- ArUco 위치 추정 노드는 아직 구현되지 않았습니다.
