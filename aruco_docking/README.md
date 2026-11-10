# ArUco 도킹 및 위치 인식

현재 확정된 Marker 역할은 다음과 같습니다.

| ID | 지점 |
|---:|---|
| 66 | 1 대기 |
| 24 | 2 픽업 |
| 87 | 3 배송 1 |
| 42 | 4 배송 2 |

매핑 원본은 `pinky_mission/config/aruco_markers.yaml`에 한 번만 저장합니다.

현재 OpenCV의 ArUco 기능 사용 가능 여부와 Pinky URDF의 `front_camera_link`는 확인했습니다. 그러나 실제 카메라 `/camera/image_raw` 발행, `/camera/camera_info` 보정값, Marker 실측 크기, 각 Marker의 `map` 자세가 확정되지 않아 위치 추정 Node는 아직 포함하지 않습니다.

후속 `aruco_localizer` Node는 이미지와 CameraInfo를 구독하고, `camera→marker`와 고정 `map→marker`, `base_footprint→camera`를 조합해 `map→base_footprint` 후보 자세를 계산해야 합니다. 로봇이 정지한 상태에서 다중 프레임 검증을 통과한 경우에만 AMCL `/initialpose`를 갱신합니다.
