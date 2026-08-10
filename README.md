# 산업안전 순찰 로봇 프로젝트

스케줄 기반으로 순찰하다가 위험을 감지하면 우선순위를 재조정하고, 다시 원래 스케줄로 복귀하는 산업안전 순찰 로봇 (2인 팀, 3주 일정).

- 은경: 파트 1(로봇/시뮬레이션) Day 1~3 → 파트 2(백엔드/로직) Day 4~6
- 센아: 파트 2(백엔드/로직) Day 1~3 → 파트 1(로봇/시뮬레이션) Day 4~6
- 3일마다 파트 교대, Day 5·10·14는 필수 통합 체크포인트

---

## 1. 프로젝트 구조

```
safety-patrol-robot/
├── docs/
│   ├── 파트1_인수인계_Day1-3.md
│   ├── 파트2_인수인계_Day1-3.md
│   ├── 파트1_진행상황_Day4-5.md
│   └── 파트2_진행상황_Day4-5.md
├── maps/
│   ├── my_map.yaml
│   └── my_map.pgm
└── src/patrol_robot/               # ROS2 ament_python 패키지
    ├── package.xml, setup.py, setup.cfg
    ├── config/schedule.json
    ├── resource/patrol_robot
    ├── test/
    └── patrol_robot/
        ├── __init__.py
        ├── risk_logic.py                  # 위험감지 순수 로직 (ROS2 비의존)
        ├── risk_detector_node.py          # /scan 구독 → /risk_events 퍼블리시
        ├── schedule_manager_node.py       # schedule.json 기반 Nav2 goal 순차 전송
        └── nav2_action_client_test.py     # goal 취소(cancel_goal_async) 방식 확인용 예제
```

---

## 2. 환경 설정 (WSL Ubuntu-22.04 기준)

새 터미널을 열 때마다 아래 3줄을 먼저 입력합니다.

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1
```

> **주의**: `LIBGL_ALWAYS_SOFTWARE=1`은 컴퓨터마다 다르게 작동할 수 있습니다. 은경 환경(WSL2)에서는 필수였지만, 센아 환경(같은 WSL2, 다른 PC)에서는 오히려 `gzserver`가 세그폴트(exit code -11)로 죽는 원인이었습니다. 문제가 생기면 껐다 켜보며 본인 환경에 맞는 쪽으로 판단하세요.

---

## 3. 트러블슈팅 노트

| 문제 | 원인 | 해결 |
|---|---|---|
| `turtlebot3_gazebo` 패키지를 찾을 수 없음 | 패키지 미설치 | `sudo apt install -y ros-humble-turtlebot3 ros-humble-turtlebot3-simulations ros-humble-turtlebot3-msgs` |
| 설치 중 `gz-tools2` 의존성 충돌 | 신형 Gazebo(Harmonic)와 구형 gazebo11(ROS2 Humble용) 공존 불가 | `gz-*`, `libgz-*`, `libignition-*` 계열 패키지 정리 후 `ros-humble-desktop`, `nav2`, `rviz2`, `slam_toolbox`, `turtlebot3` 관련 패키지를 한 번에 재설치 |
| `spawn_entity` 서비스 타임아웃 | WSL 소프트웨어 렌더링 지연 / `GAZEBO_PLUGIN_PATH`에 ROS2 플러그인 경로 누락 | 아래 "수동 스폰" 명령 실행. 반복되면 `.bashrc`에 `export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH` 추가 |
| Gazebo/GUI 프로그램이 아예 안 뜨거나 멈춤 | WSLg(GUI 출력 시스템) 자체 오류 | Windows PowerShell에서 `wsl --shutdown` → `wsl --update` → WSL 재시작 |
| 로봇이 회전 중 넘어짐 | 회전 명령 과다 누적 + 물리 연산 지연 | 아래 "로봇 삭제 후 재스폰" 명령 실행 |
| RViz Nav2 패널 로딩 실패 (`diagnostic_updater` undefined symbol) | 패키지 재설치 과정에서 버전 꼬임 | `dpkg -l \| grep diagnostic`으로 버전 확인 후 재설치 시도 — 미해결, 확인 필요 |
| Git 커밋 시 `Author identity unknown` | git 사용자 정보 미설정 | `git config --global user.email "..."`, `git config --global user.name "..."` |
| Git push 인증 실패 | 비밀번호 인증 미지원 (GitHub 정책) | Personal Access Token(classic, `repo` scope) 발급 후 `git remote set-url origin https://<토큰>@github.com/k-eungyeong/safety-patrol-robot.git` |

### 수동 스폰
```bash
ros2 run gazebo_ros spawn_entity.py -entity burger -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf -x 0.0 -y 0.0 -z 0.01
```

### 로봇 삭제 후 재스폰 (넘어졌을 때)
```bash
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'burger'}"
```

### 3-1. 팀원(센아) 환경에서 추가로 겪은 이슈 (같은 WSL2, 다른 PC)

| 문제 | 원인 | 해결 |
|---|---|---|
| `bt_navigator` 활성화 실패 (`Node not recognized: SmoothPath`) | `turtlebot3_navigation2`의 `burger.yaml` plugin 목록이 구버전이라 `nav2_smooth_path_action_bt_node` 누락 | `sudo sed -i '/nav2_wait_action_bt_node/a\    - nav2_smooth_path_action_bt_node' /opt/ros/humble/share/turtlebot3_navigation2/param/humble/burger.yaml` |
| `bt_navigator` 활성화 실패 (`RateController` 포트 이름 불일치) | BT XML은 `rate=`를 쓰는데 실제 설치된 `RateController`는 `hz`로 이름 변경됨 | `sudo sed -i 's/RateController rate=/RateController hz=/' /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/*.xml` |
| `controller_server` Configuring 단계 실패 (원인 불명) | 버전 정합성 문제로 추정 | `sudo apt install --reinstall ros-humble-navigation2 ros-humble-nav2-bringup -y` |

---

## 4. 진행 상황

### 완료 (Day 1~5)
- ROS2 Humble + Gazebo + TurtleBot3 환경 구축, teleop 조작 확인
- SLAM(Cartographer)으로 맵 생성 및 저장 (`maps/my_map.yaml`, `.pgm`)
- Nav2 + 저장된 맵으로 2D Pose Estimate → 2D Nav Goal 자율 이동 확인
- `risk_logic.py`: 위험감지 순수 로직 + 단위 테스트
- `risk_detector_node.py`: `/scan` 구독 → `/risk_events` 퍼블리시까지 파이프라인 완성 및 검증
- `schedule_manager_node.py`: 실제 맵 좌표로 `schedule.json` 갱신, Nav2 goal 순차 전송 통합 테스트 성공 (`SUCCEEDED`)
- `risk_detector_node` + `schedule_manager_node` 동시 실행 검증 (서로 충돌 없이 독립 동작)
- `nav2_action_client_test.py`: rclpy 순수 ActionClient로 goal 전송/취소(`cancel_goal_async`) 패턴 확인

### 알려진 한계 (Day 6에서 처리 예정)
- 위험 감지 시 goal 취소/재전송(동적 재조정) 미구현 — **Day6 핵심 작업**
- `/risk_events`의 `zone_id`가 항상 `null` — 현재 순찰 중인 구역 정보와 연동 필요
- 위험 이벤트가 로봇 기준 상대좌표(`distance`, `angle_rad`)만 제공 — Nav2 goal 전송을 위해 map 좌표계 절대 x/y로 변환하는 로직 필요 (로봇 현재 위치, `/amcl_pose` 또는 tf `map`→`base_link` 활용)
- 야간(`night`) 블록처럼 자정을 넘는 시간대(`18:00~08:00`) 매칭 로직 미구현
- `schedule_manager_node`는 현재 한 바퀴만 순회(`run_once`) — 무한 반복 순찰 로직 없음
- `config/schedule.json`이 `setup.py`의 `data_files`에 미등록 — symlink 없는 일반 빌드 시 파일 못 찾을 수 있음
- `operation` 블록 `end_time`이 테스트용으로 임시 확장된 상태 — 실제 운영 시간대로 재조정 필요

### 다음 계획
- **Day 6**: 위험 이벤트 좌표 변환(상대→절대) 설계, `schedule_manager_node`에 `cancel_goal_async` 통합, 위험 감지 시 재전송 후 원래 스케줄 복귀 로직
- **Day 10 / Day 14**: 다음 필수 통합 체크포인트, Day 14는 실제 발표 PC에서 리허설

---

## 5. Git 관련

- 저장소: `https://github.com/k-eungyeong/safety-patrol-robot` (본체, 센아의 `se-ny/patrol-robot`은 Day1~3 히스토리 백업용으로 유지)
- 커밋 메시지 컨벤션: `Day숫자: 무엇을 했는지`
- push 인증에 Personal Access Token 필요

---

## 6. 참고 문서

각 파트의 상세 작업 기록은 `docs/` 폴더 참고:
- [파트1 인수인계 (Day1~3)](docs/파트1_인수인계_Day1-3.md)
- [파트2 인수인계 (Day1~3)](docs/파트2_인수인계_Day1-3.md)
- [파트1 진행상황 (Day4~5)](docs/파트1_진행상황_Day4-5.md)
- [파트2 진행상황 (Day4~5)](docs/파트2_진행상황_Day4-5.md)
