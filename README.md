# 산업안전 순찰 로봇 프로젝트 - 인수인계 (Day 1~3, 파트 1)
---

## 1. 환경 설정 (필수)

새 터미널을 열 때마다 아래 3줄을 항상 먼저 입력

```bash
# ROS2 환경 활성화
source /opt/ros/humble/setup.bash

# 사용할 터틀봇 모델 지정
export TURTLEBOT3_MODEL=burger

# WSL 렌더링 이슈 방지 (소프트웨어 렌더링 강제)
export LIBGL_ALWAYS_SOFTWARE=1
```

`LIBGL_ALWAYS_SOFTWARE=1`은 WSL에서 Gazebo 화면이 안 뜨거나 멈추는 문제를 예방하는 설정, 없어도 되는 경우도 있지만, Day 1~3 진행 중 계속 필요했으니 습관처럼 넣어주세요.

---

## 2. 겪었던 문제 + 해결법 (트러블슈팅 노트)


| 문제 | 원인 | 해결 방법 |
|---|---|---|
| `turtlebot3_gazebo` 패키지를 찾을 수 없음 | 패키지 미설치 | `sudo apt install -y ros-humble-turtlebot3 ros-humble-turtlebot3-simulations ros-humble-turtlebot3-msgs` |
| 설치 중 `gz-tools2` 의존성 충돌 에러 | 신버전 Gazebo 도구와 ROS2 Humble용 구버전 gazebo11 충돌 | `sudo apt remove -y gz-tools2` 실행 후 위 설치 명령 재시도 |
| `Waiting for service /spawn_entity, timeout = 30` → 결국 타임아웃 실패 | WSL 소프트웨어 렌더링이 느려서 자동 스폰이 제시간에 안 됨 (정상적으로 자주 발생) | 새 터미널에서 아래 "수동 스폰" 명령어 실행 |
| Gazebo 창이 안 뜨거나, `xeyes` 같은 GUI 프로그램도 반응 없음 | WSLg(GUI 출력 시스템) 자체가 먹통된 상태 | **Windows PowerShell**에서 `wsl --shutdown` → `wsl --update` → WSL 재시작 |
| teleop으로 회전시키다 로봇이 넘어짐 (라이다 광선이 옆으로 누움) | 회전 명령을 연속으로 눌러 각속도 과다 누적 + 물리 연산 지연 | 아래 "로봇 삭제 후 재스폰" 명령어로 초기화 |
| `git commit` 시 `Author identity unknown` 에러 | 이 PC에 git 사용자 정보가 한 번도 설정 안 됨 | `git config --global user.email "..."`, `git config --global user.name "..."` 최초 1회 설정 |
| `git push` 시 비밀번호 계속 요구, 인증 실패 | GitHub이 일반 비밀번호 인증을 지원 안 함 (Personal Access Token 필요) | GitHub에서 PAT(classic, `repo` scope 체크) 생성 → `git remote set-url origin https://<토큰>@github.com/k-eungyeong/safety-patrol-robot.git` |

### 수동 스폰 명령어

```bash
ros2 run gazebo_ros spawn_entity.py -entity burger -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf -x 0.0 -y 0.0 -z 0.01
```

### 로봇 삭제 후 재스폰 (넘어졌을 때)

```bash
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'burger'}"
# 삭제 확인 후 위의 수동 스폰 명령어 재실행
```

---

## 3. 결과물 파일 위치

저장소 클론 후 아래 경로에서 확인 가능합니다.

- `~/safety-patrol-robot/maps/my_map.yaml`, `my_map.pgm` — SLAM(Cartographer)으로 생성한 turtlebot3_world 맵
- `~/safety-patrol-robot/schedule/patrol_schedule.json` — 순찰 스케줄 JSON 초안

---

## 4. 현재 상태에서 이어서 실행하는 법

**터미널 1 — Gazebo (장애물 있는 맵으로 실행)**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
→ `spawn_entity` 타임아웃이 뜨면 위 "수동 스폰 명령어" 실행

**터미널 2 — Nav2 (저장된 맵 불러오기)**
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/dmin/safety-patrol-robot/maps/my_map.yaml
```
> 경로의 `dmin` 부분은 실제 계정명에 맞게 바꿔주세요 (`echo $HOME`으로 확인 가능).

RViz 창이 뜨면:
1. **2D Pose Estimate** — 로봇의 실제 위치/방향 지정 (Gazebo 화면 보면서 맞추기)
2. **2D Nav Goal** — 목표 지점 지정 → 로봇이 알아서 경로 계산 후 이동

---

## 5. Git 관련

- 저장소: `https://github.com/k-eungyeong/safety-patrol-robot`
- 커밋 이력은 `git log --oneline`으로 확인 가능 (Day1~3 작업 내용 전부 기록됨)
- 클론 또는 pull 받으면 위 결과물 파일들 그대로 이어받을 수 있음
- 팀원 본인 계정으로 push하려면 본인 GitHub PAT 필요 (위 트러블슈팅 표 참고)
- 커밋 메시지 컨벤션: `Day숫자: 무엇을 했는지` 형식 유지

---

## 6. 다음 할 일 (Day 4~6, 파트 1 → 파트 2 로 담당 교대)

아래 순서로 이어가면 됨!!

- **Day 4**: rclpy 액션 클라이언트로 Nav2 goal 코드 전송 방법 확인
- **Day 5**: 통합 1차 — 스케줄 노드로 TurtleBot3 순차 이동 확인 (파트 1·2 결과물 첫 통합 체크포인트)
- **Day 6**: 위험 이벤트 수신 시 goal 취소 → 위험 좌표 재전송 로직 (페어 작업 권장)

Day 5는 필수 체크포인트이니, 이 시점까지는 반드시 파트 1·2 결과물이 합쳐진 상태여야 합니다.
