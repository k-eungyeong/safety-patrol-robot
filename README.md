# 산업안전 순찰 로봇 (Schedule-based Industrial Safety Patrol Robot)

작업장의 시간대별 위험도 변화에 맞춰, 로봇이 순찰 우선순위를 스스로 재조정하며 안전을 감시하는 시스템

- 저장소: https://github.com/k-eungyeong/safety-patrol-robot
- 팀: 은경([@k-eungyeong](https://github.com/k-eungyeong)), 세은([@se-ny](https://github.com/se-ny))
- 기간: 2026.7.23 ~ 2026.8.12 (3주, 2인 팀 프로젝트)

---

## 핵심 기능

| 구분 | 내용 |
|---|---|
| 스케줄 순찰 | 시간대별(출근/가동/점심/야간) 구역별 순찰 웨이포인트를 JSON으로 정의, 순서대로 자동 순찰 |
| 위험 감지 | LiDAR(`/scan`) 거리 임계값 기반 이상치 감지 (룰 기반) |
| 동적 재조정 | 위험 감지 시 진행 중인 goal 취소 → 제자리 대기 → 상황 종료 후 중단 지점부터 순찰 재개 |
| 실시간 알림 | 위험 이벤트 발생 시 Discord 웹훅으로 알림 전송 (쿨다운 적용) |
| 실시간 대시보드 | FastAPI + Streamlit으로 순찰 상태·로봇 위치·위험 이벤트 로그 실시간 표시 |

---

## 기술 스택

- **시뮬레이션**: ROS2 Humble + Gazebo, Ubuntu 22.04 (WSL2)
- **로봇/내비게이션**: TurtleBot3 (waffle) + Nav2
- **위험 감지**: LiDAR(`/scan`) 임계값 룰 기반
- **백엔드**: FastAPI (ROS2 상태 ↔ REST API)
- **알림**: Discord Webhook
- **대시보드**: Streamlit

---

## 아키텍처
┌─────────────────────────┐
│    Gazebo + Nav2        │
│ (시뮬레이션/내비게이션) │
└────────────┬────────────┘
             │ /scan, /amcl_pose, Nav2 action
             ▼
 ┌──────────────────────────────────────┐
 │        schedule_manager_node          │
 │  - schedule.json 기반 시간대별 순찰 순서 결정 │
 │  - 위험 이벤트 수신 시 goal 취소→대기→재개    │
 │  - /patrol_status 발행 (idle/patrolling/  │
 │    risk_response)                     │
 └───────────┬───────────────┬──────────┘
             │               ▲
 /risk_events│               │goal cancel/resend
             │               │
 ┌───────────▼───────────────┴──────────┐
 │           risk_detector_node          │
 │  - /scan 구독 → 거리 임계값(0.3m) 비교   │
 │  - 이벤트 쿨다운(2s) 후 /risk_events 발행 │
 │  - Discord 웹훅 알림 전송(5s 쿨다운)      │
 └────────────────────────────────────────┘

 /patrol_status, /risk_events
             │
             ▼
 ┌───────────────────────┐       ┌────────────────────┐
 │   api_server.py         │──────▶│  dashboard_app.py    │
 │ (FastAPI, ROS2 구독 +   │  REST │ (Streamlit, 2초마다   │
 │  REST API, :8000)        │       │  자동 새로고침, :8501) │
 └───────────────────────┘       └────────────────────┘
---

## 파일 구성

| 파일 | 역할 |
|---|---|
| `src/patrol_robot/patrol_robot/schedule_manager_node.py` | 순찰 스케줄링 + 위험 대응 상태머신 (비동기 goal 체인) |
| `src/patrol_robot/patrol_robot/risk_detector_node.py` | LiDAR 기반 위험 감지, 이벤트/알림 쿨다운 |
| `src/patrol_robot/patrol_robot/discord_notifier.py` | Discord 웹훅 알림 전송 |
| `src/patrol_robot/config/schedule.json` | 시간대별(commute/operation/lunch/night) 순찰 스케줄 정의 |
| `src/patrol_robot/config/nav2_params.yaml` | Nav2 파라미터 (`use_sim_time`, goal tolerance 등) |
| `maps/my_map.yaml`, `maps/my_map.pgm` | 시뮬레이션 맵 |
| `api_server.py` | FastAPI 서버 — ROS2 상태 구독 후 REST API로 노출 |
| `dashboard_app.py` | Streamlit 대시보드 — 순찰 상태·위치·이벤트 로그 시각화 |

---

## 환경 세팅

### 1. 사전 요구사항
- ROS2 Humble, Gazebo, TurtleBot3, Nav2 (학원 PC 기설치)
- Python 3.10

### 2. 패키지 설치
```bash
pip install python-dotenv requests fastapi uvicorn streamlit pandas
# ROS2 tf_transformations가 numpy 2.x와 호환되지 않으므로 반드시 다운그레이드
pip install "numpy<1.24" --user
```

### 3. 환경변수
프로젝트 루트에 `.env` 파일 생성:

​```
DISCORD_WEBHOOK_URL=<Discord 채널 웹훅 URL>
​```

### 4. 빌드
```bash
cd ~/safety-patrol-robot
colcon build --packages-select patrol_robot
source install/setup.bash
```
⚠️ `schedule_manager_node.py` 등 노드 코드를 수정한 경우, 위 빌드+source를 다시 해야 변경사항이 반영됩니다 (노드는 `install/` 하위 복사본에서 실행됨).

---

## 실행 방법

아래 순서대로 각각 별도 터미널에서 실행합니다.

```bash
# 터미널 1 — Gazebo
cd ~/safety-patrol-robot && source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 터미널 2 — Localization (AMCL)
cd ~/safety-patrol-robot && source install/setup.bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/dmin/safety-patrol-robot/maps/my_map.yaml \
  use_sim_time:=True

# 터미널 3 — Nav2
cd ~/safety-patrol-robot && source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

# 터미널 4 — RViz2 (반드시 이 launch로 열 것 — QoS 관련, 아래 알려진 이슈 참고)
cd ~/safety-patrol-robot && source install/setup.bash
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=True

# 터미널 5 — schedule_manager_node
cd ~/safety-patrol-robot && source install/setup.bash
ros2 run patrol_robot schedule_manager_node

# 터미널 6 — risk_detector_node
cd ~/safety-patrol-robot && source install/setup.bash
ros2 run patrol_robot risk_detector_node

# 터미널 7 — FastAPI 서버
cd ~/safety-patrol-robot && source install/setup.bash
python3 api_server.py

# 터미널 8 — Streamlit 대시보드
cd ~/safety-patrol-robot
streamlit run dashboard_app.py
# → http://localhost:8501 접속
```

RViz가 뜨면 **"2D Pose Estimate"** 로 Gazebo 상 로봇의 실제 위치에 초기 pose를 지정해야 AMCL이 정상 작동합니다.

⚠️ **로봇 스폰 위치 주의**: 기본 스폰 좌표가 필러 구조물에 가까워 위험 감지 임계값(0.3m) 안쪽일 수 있습니다. 이 상태로 바로 순찰을 시작하면 goal 전송 직후 위험 감지→취소→재시도가 반복될 수 있으니, 필요시 `teleop_keyboard`로 로봇을 살짝 이동시킨 뒤 `schedule_manager_node`를 실행하세요.
```bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## 스케줄 설정 (schedule.json)

| 시간대(block_id) | 시간 | 순찰 순서 |
|---|---|---|
| commute | 08:00–09:00 | zone_a → zone_b |
| operation | 09:00–16:00 | zone_a → zone_a → zone_b |
| lunch | 12:00–13:00 | zone_b |
| night | 18:00–08:00 (자정 넘김) | zone_a → zone_b |

웨이포인트: `a_1=(0.55, 0.55)`, `a_2=(0.55, -0.55)`, `b_1=(-0.55, -0.55)` — 맵 중앙 필러 구조물로부터 균등 거리(0.78m)를 확보한 좌표입니다.

> 참고: 16:00~18:00 구간은 로봇 순찰 스케줄에 정의되어 있지 않습니다 — 이 시간대는 사람이 직접 순찰하는 것으로 설계되어 있어 의도된 공백입니다.

---

## 알려진 이슈 (미해결)

- **로봇 스폰 위치**: 기본 스폰 좌표가 위험 감지 임계값 안쪽이라 순찰 시작 전 수동 이동이 필요합니다. (위 "실행 방법" 참고)
- **AMCL 로컬라이제이션 어긋남**: 장시간 구동 시 로컬 코스트맵이 전역 맵과 어긋나는(회전) 현상이 간헐적으로 발생합니다. 위험 대응 시 급격한 방향 전환이 잦아 스캔 매칭 시간이 부족한 것으로 추정되나 원인 미확정입니다. 발생 시 RViz의 "2D Pose Estimate"로 수동 재정렬하면 임시 해결됩니다.
- **schedule_manager_node 종료 시 트레이스백**: 순찰 완료 후에도 위험 이벤트 감시를 위해 노드가 계속 실행되도록 설계되어 있어, Ctrl+C로 종료할 때 `rcl_shutdown already called` 관련 트레이스백이 출력될 수 있습니다. 기능상 문제는 없는 사소한 이슈입니다.

---

## 발표 환경

발표는 학원 PC에서 진행하며 ROS2/Gazebo가 이미 설치되어 있어 별도 클라우드 배포는 필수가 아닙니다. 대신 환경 재현성 점검(패키지 버전 고정, 상대경로/`.env` 분리)과 발표 전 실제 학원 PC 리허설에 집중합니다.
