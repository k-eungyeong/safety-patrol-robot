#!/usr/bin/env python3
"""
schedule_manager_node.py (Day6 업데이트)

Day3 초안에서 추가된 것:
- /risk_events 구독 → 위험 감지 시 현재 goal 취소하고 위험 좌표로 재전송
- /amcl_pose 구독 → 위험 좌표(로봇 기준 상대좌표) 계산에 필요한 로봇 현재 위치 추적

동작 방식:
1. 평소엔 스케줄대로 순찰 (기존 run_once 로직)
2. /risk_events가 오면:
   a. 지금 가고 있던 goal을 취소
   b. distance/angle_rad + 로봇 현재 pose로 위험 지점의 map 좌표 계산
   c. 그 좌표로 새 goal 전송 (확인차 이동)
   d. 도착하면 잠깐 대기 후 원래 순찰 스케줄로 복귀
"""

import json
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


DEFAULT_SCHEDULE_PATH = (
    Path(get_package_share_directory("patrol_robot")) / "config" / "schedule.json"
)

# 위험 지점 확인 후 대기 시간(초) - 너무 짧으면 확인이 무의미, 너무 길면 순찰 지연
RISK_INVESTIGATE_WAIT_SEC = 3.0
GOAL_RETRY_LIMIT = 2
GOAL_RETRY_WAIT_SEC = 1.0


class ScheduleManagerNode(Node):
    def __init__(self, schedule_path: Path = DEFAULT_SCHEDULE_PATH):
        super().__init__("schedule_manager_node")

        self.declare_parameter("schedule_path", str(schedule_path))
        resolved_path = Path(self.get_parameter("schedule_path").value)

        self.schedule = self._load_schedule(resolved_path)
        self.zones_by_id = {z["zone_id"]: z for z in self.schedule["zones"]}

        # --- Nav2 액션 클라이언트 (Day4에서 확인한 방식 그대로 적용) ---
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._current_goal_handle = None
        self._interrupted_by_risk = False

        # 위험 대응으로 순찰이 중단됐을 때 "어디까지 갔는지" 기억해두는 상태.
        # (block_id, zone_index, waypoint_index) 튜플, 없으면 처음부터 순찰.
        # 이게 없으면 위험 이벤트가 걸릴 때마다 run_once()가 patrol_order를
        # 처음부터 다시 순회해서 항상 첫 waypoint에만 머무르게 됨.
        self._patrol_progress: tuple[str, int, int] | None = None
        self._active_block: dict | None = None
        self._active_patrol_order: list | None = None
        self._active_zone_idx = 0
        self._active_wp_idx = 0
        self._pending_zone_id: str | None = None
        self._pending_waypoint: dict | None = None
        self._pending_retry_count = 0
        self._retry_timer = None

        # --- 로봇 현재 위치 추적 ---
        # AMCL은 /amcl_pose를 RELIABLE + TRANSIENT_LOCAL QoS로 발행함.
        # 기본 QoS(depth=10, VOLATILE)로 구독하면 durability 불일치로
        # 메시지를 아예 못 받는 문제가 있어 AMCL과 동일한 QoS로 맞춰줌.
        amcl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._current_pose = None  # (x, y, yaw) 튜플
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_callback, amcl_qos
        )

        # --- 위험 이벤트 구독 ---
        self.create_subscription(String, "/risk_events", self._risk_event_callback, 10)

        # --- Day9(대시보드) 이식: 순찰 상태 발행 ---
        # 대시보드(api_server.py)가 구독해서 화면에 표시함.
        # TRANSIENT_LOCAL로 발행해서 대시보드가 로봇보다 늦게 켜져도
        # 마지막 상태를 즉시 받게 함.
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._status_publisher = self.create_publisher(String, "/patrol_status", status_qos)

        self.get_logger().info("Nav2 액션 서버 대기 중...")
        self._nav_client.wait_for_server()
        self.get_logger().info("Nav2 준비 완료. 스케줄 매니저 시작.")

    # ------------------------------------------------------------------
    # 초기화/유틸
    # ------------------------------------------------------------------

    def _load_schedule(self, path: Path) -> dict:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.get_logger().info(
            f"스케줄 로드 완료: {path} (time_blocks={len(data['time_blocks'])})"
        )
        return data

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._current_pose = (x, y, yaw)

    def _get_current_time_block(self) -> dict | None:
        now_str = datetime.now().strftime("%H:%M")
        for block in self.schedule["time_blocks"]:
            start, end = block["start_time"], block["end_time"]
            if start <= end:
                # 일반적인 경우 (자정 안 넘음)
                if start <= now_str < end:
                    return block
            else:
                # 자정을 넘는 경우 (예: 18:00~08:00)
                if now_str >= start or now_str < end:
                    return block
        return None

    def _waypoint_to_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def _zone_waypoints(self, zone_id: str) -> list[dict]:
        zone = self.zones_by_id.get(zone_id)
        if zone is None:
            self.get_logger().warn(f"알 수 없는 zone_id: {zone_id}, 스킵")
            return []
        return zone["waypoints"]

    def _publish_status(
        self,
        state: str,
        block: dict | None = None,
        zone_id: str | None = None,
        waypoint_id: str | None = None,
        extra: dict | None = None,
    ):
        """대시보드용 순찰 상태 발행. state 예: idle/patrolling/risk_response."""
        payload = {
            "state": state,
            "block_id": block["block_id"] if block else None,
            "block_name": block["block_name"] if block else None,
            "zone_id": zone_id,
            "waypoint_id": waypoint_id,
            "timestamp": datetime.now().isoformat(),
        }
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_publisher.publish(msg)

    # ------------------------------------------------------------------
    # 위험 이벤트 처리 (Day6 핵심)
    # ------------------------------------------------------------------

    def _risk_event_callback(self, msg: String):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"위험 이벤트 파싱 실패: {msg.data}")
            return

        if self._current_pose is None:
            self.get_logger().warn(
                "위험 이벤트 수신했지만 로봇 현재 위치를 아직 모름 (amcl_pose 대기 중). 무시."
            )
            return

        if self._interrupted_by_risk:
            self.get_logger().info("이미 위험 대응 중, 새 이벤트 무시.")
            return

        if self._current_goal_handle is None:
            self.get_logger().info("현재 이동 중인 목표가 없어 위험 이벤트 무시.")
            return

        distance = event.get("distance")
        angle_rad = event.get("angle_rad")
        if distance is None or angle_rad is None:
            self.get_logger().warn(f"위험 이벤트에 distance/angle_rad 없음: {event}")
            return

        self.get_logger().warn(
            f"⚠ 위험 이벤트 수신 : distance={distance:.2f}m, angle_rad={angle_rad:.2f}"
        )

        if self._active_block is not None:
            self._patrol_progress = (
                self._active_block["block_id"],
                self._active_zone_idx,
                self._active_wp_idx,
            )
        self._interrupted_by_risk = True
        self._publish_status(
            "risk_response", extra={"distance": distance, "angle_rad": angle_rad}
        )
        self._cancel_current_goal_and_wait()

    def _cancel_current_goal_and_wait(self):
        if self._current_goal_handle is not None:
            self.get_logger().info("현재 순찰 goal 취소 요청.")
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_cancel_done)
        else:
            self._on_cancel_done(None)

    def _on_cancel_done(self, future):
        self.get_logger().info("현재 goal 취소 완료. 제자리에서 대기 시작.")
        self._current_goal_handle = None
        self.get_logger().info(
            f"위험 상황 대기 : {RISK_INVESTIGATE_WAIT_SEC}초 대기 후 순찰 복귀."
        )
        self._risk_wait_timer = self.create_timer(
            RISK_INVESTIGATE_WAIT_SEC, self._resume_patrol_after_wait
        )

    def _resume_patrol_after_wait(self):
        self._risk_wait_timer.cancel()
        self._risk_wait_timer = None
        self._interrupted_by_risk = False
        self.get_logger().info("순찰 스케줄로 복귀.")
        self.run_once()

    # ------------------------------------------------------------------
    # 기존 순찰 로직 (Day3, goal 전송 부분만 액션클라이언트 직접 호출로 교체)
    # ------------------------------------------------------------------

    def _make_goal_msg(self, pose: PoseStamped) -> NavigateToPose.Goal:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        return goal_msg

    def run_once(self):
        block = self._get_current_time_block()
        if block is None:
            self.get_logger().warn("현재 시각에 해당하는 time_block이 없습니다.")
            self._patrol_progress = None
            self._publish_status("idle")
            return

        patrol_order = block["patrol_order"]

        start_zone_idx = 0
        start_wp_idx = 0
        if self._patrol_progress is not None and self._patrol_progress[0] == block["block_id"]:
            _, start_zone_idx, start_wp_idx = self._patrol_progress
            self.get_logger().info(
                f"중단된 지점부터 순찰 재개 : zone_index={start_zone_idx}, waypoint_index={start_wp_idx}"
            )
        else:
            self.get_logger().info(
                f"현재 블록 : {block['block_name']} ({block['block_id']}), "
                f"순찰 순서 : {patrol_order}"
            )

        self._active_block = block
        self._active_patrol_order = patrol_order
        self._active_zone_idx = start_zone_idx
        self._active_wp_idx = start_wp_idx
        self._advance_patrol()


    def _advance_patrol(self):
        if self._interrupted_by_risk:
            return

        block = self._active_block
        patrol_order = self._active_patrol_order

        while self._active_zone_idx < len(patrol_order):
            zone_id = patrol_order[self._active_zone_idx]
            waypoints = self._zone_waypoints(zone_id)

            if self._active_wp_idx < len(waypoints):
                waypoint = waypoints[self._active_wp_idx]
                self._publish_status(
                    "patrolling", block=block, zone_id=zone_id, waypoint_id=waypoint["waypoint_id"]
                )
                self._go_to_waypoint_async(zone_id, waypoint)
                return

            self._active_zone_idx += 1
            self._active_wp_idx = 0

        self._patrol_progress = None
        self.get_logger().info(f"{block['block_name']} 순찰 완료.")

    def _go_to_waypoint_async(self, zone_id: str, waypoint: dict, retry_count: int = 0):
        pose = self._waypoint_to_pose(waypoint["x"], waypoint["y"], waypoint["yaw"])
        self.get_logger().info(
            f"[{zone_id}] goal 전송 : {waypoint['waypoint_id']} "
            f"(x={waypoint['x']}, y={waypoint['y']})"
        )

        self._pending_zone_id = zone_id
        self._pending_waypoint = waypoint
        self._pending_retry_count = retry_count

        send_goal_future = self._nav_client.send_goal_async(self._make_goal_msg(pose))
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(f"goal 거부됨 : {self._pending_waypoint['waypoint_id']}")
            self._advance_to_next_waypoint()
            return

        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._current_goal_handle = None

        if self._interrupted_by_risk:
            return

        result = future.result()
        if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"[{self._pending_zone_id}] {self._pending_waypoint['waypoint_id']} 도착 완료."
            )
            self._advance_to_next_waypoint()
            return

        status = result.status if result is not None else "unknown"
        self.get_logger().warn(
            f"[{self._pending_zone_id}] {self._pending_waypoint['waypoint_id']} 이동 실패 (status={status})."
        )

        if self._pending_retry_count < GOAL_RETRY_LIMIT:
            self.get_logger().info(
                f"{GOAL_RETRY_WAIT_SEC}초 후 재시도 ({self._pending_retry_count + 1}/{GOAL_RETRY_LIMIT})."
            )
            self._retry_timer = self.create_timer(GOAL_RETRY_WAIT_SEC, self._on_retry_timer)
        else:
            self.get_logger().warn(
                f"[{self._pending_zone_id}] {self._pending_waypoint['waypoint_id']} "
                f"재시도 {GOAL_RETRY_LIMIT}회 모두 실패, 다음 지점으로 넘어감."
            )
            self._advance_to_next_waypoint()

    def _on_retry_timer(self):
        self._retry_timer.cancel()
        self._retry_timer = None
        self._go_to_waypoint_async(
            self._pending_zone_id, self._pending_waypoint, self._pending_retry_count + 1
        )

    def _advance_to_next_waypoint(self):
        self._active_wp_idx += 1
        self._advance_patrol()


    def _spin_until_future_complete(self, future):
        """위험 이벤트 콜백도 계속 처리되도록 spin_until_future_complete 사용."""
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = ScheduleManagerNode()
    try:
        node.run_once()
        rclpy.spin(node)  # run_once 끝나도 위험 이벤트는 계속 감시
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
