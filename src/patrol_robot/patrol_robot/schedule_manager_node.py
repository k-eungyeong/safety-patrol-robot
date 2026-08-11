#!/usr/bin/env python3
"""
schedule_manager_node.py (Day6)

schedule.json 기반 순찰 + 위험 이벤트 수신 시 동적 재조정.

동작:
1. patrol_order 순서대로 waypoint에 goal 전송
2. 이동 중 /risk_events 수신되면: 현재 goal 취소 → 위험 좌표(map 기준 절대좌표)로
   goal 전송 → 도착 대기 → 원래 waypoint로 재전송(재개)
"""

import json
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros


DEFAULT_SCHEDULE_PATH = (
    Path(__file__).resolve().parent.parent / "config" / "schedule.json"
)

RISK_REACT_COOLDOWN_SEC = 5.0


class ScheduleManagerNode(Node):
    def __init__(self, schedule_path: Path = DEFAULT_SCHEDULE_PATH):
        super().__init__("schedule_manager_node")

        self.declare_parameter("schedule_path", str(schedule_path))
        resolved_path = Path(self.get_parameter("schedule_path").value)

        self.schedule = self._load_schedule(resolved_path)
        self.zones_by_id = {z["zone_id"]: z for z in self.schedule["zones"]}

        self._pending_risk_event = None
        self._last_risk_react_time = 0.0
        self.create_subscription(String, "/risk_events", self._risk_event_callback, 10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.navigator = BasicNavigator()
        self.get_logger().info("Nav2 활성화 대기 중...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 준비 완료. 스케줄 매니저 시작.")

    def _load_schedule(self, path: Path) -> dict:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.get_logger().info(
            f"스케줄 로드 완료: {path} (time_blocks={len(data['time_blocks'])})"
        )
        return data

    def _get_current_time_block(self):
        now_str = datetime.now().strftime("%H:%M")
        for block in self.schedule["time_blocks"]:
            if block["start_time"] <= now_str < block["end_time"]:
                return block
        return None

    def _zone_waypoints(self, zone_id: str):
        zone = self.zones_by_id.get(zone_id)
        if zone is None:
            self.get_logger().warn(f"알 수 없는 zone_id: {zone_id}, 스킵")
            return []
        return zone["waypoints"]
    def _risk_event_callback(self, msg: String):
        self.get_logger().info(f"[디버그] 위험 이벤트 수신: {msg.data}")
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"[디버그] JSON 파싱 실패: {msg.data}")
            return
        self._pending_risk_event = event

    def _get_robot_pose(self):
        try:
            now = self.get_clock().now()
            trans = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warn(f"로봇 위치(tf) 조회 실패: {e}")
            return None

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return x, y, yaw

    def _risk_event_to_map_pose(self, event: dict):
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return None
        rx, ry, ryaw = robot_pose

        distance = event["distance"]
        angle_rad = event["angle_rad"]

        target_angle = ryaw + angle_rad
        danger_x = rx + distance * math.cos(target_angle)
        danger_y = ry + distance * math.sin(target_angle)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = danger_x
        pose.pose.position.y = danger_y
        q = quaternion_from_euler(0.0, 0.0, target_angle)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def _handle_pending_risk_if_any(self) -> bool:
        event = self._pending_risk_event
        if event is None:
            return False

        now = time.time()
        if now - self._last_risk_react_time < RISK_REACT_COOLDOWN_SEC:
            return False

        self._pending_risk_event = None
        self._last_risk_react_time = now

        danger_pose = self._risk_event_to_map_pose(event)
        if danger_pose is None:
            self.get_logger().warn("위험좌표 변환 실패, 이번 이벤트는 무시하고 원래 goal 계속 진행")
            return False

        self.get_logger().warn(
            f"위험 감지! 현재 goal 취소 → 위험좌표로 이동: "
            f"(x={danger_pose.pose.position.x:.2f}, y={danger_pose.pose.position.y:.2f})"
        )
        self.navigator.cancelTask()
        time.sleep(0.5)

        self.navigator.goToPose(danger_pose)
        self._wait_for_task_complete(label="위험지점 이동")

        self.get_logger().info("위험지점 확인 완료. 원래 순찰 경로로 복귀합니다.")
        return True

    def _waypoint_to_pose(self, waypoint: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = waypoint["x"]
        pose.pose.position.y = waypoint["y"]

        q = quaternion_from_euler(0.0, 0.0, waypoint["yaw"])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def _wait_for_task_complete(self, label: str = "이동"):
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f"  {label} 중... 남은 거리: {feedback.distance_remaining:.2f}m",
                    throttle_duration_sec=2.0,
                )
        result = self.navigator.getResult()
        self.get_logger().info(f"  {label} 결과: {result}")

    def _go_to_waypoint(self, zone_id: str, waypoint: dict):
        pose = self._waypoint_to_pose(waypoint)
        self.get_logger().info(
            f"[{zone_id}] goal 전송: {waypoint['waypoint_id']} "
            f"(x={waypoint['x']}, y={waypoint['y']})"
        )
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self._handle_pending_risk_if_any():
                self.navigator.goToPose(pose)
                continue

            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f"  이동 중... 남은 거리: {feedback.distance_remaining:.2f}m",
                    throttle_duration_sec=2.0,
                )

        result = self.navigator.getResult()
        self.get_logger().info(f"  결과: {result}")

    def run_once(self):
        block = self._get_current_time_block()
        if block is None:
            self.get_logger().warn("현재 시각에 해당하는 time_block이 없습니다.")
            return

        self.get_logger().info(
            f"현재 블록: {block['block_name']} ({block['block_id']}), "
            f"순찰 순서: {block['patrol_order']}"
        )

        for zone_id in block["patrol_order"]:
            waypoints = self._zone_waypoints(zone_id)
            for wp in waypoints:
                self._go_to_waypoint(zone_id, wp)


def main(args=None):
    rclpy.init(args=args)
    node = ScheduleManagerNode()
    try:
        while rclpy.ok():
            node.run_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
