#!/usr/bin/env python3
"""
nav2_action_client_test.py (Day4 파트1 - 방법 확인용)

nav2_simple_commander 없이, rclpy의 순수 ActionClient로
Nav2의 NavigateToPose 액션에 직접 goal을 보내는 방법을 확인한다.

왜 이걸 따로 확인하냐면:
- schedule_manager_node.py(Day3)는 nav2_simple_commander를 썼는데, 이건 내부적으로
  액션 클라이언트를 감싸서 편하게 만든 것 뿐임
- Day6에서 "위험 감지 시 goal 취소 → 재전송"을 만들려면 액션 클라이언트를
  더 세밀하게 제어할 줄 알아야 함 (goal handle을 직접 들고 있다가 cancel_goal_async 호출 등)
- 이 스크립트는 그 원리를 확인하기 위한 최소 예제
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class Nav2ActionClientTest(Node):
    def __init__(self):
        super().__init__("nav2_action_client_test")
        # NavigateToPose 액션 서버(bt_navigator가 제공)에 연결할 클라이언트 생성
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._goal_handle = None

    def send_goal(self, x: float, y: float, yaw_w: float = 1.0):
        """
        간단하게 하기 위해 방향(orientation)은 z=0, w=yaw_w로 고정.
        실전에서는 schedule_manager_node.py처럼 quaternion_from_euler로 변환해서 써야 함.
        """
        self.get_logger().info("액션 서버(navigate_to_pose) 연결 대기 중...")
        self._action_client.wait_for_server()
        self.get_logger().info("액션 서버 연결됨. goal 전송.")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = yaw_w

        # goal 비동기 전송 + 피드백 콜백 등록
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal이 서버에 의해 거부됨.")
            return

        self.get_logger().info("Goal이 수락됨.")
        self._goal_handle = goal_handle  # Day6에서 취소할 때 이 handle이 필요함

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"이동 중... 남은 거리: {remaining:.2f}m")

    def _result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"완료. status={status}, result={result}")
        rclpy.shutdown()

    def cancel_goal(self):
        """
        Day6에서 위험감지 시 이걸 호출하게 될 예정.
        지금은 실제로 안 쓰지만, 구조 확인용으로 미리 만들어둠.
        """
        if self._goal_handle is not None:
            self.get_logger().info("현재 goal 취소 요청.")
            self._goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ActionClientTest()

    # 테스트용: map 좌표 (1.0, 0.5)로 이동 시도
    node.send_goal(x=1.0, y=0.5)

    rclpy.spin(node)


if __name__ == "__main__":
    main()
