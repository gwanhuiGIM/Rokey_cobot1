"""웹 테스트 명령과 실시간 작동 속도 서비스를 중계하는 ROS 노드."""

from __future__ import annotations

import json
import threading
import time
from collections import deque
from typing import Any, Optional

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .config import (
    ROBOT_ID, NODE_NAME, STATUS_TOPIC, CONTROL_TOPIC,
    OPERATION_SPEED_MIN_PERCENT, OPERATION_SPEED_MAX_PERCENT,
    OPERATION_SPEED_DEFAULT_PERCENT, TEST_STAGE_NAMES, TEST_GRIP_OPEN_MODES,
)
from .speed import _effective_speed_payload, _speed_change_lines


class CoffeeControlBridge:
    """웹 테스트 명령 큐와 실시간 작동 속도 서비스를 관리한다.

    메인 로봇 노드와 별도 ROS 노드/Executor를 사용하므로 동기 movej/movel 실행
    중에도 ``change_operation_speed`` 서비스 요청을 처리할 수 있다.
    """

    def __init__(self, speed_service_type: Any) -> None:
        self.node = rclpy.create_node(
            f"{NODE_NAME}_web_control",
            namespace=ROBOT_ID,
        )
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        status_qos = QoSProfile(depth=10)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._lock = threading.RLock()
        self._test_commands: deque[dict[str, Any]] = deque(maxlen=4)
        self._busy = False
        self._desired_speed = OPERATION_SPEED_DEFAULT_PERCENT
        self._applied_speed = OPERATION_SPEED_DEFAULT_PERCENT
        self._speed_dirty = True
        self._speed_future = None
        self._last_speed_error = ""
        self._last_speed_change_log = _speed_change_lines(
            OPERATION_SPEED_DEFAULT_PERCENT,
            OPERATION_SPEED_DEFAULT_PERCENT,
        )
        self._speed_service_type = speed_service_type
        self._speed_client = None

        self._status_publisher = self.node.create_publisher(
            String,
            STATUS_TOPIC,
            status_qos,
        )
        self.node.create_subscription(
            String,
            CONTROL_TOPIC,
            self._command_callback,
            qos,
        )
        if speed_service_type is not None:
            self._speed_client = self.node.create_client(
                speed_service_type,
                "motion/change_operation_speed",
            )
        self.node.create_timer(0.10, self._flush_speed_request)
        self.node.get_logger().info(
            "실시간 속도 로그 활성화: change_operation_speed 적용 시 "
            "명명된 속도 변수의 이전/신규 유효값을 출력합니다."
        )

    def _publish_partial(self, **payload: Any) -> None:
        payload["timestamp"] = time.time()
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_publisher.publish(msg)

    def _command_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return

        command = str(payload.get("cmd", "")).strip()
        if command == "set_speed":
            try:
                requested = int(round(float(payload.get("speed_percent"))))
            except (TypeError, ValueError):
                self._publish_partial(
                    speed_update_error="속도 값이 숫자가 아닙니다.",
                )
                return
            requested = max(
                OPERATION_SPEED_MIN_PERCENT,
                min(OPERATION_SPEED_MAX_PERCENT, requested),
            )
            with self._lock:
                previous_applied = int(self._applied_speed)
                self._desired_speed = requested
                self._speed_dirty = True
                self._last_speed_error = ""
            self.node.get_logger().info(
                f"[속도 변경 요청] 현재 적용={previous_applied}%, 요청={requested}%"
            )
            self._publish_partial(
                operation_speed_percent=requested,
                speed_previous_percent=previous_applied,
                speed_update_pending=True,
                speed_update_error="",
            )
            return

        if command != "start_test":
            return

        stage = str(payload.get("stage", "")).strip()
        if stage not in TEST_STAGE_NAMES:
            self._publish_partial(
                test_command_error=f"허용되지 않은 테스트 단계: {stage}",
            )
            return

        try:
            grind_turns = int(payload.get("grind_turns", 3))
        except (TypeError, ValueError):
            grind_turns = 3
        if grind_turns not in {3, 5, 7, 10}:
            grind_turns = 3

        open_mode = str(
            payload.get("gripper_open_mode", "spoon_cup")
        ).strip()
        if open_mode not in TEST_GRIP_OPEN_MODES:
            open_mode = "spoon_cup"

        normalized = {
            "stage": stage,
            "grind_turns": grind_turns,
            "gripper_open_mode": open_mode,
        }
        with self._lock:
            if self._busy or self._test_commands:
                self._publish_partial(
                    test_command_error="현재 테스트 또는 공정이 실행 중입니다.",
                )
                return
            self._test_commands.append(normalized)
        self._publish_partial(test_command_error="")

    def _flush_speed_request(self) -> None:
        with self._lock:
            if not self._speed_dirty or self._speed_future is not None:
                return
            desired = int(self._desired_speed)

        if self._speed_client is None or self._speed_service_type is None:
            with self._lock:
                self._speed_dirty = False
                self._last_speed_error = (
                    "dsr_msgs2/srv/ChangeOperationSpeed를 불러오지 못했습니다."
                )
            self.node.get_logger().error(
                f"[속도 변경 실패] {self._last_speed_error}"
            )
            self._publish_partial(
                speed_service_ready=False,
                speed_update_pending=False,
                speed_update_error=self._last_speed_error,
                speed_change_log=[self._last_speed_error],
            )
            return

        if not self._speed_client.service_is_ready():
            self._publish_partial(speed_service_ready=False)
            return

        request = self._speed_service_type.Request()
        request.speed = desired
        future = self._speed_client.call_async(request)
        with self._lock:
            self._speed_future = future
        future.add_done_callback(
            lambda done, requested=desired: self._speed_done(done, requested)
        )

    def _speed_done(self, future: Any, requested: int) -> None:
        success = False
        error_text = ""
        try:
            response = future.result()
            success = bool(getattr(response, "success", False))
            if not success:
                error_text = "change_operation_speed 서비스가 실패를 반환했습니다."
        except Exception as error:
            error_text = f"change_operation_speed 서비스 오류: {error}"

        with self._lock:
            previous_applied = int(self._applied_speed)
            self._speed_future = None
            if success:
                self._applied_speed = requested
                self._last_speed_error = ""
                self._speed_dirty = self._desired_speed != requested
                change_lines = _speed_change_lines(previous_applied, requested)
                self._last_speed_change_log = list(change_lines)
            else:
                self._last_speed_error = error_text
                self._speed_dirty = False
                change_lines = [
                    f"작동 속도 변경 실패: {previous_applied}% -> {requested}%",
                    error_text,
                ]
                self._last_speed_change_log = list(change_lines)

        if success:
            self.node.get_logger().info(
                f"[속도 변경 적용 완료] {previous_applied}% -> {requested}%"
            )
            for line in change_lines[1:]:
                self.node.get_logger().info(f"[속도 변수] {line}")
        else:
            self.node.get_logger().error(
                f"[속도 변경 실패] {previous_applied}% -> {requested}%: {error_text}"
            )

        applied_percent = requested if success else previous_applied
        self._publish_partial(
            operation_speed_percent=applied_percent,
            speed_previous_percent=previous_applied,
            speed_applied_percent=applied_percent,
            speed_effective_variables=_effective_speed_payload(applied_percent),
            speed_change_log=change_lines,
            speed_service_ready=True,
            speed_update_pending=False,
            speed_update_error=error_text,
        )

    def pop_test_command(self) -> Optional[dict[str, Any]]:
        with self._lock:
            if not self._test_commands:
                return None
            return self._test_commands.popleft()

    def set_busy(self, busy: bool) -> None:
        with self._lock:
            self._busy = bool(busy)
            if not busy:
                self._test_commands.clear()

    def speed_percent(self) -> int:
        with self._lock:
            return int(self._desired_speed)

    def destroy(self) -> None:
        self.node.destroy_node()
