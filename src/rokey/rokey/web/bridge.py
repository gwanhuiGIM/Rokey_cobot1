"""커피 시스템/시스템 모니터 상태를 모으고 명령을 발행하는 ROS 노드."""

from __future__ import annotations

import json
import threading
from collections import deque
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .constants import (
    ADMIN_CMD_TOPIC, ADMIN_LOG_TOPIC, ADMIN_STATUS_TOPIC, CONTROL_TOPIC,
    DEFAULT_STATE, STATUS_TOPIC,
)


class CoffeeWebBridge(Node):
    def __init__(self) -> None:
        super().__init__("coffee_webui_bridge")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        admin_qos = QoSProfile(depth=10)
        admin_qos.reliability = ReliabilityPolicy.RELIABLE
        admin_qos.durability = DurabilityPolicy.VOLATILE
        self._lock = threading.RLock()
        self._state = dict(DEFAULT_STATE)
        self._admin_state: dict[str, Any] = {}
        self._admin_logs: deque[str] = deque(maxlen=300)
        self.create_subscription(String, STATUS_TOPIC, self._callback, qos)
        self.create_subscription(
            String, ADMIN_STATUS_TOPIC, self._admin_callback, admin_qos)
        self.create_subscription(
            String, ADMIN_LOG_TOPIC, self._log_callback, admin_qos)
        self._admin_cmd = self.create_publisher(
            String, ADMIN_CMD_TOPIC, admin_qos)
        self._coffee_cmd = self.create_publisher(
            String, CONTROL_TOPIC, admin_qos)

    def _callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
        except json.JSONDecodeError:
            return
        with self._lock:
            self._state.update(payload)

    def state(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._state)
        result["connected"] = self.count_publishers(STATUS_TOPIC) > 0
        result["control_connected"] = (
            self.count_subscribers(CONTROL_TOPIC) > 0
        )
        return result

    def _admin_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                with self._lock:
                    self._admin_state = payload
        except json.JSONDecodeError:
            return

    def _log_callback(self, msg: String) -> None:
        with self._lock:
            self._admin_logs.append(msg.data)

    def admin_state(self) -> dict[str, Any]:
        with self._lock:
            state = dict(self._admin_state)
            state["logs"] = list(self._admin_logs)
        try:
            state["connected"] = (
                rclpy.ok() and self.count_publishers(ADMIN_STATUS_TOPIC) > 0)
        except Exception:
            state["connected"] = False
        return state

    def admin_command(self, payload: dict[str, Any]) -> None:
        self._admin_cmd.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

    def coffee_command(self, payload: dict[str, Any]) -> None:
        self._coffee_cmd.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))
