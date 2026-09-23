"""DI 13~16 물리 버튼 입력 감지."""

from __future__ import annotations

import time
from typing import Any, Callable, Optional

import rclpy

from dsr_msgs2.msg import RobotState

from .config import (
    PHYSICAL_BUTTONS, BUTTON_POLL_SEC, BUTTON_DEBOUNCE_SEC,
    BUTTON_BASELINE_STABLE_SEC, BUTTON_STATE_STALE_SEC,
    BUTTON_SOURCE_WAIT_SEC, ROBOT_STATE_TYPE, ROBOT_STATE_TOPIC_CANDIDATES,
)
from .errors import ButtonWaitCancelled


class PhysicalButtonInput:
    """DI 13~16을 RobotState 우선, get_digital_input() 보조로 감지한다.

    버튼 해제 상태를 0으로 가정하지 않고 안정된 현재값을 기준값으로 사용하므로
    Active-High와 Active-Low 배선을 모두 처리한다.
    """

    def __init__(
        self,
        node,
        get_digital_input: Callable[[int], int],
    ) -> None:
        self._node = node
        self._get_digital_input = get_digital_input
        self._subscriptions = {}
        self._robot_state_buttons = {}
        self._robot_state_stamp = 0.0
        self._source = "uninitialized"
        self._last_wait_defaulted = False

        for topic in ROBOT_STATE_TOPIC_CANDIDATES:
            self._register_topic(topic)

    def _register_topic(self, topic: str) -> None:
        if not topic or topic in self._subscriptions:
            return

        subscription = self._node.create_subscription(
            RobotState,
            topic,
            lambda msg, source=topic: self._robot_state_callback(msg, source),
            10,
        )
        self._subscriptions[topic] = subscription
        self._node.get_logger().info(f"RobotState 구독 등록: {topic}")

    def _discover_topics(self) -> None:
        try:
            topics = self._node.get_topic_names_and_types()
        except Exception as error:
            self._node.get_logger().warning(f"RobotState 토픽 탐색 실패: {error}")
            return

        for topic, type_names in topics:
            if ROBOT_STATE_TYPE in type_names:
                self._register_topic(topic)

    def _robot_state_callback(self, msg: RobotState, source: str) -> None:
        values = {}

        if hasattr(msg, "controller_digital_input"):
            mask = int(getattr(msg, "controller_digital_input"))
            values = {
                index: (mask >> (index - 1)) & 0x1
                for index in PHYSICAL_BUTTONS
            }
        elif hasattr(msg, "ctrlbox_digital_input"):
            raw = list(getattr(msg, "ctrlbox_digital_input"))
            if len(raw) >= max(PHYSICAL_BUTTONS):
                values = {
                    index: int(raw[index - 1])
                    for index in PHYSICAL_BUTTONS
                }

        if not values:
            return

        self._robot_state_buttons = values
        self._robot_state_stamp = time.monotonic()
        new_source = f"RobotState:{source}"

        if self._source != new_source:
            self._source = new_source
            self._node.get_logger().info(
                f"물리 버튼 입력 소스 전환: {self._source}, raw={values}"
            )

    def _spin_once(self, timeout_sec: float = 0.0) -> None:
        rclpy.spin_once(self._node, timeout_sec=max(0.0, timeout_sec))

    def _robot_state_is_fresh(self) -> bool:
        return (
            bool(self._robot_state_buttons)
            and time.monotonic() - self._robot_state_stamp
            <= BUTTON_STATE_STALE_SEC
        )

    @staticmethod
    def _raise_if_cancelled(
        cancel_if: Optional[Callable[[], bool]],
    ) -> None:
        if cancel_if is not None and cancel_if():
            raise ButtonWaitCancelled("안전 조건이 해제되어 버튼 대기를 취소합니다.")

    def _wait_for_source(
        self,
        cancel_if: Optional[Callable[[], bool]] = None,
    ) -> None:
        deadline = time.monotonic() + BUTTON_SOURCE_WAIT_SEC
        self._raise_if_cancelled(cancel_if)
        self._discover_topics()

        while rclpy.ok():
            self._raise_if_cancelled(cancel_if)
            self._spin_once(0.05)
            self._raise_if_cancelled(cancel_if)
            self._discover_topics()

            if self._robot_state_is_fresh():
                return

            if time.monotonic() >= deadline:
                self._node.get_logger().warning(
                    "RobotState DI를 받지 못해 get_digital_input()으로 대체합니다."
                )
                return

    def read(self) -> dict[int, int]:
        self._spin_once(0.0)
        self._discover_topics()

        if self._robot_state_is_fresh():
            return dict(self._robot_state_buttons)

        values = {
            index: int(self._get_digital_input(index))
            for index in PHYSICAL_BUTTONS
        }

        if self._source != "DSR:get_digital_input":
            self._source = "DSR:get_digital_input"
            self._node.get_logger().warning(
                f"물리 버튼 입력 소스: {self._source}, raw={values}"
            )

        return values

    def _stable_baseline(
        self,
        cancel_if: Optional[Callable[[], bool]] = None,
    ) -> dict[int, int]:
        self._wait_for_source(cancel_if)
        baseline = None
        stable_since = time.monotonic()

        while rclpy.ok():
            self._raise_if_cancelled(cancel_if)
            current = self.read()
            self._raise_if_cancelled(cancel_if)
            now = time.monotonic()

            if baseline != current:
                baseline = dict(current)
                stable_since = now
                self._node.get_logger().info(
                    f"버튼 기준값 확인 ({self._source}): {baseline}"
                )

            if now - stable_since >= BUTTON_BASELINE_STABLE_SEC:
                return dict(baseline)

            time.sleep(BUTTON_POLL_SEC)

        raise KeyboardInterrupt

    def wait_for_button(
        self,
        *,
        timeout_sec: Optional[float] = None,
        default_button: Optional[int] = None,
        allowed_buttons: Optional[set[int]] = None,
        cancel_if: Optional[Callable[[], bool]] = None,
    ) -> int:
        self._last_wait_defaulted = False
        wait_started = time.monotonic()
        baseline = self._stable_baseline(cancel_if)
        allowed = set(PHYSICAL_BUTTONS)
        if allowed_buttons is not None:
            allowed &= {int(index) for index in allowed_buttons}
        if not allowed:
            raise ValueError("대기할 물리 버튼이 하나도 없습니다.")
        if default_button is not None and default_button not in allowed:
            raise ValueError("기본 버튼은 허용된 버튼 중 하나여야 합니다.")
        deadline = (
            wait_started + max(0.0, float(timeout_sec))
            if timeout_sec is not None
            else None
        )
        self._node.get_logger().info(
            f"DI {sorted(allowed)} 버튼 입력 대기: "
            f"source={self._source}, baseline={baseline}, "
            f"timeout={timeout_sec}"
        )

        while rclpy.ok():
            self._raise_if_cancelled(cancel_if)
            if deadline is not None and time.monotonic() >= deadline:
                if default_button is None:
                    raise TimeoutError("물리 버튼 입력 제한시간을 초과했습니다.")
                self._node.get_logger().warning(
                    f"물리 버튼 입력 {float(timeout_sec):.1f}초 초과: "
                    f"DI {default_button}을 기본값으로 선택합니다."
                )
                self._last_wait_defaulted = True
                return int(default_button)

            current = self.read()
            self._raise_if_cancelled(cancel_if)
            changed = [
                index
                for index in sorted(allowed)
                if current[index] != baseline[index]
            ]

            if changed:
                index = changed[0]
                time.sleep(BUTTON_DEBOUNCE_SEC)
                self._raise_if_cancelled(cancel_if)
                confirmed = self.read()
                self._raise_if_cancelled(cancel_if)

                if confirmed[index] != baseline[index]:
                    self._node.get_logger().info(
                        f"물리 버튼 DI {index} 감지: "
                        f"{baseline[index]} -> {confirmed[index]}"
                    )

                    while rclpy.ok():
                        self._raise_if_cancelled(cancel_if)
                        released = self.read()
                        self._raise_if_cancelled(cancel_if)
                        if released[index] == baseline[index]:
                            self._node.get_logger().info(
                                f"물리 버튼 DI {index} 해제 확인"
                            )
                            return index
                        time.sleep(BUTTON_POLL_SEC)

            time.sleep(BUTTON_POLL_SEC)

        raise KeyboardInterrupt

    def wait_for_button_or_command(
        self,
        command_getter: Callable[[], Optional[dict[str, Any]]],
        *,
        timeout_sec: Optional[float] = None,
        default_button: Optional[int] = None,
    ) -> tuple[str, int | dict[str, Any]]:
        """대기 중 DI 버튼과 웹 테스트 명령 중 먼저 들어온 항목을 반환한다."""
        self._last_wait_defaulted = False
        wait_started = time.monotonic()
        baseline = self._stable_baseline()
        deadline = (
            wait_started + max(0.0, float(timeout_sec))
            if timeout_sec is not None
            else None
        )
        if default_button is not None and default_button not in PHYSICAL_BUTTONS:
            raise ValueError("기본 버튼은 DI 13~16 중 하나여야 합니다.")
        self._node.get_logger().info(
            "대기 상태: DI 13~16 또는 웹 단계 테스트 명령 입력 대기, "
            f"source={self._source}, baseline={baseline}, "
            f"timeout={timeout_sec}"
        )

        while rclpy.ok():
            command = command_getter()
            if command is not None:
                return "command", command

            if deadline is not None and time.monotonic() >= deadline:
                if default_button is None:
                    raise TimeoutError("버튼/명령 입력 제한시간을 초과했습니다.")
                self._node.get_logger().warning(
                    f"원두 선택 {float(timeout_sec):.1f}초 초과: "
                    f"DI {default_button}을 기본값으로 선택합니다."
                )
                self._last_wait_defaulted = True
                return "button", int(default_button)

            current = self.read()
            changed = [
                index
                for index in PHYSICAL_BUTTONS
                if current[index] != baseline[index]
            ]
            if changed:
                index = changed[0]
                time.sleep(BUTTON_DEBOUNCE_SEC)
                confirmed = self.read()
                if confirmed[index] != baseline[index]:
                    self._node.get_logger().info(
                        f"물리 버튼 DI {index} 감지: "
                        f"{baseline[index]} -> {confirmed[index]}"
                    )
                    while rclpy.ok():
                        command = command_getter()
                        if command is not None:
                            return "command", command
                        released = self.read()
                        if released[index] == baseline[index]:
                            self._node.get_logger().info(
                                f"물리 버튼 DI {index} 해제 확인"
                            )
                            return "button", index
                        time.sleep(BUTTON_POLL_SEC)
            time.sleep(BUTTON_POLL_SEC)

        raise KeyboardInterrupt

    def last_wait_defaulted(self) -> bool:
        return bool(self._last_wait_defaulted)
