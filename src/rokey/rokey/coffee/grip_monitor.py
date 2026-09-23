"""RG2 그립 판정과 통신/안전 상태 감시."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Optional

import rclpy
from onrobot_rg_msgs.msg import OnRobotRGInput
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from dsr_msgs2.srv import MoveStop

from .config import (
    GRIP_STATUS_TOPIC, GRIP_DETECTED_TOPIC, GRIP_JOINT_STATE_TOPIC,
    GRIP_VERIFY_TIMEOUT_SEC, GRIP_SIGNAL_TIMEOUT_SEC,
    GRIP_EMPTY_CLOSED_POSITION_RAD, GRIP_POSITION_MARGIN_RAD,
    GRIP_EFFORT_MIN, GRIP_SIGNAL_STOP_MODE, RG2_GSTA_SAFETY_MASK,
    _rg2_gsta_safety_names,
)
from .errors import GripperSignalLostError


class GripMonitor:
    """RG2 그립 판정과 실행 중 통신/안전 상태 감시를 담당한다.

    ``/OnRobotRGInput``의 gSTA와 수신 주기를 장비 건강 상태의 권위 있는
    신호로 사용한다. 파지 여부는 ``/onrobot/grip_detected``를 우선하고
    ``/onrobot_joint_states``의 effort/position을 보조 판정에 사용한다.
    """

    def __init__(self, node) -> None:
        self._node = node
        self._lock = threading.RLock()
        self._bit_value: Optional[bool] = None
        self._bit_stamp = 0.0
        self._joint_position: Optional[float] = None
        self._joint_effort: Optional[float] = None
        self._joint_stamp = 0.0
        self._status_gsta: Optional[int] = None
        self._status_stamp = 0.0
        self._stage_active = False
        self._stage_id = ""
        self._stage_name = ""
        self._signal_lost = False
        self._loss_reason = ""
        self._stop_requested = False
        self._stop_completed = False
        self._stop_success: Optional[bool] = None
        self._last_stop_request_stamp = 0.0
        self._signal_loss_callback: Optional[
            Callable[[GripperSignalLostError], None]
        ] = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self._bit_subscription = node.create_subscription(
            Bool,
            GRIP_DETECTED_TOPIC,
            self._bit_callback,
            qos,
        )
        self._joint_subscription = node.create_subscription(
            JointState,
            GRIP_JOINT_STATE_TOPIC,
            self._joint_callback,
            qos,
        )
        self._status_subscription = node.create_subscription(
            OnRobotRGInput,
            GRIP_STATUS_TOPIC,
            self._status_callback,
            qos,
        )
        self._stop_client = node.create_client(MoveStop, "motion/move_stop")
        self._watchdog_timer = node.create_timer(0.05, self._watchdog)

    def _bit_callback(self, msg: Bool) -> None:
        with self._lock:
            self._bit_value = bool(msg.data)
            self._bit_stamp = time.monotonic()

    def _joint_callback(self, msg: JointState) -> None:
        position = None
        effort = None
        if msg.position:
            candidate = float(msg.position[0])
            if math.isfinite(candidate):
                position = candidate
        if msg.effort:
            candidate = float(msg.effort[0])
            if math.isfinite(candidate):
                effort = candidate
        with self._lock:
            self._joint_position = position
            self._joint_effort = effort
            self._joint_stamp = time.monotonic()

    def _status_callback(self, msg: OnRobotRGInput) -> None:
        with self._lock:
            self._status_gsta = int(msg.gsta)
            self._status_stamp = time.monotonic()

    def set_signal_loss_callback(
        self,
        callback: Callable[[GripperSignalLostError], None],
    ) -> None:
        """최초 오류 latch 순간 호출할 비차단 UI 알림 콜백을 등록한다."""
        with self._lock:
            self._signal_loss_callback = callback

    def _signal_health_locked(self, now: float) -> tuple[bool, str]:
        if self._status_stamp <= 0.0:
            return False, f"{GRIP_STATUS_TOPIC} 상태를 한 번도 받지 못했습니다."

        status_age = now - self._status_stamp
        if status_age > GRIP_SIGNAL_TIMEOUT_SEC:
            return (
                False,
                f"{GRIP_STATUS_TOPIC}을 {status_age:.1f}초 동안 받지 못했습니다.",
            )

        gsta = int(self._status_gsta or 0)
        safety_names = _rg2_gsta_safety_names(gsta)
        if safety_names:
            return (
                False,
                "RG2 안전 상태 이상 "
                f"(gSTA=0x{gsta:02X}: {', '.join(safety_names)})",
            )

        return True, ""

    def _signal_online_locked(self, now: float) -> bool:
        return self._signal_health_locked(now)[0]

    def signal_online(self) -> bool:
        with self._lock:
            return self._signal_online_locked(time.monotonic())

    def signal_health_reason(self) -> str:
        with self._lock:
            return self._signal_health_locked(time.monotonic())[1]

    def safety_fault_active(self) -> bool:
        now = time.monotonic()
        with self._lock:
            return bool(
                self._status_stamp > 0.0
                and now - self._status_stamp <= GRIP_SIGNAL_TIMEOUT_SEC
                and int(self._status_gsta or 0) & RG2_GSTA_SAFETY_MASK
            )

    def wait_until_online(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok():
            if self.signal_online():
                return True
            if self.safety_fault_active():
                return False
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)
        raise KeyboardInterrupt

    def begin_stage(self, stage_id: str, stage_name: str) -> None:
        with self._lock:
            self._stage_active = True
            self._stage_id = str(stage_id)
            self._stage_name = str(stage_name)

    def end_stage(self) -> None:
        with self._lock:
            self._stage_active = False

    def stage_context(self) -> tuple[str, str]:
        with self._lock:
            return self._stage_id, self._stage_name

    def diagnostics(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            status_age = (
                round(now - self._status_stamp, 3)
                if self._status_stamp > 0.0
                else None
            )
            bit_age = (
                round(now - self._bit_stamp, 3)
                if self._bit_stamp > 0.0
                else None
            )
            joint_age = (
                round(now - self._joint_stamp, 3)
                if self._joint_stamp > 0.0
                else None
            )
            signal_online, signal_reason = self._signal_health_locked(now)
            gsta = self._status_gsta
            return {
                "status_topic": GRIP_STATUS_TOPIC,
                "status_age_sec": status_age,
                "gsta": gsta,
                "gsta_hex": (
                    f"0x{int(gsta):02X}"
                    if gsta is not None
                    else None
                ),
                "gsta_safety_flags": (
                    _rg2_gsta_safety_names(gsta)
                    if gsta is not None
                    else []
                ),
                "grip_bit": self._bit_value,
                "grip_bit_age_sec": bit_age,
                "joint_position_rad": (
                    round(self._joint_position, 5)
                    if self._joint_position is not None
                    else None
                ),
                "joint_effort": (
                    round(self._joint_effort, 5)
                    if self._joint_effort is not None
                    else None
                ),
                "joint_state_age_sec": joint_age,
                "signal_online": signal_online,
                "signal_reason": signal_reason,
                "empty_closed_position_rad": GRIP_EMPTY_CLOSED_POSITION_RAD,
                "position_margin_rad": GRIP_POSITION_MARGIN_RAD,
            }

    def _notify_signal_loss(
        self,
        error: GripperSignalLostError,
        callback: Optional[Callable[[GripperSignalLostError], None]],
    ) -> None:
        if callback is None:
            return
        try:
            callback(error)
        except Exception as callback_error:
            self._node.get_logger().error(
                "그리퍼 장비 오류 UI 즉시 알림 실패: "
                f"{type(callback_error).__name__}: {callback_error}"
            )

    def _watchdog(self) -> None:
        now = time.monotonic()
        with self._lock:
            signal_online, signal_reason = self._signal_health_locked(now)
            should_stop = (
                self._stage_active
                and not self._signal_lost
                and not signal_online
            )
            if not should_stop:
                return
            self._signal_lost = True
            self._loss_reason = signal_reason
            stage_id = self._stage_id
            stage_name = self._stage_name
            callback = self._signal_loss_callback

        error = GripperSignalLostError(
            stage_id,
            stage_name,
            signal_reason,
        )

        self._node.get_logger().error(
            f"[그리퍼 장비 오류] 단계={stage_name}: {signal_reason}"
        )
        self.request_motion_stop("그리퍼 장비 오류")
        self._notify_signal_loss(error, callback)

    def force_signal_loss(self, reason: str) -> None:
        with self._lock:
            if self._signal_lost:
                return
            self._signal_lost = True
            self._loss_reason = str(reason)
            stage_id = self._stage_id
            stage_name = self._stage_name
            callback = self._signal_loss_callback
        error = GripperSignalLostError(stage_id, stage_name, str(reason))
        self._node.get_logger().error(f"[그리퍼 장비 오류] {reason}")
        self.request_motion_stop("그리퍼 장비 오류")
        self._notify_signal_loss(error, callback)

    def request_motion_stop(self, reason: str) -> None:
        with self._lock:
            if self._stop_requested:
                return
            self._stop_requested = True
            self._stop_completed = False
            self._stop_success = None
            self._last_stop_request_stamp = time.monotonic()

        if not self._stop_client.service_is_ready():
            with self._lock:
                self._stop_completed = True
                self._stop_success = False
            self._node.get_logger().error(
                f"[{reason}] MoveStop 서비스를 찾지 못해 정지 확인에 실패했습니다."
            )
            return

        request = MoveStop.Request()
        request.stop_mode = GRIP_SIGNAL_STOP_MODE
        future = self._stop_client.call_async(request)
        future.add_done_callback(
            lambda done, stop_reason=reason: self._motion_stop_done(
                done,
                stop_reason,
            )
        )

    def _motion_stop_done(self, future: Any, reason: str) -> None:
        success = False
        error_text = ""
        try:
            response = future.result()
            success = bool(getattr(response, "success", False))
            if not success:
                error_text = "MoveStop 서비스가 실패를 반환했습니다."
        except Exception as error:
            error_text = f"MoveStop 서비스 오류: {error}"
        with self._lock:
            self._stop_completed = True
            self._stop_success = success
        if success:
            self._node.get_logger().warning(f"[{reason}] 로봇 모션 정지 완료")
        else:
            self._node.get_logger().error(f"[{reason}] {error_text}")

    def wait_for_stop_result(self, timeout_sec: float = 3.0) -> Optional[bool]:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok():
            with self._lock:
                if not self._stop_requested:
                    return True
                if self._stop_completed:
                    return self._stop_success
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.05)
        raise KeyboardInterrupt

    def retry_motion_stop(self, reason: str) -> None:
        """이전 정지 요청이 실패한 경우 서비스가 복구됐을 때 다시 요청한다."""
        with self._lock:
            if self._stop_success is True or not self._stop_completed:
                return
            if time.monotonic() - self._last_stop_request_stamp < 1.0:
                return
            self._stop_requested = False
            self._stop_completed = False
            self._stop_success = None
            self._last_stop_request_stamp = 0.0
        self.request_motion_stop(reason)

    def raise_if_signal_lost(self) -> None:
        with self._lock:
            if not self._signal_lost:
                return
            stage_id = self._stage_id
            stage_name = self._stage_name
            reason = self._loss_reason
        raise GripperSignalLostError(stage_id, stage_name, reason)

    def clear_signal_loss_if_healthy(self) -> bool:
        """lock 안에서 최종 건강 상태를 확인하고 오류 latch를 해제한다."""
        with self._lock:
            signal_online, _ = self._signal_health_locked(time.monotonic())
            if not signal_online:
                return False
            self._signal_lost = False
            self._loss_reason = ""
            self._stop_requested = False
            self._stop_completed = False
            self._stop_success = None
            self._last_stop_request_stamp = 0.0
            return True

    def verify_grip(
        self,
        command_stamp: float,
        timeout_sec: float = GRIP_VERIFY_TIMEOUT_SEC,
    ) -> tuple[bool, dict[str, Any]]:
        """그리퍼 닫기 이후 새 상태 표본으로 파지 여부를 판정한다."""
        deadline = time.monotonic() + max(0.0, float(timeout_sec))

        while rclpy.ok():
            self.raise_if_signal_lost()
            now = time.monotonic()
            with self._lock:
                bit_is_new = self._bit_stamp >= command_stamp
                bit_is_fresh = (
                    bit_is_new
                    and now - self._bit_stamp <= GRIP_SIGNAL_TIMEOUT_SEC
                )
                bit_value = self._bit_value

            # 하드웨어 비트 true는 즉시 성공으로 확정한다. false는 닫힘 동작이
            # 끝날 때까지 바뀔 수 있으므로 검증 제한시간까지 기다린다.
            if bit_is_fresh and bit_value is True:
                diagnostics = self.diagnostics()
                diagnostics["decision_source"] = "hardware_grip_bit"
                return True, diagnostics

            if now >= deadline:
                break
            time.sleep(0.02)

        now = time.monotonic()
        diagnostics = self.diagnostics()
        with self._lock:
            bit_is_fresh = (
                self._bit_stamp >= command_stamp
                and now - self._bit_stamp <= GRIP_SIGNAL_TIMEOUT_SEC
            )
            joint_is_fresh = (
                self._joint_stamp >= command_stamp
                and now - self._joint_stamp <= GRIP_SIGNAL_TIMEOUT_SEC
            )
            bit_value = self._bit_value
            joint_position = self._joint_position
            joint_effort = self._joint_effort

        if bit_is_fresh:
            diagnostics["decision_source"] = "hardware_grip_bit"
            return bool(bit_value), diagnostics

        if joint_is_fresh:
            effort_grip = (
                joint_effort is not None
                and abs(joint_effort) > GRIP_EFFORT_MIN
            )
            position_grip = (
                joint_position is not None
                and joint_position
                < GRIP_EMPTY_CLOSED_POSITION_RAD - GRIP_POSITION_MARGIN_RAD
            )
            diagnostics["decision_source"] = (
                "joint_effort" if effort_grip else "joint_position"
            )
            return effort_grip or position_grip, diagnostics

        self.force_signal_loss("그립 확인용 새 상태 표본을 받지 못했습니다.")
        self.raise_if_signal_lost()
        raise AssertionError("unreachable")
