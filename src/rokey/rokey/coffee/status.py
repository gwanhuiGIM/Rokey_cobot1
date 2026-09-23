"""FastAPI Web UI가 구독할 JSON 상태 발행기."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Callable, Optional

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .config import (
    TCP_NAME, VELX_LIN_DEFAULT, ACCX_LIN_DEFAULT, SPIRAL_RADIUS_MM,
    SPIRAL_REVOLUTIONS, SPIRAL_DURATION_SEC, SPIRAL_J6_DELTA_DEG,
    CENTER_PIVOT_RETURN_DURATION_SEC, FINAL_POUR_TCP_NAME,
    FINAL_POUR_J6_DELTA_DEG, FINAL_POUR_ANGULAR_VEL_DEG_S,
    FINAL_POUR_ANGULAR_ACC_DEG_S2, FINAL_RETURN_ANGULAR_VEL_DEG_S,
    FINAL_RETURN_ANGULAR_ACC_DEG_S2, STATUS_TOPIC,
    OPERATION_SPEED_DEFAULT_PERCENT, EXTERNAL_FORCE_TRIGGER_N,
    EXTERNAL_FORCE_TIMEOUT_SEC,
)


def _json_turn_value(value: float) -> int | float:
    """정수 회전은 JSON 정수로, 부분 회전은 소수로 반환한다."""
    rounded = round(float(value), 3)
    nearest = round(rounded)
    if abs(rounded - nearest) < 1.0e-6:
        return int(nearest)
    return rounded


class StatusReporter:
    """FastAPI Web UI가 구독할 JSON 상태를 발행한다."""

    def __init__(
        self,
        node,
        speed_getter: Optional[Callable[[], int]] = None,
    ) -> None:
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = node.create_publisher(String, STATUS_TOPIC, qos)
        self._speed_getter = speed_getter
        self._publish_lock = threading.RLock()
        self._equipment_error_active = False
        self._selected_bean = ""
        self._selected_button: Optional[int] = None
        self._selected_grind = ""
        self._selected_grind_button: Optional[int] = None
        self._grind_turns = 0
        self._grind_current_turns = 0.0
        self._force_delta_n = 0.0
        self._force_peak_n = 0.0
        self._spiral_stage = "대기"
        self._spiral_progress = 0.0
        self._final_drip_stage = "대기"
        self._final_drip_progress = 0.0
        self._cycle_id = 0
        self._test_mode = False
        self._test_stage_id = ""
        self._test_stage_name = ""
        self._test_result = ""

    def begin_cycle(
        self,
        *,
        test_mode: bool,
        test_stage_id: str = "",
        test_stage_name: str = "",
    ) -> None:
        self._cycle_id += 1
        self._test_mode = bool(test_mode)
        self._test_stage_id = str(test_stage_id)
        self._test_stage_name = str(test_stage_name)
        self._test_result = "RUNNING"
        self._selected_bean = ""
        self._selected_button = None
        self._selected_grind = ""
        self._selected_grind_button = None
        self._grind_turns = 0
        self._grind_current_turns = 0.0
        self._force_delta_n = 0.0
        self._force_peak_n = 0.0
        self._spiral_stage = "대기"
        self._spiral_progress = 0.0
        self._final_drip_stage = "대기"
        self._final_drip_progress = 0.0

    def finish_cycle(self, result: str) -> None:
        self._test_result = str(result)

    def clear_test_context(self) -> None:
        self._test_mode = False
        self._test_stage_id = ""
        self._test_stage_name = ""
        self._test_result = ""

    def reset_order_state(self) -> None:
        """새 주문 초기 화면에 이전 주문의 선택/진행 상태를 남기지 않는다."""
        self._selected_bean = ""
        self._selected_button = None
        self._selected_grind = ""
        self._selected_grind_button = None
        self._grind_turns = 0
        self._grind_current_turns = 0.0
        self._force_delta_n = 0.0
        self._force_peak_n = 0.0
        self._spiral_stage = "대기"
        self._spiral_progress = 0.0
        self._final_drip_stage = "대기"
        self._final_drip_progress = 0.0

    def set_selection(
        self,
        button: Optional[int],
        bean_name: str,
    ) -> None:
        self._selected_button = button
        self._selected_bean = bean_name

    def set_grind_selection(
        self,
        button: Optional[int],
        grind_name: str,
        turns: int,
    ) -> None:
        self._selected_grind_button = button
        self._selected_grind = grind_name
        self._grind_turns = turns
        self._grind_current_turns = 0.0

    def set_equipment_error_active(self, active: bool) -> None:
        """장비 오류 중 일반 공정 화면이 오류 화면을 덮지 못하게 한다."""
        with self._publish_lock:
            self._equipment_error_active = bool(active)

    def publish(self, **kwargs: Any) -> None:
        with self._publish_lock:
            if (
                self._equipment_error_active
                and not bool(kwargs.get("equipment_error", False))
            ):
                return
            self._publish_impl(**kwargs)

    def _publish_impl(
        self,
        *,
        phase: str,
        screen: int,
        progress: int,
        title: str,
        message: str,
        busy: bool,
        waiting_physical_button: bool = False,
        waiting_external_force: bool = False,
        force_delta_n: Optional[float] = None,
        force_peak_n: Optional[float] = None,
        grind_current_turns: Optional[float] = None,
        spiral_stage: Optional[str] = None,
        spiral_progress: Optional[float] = None,
        final_drip_stage: Optional[str] = None,
        final_drip_progress: Optional[float] = None,
        selection_timeout_sec: Optional[float] = None,
        wait_remaining_sec: Optional[float] = None,
        grip_failure: bool = False,
        equipment_error: bool = False,
        failed_stage_id: str = "",
        failed_stage_name: str = "",
        failed_grip_task: str = "",
        recovery_kind: str = "",
        recovery_state: str = "",
        grip_diagnostics: Optional[dict[str, Any]] = None,
        gripper_signal_online: Optional[bool] = None,
        recovery_countdown_sec: float = 0.0,
        button: Optional[int] = None,
        error: str = "",
    ) -> None:
        if force_delta_n is not None:
            self._force_delta_n = float(force_delta_n)
        if force_peak_n is not None:
            self._force_peak_n = float(force_peak_n)
        if grind_current_turns is not None:
            current = max(0.0, float(grind_current_turns))
            if self._grind_turns > 0:
                current = min(current, float(self._grind_turns))
            self._grind_current_turns = current
        if spiral_stage is not None:
            self._spiral_stage = str(spiral_stage)
        if spiral_progress is not None:
            self._spiral_progress = max(
                0.0,
                min(100.0, float(spiral_progress)),
            )
        if final_drip_stage is not None:
            self._final_drip_stage = str(final_drip_stage)
        if final_drip_progress is not None:
            self._final_drip_progress = max(
                0.0,
                min(100.0, float(final_drip_progress)),
            )

        grind_progress = 0.0
        if self._grind_turns > 0:
            grind_progress = (
                self._grind_current_turns / float(self._grind_turns) * 100.0
            )

        speed_percent = OPERATION_SPEED_DEFAULT_PERCENT
        if self._speed_getter is not None:
            try:
                speed_percent = int(self._speed_getter())
            except Exception:
                speed_percent = OPERATION_SPEED_DEFAULT_PERCENT

        payload = {
            "phase": phase,
            "screen": screen,
            "progress": progress,
            "title": title,
            "message": message,
            "busy": busy,
            "waiting_physical_button": waiting_physical_button,
            "waiting_external_force": waiting_external_force,
            "selection_timeout_sec": (
                round(max(0.0, float(selection_timeout_sec)), 1)
                if selection_timeout_sec is not None
                else None
            ),
            "external_force_timeout_sec": EXTERNAL_FORCE_TIMEOUT_SEC,
            "wait_remaining_sec": (
                round(max(0.0, float(wait_remaining_sec)), 1)
                if wait_remaining_sec is not None
                else None
            ),
            "force_threshold_n": EXTERNAL_FORCE_TRIGGER_N,
            "force_delta_n": round(self._force_delta_n, 3),
            "force_peak_n": round(self._force_peak_n, 3),
            "selected_bean": self._selected_bean,
            "selected_button": self._selected_button,
            "selected_grind": self._selected_grind,
            "selected_grind_button": self._selected_grind_button,
            "grind_turns": self._grind_turns,
            "grind_current_turns": _json_turn_value(
                self._grind_current_turns
            ),
            "grind_progress": round(grind_progress, 1),
            "spiral_stage": self._spiral_stage,
            "spiral_progress": round(self._spiral_progress, 1),
            "spiral_radius_mm": SPIRAL_RADIUS_MM,
            "spiral_revolutions": SPIRAL_REVOLUTIONS,
            "spiral_duration_sec": SPIRAL_DURATION_SEC,
            "spiral_return_duration_sec": CENTER_PIVOT_RETURN_DURATION_SEC,
            "spiral_j6_delta_deg": SPIRAL_J6_DELTA_DEG,
            "final_drip_stage": self._final_drip_stage,
            "final_drip_progress": round(self._final_drip_progress, 1),
            "final_pour_linear_vel_mm_s": VELX_LIN_DEFAULT,
            "final_pour_angular_vel_deg_s": FINAL_POUR_ANGULAR_VEL_DEG_S,
            "final_pour_linear_acc_mm_s2": ACCX_LIN_DEFAULT,
            "final_pour_angular_acc_deg_s2": FINAL_POUR_ANGULAR_ACC_DEG_S2,
            "final_return_angular_vel_deg_s": FINAL_RETURN_ANGULAR_VEL_DEG_S,
            "final_return_angular_acc_deg_s2": FINAL_RETURN_ANGULAR_ACC_DEG_S2,
            "final_pour_j6_delta_deg": FINAL_POUR_J6_DELTA_DEG,
            "final_pour_tcp_name": FINAL_POUR_TCP_NAME,
            "final_pour_restore_tcp_name": TCP_NAME,
            "operation_speed_percent": speed_percent,
            "test_mode": self._test_mode,
            "test_stage_id": self._test_stage_id,
            "test_stage_name": self._test_stage_name,
            "test_result": self._test_result,
            "grip_failure": bool(grip_failure),
            "equipment_error": bool(equipment_error),
            "failed_stage_id": str(failed_stage_id),
            "failed_stage_name": str(failed_stage_name),
            "failed_grip_task": str(failed_grip_task),
            "recovery_kind": str(recovery_kind),
            "recovery_state": str(recovery_state),
            "grip_diagnostics": dict(grip_diagnostics or {}),
            "gripper_signal_online": gripper_signal_online,
            "recovery_countdown_sec": round(
                max(0.0, float(recovery_countdown_sec)),
                1,
            ),
            "button": button,
            "cycle_id": self._cycle_id,
            "error": error,
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._publisher.publish(msg)
