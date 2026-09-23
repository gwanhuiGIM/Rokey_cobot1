"""Web UI 토픽, 허용값, 기본 상태 스냅샷."""

from __future__ import annotations

from typing import Any


STATUS_TOPIC = "/coffee_system/status"
CONTROL_TOPIC = "/coffee_system/control"
ADMIN_STATUS_TOPIC = "/system_monitor/status"
ADMIN_LOG_TOPIC = "/system_monitor/log"
ADMIN_CMD_TOPIC = "/system_monitor/cmd"

TEST_STAGE_NAMES = {
    "full_sequence": "전체 공정",
    "bean_drop": "원두 투입",
    "grinder": "그라인더",
    "dripper_in": "필터 투입",
    "spiral_pour": "스파이럴 드립",
    "final_drip": "최종 드립",
    "gripper_open": "그리퍼 열기",
    "gripper_close": "그리퍼 닫기",
}
TEST_GRIND_TURNS = {3, 5, 7, 10}
SPEED_MIN_PERCENT = 10
SPEED_MAX_PERCENT = 100
TEST_GRIP_OPEN_MODES = {
    "spoon_cup": "스푼·컵 열기",
    "jar": "병 열기",
    "handle": "손잡이 열기",
}

DEFAULT_STATE: dict[str, Any] = {
    "phase": "WAITING_CONTROLLER",
    "screen": 1,
    "progress": 0,
    "title": "원두 선택 또는 단계 테스트를 시작해 주세요",
    "message": "DI 13~16 또는 단계 테스트를 선택할 때까지 시간 제한 없이 기다립니다.",
    "busy": False,
    "waiting_physical_button": True,
    "waiting_external_force": False,
    "selection_timeout_sec": None,
    "external_force_timeout_sec": 10.0,
    "wait_remaining_sec": None,
    "force_threshold_n": 4.0,
    "force_delta_n": 0.0,
    "force_peak_n": 0.0,
    "selected_bean": "",
    "selected_button": None,
    "selected_grind": "",
    "selected_grind_button": None,
    "grind_turns": 0,
    "grind_current_turns": 0.0,
    "spiral_stage": "대기",
    "spiral_progress": 0.0,
    "spiral_radius_mm": 44.0,
    "spiral_revolutions": 5.0,
    "spiral_duration_sec": 15.0,
    "spiral_j6_delta_deg": -60.0,
    "final_drip_stage": "대기",
    "final_drip_progress": 0.0,
    "final_pour_j6_delta_deg": 45.0,
    "final_pour_tcp_name": "mug",
    "final_pour_restore_tcp_name": "GripperDA_v1",
    "final_pour_linear_vel_mm_s": 80.0,
    "final_pour_angular_vel_deg_s": 25.0,
    "operation_speed_percent": 100,
    "speed_service_ready": False,
    "speed_update_pending": False,
    "speed_update_error": "",
    "speed_previous_percent": 100,
    "speed_applied_percent": 100,
    "speed_effective_variables": {},
    "speed_change_log": [],
    "test_result": "",
    "test_command_error": "",
    "test_mode": False,
    "test_stage_id": "",
    "test_stage_name": "",
    "grip_failure": False,
    "equipment_error": False,
    "failed_stage_id": "",
    "failed_stage_name": "",
    "failed_grip_task": "",
    "recovery_kind": "",
    "recovery_state": "",
    "grip_diagnostics": {},
    "gripper_signal_online": None,
    "recovery_countdown_sec": 0.0,
    "error": "",
    "connected": False,
    "control_connected": False,
}
