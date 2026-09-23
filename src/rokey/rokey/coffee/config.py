"""커피 시스템 전역 설정값과 상수 테이블."""

from __future__ import annotations

import DR_init

# =============================================================================
# 사용자 설정
# =============================================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_coffee_system_single_tilt_v2"

# 티치펜던트에 등록된 이름 (gear.py/move.py와 동일한 이름 사용)
TOOL_NAME = "Tool Weight_gripper"
TCP_NAME = "GripperDA_v1"

# 원본 DRL 하단 전역 설정값
VELJ_DEFAULT = 60.0
ACCJ_DEFAULT = 100.0
VELX_LIN_DEFAULT = 250.0
VELX_ROT_DEFAULT = 80.0
ACCX_LIN_DEFAULT = 1000.0
ACCX_ROT_DEFAULT = 322.5


# =============================================================================
# 스파이럴 드립 Sub 설정
# =============================================================================

SPOUT_TCP_NAME = "pot"

SPIRAL_RADIUS_MM = 44.0
SPIRAL_REVOLUTIONS = 5.0
SPIRAL_DURATION_SEC = 15.0
SPIRAL_J6_DELTA_DEG = -60.0
SPIRAL_PATH_POINTS = 100

# region 드립 주전자 복귀 시간
CENTER_PIVOT_RETURN_DURATION_SEC = 2.0
CENTER_PIVOT_RETURN_POINTS = 30

POT_RELEASE_BASE_Z_OFFSET_MM = 10.0
POT_RELEASE_SETTLE_SEC = 0.8

MOVESX_MAX_LINEAR_STEP_MM = 15.0
MOVESX_MAX_ANGULAR_STEP_DEG = 5.0
MOVESX_LINEAR_VEL_MM_S = 120.0
MOVESX_ANGULAR_VEL_DEG_S = 25.0
MOVESX_LINEAR_ACC_MM_S2 = 300.0
MOVESX_ANGULAR_ACC_DEG_S2 = 80.0

SPIRAL_CENTER_OFFSET_SIGN_X = -1.0
SPIRAL_ROTATION_SIGN = +1.0

# 주전자 교시 좌표
SYSTEM_POT_GRIP_VALUES = [831.36, -177.93, 108.26, 3.26, 90.23, 86.66]
SYSTEM_POT_GRIP_JOINT_VALUES = [-28.73, 38.17, 112.55, 146.06, 65.17, -73.99]
POUR_START_JOINT_VALUES = [-13.00, 23.41, 100.08, 144.04, 35.00, -122.57]
PICKUP_APPROACH_REL_VALUES = [0.0, -80.0, 100.0, 0.0, 0.0, 0.0]
PICKUP_LIFT_REL_VALUES = [-150.0, 0.0, 200.0, 0.0, 0.0, 0.0]


# =============================================================================
# final_drip 물 붓기 설정
# =============================================================================

# plate_outline(2).py의 회전 및 복귀 구조를 final_drip에 통합한다.
# System_final_l 도달 후 J6 상대 +30 deg를 적용하고 현재 자세를 물 붓기 시작점으로 사용한다.
# System_final_j2 교시 MoveJ는 실행하지 않고 고정 mug TCP 회전 경로로 이어 간다.
FINAL_POUR_TCP_NAME = "mug"
FINAL_PRE_POUR_J6_REL_DEG = 30.0
FINAL_POUR_J6_DELTA_DEG = 55.0
FINAL_POUR_PATH_POINTS = 100

# region 물 붓기 시간
FINAL_POUR_ANGULAR_VEL_DEG_S = 120.0
FINAL_POUR_ANGULAR_ACC_DEG_S2 = 350.0
FINAL_RETURN_ANGULAR_VEL_DEG_S = 120.0
FINAL_RETURN_ANGULAR_ACC_DEG_S2 = 350.0
FINAL_MUG_RETURN_BASE_X_OFFSET_MM = -200.0
FINAL_MUG_RETURN_Z_OFFSET_MM = 5.0
FINAL_MUG_RELEASE_TOOL_Z_RETREAT_MM = -150.0
FINAL_MUG_RELEASE_BASE_Z_LIFT_MM = 200.0

FINAL_POUR_MAX_LINEAR_STEP_MM = 5.0
FINAL_POUR_MAX_ANGULAR_STEP_DEG = 3.0

# final_drip의 물 붓기와 시작 자세 복귀 회전에 전용 속도/가속도를 사용한다.
# 물컵을 원래 위치에 내려놓는 Cartesian 모션은 전역 기본값을 유지한다.

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# =============================================================================
# 물리 버튼 / Web UI 상태 연동
# =============================================================================

STATUS_TOPIC = "/coffee_system/status"
CONTROL_TOPIC = "/coffee_system/control"

OPERATION_SPEED_MIN_PERCENT = 10
OPERATION_SPEED_MAX_PERCENT = 100
OPERATION_SPEED_DEFAULT_PERCENT = 100

# change_operation_speed()가 상대 비율을 적용하는 명명된 속도 설정값.
# 아래 Python 상수 자체를 덮어쓰는 것은 아니며, 로그에서는 "명령 기준값 → 유효 적용값"을 표시한다.
OPERATION_SPEED_VARIABLES: tuple[tuple[str, float, str], ...] = (
    ("VELJ_DEFAULT", VELJ_DEFAULT, "deg/s"),
    ("VELX_LIN_DEFAULT", VELX_LIN_DEFAULT, "mm/s"),
    ("VELX_ROT_DEFAULT", VELX_ROT_DEFAULT, "deg/s"),
    ("MOVESX_LINEAR_VEL_MM_S", MOVESX_LINEAR_VEL_MM_S, "mm/s"),
    ("MOVESX_ANGULAR_VEL_DEG_S", MOVESX_ANGULAR_VEL_DEG_S, "deg/s"),
    (
        "FINAL_POUR_ANGULAR_VEL_DEG_S",
        FINAL_POUR_ANGULAR_VEL_DEG_S,
        "deg/s",
    ),
    (
        "FINAL_RETURN_ANGULAR_VEL_DEG_S",
        FINAL_RETURN_ANGULAR_VEL_DEG_S,
        "deg/s",
    ),
)

# change_operation_speed()는 현재 설정된 모션 속도에 상대 비율을 적용한다.
# 가속도 상수는 재할당하지 않으므로 별도 "변경 없음" 로그로 표시한다.
OPERATION_ACCELERATION_VARIABLES: tuple[tuple[str, float, str], ...] = (
    ("ACCJ_DEFAULT", ACCJ_DEFAULT, "deg/s^2"),
    ("ACCX_LIN_DEFAULT", ACCX_LIN_DEFAULT, "mm/s^2"),
    ("ACCX_ROT_DEFAULT", ACCX_ROT_DEFAULT, "deg/s^2"),
    ("MOVESX_LINEAR_ACC_MM_S2", MOVESX_LINEAR_ACC_MM_S2, "mm/s^2"),
    ("MOVESX_ANGULAR_ACC_DEG_S2", MOVESX_ANGULAR_ACC_DEG_S2, "deg/s^2"),
    (
        "FINAL_POUR_ANGULAR_ACC_DEG_S2",
        FINAL_POUR_ANGULAR_ACC_DEG_S2,
        "deg/s^2",
    ),
    (
        "FINAL_RETURN_ANGULAR_ACC_DEG_S2",
        FINAL_RETURN_ANGULAR_ACC_DEG_S2,
        "deg/s^2",
    ),
)

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

TEST_GRIP_OPEN_MODES = {
    "spoon_cup": "스푼·컵 열기",
    "jar": "병 열기",
    "handle": "손잡이 열기",
}

PHYSICAL_BUTTONS = (13, 14, 15, 16)
BEAN_BY_BUTTON = {
    13: {"id": "ethiopia", "name": "에티오피아 예가체프"},
    14: {"id": "colombia", "name": "콜롬비아 수프리모"},
    15: {"id": "brazil", "name": "브라질 산토스"},
    16: {"id": "guatemala", "name": "과테말라 안티구아"},
}

# 분쇄 굵기 선택: 1회전은 movec angle 기준 360도입니다.
GRIND_BY_BUTTON = {
    13: {"id": "coarse", "name": "굵게 분쇄", "turns": 3},
    14: {"id": "medium_coarse", "name": "중간 굵게 분쇄", "turns": 5},
    15: {"id": "medium_fine", "name": "중간 곱게 분쇄", "turns": 7},
    16: {"id": "fine", "name": "곱게 분쇄", "turns": 10},
}
DEGREES_PER_GRINDER_TURN = 360.0

# 그라인더 실제 회전 진행률 측정 설정
GRINDER_USER_COORD = 101
GRINDER_PROGRESS_UPDATE_SEC = 0.10
GRINDER_MOTION_START_TIMEOUT_SEC = 2.0
MOTION_IDLE = 0

# move_periodic 종료 후 사람이 병을 가볍게 쳤는지 확인하는 외력 조건입니다.
# 대기 시작 시 측정한 기준 힘에서 10 N 이상 변화하면 완료로 판정합니다.
EXTERNAL_FORCE_TRIGGER_N = 4.0
EXTERNAL_FORCE_SETTLE_SEC = 0.60
EXTERNAL_FORCE_BASELINE_SAMPLES = 20
EXTERNAL_FORCE_SAMPLE_SEC = 0.02
EXTERNAL_FORCE_CONFIRM_SAMPLES = 2
EXTERNAL_FORCE_UI_UPDATE_SEC = 0.20

# region 선택 시간
SELECTION_TIMEOUT_SEC = 15.0
EXTERNAL_FORCE_TIMEOUT_SEC = 10.0
DEFAULT_SELECTION_BUTTON = 13

# GRIP_DETECTION_README의 RG2 판정 계약.
# 전체 gSTA 상태를 장비 건강 신호로 사용하고, grip 비트와 JointState는 파지 판정에 쓴다.
GRIP_STATUS_TOPIC = "/OnRobotRGInput"
GRIP_DETECTED_TOPIC = "/onrobot/grip_detected"
GRIP_JOINT_STATE_TOPIC = "/onrobot_joint_states"
GRIP_VERIFY_TIMEOUT_SEC = 2.0
GRIP_SIGNAL_TIMEOUT_SEC = 1.0
GRIP_SIGNAL_STARTUP_WAIT_SEC = 2.0
GRIP_SIGNAL_RECOVERY_STABLE_SEC = 0.5
GRIP_SIGNAL_RESTART_COUNTDOWN_SEC = 3.0
GRIP_SIGNAL_RESTART_BUTTON = 13
GRIP_EMPTY_CLOSED_POSITION_RAD = 0.7587
GRIP_POSITION_MARGIN_RAD = 0.02
GRIP_EFFORT_MIN = 1.0e-6
GRIP_SIGNAL_STOP_MODE = 1

# RG2 User Manual의 register 268 (gSTA) bit 정의. 자동 공정에서는
# 안전 스위치가 눌린 상태도 위험 상태로 포함해 fail-closed 처리한다.
RG2_GSTA_SAFETY_FLAGS: tuple[tuple[int, str], ...] = (
    (1 << 2, "S1 안전 스위치 눌림"),
    (1 << 3, "S1 안전회로 작동"),
    (1 << 4, "S2 안전 스위치 눌림"),
    (1 << 5, "S2 안전회로 작동"),
    (1 << 6, "안전 오류"),
)
RG2_GSTA_SAFETY_MASK = sum(mask for mask, _ in RG2_GSTA_SAFETY_FLAGS)


def _rg2_gsta_safety_names(gsta: int) -> list[str]:
    value = int(gsta)
    return [
        name
        for mask, name in RG2_GSTA_SAFETY_FLAGS
        if value & mask
    ]


BUTTON_POLL_SEC = 0.03
BUTTON_DEBOUNCE_SEC = 0.05
BUTTON_BASELINE_STABLE_SEC = 0.20
BUTTON_STATE_STALE_SEC = 1.00
BUTTON_SOURCE_WAIT_SEC = 3.00

ROBOT_STATE_TYPE = "dsr_msgs2/msg/RobotState"
ROBOT_STATE_TOPIC_CANDIDATES = (
    "/dsr01/msg/robot_state",
    "/dsr01/robot_state",
    "/dsr01/dsr_controller2/robot_state",
)
