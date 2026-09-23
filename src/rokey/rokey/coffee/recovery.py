"""그립 실패와 그리퍼 장비 오류에서 단계를 재시작하는 복구 흐름."""

from __future__ import annotations

import time
from typing import Any, Callable, Optional

import rclpy

from .config import (
    TCP_NAME, TEST_STAGE_NAMES, GRIP_SIGNAL_STARTUP_WAIT_SEC,
    GRIP_SIGNAL_RECOVERY_STABLE_SEC, GRIP_SIGNAL_RESTART_COUNTDOWN_SEC,
    GRIP_SIGNAL_RESTART_BUTTON,
)
from .errors import (
    ButtonWaitCancelled, GripFailureError, GripperSignalLostError,
)

stage_recovery_ui = {
    "bean_drop": (2, 10),
    "grinder": (4, 42),
    "dripper_in": (5, 68),
    "spiral_pour": (6, 75),
    "final_drip": (7, 89),
    "gripper_open": (1, 0),
    "gripper_close": (1, 0),
}


def restore_default_context_after_recovery(ctx) -> None:
    """중단 지점의 전용 TCP가 남았으면 기본 TCP로 되돌린다."""
    get_tcp = ctx.api.get_tcp
    apply_tcp = ctx.apply_tcp
    ensure_tool_tcp = ctx.ensure_tool_tcp

    active_tcp = get_tcp()
    if active_tcp != TCP_NAME and not apply_tcp(TCP_NAME):
        raise RuntimeError(
            f"복구 후 기본 TCP({TCP_NAME})로 되돌리지 못했습니다."
        )
    ensure_tool_tcp()


def recover_grip_failure(ctx, error: GripFailureError) -> None:
    """DI13 관리자 호출, DI14 환경 정리 완료 순서로 복구한다."""
    node = ctx.node
    status = ctx.status
    buttons = ctx.buttons
    grip_monitor = ctx.grip

    _, progress = stage_recovery_ui.get(error.stage_id, (10, 0))
    status.publish(
        phase="GRIP_FAILURE",
        screen=10,
        progress=progress,
        title="그립 실패로 작업을 정지했습니다",
        message=(
            f"기억한 단계: {error.stage_name} / 작업: {error.grip_task}. "
            "관리자를 호출하려면 DI 13번 버튼을 누르세요."
        ),
        busy=True,
        waiting_physical_button=True,
        grip_failure=True,
        failed_stage_id=error.stage_id,
        failed_stage_name=error.stage_name,
        failed_grip_task=error.grip_task,
        recovery_kind="GRIP_FAILURE",
        recovery_state="WAIT_ADMIN_CALL",
        grip_diagnostics=error.diagnostics,
        gripper_signal_online=grip_monitor.signal_online(),
        error=str(error),
    )
    node.get_logger().warning(
        f"그립 실패 복구 대기: 단계={error.stage_name}, "
        f"작업={error.grip_task}, DI13 관리자 호출 대기"
    )

    buttons.wait_for_button(allowed_buttons={13})
    status.publish(
        phase="ADMIN_CALLED",
        screen=10,
        progress=progress,
        title="관리자가 호출되었습니다",
        message=(
            "관리자가 작업 환경을 정리한 뒤 DI 14번 버튼을 누르면 "
            f"기억한 {error.stage_name} 단계를 처음부터 다시 시작합니다."
        ),
        busy=True,
        waiting_physical_button=True,
        grip_failure=True,
        failed_stage_id=error.stage_id,
        failed_stage_name=error.stage_name,
        failed_grip_task=error.grip_task,
        recovery_kind="GRIP_FAILURE",
        recovery_state="ADMIN_CALLED",
        grip_diagnostics=error.diagnostics,
        gripper_signal_online=grip_monitor.signal_online(),
        button=13,
        error=str(error),
    )

    buttons.wait_for_button(allowed_buttons={14})
    status.publish(
        phase="STAGE_RESTARTING",
        screen=10,
        progress=progress,
        title=f"{error.stage_name} 단계를 다시 시작합니다",
        message="DI 14번 환경 정리 완료 입력을 확인했습니다.",
        busy=True,
        grip_failure=True,
        failed_stage_id=error.stage_id,
        failed_stage_name=error.stage_name,
        failed_grip_task=error.grip_task,
        recovery_kind="GRIP_FAILURE",
        recovery_state="RESTARTING",
        grip_diagnostics=error.diagnostics,
        gripper_signal_online=grip_monitor.signal_online(),
        button=14,
    )
    node.get_logger().warning(
        f"환경 정리 완료 입력 확인: {error.stage_name} 단계 재시작"
    )
    time.sleep(0.3)


def publish_signal_recovery(
    ctx,
    error: GripperSignalLostError,
    *,
    recovery_state: str,
    message: str,
    countdown_sec: float = 0.0,
    waiting_physical_button: bool = False,
    button: Optional[int] = None,
) -> None:
    status = ctx.status
    grip_monitor = ctx.grip

    _, progress = stage_recovery_ui.get(error.stage_id, (11, 0))
    status.publish(
        phase="GRIPPER_SIGNAL_ERROR",
        screen=11,
        progress=progress,
        title="장비 오류: 그리퍼 상태를 확인해 주세요",
        message=message,
        busy=True,
        waiting_physical_button=waiting_physical_button,
        equipment_error=True,
        failed_stage_id=error.stage_id,
        failed_stage_name=error.stage_name,
        recovery_kind="GRIPPER_SIGNAL_LOSS",
        recovery_state=recovery_state,
        grip_diagnostics=grip_monitor.diagnostics(),
        gripper_signal_online=grip_monitor.signal_online(),
        recovery_countdown_sec=countdown_sec,
        button=button,
        error=str(error),
    )


def notify_signal_loss_immediately(
    ctx,
    error: GripperSignalLostError,
) -> None:
    """watchdog 스레드에서 화면 11을 즉시 latch하고 표시한다."""
    status = ctx.status

    status.set_equipment_error_active(True)
    publish_signal_recovery(
        ctx,
        error,
        recovery_state="WAIT_MOTION_STOP",
        message=(
            f"{error.reason} 로봇 모션 정지를 요청했습니다. "
            "정지 응답이 확인될 때까지 재시작할 수 없습니다."
        ),
    )


def recover_gripper_signal(ctx, error: GripperSignalLostError) -> None:
    """정지와 신호 정상 확인 후 DI13+3초 승인으로 기억 단계를 재시작한다."""
    node = ctx.node
    status = ctx.status
    buttons = ctx.buttons
    grip_monitor = ctx.grip

    status.set_equipment_error_active(True)
    node.get_logger().error(
        f"장비 오류 복구 대기: 단계={error.stage_name}, 원인={error.reason}"
    )
    online_since: Optional[float] = None
    next_publish = 0.0

    while rclpy.ok():
        stop_result = grip_monitor.wait_for_stop_result(0.1)
        if stop_result is not True:
            if stop_result is False:
                grip_monitor.retry_motion_stop("그리퍼 장비 오류 재정지")
            now = time.monotonic()
            if now >= next_publish:
                publish_signal_recovery(
                    ctx,
                    error,
                    recovery_state="WAIT_MOTION_STOP",
                    message=(
                        "로봇 모션 정지 응답을 확인하는 중입니다. "
                        "정지가 확인될 때까지 재시작 승인을 받지 않습니다."
                    ),
                )
                next_publish = now + 0.5
            time.sleep(0.05)
            continue

        now = time.monotonic()
        if not grip_monitor.signal_online():
            online_since = None
            if now >= next_publish:
                health_reason = grip_monitor.signal_health_reason()
                publish_signal_recovery(
                    ctx,
                    error,
                    recovery_state="WAIT_GRIPPER_SIGNAL",
                    message=(
                        "그리퍼 연결과 빨간 오류 표시를 확인해 주세요. "
                        f"현재 상태: {health_reason or error.reason}"
                    ),
                )
                next_publish = now + 0.5
            time.sleep(0.05)
            continue

        if online_since is None:
            online_since = now
        online_duration = now - online_since
        if online_duration < GRIP_SIGNAL_RECOVERY_STABLE_SEC:
            if now >= next_publish:
                publish_signal_recovery(
                    ctx,
                    error,
                    recovery_state="SIGNAL_STABILIZING",
                    message="그리퍼 정상 신호가 안정적으로 유지되는지 확인 중입니다.",
                )
                next_publish = now + 0.2
            time.sleep(0.05)
            continue

        publish_signal_recovery(
            ctx,
            error,
            recovery_state="WAIT_RESTART_CONFIRMATION",
            message=(
                "그리퍼 신호 정상 확인됨. 기억한 "
                f"{error.stage_name} 단계를 다시 시작하려면 "
                f"DI {GRIP_SIGNAL_RESTART_BUTTON}번 버튼을 눌러주세요."
            ),
            waiting_physical_button=True,
        )

        try:
            buttons.wait_for_button(
                allowed_buttons={GRIP_SIGNAL_RESTART_BUTTON},
                cancel_if=lambda: not grip_monitor.signal_online(),
            )
        except ButtonWaitCancelled:
            node.get_logger().warning(
                "재시작 승인 대기 중 그리퍼 상태가 다시 나빠졌습니다."
            )
            online_since = None
            next_publish = 0.0
            continue

        countdown_deadline = (
            time.monotonic() + GRIP_SIGNAL_RESTART_COUNTDOWN_SEC
        )
        countdown_cancelled = False
        next_publish = 0.0
        while rclpy.ok():
            if not grip_monitor.signal_online():
                countdown_cancelled = True
                break
            remaining = max(0.0, countdown_deadline - time.monotonic())
            if time.monotonic() >= next_publish:
                publish_signal_recovery(
                    ctx,
                    error,
                    recovery_state="RESTART_COUNTDOWN",
                    message=(
                        f"DI {GRIP_SIGNAL_RESTART_BUTTON} 입력을 확인했습니다. "
                        f"{remaining:.1f}초 후 {error.stage_name} 단계를 "
                        "처음부터 다시 시작합니다."
                    ),
                    countdown_sec=remaining,
                    button=GRIP_SIGNAL_RESTART_BUTTON,
                )
                next_publish = time.monotonic() + 0.1
            if remaining <= 0.0:
                break
            time.sleep(0.05)

        if not rclpy.ok():
            raise KeyboardInterrupt
        if countdown_cancelled:
            node.get_logger().warning(
                "3초 카운트다운 중 그리퍼 상태가 다시 나빠져 재시작을 취소합니다."
            )
            online_since = None
            next_publish = 0.0
            continue

        if not grip_monitor.clear_signal_loss_if_healthy():
            online_since = None
            next_publish = 0.0
            continue
        break

    if not rclpy.ok():
        raise KeyboardInterrupt

    _, progress = stage_recovery_ui.get(error.stage_id, (11, 0))
    status.publish(
        phase="STAGE_RESTARTING",
        screen=11,
        progress=progress,
        title=f"{error.stage_name} 단계를 다시 시작합니다",
        message=(
            f"DI {GRIP_SIGNAL_RESTART_BUTTON} 입력과 3초 카운트다운을 "
            "완료했습니다."
        ),
        busy=True,
        equipment_error=True,
        failed_stage_id=error.stage_id,
        failed_stage_name=error.stage_name,
        recovery_kind="GRIPPER_SIGNAL_LOSS",
        recovery_state="RESTARTING",
        grip_diagnostics=grip_monitor.diagnostics(),
        gripper_signal_online=True,
        button=GRIP_SIGNAL_RESTART_BUTTON,
    )
    status.set_equipment_error_active(False)
    node.get_logger().warning(
        f"그리퍼 정상 및 DI {GRIP_SIGNAL_RESTART_BUTTON} 승인 확인: "
        f"{error.stage_name} 단계 재시작"
    )


def run_protected_stage(
    ctx,
    stage_id: str,
    action: Callable[[], Any],
) -> Any:
    """현재 단계를 기억하고 그립/신호 오류 뒤 같은 단계를 재시작한다."""
    status = ctx.status
    grip_monitor = ctx.grip

    stage_name = TEST_STAGE_NAMES.get(stage_id, stage_id)
    screen, progress = stage_recovery_ui.get(stage_id, (1, 0))
    retry_count = 0

    while rclpy.ok():
        if retry_count > 0:
            status.publish(
                phase="STAGE_RESTARTED",
                screen=screen,
                progress=progress,
                title=f"{stage_name} 단계 재시작",
                message=(
                    f"기억한 {stage_name} 단계를 처음부터 다시 실행합니다. "
                    f"재시도 {retry_count}회"
                ),
                busy=True,
            )

        if not grip_monitor.wait_until_online(
            GRIP_SIGNAL_STARTUP_WAIT_SEC
        ):
            grip_monitor.begin_stage(stage_id, stage_name)
            grip_monitor.force_signal_loss(
                grip_monitor.signal_health_reason()
                or "단계 시작 전에 그리퍼 정상 상태를 확인하지 못했습니다."
            )
            try:
                grip_monitor.raise_if_signal_lost()
            except GripperSignalLostError as error:
                grip_monitor.end_stage()
                recover_gripper_signal(ctx, error)
                restore_default_context_after_recovery(ctx)
                retry_count += 1
                continue

        grip_monitor.begin_stage(stage_id, stage_name)
        try:
            grip_monitor.raise_if_signal_lost()
            result = action()
            grip_monitor.raise_if_signal_lost()
        except GripFailureError as error:
            grip_monitor.end_stage()
            recover_grip_failure(ctx, error)
            restore_default_context_after_recovery(ctx)
            retry_count += 1
            continue
        except GripperSignalLostError as error:
            grip_monitor.end_stage()
            recover_gripper_signal(ctx, error)
            restore_default_context_after_recovery(ctx)
            retry_count += 1
            continue
        except BaseException:
            grip_monitor.end_stage()
            raise
        else:
            grip_monitor.end_stage()
            return result

    raise KeyboardInterrupt
