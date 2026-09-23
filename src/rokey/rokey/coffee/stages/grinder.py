"""그라인더 분쇄 단계와 실시간 회전 진행률 측정."""

from __future__ import annotations

import math
import time

import rclpy

from ..config import (
    DEGREES_PER_GRINDER_TURN, GRINDER_USER_COORD,
    GRINDER_PROGRESS_UPDATE_SEC, GRINDER_MOTION_START_TIMEOUT_SEC,
    MOTION_IDLE,
)
from ..geometry import _pose6_from_dsr, _wrap_radians


def publish_grinding_progress(
    ctx,
    current_turns: float,
    total_turns: int,
) -> None:
    """웹 UI에 실제 그라인더 회전량과 전체 회전량을 발행한다."""
    status = ctx.status

    total = max(1, int(total_turns))
    current = max(0.0, min(float(current_turns), float(total)))
    ratio = current / float(total)
    percent = ratio * 100.0
    overall_progress = int(round(42.0 + 25.0 * ratio))

    status.publish(
        phase="GRINDING",
        screen=4,
        progress=overall_progress,
        title="원두를 갈고 있습니다",
        message=(
            f"현재 {current:.2f} / {total}회전 "
            f"({percent:.0f}%) 진행했습니다."
        ),
        busy=True,
        grind_current_turns=current,
    )


def grind_with_continuous_progress(
    ctx,
    grind_via,
    grind_end,
    grind_turns: int,
) -> None:
    """단일 amovec를 유지하며 현재 TCP 각도로 실제 회전량을 계산한다."""
    amovec = ctx.api.amovec
    get_current_posx = ctx.api.get_current_posx
    check_motion = ctx.api.check_motion
    mwait = ctx.api.mwait
    DR_MV_RA_OVERRIDE = ctx.api.DR_MV_RA_OVERRIDE
    node = ctx.node
    grip_monitor = ctx.grip

    total_angle = DEGREES_PER_GRINDER_TURN * float(grind_turns)
    start_pose = _pose6_from_dsr(
        get_current_posx(ref=GRINDER_USER_COORD)
    )
    start_angle = math.atan2(start_pose[1], start_pose[0])

    # 현재점 -> 경유점 방향으로 원호 진행 방향을 결정한다.
    via_angle = math.atan2(-124.5, 0.0)
    direction_delta = _wrap_radians(via_angle - start_angle)
    direction = -1.0 if direction_delta < 0.0 else 1.0

    publish_grinding_progress(ctx, 0.0, grind_turns)
    amovec(
        grind_via,
        grind_end,
        ref=GRINDER_USER_COORD,
        angle=[total_angle, 0.0],
        ra=DR_MV_RA_OVERRIDE,
    )

    last_angle = start_angle
    accumulated_angle = 0.0
    motion_seen = False
    motion_start_deadline = (
        time.monotonic() + GRINDER_MOTION_START_TIMEOUT_SEC
    )
    next_update = 0.0
    monitor_warning_logged = False

    while rclpy.ok():
        grip_monitor.raise_if_signal_lost()
        now = time.monotonic()

        try:
            motion_state = int(check_motion())
        except Exception as error:
            if not monitor_warning_logged:
                node.get_logger().warning(
                    "check_motion() 실패로 회전 중 실시간 측정을 중단하고 "
                    f"모션 종료만 기다립니다: {error}"
                )
                monitor_warning_logged = True
            if callable(mwait):
                mwait(0)
            break

        if motion_state != MOTION_IDLE:
            motion_seen = True

        try:
            pose = _pose6_from_dsr(
                get_current_posx(ref=GRINDER_USER_COORD)
            )
            current_angle = math.atan2(pose[1], pose[0])
            angular_step = _wrap_radians(current_angle - last_angle)
            directed_step = direction * angular_step

            # 반대 방향의 미세 진동은 진행량에서 제외한다.
            if directed_step > 0.0:
                accumulated_angle += directed_step

            last_angle = current_angle
            current_turns = min(
                accumulated_angle / (2.0 * math.pi),
                float(grind_turns),
            )

            if now >= next_update:
                publish_grinding_progress(ctx, current_turns, grind_turns)
                next_update = now + GRINDER_PROGRESS_UPDATE_SEC
        except Exception as error:
            if not monitor_warning_logged:
                node.get_logger().warning(
                    "그라인더 현재 pose를 읽지 못해 마지막 진행률을 유지합니다: "
                    f"{error}"
                )
                monitor_warning_logged = True

        if motion_seen and motion_state == MOTION_IDLE:
            break

        if not motion_seen and now >= motion_start_deadline:
            node.get_logger().warning(
                "amovec 시작 상태를 확인하지 못했습니다. 모션 종료를 기다립니다."
            )
            if callable(mwait):
                mwait(0)
            break

        time.sleep(0.03)

    if callable(mwait):
        mwait(0)
    grip_monitor.raise_if_signal_lost()

    publish_grinding_progress(ctx, float(grind_turns), grind_turns)


def grind_with_turn_steps(
    ctx,
    grind_via,
    grind_end,
    grind_turns: int,
) -> None:
    """비동기 API 미지원 시 1회전 movec 단위로 진행률을 발행한다."""
    movec = ctx.api.movec
    DR_MV_RA_OVERRIDE = ctx.api.DR_MV_RA_OVERRIDE

    publish_grinding_progress(ctx, 0.0, grind_turns)

    for completed_turns in range(1, grind_turns + 1):
        movec(
            grind_via,
            grind_end,
            radius=0.0,
            ref=GRINDER_USER_COORD,
            angle=[DEGREES_PER_GRINDER_TURN, 0.0],
            ra=DR_MV_RA_OVERRIDE,
        )
        publish_grinding_progress(ctx, float(completed_turns), grind_turns)


def grinder(ctx, grind_turns: int) -> None:
    """선택된 회전 수만큼 그라인더 손잡이를 돌려 원두를 분쇄."""
    movej = ctx.api.movej
    movel = ctx.api.movel
    amovec = ctx.api.amovec
    task_compliance_ctrl = ctx.api.task_compliance_ctrl
    release_compliance_ctrl = ctx.api.release_compliance_ctrl
    set_stiffnessx = ctx.api.set_stiffnessx
    check_motion = ctx.api.check_motion
    posj = ctx.api.posj
    posx = ctx.api.posx
    DR_BASE = ctx.api.DR_BASE
    DR_MV_MOD_ABS = ctx.api.DR_MV_MOD_ABS
    DR_MV_RA_DUPLICATE = ctx.api.DR_MV_RA_DUPLICATE
    System_handle_j = ctx.poses.System_handle_j
    System_handle_l = ctx.poses.System_handle_l
    node = ctx.node
    jar_grip_open = ctx.jar_grip_open
    close_and_verify_grip = ctx.close_and_verify_grip

    if grind_turns <= 0:
        raise ValueError(f"그라인더 회전 수는 1 이상이어야 합니다: {grind_turns}")

    # status.step("grinder", 0)  # UI 상태 발행용 — 비활성화
    movej(System_handle_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
    jar_grip_open()
    movel(
        System_handle_l, radius=0.0, ref=DR_BASE,
        mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
    )
    close_and_verify_grip("grinder", "그라인더 손잡이 잡기")

    # status.step("grinder", 1)  # UI 상태 발행용 — 비활성화
    movej(
        posj(12.35, 34.84, 27.41, 1.47, 118.92, 12.01),
        radius=0.0, ra=DR_MV_RA_DUPLICATE,
    )

    grind_via = posx(
        0.0, -124.5, 0.0, 90.0, -179.99, 88.27
    )
    grind_end = posx(
        -124.5, 0.0, 0.0, 90.0, -179.99, 88.27
    )

    # status.step("grinder", 2)  # UI 상태 발행용 — 비활성화
    task_compliance_ctrl()
    set_stiffnessx(
        [1500.0, 1500.0, 2500.0, 150.0, 150.0, 200.0],
        time=0.5,
    )

    try:
        # amovec + check_motion이 있으면 기존처럼 한 번의 연속 원호 모션을
        # 유지하면서 좌표계 101의 현재 X/Y 각도로 실제 회전량을 측정한다.
        if callable(amovec) and callable(check_motion):
            grind_with_continuous_progress(
                ctx, grind_via, grind_end, grind_turns
            )
        else:
            node.get_logger().warning(
                "amovec/check_motion 미지원: 1회전 movec 단위 진행률로 대체합니다."
            )
            grind_with_turn_steps(ctx, grind_via, grind_end, grind_turns)
    finally:
        # status.step("grinder", 4)  # UI 상태 발행용 — 비활성화
        release_compliance_ctrl()
        jar_grip_open()
