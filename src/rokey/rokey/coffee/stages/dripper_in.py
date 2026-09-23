"""분쇄 원두가 담긴 병을 커피 필터에 투입하는 단계."""

from __future__ import annotations

from ..config import (
    TCP_NAME, EXTERNAL_FORCE_TRIGGER_N, EXTERNAL_FORCE_TIMEOUT_SEC,
)


def dripper_in(ctx) -> None:
    """드립 병을 잡고 주기 운동으로 커피를 추출."""
    movej = ctx.api.movej
    movel = ctx.api.movel
    move_periodic = ctx.api.move_periodic
    posx = ctx.api.posx
    DR_BASE = ctx.api.DR_BASE
    DR_TOOL = ctx.api.DR_TOOL
    DR_MV_MOD_ABS = ctx.api.DR_MV_MOD_ABS
    DR_MV_MOD_REL = ctx.api.DR_MV_MOD_REL
    DR_MV_RA_DUPLICATE = ctx.api.DR_MV_RA_DUPLICATE
    System_bottle_j_1 = ctx.poses.System_bottle_j_1
    System_bottle_j_2 = ctx.poses.System_bottle_j_2
    System_bottle_l = ctx.poses.System_bottle_l
    System_bottle_l2 = ctx.poses.System_bottle_l2
    System_drip_j = ctx.poses.System_drip_j
    System_drip_l = ctx.poses.System_drip_l
    status = ctx.status
    jar_grip_open = ctx.jar_grip_open
    close_and_verify_grip = ctx.close_and_verify_grip
    apply_tcp = ctx.apply_tcp
    wait_for_external_force = ctx.wait_for_external_force

    # status.step("dripper_in", 0)  # UI 상태 발행용 — 비활성화
    jar_grip_open()
    movej(System_bottle_j_1, radius=0.0, ra=DR_MV_RA_DUPLICATE)
    movej(System_bottle_j_2, radius=0.0, ra=DR_MV_RA_DUPLICATE)
    movel(
        System_bottle_l, radius=0.0, ref=DR_BASE,
        mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
    )
    close_and_verify_grip("dripper_in", "분쇄 원두 병 잡기")
    movel(
        posx(0.0, 0.0, 5.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
        mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
    )

    # status.step("dripper_in", 1)  # UI 상태 발행용 — 비활성화
    movej(System_bottle_j_2, radius=0.0, ra=DR_MV_RA_DUPLICATE)
    movel(
        posx(0.0, 0.0, 300.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
        mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
    )
    movej(System_drip_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
    movel(
        System_drip_l, radius=0.0, ref=DR_BASE,
        mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
    )
    # 반영될 때까지 재시도 (apply_tcp 참고). 끝내 실패해도 move_periodic은
    # 진행한다 — 병을 잡고 있는 상태라 여기서 멈추는 게 더 위험할 수 있다.
    apply_tcp("joint4")

    # status.step("dripper_in", 2)  # UI 상태 발행용 — 비활성화
    move_periodic(
        amp=[0.0, 0.0, 0.0, 2.0, 0.0, 0.0],
        period=[1.0, 1.0, 1.0, 0.3, 1.0, 1.0],
        atime=0.0,
        repeat=5,
        ref=DR_TOOL,
    )

    status.publish(
        phase="WAIT_EXTERNAL_FORCE",
        screen=5,
        progress=70,
        title="병 바닥을 가볍게 쳐주세요",
        message=(
            f"기준 힘 대비 {EXTERNAL_FORCE_TRIGGER_N:.1f} N 이상의 외력이 "
            f"감지되면 진행합니다. {EXTERNAL_FORCE_TIMEOUT_SEC:.0f}초 동안 "
            "감지되지 않아도 자동으로 다음 단계로 진행합니다."
        ),
        busy=True,
        waiting_external_force=True,
        force_delta_n=0.0,
        force_peak_n=0.0,
    )
    detected_force_n = wait_for_external_force(
        EXTERNAL_FORCE_TRIGGER_N,
        EXTERNAL_FORCE_TIMEOUT_SEC,
    )
    if detected_force_n is None:
        finishing_message = (
            f"{EXTERNAL_FORCE_TIMEOUT_SEC:.0f}초 동안 외력이 감지되지 않아 "
            "자동으로 다음 동작을 진행합니다. 병을 제자리로 옮깁니다."
        )
        reported_force_n = 0.0
    else:
        finishing_message = (
            f"{detected_force_n:.1f} N의 외력을 감지했습니다. "
            "병을 제자리로 옮깁니다."
        )
        reported_force_n = detected_force_n
    status.publish(
        phase="FILTER_FINISHING",
        screen=5,
        progress=74,
        title="커피 필터 투입을 마무리하는 중",
        message=finishing_message,
        busy=True,
        force_delta_n=reported_force_n,
        force_peak_n=reported_force_n,
    )

    # status.step("dripper_in", 3)  # UI 상태 발행용 — 비활성화
    apply_tcp(TCP_NAME)
    movel(
        posx(0.0, 0.0, -150.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_TOOL,
        mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
    )
    movej(System_bottle_j_2, radius=0.0, ra=DR_MV_RA_DUPLICATE)
    movel(
        System_bottle_l2, radius=0.0, ref=DR_BASE,
        mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
    )

    # status.step("dripper_in", 4)  # UI 상태 발행용 — 비활성화
    jar_grip_open()
