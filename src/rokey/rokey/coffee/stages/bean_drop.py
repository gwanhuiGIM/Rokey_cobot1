"""원두를 스푼으로 퍼서 그라인더 호퍼에 투입하는 단계."""

from __future__ import annotations


def bean_drop(ctx) -> None:
    """스푼을 잡고 원두를 퍼서 그라인더 호퍼에 투입."""
    movej = ctx.api.movej
    movel = ctx.api.movel
    amovel = ctx.api.amovel
    wait = ctx.api.wait
    posj = ctx.api.posj
    posx = ctx.api.posx
    DR_BASE = ctx.api.DR_BASE
    DR_MV_MOD_ABS = ctx.api.DR_MV_MOD_ABS
    DR_MV_MOD_REL = ctx.api.DR_MV_MOD_REL
    DR_MV_RA_DUPLICATE = ctx.api.DR_MV_RA_DUPLICATE
    System_spoon_j = ctx.poses.System_spoon_j
    System_spoon_l = ctx.poses.System_spoon_l
    System_grinder_j = ctx.poses.System_grinder_j
    System_grinder_l = ctx.poses.System_grinder_l
    System_home = ctx.poses.System_home
    System_spoon_l_2 = ctx.poses.System_spoon_l_2
    spoon_cup_grip_open = ctx.spoon_cup_grip_open
    close_and_verify_grip = ctx.close_and_verify_grip

    # status.step("bean_drop", 0)  # UI 상태 발행용 — 비활성화
    spoon_cup_grip_open()

    # status.step("bean_drop", 1)  # UI 상태 발행용 — 비활성화
    movej(System_home, radius=0.0, ra=DR_MV_RA_DUPLICATE)

    # status.step("bean_drop", 2)  # UI 상태 발행용 — 비활성화
    movej(System_spoon_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
    movel(
        System_spoon_l, radius=0.0, ref=DR_BASE,
        mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
    )
    close_and_verify_grip("bean_drop", "스푼 잡기")

    # status.step("bean_drop", 3)  # UI 상태 발행용 — 비활성화
    movel(
        posx(0.0, 0.0, 100.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
        mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
    )
    movej(System_grinder_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
    movel(
        System_grinder_l, radius=0.0, ref=DR_BASE,
        mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
    )

    # status.step("bean_drop", 4)  # UI 상태 발행용 — 비활성화
    amovel(
        posx(32.0, -15.0, 0.0, 0.0, 0.0, 0.0), time=1.0, ref=DR_BASE,
        mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
    )
    movej(
        posj(0.0, 0.0, 0.0, 0.0, 0.0, 45.0), time=1.0,
        radius=0.0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
    )
    movel(
        posx(0.0, 0.0, 5.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
        mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
    )

    # status.step("bean_drop", 5)  # UI 상태 발행용 — 비활성화
    movej(System_grinder_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)

    # status.step("bean_drop", 6)  # UI 상태 발행용 — 비활성화
    movel(
        System_spoon_l_2, radius=0.0, ref=DR_BASE,
        mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
    )
    movel(
        posx(0.0, 0.0, -10.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
        mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
    )
    spoon_cup_grip_open()
    wait(1.0)

    # status.step("bean_drop", 7)  # UI 상태 발행용 — 비활성화
    movej(System_home, radius=0.0, ra=DR_MV_RA_DUPLICATE)
