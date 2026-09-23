"""주전자 파지부터 보정 스파이럴 드립, 반환까지 수행하는 단계."""

from __future__ import annotations

import math
import time

import numpy as np

from ..config import (
    TCP_NAME, SPOUT_TCP_NAME, SPIRAL_RADIUS_MM, SPIRAL_REVOLUTIONS,
    SPIRAL_DURATION_SEC, SPIRAL_J6_DELTA_DEG, SPIRAL_PATH_POINTS,
    CENTER_PIVOT_RETURN_DURATION_SEC, CENTER_PIVOT_RETURN_POINTS,
    POT_RELEASE_BASE_Z_OFFSET_MM, POT_RELEASE_SETTLE_SEC,
    MOVESX_MAX_LINEAR_STEP_MM, MOVESX_MAX_ANGULAR_STEP_DEG,
    MOVESX_LINEAR_VEL_MM_S, MOVESX_ANGULAR_VEL_DEG_S,
    MOVESX_LINEAR_ACC_MM_S2, MOVESX_ANGULAR_ACC_DEG_S2,
    SPIRAL_CENTER_OFFSET_SIGN_X, SPIRAL_ROTATION_SIGN,
    SYSTEM_POT_GRIP_VALUES,
)
from ..geometry import (
    _smoothstep5, _zyz_to_rotm, _rotvec_to_rotm, _rotm_to_rotvec,
    _rotm_to_zyz_near, _numeric6,
)


def spiral_pour(ctx) -> None:
    """주전자 파지부터 보상 스파이럴, 중심 회전, 반환까지 수행하는 Sub."""
    movej = ctx.api.movej
    movel = ctx.api.movel
    movesx = ctx.api.movesx
    get_current_posx = ctx.api.get_current_posx
    get_current_posj = ctx.api.get_current_posj
    fkin = ctx.api.fkin
    posj = ctx.api.posj
    posx = ctx.api.posx
    DR_BASE = ctx.api.DR_BASE
    DR_MV_MOD_ABS = ctx.api.DR_MV_MOD_ABS
    DR_MV_MOD_REL = ctx.api.DR_MV_MOD_REL
    DR_MV_RA_DUPLICATE = ctx.api.DR_MV_RA_DUPLICATE
    DR_MVS_VEL_NONE = ctx.api.DR_MVS_VEL_NONE
    System_bottle_j_2 = ctx.poses.System_bottle_j_2
    System_pot_grip = ctx.poses.System_pot_grip
    System_pot_grip_joint = ctx.poses.System_pot_grip_joint
    Pour_start_joint = ctx.poses.Pour_start_joint
    Pickup_approach_rel = ctx.poses.Pickup_approach_rel
    Pickup_lift_rel = ctx.poses.Pickup_lift_rel
    node = ctx.node
    status = ctx.status
    handle_grip_open = ctx.handle_grip_open
    close_and_verify_grip = ctx.close_and_verify_grip
    apply_tcp = ctx.apply_tcp

    def publish_spiral(
        *,
        phase: str,
        progress: int,
        spiral_progress: float,
        stage: str,
        title: str,
        message: str,
    ) -> None:
        overall_progress = max(
            75,
            min(88, 75 + round(float(spiral_progress) * 0.13)),
        )
        status.publish(
            phase=phase,
            screen=6,
            progress=overall_progress,
            title=title,
            message=message,
            busy=True,
            spiral_stage=stage,
            spiral_progress=spiral_progress,
        )

    def current_pose_base() -> np.ndarray:
        try:
            value = get_current_posx(ref=DR_BASE)
        except TypeError:
            value = get_current_posx(DR_BASE)
        return _numeric6(value, label="현재 TCP Base pose")

    def current_joint() -> np.ndarray:
        return _numeric6(
            get_current_posj(),
            label="현재 joint pose",
        )

    def forward_kinematics_base(joint_deg) -> np.ndarray:
        joint_pose = posj(
            *[float(value) for value in joint_deg[:6]]
        )
        try:
            value = fkin(joint_pose, ref=DR_BASE)
        except TypeError:
            value = fkin(joint_pose, DR_BASE)
        return _numeric6(value, label="fkin 결과")

    def rotation_distance_deg(first_abc, second_abc) -> float:
        first_rotation = _zyz_to_rotm(first_abc)
        second_rotation = _zyz_to_rotm(second_abc)
        relative = _rotm_to_rotvec(
            first_rotation.T @ second_rotation
        )
        return math.degrees(float(np.linalg.norm(relative)))

    def validate_path(
        start_pose: np.ndarray,
        numeric_path: list[np.ndarray],
        *,
        label: str,
    ) -> None:
        if not numeric_path:
            raise RuntimeError(f"{label} 경로가 비어 있습니다.")

        previous = start_pose
        max_linear_step = 0.0
        max_angular_step = 0.0

        for index, point in enumerate(numeric_path, start=1):
            if point.shape != (6,) or not np.all(np.isfinite(point)):
                raise RuntimeError(
                    f"{label} 경유점 {index}가 올바르지 않습니다: "
                    f"{point!r}"
                )

            linear_step = float(
                np.linalg.norm(point[:3] - previous[:3])
            )
            angular_step = rotation_distance_deg(
                previous[3:],
                point[3:],
            )
            max_linear_step = max(max_linear_step, linear_step)
            max_angular_step = max(
                max_angular_step,
                angular_step,
            )
            previous = point

        node.get_logger().info(
            f"{label} 경로 검증: points={len(numeric_path)}, "
            f"max linear step={max_linear_step:.3f} mm, "
            f"max angular step={max_angular_step:.3f} deg"
        )

        if max_linear_step > MOVESX_MAX_LINEAR_STEP_MM:
            raise RuntimeError(
                f"{label} 경유점 병진 간격이 너무 큽니다: "
                f"{max_linear_step:.3f} mm"
            )
        if max_angular_step > MOVESX_MAX_ANGULAR_STEP_DEG:
            raise RuntimeError(
                f"{label} 경유점 회전 간격이 너무 큽니다: "
                f"{max_angular_step:.3f} deg"
            )

    def call_movesx(
        path,
        *,
        duration_sec: float,
    ):
        kwargs = {
            "ref": DR_BASE,
            "mod": DR_MV_MOD_ABS,
            "vel_opt": DR_MVS_VEL_NONE,
        }
        attempts = (
            lambda: movesx(
                path,
                time=duration_sec,
                **kwargs,
            ),
            lambda: movesx(
                path,
                t=duration_sec,
                **kwargs,
            ),
            lambda: movesx(
                path,
                vel=[
                    MOVESX_LINEAR_VEL_MM_S,
                    MOVESX_ANGULAR_VEL_DEG_S,
                ],
                acc=[
                    MOVESX_LINEAR_ACC_MM_S2,
                    MOVESX_ANGULAR_ACC_DEG_S2,
                ],
                time=duration_sec,
                **kwargs,
            ),
            lambda: movesx(
                path,
                [
                    MOVESX_LINEAR_VEL_MM_S,
                    MOVESX_ANGULAR_VEL_DEG_S,
                ],
                [
                    MOVESX_LINEAR_ACC_MM_S2,
                    MOVESX_ANGULAR_ACC_DEG_S2,
                ],
                duration_sec,
                DR_BASE,
                DR_MV_MOD_ABS,
                DR_MVS_VEL_NONE,
            ),
        )

        last_type_error = None
        for attempt in attempts:
            try:
                return attempt()
            except TypeError as error:
                last_type_error = error

        raise RuntimeError(
            "현재 DSR_ROBOT2 movesx() 시그니처와 맞지 않습니다: "
            f"{last_type_error}"
        )

    # -------------------------------------------------------------
    # 1. 주전자 접근 및 파지
    # -------------------------------------------------------------
    publish_spiral(
        phase="SPIRAL_PICKUP",
        progress=76,
        spiral_progress=5,
        stage="주전자 파지",
        title="드립 주전자를 집는 중",
        message="주전자를 파지한 뒤 스파이럴 시작 자세로 이동합니다.",
    )
    apply_tcp(TCP_NAME)

    movej(
        System_bottle_j_2,
        radius=0.0,
        ra=DR_MV_RA_DUPLICATE,
    )
    movel(
        Pickup_approach_rel,
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_REL,
        ra=DR_MV_RA_DUPLICATE,
    )
    movej(
        System_pot_grip_joint,
        radius=0.0,
        ra=DR_MV_RA_DUPLICATE,
    )

    handle_grip_open()
    time.sleep(0.30)

    movel(
        System_pot_grip,
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_ABS,
        ra=DR_MV_RA_DUPLICATE,
    )
    close_and_verify_grip("spiral_pour", "드립 주전자 잡기")

    movel(
        Pickup_lift_rel,
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_REL,
        ra=DR_MV_RA_DUPLICATE,
    )
    movej(
        Pour_start_joint,
        radius=0.0,
        ra=DR_MV_RA_DUPLICATE,
    )
    movel(
        posx(-5.0, -5.0, 0.0, 0.0, 0.0, 0.0),
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_REL,
        ra=DR_MV_RA_DUPLICATE,
    )

    # -------------------------------------------------------------
    # 2. pot TCP 기준 내향 스파이럴 경로 생성
    # -------------------------------------------------------------
    publish_spiral(
        phase="SPIRAL_PREPARING",
        progress=81,
        spiral_progress=20,
        stage="경로 준비",
        title="스파이럴 드립 경로를 준비하는 중",
        message=(
            f"반경 {SPIRAL_RADIUS_MM:.0f} mm, "
            f"{SPIRAL_REVOLUTIONS:.0f}회전 경로를 계산합니다."
        ),
    )
    if not apply_tcp(SPOUT_TCP_NAME):
        raise RuntimeError(
            f"주둥이 TCP '{SPOUT_TCP_NAME}' 적용 실패"
        )

    start_pose = current_pose_base()
    start_joint = current_joint()

    final_joint = start_joint.copy()
    final_joint[5] += SPIRAL_J6_DELTA_DEG
    final_virtual_pose = forward_kinematics_base(final_joint)

    virtual_drift = final_virtual_pose[:3] - start_pose[:3]
    required_compensation = -virtual_drift

    center_x = (
        start_pose[0]
        + SPIRAL_CENTER_OFFSET_SIGN_X * SPIRAL_RADIUS_MM
    )
    center_y = start_pose[1]
    hold_z = start_pose[2]

    start_rotation = _zyz_to_rotm(start_pose[3:])
    final_rotation = _zyz_to_rotm(final_virtual_pose[3:])
    relative_rotvec = _rotm_to_rotvec(
        start_rotation.T @ final_rotation
    )

    node.get_logger().info(
        "Joint6 단독 가상 회전 시 주둥이 Base 변위 [mm]: "
        f"dX={virtual_drift[0]:+.3f}, "
        f"dY={virtual_drift[1]:+.3f}, "
        f"dZ={virtual_drift[2]:+.3f}"
    )
    node.get_logger().info(
        "로봇 본체의 반대 보상 [mm]: "
        f"X={required_compensation[0]:+.3f}, "
        f"Y={required_compensation[1]:+.3f}, "
        f"Z={required_compensation[2]:+.3f}"
    )

    dense_count = max(4000, SPIRAL_PATH_POINTS * 80)
    dense_s = np.linspace(
        0.0,
        1.0,
        dense_count + 1,
        dtype=float,
    )
    dense_radius = SPIRAL_RADIUS_MM * (1.0 - dense_s)
    dense_theta = (
        SPIRAL_ROTATION_SIGN
        * 2.0
        * math.pi
        * SPIRAL_REVOLUTIONS
        * dense_s
    )
    dense_x = center_x + dense_radius * np.cos(dense_theta)
    dense_y = center_y + dense_radius * np.sin(dense_theta)
    dense_xyz = np.column_stack(
        (
            dense_x,
            dense_y,
            np.full_like(dense_x, hold_z),
        )
    )

    segment_length = np.linalg.norm(
        np.diff(dense_xyz, axis=0),
        axis=1,
    )
    cumulative_length = np.concatenate(
        (
            np.array([0.0], dtype=float),
            np.cumsum(segment_length),
        )
    )
    total_path_length = float(cumulative_length[-1])
    if (
        not math.isfinite(total_path_length)
        or total_path_length <= 0.0
    ):
        raise RuntimeError(
            f"스파이럴 호길이 계산 실패: {total_path_length}"
        )

    target_lengths = np.linspace(
        total_path_length / float(SPIRAL_PATH_POINTS),
        total_path_length,
        SPIRAL_PATH_POINTS,
        dtype=float,
    )
    spiral_s_samples = np.interp(
        target_lengths,
        cumulative_length,
        dense_s,
    )

    numeric_path: list[np.ndarray] = []
    path = []
    previous_abc = start_pose[3:].copy()

    for index, spiral_s in enumerate(
        spiral_s_samples,
        start=1,
    ):
        radius = SPIRAL_RADIUS_MM * (
            1.0 - float(spiral_s)
        )
        theta = (
            SPIRAL_ROTATION_SIGN
            * 2.0
            * math.pi
            * SPIRAL_REVOLUTIONS
            * float(spiral_s)
        )

        target_x = center_x + radius * math.cos(theta)
        target_y = center_y + radius * math.sin(theta)
        target_z = hold_z

        orientation_progress = _smoothstep5(
            index / float(SPIRAL_PATH_POINTS)
        )
        target_rotation = start_rotation @ _rotvec_to_rotm(
            relative_rotvec * orientation_progress
        )
        target_abc = _rotm_to_zyz_near(
            target_rotation,
            previous_abc,
        )
        previous_abc = target_abc

        target = np.array(
            [
                target_x,
                target_y,
                target_z,
                target_abc[0],
                target_abc[1],
                target_abc[2],
            ],
            dtype=float,
        )
        numeric_path.append(target)
        path.append(posx(*[float(value) for value in target]))

    validate_path(
        start_pose,
        numeric_path,
        label="스파이럴",
    )

    # -------------------------------------------------------------
    # 3. 내향 스파이럴 + Joint6 등가 기울임
    # -------------------------------------------------------------
    publish_spiral(
        phase="SPIRAL_POURING",
        progress=84,
        spiral_progress=35,
        stage="스파이럴 드립",
        title="스파이럴 방식으로 물을 붓는 중",
        message=(
            f"{SPIRAL_DURATION_SEC:.0f}초 동안 "
            f"{SPIRAL_REVOLUTIONS:.0f}회 내향 스파이럴을 수행합니다."
        ),
    )
    node.get_logger().info(
        "통합 워크플로우 스파이럴 Sub 시작: "
        f"radius={SPIRAL_RADIUS_MM:.1f} mm, "
        f"rev={SPIRAL_REVOLUTIONS:.1f}, "
        f"duration={SPIRAL_DURATION_SEC:.1f} s, "
        f"J6 equivalent={SPIRAL_J6_DELTA_DEG:.1f} deg"
    )

    result = call_movesx(
        path,
        duration_sec=SPIRAL_DURATION_SEC,
    )
    if isinstance(result, (int, float)) and result < 0:
        raise RuntimeError(
            f"스파이럴 movesx 실패 반환값={result}"
        )

    end_pose = current_pose_base()
    target_end = numeric_path[-1]
    end_error = target_end[:3] - end_pose[:3]
    node.get_logger().info(
        "스파이럴 완료 주둥이 Base 오차 [mm]: "
        f"X={end_error[0]:+.3f}, "
        f"Y={end_error[1]:+.3f}, "
        f"Z={end_error[2]:+.3f}"
    )

    # -------------------------------------------------------------
    # 4. 스파이럴 중심에서 주둥이 XYZ 고정, 주전자 자세만 원복
    # -------------------------------------------------------------
    publish_spiral(
        phase="SPIRAL_CENTER_RETURN",
        progress=92,
        spiral_progress=75,
        stage="중심 고정 자세 복원",
        title="주둥이를 고정한 채 주전자를 세우는 중",
        message=(
            "스파이럴 중심에서 주둥이 위치를 유지하며 "
            f"{CENTER_PIVOT_RETURN_DURATION_SEC:.1f}초 동안 "
            "주전자 자세를 빠르게 복원합니다."
        ),
    )

    fixed_center_xyz = end_pose[:3].copy()
    current_rotation = _zyz_to_rotm(end_pose[3:])
    target_rotation = _zyz_to_rotm(start_pose[3:])
    return_rotvec = _rotm_to_rotvec(
        current_rotation.T @ target_rotation
    )
    total_return_deg = math.degrees(
        float(np.linalg.norm(return_rotvec))
    )

    if total_return_deg >= 0.05:
        return_numeric_path: list[np.ndarray] = []
        return_path = []
        previous_abc = end_pose[3:].copy()

        for index in range(
            1,
            CENTER_PIVOT_RETURN_POINTS + 1,
        ):
            progress_value = _smoothstep5(
                index / float(CENTER_PIVOT_RETURN_POINTS)
            )
            interpolated_rotation = (
                current_rotation
                @ _rotvec_to_rotm(
                    return_rotvec * progress_value
                )
            )
            target_abc = _rotm_to_zyz_near(
                interpolated_rotation,
                previous_abc,
            )
            previous_abc = target_abc

            target = np.array(
                [
                    fixed_center_xyz[0],
                    fixed_center_xyz[1],
                    fixed_center_xyz[2],
                    target_abc[0],
                    target_abc[1],
                    target_abc[2],
                ],
                dtype=float,
            )
            return_numeric_path.append(target)
            return_path.append(
                posx(*[float(value) for value in target])
            )

        validate_path(
            end_pose,
            return_numeric_path,
            label="중심 고정 자세복원",
        )
        return_start_time = time.monotonic()
        result = call_movesx(
            return_path,
            duration_sec=CENTER_PIVOT_RETURN_DURATION_SEC,
        )
        return_elapsed = time.monotonic() - return_start_time
        if (
            isinstance(result, (int, float))
            and result < 0
        ):
            raise RuntimeError(
                "중심 고정 자세복원 movesx 실패 "
                f"반환값={result}"
            )

        pivot_end_pose = current_pose_base()
        pivot_drift = (
            pivot_end_pose[:3] - fixed_center_xyz
        )
        node.get_logger().info(
            "중심 고정 자세복원 결과 [mm]: "
            f"dX={pivot_drift[0]:+.3f}, "
            f"dY={pivot_drift[1]:+.3f}, "
            f"dZ={pivot_drift[2]:+.3f}, "
            f"설정={CENTER_PIVOT_RETURN_DURATION_SEC:.1f}s, "
            f"실측={return_elapsed:.3f}s"
        )

    # -------------------------------------------------------------
    # 5. 원래 파지 위치 + Base Z 10 mm로 이동 후 놓기
    # -------------------------------------------------------------
    publish_spiral(
        phase="SPIRAL_RETURNING",
        progress=96,
        spiral_progress=90,
        stage="주전자 반환",
        title="주전자를 원래 위치로 옮기는 중",
        message="원래 파지 위치의 Base +Z 10 mm 지점으로 이동합니다.",
    )

    if not apply_tcp(TCP_NAME):
        raise RuntimeError(
            f"기본 TCP '{TCP_NAME}' 복원 실패"
        )

    movej(
        System_pot_grip_joint,
        radius=0.0,
        ra=DR_MV_RA_DUPLICATE,
    )

    release_values = [
        float(value) for value in SYSTEM_POT_GRIP_VALUES
    ]
    release_values[2] += POT_RELEASE_BASE_Z_OFFSET_MM
    release_target = posx(*release_values)

    movel(
        release_target,
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_ABS,
        ra=DR_MV_RA_DUPLICATE,
    )

    publish_spiral(
        phase="SPIRAL_RELEASING",
        progress=99,
        spiral_progress=98,
        stage="주전자 놓기",
        title="주전자를 내려놓는 중",
        message="그리퍼를 열어 주전자를 배치합니다.",
    )
    handle_grip_open()
    time.sleep(POT_RELEASE_SETTLE_SEC)

    publish_spiral(
        phase="SPIRAL_DONE",
        progress=99,
        spiral_progress=100,
        stage="완료",
        title="스파이럴 드립 완료",
        message="주전자를 원래 위치에 놓았습니다.",
    )
    node.get_logger().info(
        "통합 워크플로우 스파이럴 Sub 완료"
    )
