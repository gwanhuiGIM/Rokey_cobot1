"""필터 홀더/물컵 배치와 mug TCP 고정 물 붓기를 수행하는 최종 단계."""

from __future__ import annotations

import math
import time
from typing import Any, Optional

import numpy as np

from ..config import (
    TCP_NAME, VELJ_DEFAULT, ACCJ_DEFAULT, VELX_LIN_DEFAULT,
    VELX_ROT_DEFAULT, ACCX_LIN_DEFAULT, ACCX_ROT_DEFAULT,
    FINAL_POUR_TCP_NAME, FINAL_PRE_POUR_J6_REL_DEG,
    FINAL_POUR_J6_DELTA_DEG, FINAL_POUR_PATH_POINTS,
    FINAL_POUR_ANGULAR_VEL_DEG_S, FINAL_POUR_ANGULAR_ACC_DEG_S2,
    FINAL_RETURN_ANGULAR_VEL_DEG_S, FINAL_RETURN_ANGULAR_ACC_DEG_S2,
    FINAL_MUG_RETURN_BASE_X_OFFSET_MM, FINAL_MUG_RETURN_Z_OFFSET_MM,
    FINAL_MUG_RELEASE_TOOL_Z_RETREAT_MM, FINAL_MUG_RELEASE_BASE_Z_LIFT_MM,
    FINAL_POUR_MAX_LINEAR_STEP_MM, FINAL_POUR_MAX_ANGULAR_STEP_DEG,
)
from ..geometry import (
    _smoothstep5, _zyz_to_rotm, _project_rotation, _rotvec_to_rotm,
    _rotm_to_rotvec, _rotm_to_zyz_near, _numeric6,
)


def final_drip(ctx) -> None:
    """DRL final_drip 동작 후 plate_outline 기반 물 붓기를 수행한다.

    순서
    ----
    1. 필터 홀더를 집어 지정 위치로 이동
    2. 물컵을 집어 최종 드립 시작 자세로 이동
    3. 정확히 교시된 ``mug`` TCP를 활성화
    4. Joint6 상대 +30 deg 후 mug TCP 원점을 고정한 +55 deg 회전 경로 실행
    5. 저장한 물 붓기 시작 회전행렬을 직접 목표로 새 복귀 경로를 계산
    6. final_drip 진입 전 활성 TCP로 복원
    7. Base -X 200 mm 상대 이동 후 ``System_mug_l``의 Base +Z 5 mm에서 물컵 해제
    8. Tool -Z 150 mm, Base +Z 150 mm 순서로 이동한 뒤 ``System_home`` 복귀

    mug TCP 자체가 실제 주둥이 끝이므로 별도의 Tool XYZ 오프셋은 사용하지 않는다.
    """
    movej = ctx.api.movej
    movel = ctx.api.movel
    movesx = ctx.api.movesx
    get_current_posx = ctx.api.get_current_posx
    get_current_posj = ctx.api.get_current_posj
    fkin = ctx.api.fkin
    get_tcp = ctx.api.get_tcp
    set_velj = ctx.api.set_velj
    set_accj = ctx.api.set_accj
    set_velx = ctx.api.set_velx
    set_accx = ctx.api.set_accx
    wait = ctx.api.wait
    posj = ctx.api.posj
    posx = ctx.api.posx
    DR_BASE = ctx.api.DR_BASE
    DR_TOOL = ctx.api.DR_TOOL
    DR_MV_MOD_ABS = ctx.api.DR_MV_MOD_ABS
    DR_MV_MOD_REL = ctx.api.DR_MV_MOD_REL
    DR_MV_RA_DUPLICATE = ctx.api.DR_MV_RA_DUPLICATE
    DR_MVS_VEL_NONE = ctx.api.DR_MVS_VEL_NONE
    System_home = ctx.poses.System_home
    System_fitter_j = ctx.poses.System_fitter_j
    System_filtter_l = ctx.poses.System_filtter_l
    System_filtter_l2 = ctx.poses.System_filtter_l2
    System_mug_j = ctx.poses.System_mug_j
    System_mug_l = ctx.poses.System_mug_l
    System_mug_release_l = ctx.poses.System_mug_release_l
    System_final_l = ctx.poses.System_final_l
    System_final_approach_j = ctx.poses.System_final_approach_j
    node = ctx.node
    status = ctx.status
    handle_grip_open = ctx.handle_grip_open
    spoon_cup_grip_open = ctx.spoon_cup_grip_open
    close_and_verify_grip = ctx.close_and_verify_grip
    apply_tcp = ctx.apply_tcp

    # 다른 모든 Sub와 동일한 전역 속도/가속도를 적용한다.
    set_velj(VELJ_DEFAULT)
    set_accj(ACCJ_DEFAULT)
    set_velx(VELX_LIN_DEFAULT, VELX_ROT_DEFAULT)
    set_accx(ACCX_LIN_DEFAULT, ACCX_ROT_DEFAULT)

    def publish_final(
        *,
        phase: str,
        overall_progress: int,
        final_progress: float,
        stage: str,
        title: str,
        message: str,
    ) -> None:
        status.publish(
            phase=phase,
            screen=7,
            progress=overall_progress,
            title=title,
            message=message,
            busy=True,
            final_drip_stage=stage,
            final_drip_progress=final_progress,
        )

    def current_pose_base() -> np.ndarray:
        try:
            value = get_current_posx(ref=DR_BASE)
        except TypeError:
            value = get_current_posx(DR_BASE)
        return _numeric6(value, label="현재 mug TCP Base pose")

    def current_joint() -> np.ndarray:
        return _numeric6(get_current_posj(), label="현재 joint pose")

    def forward_kinematics_base(joint_deg: np.ndarray) -> np.ndarray:
        joint = posj(*[float(value) for value in joint_deg[:6]])
        try:
            value = fkin(joint, ref=DR_BASE)
        except TypeError:
            value = fkin(joint, DR_BASE)
        return _numeric6(value, label="fkin 결과")

    def rotation_distance_deg(first_abc, second_abc) -> float:
        first_rotation = _zyz_to_rotm(first_abc)
        second_rotation = _zyz_to_rotm(second_abc)
        relative = _rotm_to_rotvec(first_rotation.T @ second_rotation)
        return math.degrees(float(np.linalg.norm(relative)))

    def mug_tcp_point_base(tcp_pose: np.ndarray) -> np.ndarray:
        """mug TCP 원점은 실제 주둥이 끝과 동일하므로 Base XYZ만 반환한다."""
        return np.asarray(tcp_pose[:3], dtype=float).copy()

    def validate_path(
        start_pose: np.ndarray,
        numeric_path: list[np.ndarray],
    ) -> None:
        if not numeric_path:
            raise RuntimeError("final_drip 물 붓기 MoveSX 경로가 비어 있습니다.")

        previous = start_pose
        max_linear_step = 0.0
        max_angular_step = 0.0

        for index, point in enumerate(numeric_path, start=1):
            if point.shape != (6,) or not np.all(np.isfinite(point)):
                raise RuntimeError(
                    f"물 붓기 경유점 {index}가 올바르지 않습니다: {point!r}"
                )
            linear_step = float(np.linalg.norm(point[:3] - previous[:3]))
            angular_step = rotation_distance_deg(
                previous[3:], point[3:]
            )
            max_linear_step = max(max_linear_step, linear_step)
            max_angular_step = max(max_angular_step, angular_step)
            previous = point

        node.get_logger().info(
            "final_drip 물 붓기 경로 검증: "
            f"points={len(numeric_path)}, "
            f"max linear step={max_linear_step:.3f} mm, "
            f"max angular step={max_angular_step:.3f} deg"
        )

        if max_linear_step > FINAL_POUR_MAX_LINEAR_STEP_MM:
            raise RuntimeError(
                "물 붓기 경유점 병진 간격이 "
                f"{max_linear_step:.3f} mm로 제한값 "
                f"{FINAL_POUR_MAX_LINEAR_STEP_MM:.3f} mm를 초과합니다."
            )
        if max_angular_step > FINAL_POUR_MAX_ANGULAR_STEP_DEG:
            raise RuntimeError(
                "물 붓기 경유점 회전 간격이 "
                f"{max_angular_step:.3f} deg로 제한값 "
                f"{FINAL_POUR_MAX_ANGULAR_STEP_DEG:.3f} deg를 초과합니다."
            )

    def call_final_movesx(
        path,
        *,
        angular_vel_deg_s: float = VELX_ROT_DEFAULT,
        angular_acc_deg_s2: float = ACCX_ROT_DEFAULT,
    ) -> Any:
        """지정된 회전 속도/가속도로 final_drip MoveSX를 실행한다."""
        kwargs = {
            "ref": DR_BASE,
            "mod": DR_MV_MOD_ABS,
            "vel_opt": DR_MVS_VEL_NONE,
        }
        common_vel = [VELX_LIN_DEFAULT, float(angular_vel_deg_s)]
        common_acc = [ACCX_LIN_DEFAULT, float(angular_acc_deg_s2)]
        attempts = (
            lambda: movesx(
                path,
                vel=common_vel,
                acc=common_acc,
                **kwargs,
            ),
            lambda: movesx(
                path,
                common_vel,
                common_acc,
                0.0,
                DR_BASE,
                DR_MV_MOD_ABS,
                DR_MVS_VEL_NONE,
            ),
        )

        last_type_error: Optional[TypeError] = None
        for attempt in attempts:
            try:
                return attempt()
            except TypeError as error:
                last_type_error = error
        raise RuntimeError(
            "현재 DSR_ROBOT2 movesx() 속도 인자 시그니처와 맞지 않습니다: "
            f"{last_type_error}"
        )

    def build_fixed_spout_orientation_path(
        start_pose: np.ndarray,
        target_rotation: np.ndarray,
        fixed_spout_base: np.ndarray,
        *,
        label: str,
    ) -> tuple[list[Any], list[np.ndarray]]:
        """mug TCP XYZ를 고정하고 지정 회전행렬까지 한 구간을 생성한다."""
        start_rotation = _zyz_to_rotm(start_pose[3:])
        requested_target_rotation = _project_rotation(target_rotation)
        fixed_spout_base = np.asarray(
            fixed_spout_base,
            dtype=float,
        ).reshape(3)
        relative_rotvec = _rotm_to_rotvec(
            start_rotation.T @ requested_target_rotation
        )
        rotation_change_deg = math.degrees(
            float(np.linalg.norm(relative_rotvec))
        )
        node.get_logger().info(
            f"{label} 경로 생성: 회전 변화={rotation_change_deg:.3f} deg, "
            "mug TCP Base XYZ 고정"
        )

        numeric_path: list[np.ndarray] = []
        path = []
        previous_abc = start_pose[3:].copy()

        for index in range(1, FINAL_POUR_PATH_POINTS + 1):
            progress_value = _smoothstep5(
                index / float(FINAL_POUR_PATH_POINTS)
            )
            interpolated_rotation = start_rotation @ _rotvec_to_rotm(
                relative_rotvec * progress_value
            )
            target_abc = _rotm_to_zyz_near(
                interpolated_rotation,
                previous_abc,
            )
            previous_abc = target_abc

            # mug TCP가 실제 주둥이 끝이므로 회전 중 XYZ는 시작값으로 고정한다.
            target_tcp_xyz = fixed_spout_base.copy()
            target = np.array(
                [
                    target_tcp_xyz[0],
                    target_tcp_xyz[1],
                    target_tcp_xyz[2],
                    target_abc[0],
                    target_abc[1],
                    target_abc[2],
                ],
                dtype=float,
            )

            predicted_spout = target_tcp_xyz
            residual = predicted_spout - fixed_spout_base
            if float(np.linalg.norm(residual)) > 1.0e-6:
                raise RuntimeError(
                    f"{label} 경유점 {index} 보상 계산 오류: "
                    f"residual={residual.tolist()}"
                )

            numeric_path.append(target)
            path.append(
                posx(*[float(value) for value in target])
            )

        validate_path(start_pose, numeric_path)
        planned_target_error_deg = rotation_distance_deg(
            numeric_path[-1][3:],
            _rotm_to_zyz_near(
                requested_target_rotation,
                numeric_path[-1][3:],
            ),
        )
        if planned_target_error_deg > 0.01:
            raise RuntimeError(
                f"{label} 최종 회전 목표 오차가 큽니다: "
                f"{planned_target_error_deg:.6f} deg"
            )
        return path, numeric_path

    def build_fixed_spout_path(
        start_pose: np.ndarray,
        start_joint: np.ndarray,
        j6_delta_deg: float,
    ) -> tuple[list[Any], list[np.ndarray], np.ndarray]:
        # mug TCP 원점 자체가 주둥이 끝이므로 별도 Tool 오프셋 없이
        # 시작 TCP Base XYZ를 회전 중심으로 그대로 사용한다.
        fixed_spout_base = start_pose[:3].copy()

        virtual_final_joint = start_joint.copy()
        virtual_final_joint[5] += float(j6_delta_deg)
        virtual_final_pose = forward_kinematics_base(virtual_final_joint)
        final_rotation = _zyz_to_rotm(virtual_final_pose[3:])

        virtual_uncompensated_spout = virtual_final_pose[:3].copy()
        uncompensated_drift = (
            virtual_uncompensated_spout - fixed_spout_base
        )
        node.get_logger().info(
            "가상 J6 단독 회전 시 mug TCP 원점 변위 [mm]: "
            f"dX={uncompensated_drift[0]:+.3f}, "
            f"dY={uncompensated_drift[1]:+.3f}, "
            f"dZ={uncompensated_drift[2]:+.3f}"
        )

        path, numeric_path = build_fixed_spout_orientation_path(
            start_pose,
            final_rotation,
            fixed_spout_base,
            label=f"J6 등가 {j6_delta_deg:+.1f} deg 기울임",
        )
        return path, numeric_path, fixed_spout_base

    # -------------------------------------------------------------
    # 1. DRL final_drip: 필터 홀더 이동
    # -------------------------------------------------------------
    publish_final(
        phase="FINAL_DRIP_FILTER_HOLDER",
        overall_progress=90,
        final_progress=5,
        stage="필터 홀더 이동",
        title="필터 홀더를 옮기는 중",
        message="필터 홀더를 집어 최종 드립 위치에 배치합니다.",
    )
    handle_grip_open()
    movel(
        posx(0.0, 0.0, -150.0, 0.0, 0.0, 0.0),
        radius=0.0,
        ref=DR_TOOL,
        mod=DR_MV_MOD_REL,
        ra=DR_MV_RA_DUPLICATE,
    )
    movej(System_fitter_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
    movel(
        System_filtter_l,
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_ABS,
        ra=DR_MV_RA_DUPLICATE,
    )
    close_and_verify_grip("final_drip", "필터 홀더 잡기")
    movel(
        posx(0.0, 0.0, 100.0, 0.0, 0.0, 0.0),
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_REL,
        ra=DR_MV_RA_DUPLICATE,
    )
    movel(
        System_filtter_l2,
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_ABS,
        ra=DR_MV_RA_DUPLICATE,
    )
    spoon_cup_grip_open()
    movel(
        posx(0.0, 0.0, -90.0, 0.0, 0.0, 0.0),
        radius=0.0,
        ref=DR_TOOL,
        mod=DR_MV_MOD_REL,
        ra=DR_MV_RA_DUPLICATE,
    )
    movel(
        posx(200.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_REL,
        ra=DR_MV_RA_DUPLICATE,
    )

    # -------------------------------------------------------------
    # 2. DRL final_drip: 물컵 파지 및 붓기 시작 자세 이동
    # -------------------------------------------------------------
    publish_final(
        phase="FINAL_DRIP_MUG_PICKUP",
        overall_progress=92,
        final_progress=30,
        stage="물컵 파지",
        title="물컵을 집는 중",
        message="물컵을 파지한 뒤 드리퍼 위 최종 물 붓기 자세로 이동합니다.",
    )
    movej(System_mug_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
    movel(
        System_mug_l,
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_ABS,
        ra=DR_MV_RA_DUPLICATE,
    )
    close_and_verify_grip("final_drip", "물컵 잡기")
    movel(
        posx(0.0, 0.0, 200.0, 0.0, 0.0, 0.0),
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_REL,
        ra=DR_MV_RA_DUPLICATE,
    )
    movel(
        posx(-100.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_REL,
        ra=DR_MV_RA_DUPLICATE,
    )

    publish_final(
        phase="FINAL_DRIP_POSITIONING",
        overall_progress=94,
        final_progress=55,
        stage="드립 위치 이동",
        title="물 붓기 시작 자세로 이동하는 중",
        message=(
            "교시된 최종 드립 위치 도달 후 Joint6을 상대 "
            f"{FINAL_PRE_POUR_J6_REL_DEG:+.0f}° 회전하고 "
            "mug TCP 고정 물 붓기를 실행합니다."
        ),
    )
    movej(System_final_approach_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
    movel(
        System_final_l,
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_ABS,
        ra=DR_MV_RA_DUPLICATE,
    )
    movej(
        posj(0.0, 0.0, 0.0, 0.0, 0.0, FINAL_PRE_POUR_J6_REL_DEG),
        radius=0.0,
        mod=DR_MV_MOD_REL,
        ra=DR_MV_RA_DUPLICATE,
    )
    node.get_logger().info(
        f"final_drip 사전 Joint6 상대 회전: {FINAL_PRE_POUR_J6_REL_DEG:+.1f} deg"
    )

    # -------------------------------------------------------------
    # 3. DRL의 tp_log("물 붓기") 위치: plate_outline 통합
    # -------------------------------------------------------------
    original_tcp_name = get_tcp()
    if not isinstance(original_tcp_name, str) or not original_tcp_name.strip():
        original_tcp_name = TCP_NAME
    original_tcp_name = original_tcp_name.strip()

    publish_final(
        phase="FINAL_DRIP_POUR_READY",
        overall_progress=96,
        final_progress=70,
        stage="물 붓기 준비",
        title="mug TCP 회전 경로를 준비하는 중",
        message=(
            f"TCP '{FINAL_POUR_TCP_NAME}'를 적용하고 mug TCP 원점을 고정한 "
            f"Joint6 등가 {FINAL_POUR_J6_DELTA_DEG:+.0f}° 경로를 계산합니다."
        ),
    )
    if not apply_tcp(FINAL_POUR_TCP_NAME):
        raise RuntimeError(
            f"물 붓기 TCP '{FINAL_POUR_TCP_NAME}' 적용 실패"
        )

    start_pose = current_pose_base()
    start_joint = current_joint()
    path, numeric_path, fixed_spout_base = build_fixed_spout_path(
        start_pose,
        start_joint,
        FINAL_POUR_J6_DELTA_DEG,
    )

    node.get_logger().warning(
        "final_drip 물 붓기 시작: 고정 mug TCP Base XYZ="
        f"[{fixed_spout_base[0]:.3f}, "
        f"{fixed_spout_base[1]:.3f}, "
        f"{fixed_spout_base[2]:.3f}] mm"
    )
    publish_final(
        phase="FINAL_DRIP_POURING",
        overall_progress=97,
        final_progress=78,
        stage="물 붓기",
        title="mug TCP 원점을 유지하며 물을 붓는 중",
        message=(
            f"공통 Cartesian 속도 [{VELX_LIN_DEFAULT:.0f} mm/s, "
            f"{FINAL_POUR_ANGULAR_VEL_DEG_S:.0f} deg/s]로 "
            "mug TCP Base XYZ를 고정하고 "
            f"Joint6 등가 {FINAL_POUR_J6_DELTA_DEG:+.0f}°로 기울입니다."
        ),
    )

    start_time = time.monotonic()
    result = call_final_movesx(
        path,
        angular_vel_deg_s=FINAL_POUR_ANGULAR_VEL_DEG_S,
        angular_acc_deg_s2=FINAL_POUR_ANGULAR_ACC_DEG_S2,
    )
    elapsed = time.monotonic() - start_time
    if isinstance(result, (int, float)) and result < 0:
        raise RuntimeError(
            f"final_drip 물 붓기 movesx 실패 반환값={result}"
        )

    # 동기 MoveSX 완료 후 0.5초 정지하여 실제 종료 상태를 안정화한다.
    # 이후 읽은 TCP/관절값을 복귀 경로의 새 시작 상태로 그대로 사용한다.
    wait(0.50)
    end_pose = current_pose_base()
    end_joint = current_joint()
    measured_spout_base = mug_tcp_point_base(end_pose)
    spout_error = measured_spout_base - fixed_spout_base
    final_pose_error = numeric_path[-1][:3] - end_pose[:3]
    joint_delta = end_joint - start_joint

    node.get_logger().info(
        "final_drip 물 붓기 완료: "
        f"ret={result}, elapsed={elapsed:.3f}s"
    )
    node.get_logger().info(
        "mug TCP 원점 고정 오차 [mm]: "
        f"X={spout_error[0]:+.3f}, "
        f"Y={spout_error[1]:+.3f}, "
        f"Z={spout_error[2]:+.3f}, "
        f"norm={float(np.linalg.norm(spout_error)):.3f}"
    )
    node.get_logger().info(
        "실제 관절 변화 [deg]: ["
        + ", ".join(f"{value:+.3f}" for value in joint_delta)
        + "]"
    )
    node.get_logger().info(
        "최종 mug TCP 목표 오차 [mm]: "
        f"X={final_pose_error[0]:+.3f}, "
        f"Y={final_pose_error[1]:+.3f}, "
        f"Z={final_pose_error[2]:+.3f}"
    )

    if float(np.linalg.norm(spout_error)) > 3.0:
        node.get_logger().warning(
            "mug TCP 원점 고정 오차가 3 mm를 초과했습니다. "
            "티치펜던트의 mug TCP 교시값을 확인하십시오."
        )

    # -------------------------------------------------------------
    # 4. 저장해 둔 물 붓기 시작 회전행렬로 직접 복귀
    # -------------------------------------------------------------
    publish_final(
        phase="FINAL_DRIP_RETURNING",
        overall_progress=98,
        final_progress=90,
        stage="시작 자세 복귀",
        title="물 붓기 전 자세로 돌아가는 중",
        message=(
            "현재 관절에 반대 J6 값을 다시 적용하지 않고, 기울이기 직전에 "
            "저장한 mug TCP 자세로 직접 복귀합니다. "
            f"복귀 회전 속도는 {FINAL_RETURN_ANGULAR_VEL_DEG_S:.0f} deg/s입니다."
        ),
    )

    # 현재 실제 자세에서 저장한 최초 회전행렬을 직접 목표로 삼는다.
    # 따라서 첫 구간의 실제 IK가 J6 회전을 다른 관절에 분배했더라도 복귀 목표가
    # 다시 +방향으로 계산되지 않으며, 계획 자체가 시작 자세로 끝난다.
    return_fixed_spout_base = fixed_spout_base.copy()
    return_target_rotation = _zyz_to_rotm(start_pose[3:])
    return_path, return_numeric_path = build_fixed_spout_orientation_path(
        end_pose,
        return_target_rotation,
        return_fixed_spout_base,
        label="저장한 물 붓기 시작 자세 복귀",
    )

    return_start_time = time.monotonic()
    return_result = call_final_movesx(
        return_path,
        angular_vel_deg_s=FINAL_RETURN_ANGULAR_VEL_DEG_S,
        angular_acc_deg_s2=FINAL_RETURN_ANGULAR_ACC_DEG_S2,
    )
    return_elapsed = time.monotonic() - return_start_time
    node.get_logger().info(
        "final_drip 복귀 구간 고정 mug TCP Base XYZ="
        f"[{return_fixed_spout_base[0]:.3f}, "
        f"{return_fixed_spout_base[1]:.3f}, "
        f"{return_fixed_spout_base[2]:.3f}] mm"
    )
    if isinstance(return_result, (int, float)) and return_result < 0:
        raise RuntimeError(
            f"final_drip 시작 자세 복귀 movesx 실패 반환값={return_result}"
        )

    returned_pose = current_pose_base()
    returned_joint = current_joint()
    return_xyz_error = returned_pose[:3] - start_pose[:3]
    return_orientation_error = rotation_distance_deg(
        start_pose[3:],
        returned_pose[3:],
    )
    return_joint_error = returned_joint - start_joint

    node.get_logger().info(
        "final_drip 시작 자세 복귀 완료: "
        f"ret={return_result}, elapsed={return_elapsed:.3f}s, "
        f"XYZ error={float(np.linalg.norm(return_xyz_error)):.3f} mm, "
        f"orientation error={return_orientation_error:.3f} deg"
    )
    node.get_logger().info(
        "복귀 후 관절 오차 [deg]: ["
        + ", ".join(f"{value:+.3f}" for value in return_joint_error)
        + "]"
    )

    # -------------------------------------------------------------
    # 5. final_drip 진입 전 TCP로 복원
    # -------------------------------------------------------------
    publish_final(
        phase="FINAL_DRIP_TCP_RESTORE",
        overall_progress=99,
        final_progress=97,
        stage="TCP 복원",
        title="기본 TCP로 복원하는 중",
        message=f"활성 TCP를 '{original_tcp_name}'로 되돌립니다.",
    )
    if not apply_tcp(original_tcp_name):
        raise RuntimeError(
            f"원래 TCP '{original_tcp_name}' 복원 실패"
        )

    # -------------------------------------------------------------
    # 6. Base -X 200 mm 이격 후 원위치 위 5 mm에서 그리퍼 열기
    # -------------------------------------------------------------
    publish_final(
        phase="FINAL_DRIP_MUG_RETURN",
        overall_progress=99,
        final_progress=99,
        stage="물컵 원위치 복귀",
        title="물컵을 원위치로 옮기는 중",
        message=(
            f"Base X {FINAL_MUG_RETURN_BASE_X_OFFSET_MM:+.1f} mm 상대 이동 후 "
            "System_mug_l의 Base +Z "
            f"{FINAL_MUG_RETURN_Z_OFFSET_MM:.1f} mm 위치에서 그리퍼를 엽니다."
        ),
    )
    movel(
        posx(
            FINAL_MUG_RETURN_BASE_X_OFFSET_MM,
            80.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ),
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_REL,
        ra=DR_MV_RA_DUPLICATE,
    )
    movel(
        System_mug_release_l,
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_ABS,
        ra=DR_MV_RA_DUPLICATE,
    )
    spoon_cup_grip_open()
    movel(
        posx(
            0.0,
            0.0,
            FINAL_MUG_RELEASE_TOOL_Z_RETREAT_MM,
            0.0,
            0.0,
            0.0,
        ),
        radius=0.0,
        ref=DR_TOOL,
        mod=DR_MV_MOD_REL,
        ra=DR_MV_RA_DUPLICATE,
    )
    movel(
        posx(
            0.0,
            0.0,
            FINAL_MUG_RELEASE_BASE_Z_LIFT_MM,
            0.0,
            0.0,
            0.0,
        ),
        radius=0.0,
        ref=DR_BASE,
        mod=DR_MV_MOD_REL,
        ra=DR_MV_RA_DUPLICATE,
    )
    movej(System_home, radius=0.0, ra=DR_MV_RA_DUPLICATE)
    node.get_logger().info(
        "final_drip 물컵 반환 완료: "
        f"Base X {FINAL_MUG_RETURN_BASE_X_OFFSET_MM:+.1f} mm 상대 이동, "
        f"System_mug_l Base +Z {FINAL_MUG_RETURN_Z_OFFSET_MM:.1f} mm, "
        f"그리퍼 열림, Tool Z {FINAL_MUG_RELEASE_TOOL_Z_RETREAT_MM:+.1f} mm, "
        f"Base Z {FINAL_MUG_RELEASE_BASE_Z_LIFT_MM:+.1f} mm, 홈 복귀"
    )

    publish_final(
        phase="FINAL_DRIP_DONE",
        overall_progress=99,
        final_progress=100,
        stage="완료",
        title="최종 물 붓기 완료",
        message=(
            "물 붓기 전 자세로 복귀하고 활성 TCP를 "
            f"'{original_tcp_name}'로 복원한 뒤 물컵을 반환하고 홈 자세로 복귀했습니다."
        ),
    )
    node.get_logger().info(
        f"final_drip Sub 완료: TCP={original_tcp_name}"
    )
