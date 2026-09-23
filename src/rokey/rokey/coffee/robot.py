"""로봇 API 바인딩, 교시 포즈, 그리퍼/TCP/외력 헬퍼를 묶은 실행 컨텍스트."""

from __future__ import annotations

import json
import math
import sys
import time
from typing import Any, Callable, Optional

import rclpy

import DR_init
from dsr_msgs2.srv import MoveJoint

from .config import (
    ROBOT_ID, NODE_NAME, TOOL_NAME, TCP_NAME, VELJ_DEFAULT, ACCJ_DEFAULT,
    VELX_LIN_DEFAULT, VELX_ROT_DEFAULT, ACCX_LIN_DEFAULT, ACCX_ROT_DEFAULT,
    SYSTEM_POT_GRIP_VALUES, SYSTEM_POT_GRIP_JOINT_VALUES,
    POUR_START_JOINT_VALUES, PICKUP_APPROACH_REL_VALUES,
    PICKUP_LIFT_REL_VALUES, TEST_STAGE_NAMES, EXTERNAL_FORCE_TRIGGER_N,
    EXTERNAL_FORCE_SETTLE_SEC, EXTERNAL_FORCE_BASELINE_SAMPLES,
    EXTERNAL_FORCE_SAMPLE_SEC, EXTERNAL_FORCE_CONFIRM_SAMPLES,
    EXTERNAL_FORCE_UI_UPDATE_SEC, EXTERNAL_FORCE_TIMEOUT_SEC,
)
from .buttons import PhysicalButtonInput
from .control_bridge import CoffeeControlBridge
from .errors import GripFailureError
from .grip_monitor import GripMonitor
from .status import StatusReporter


class RobotApi:
    """DSR_ROBOT2 함수와 상수를 DRL 원본과 동일한 이름으로 보관한다."""

    def __init__(self, dsr2: Any) -> None:
        self.movej = dsr2.movej
        self.movel = dsr2.movel
        self.movec = dsr2.movec
        self.movesx = dsr2.movesx
        self.amovec = getattr(dsr2, "amovec", None)
        self.amovel = dsr2.amovel
        self.move_periodic = dsr2.move_periodic
        self.task_compliance_ctrl = dsr2.task_compliance_ctrl
        self.release_compliance_ctrl = dsr2.release_compliance_ctrl
        self.set_stiffnessx = dsr2.set_stiffnessx
        self.set_digital_output = dsr2.set_digital_output
        self.get_digital_input = dsr2.get_digital_input
        self.get_tool_force = dsr2.get_tool_force
        self.get_current_posx = dsr2.get_current_posx
        self.get_current_posj = dsr2.get_current_posj
        self.fkin = dsr2.fkin
        self.check_motion = getattr(dsr2, "check_motion", None)
        self.mwait = getattr(dsr2, "mwait", None)
        self.set_tool = dsr2.set_tool
        self.set_tcp = dsr2.set_tcp
        self.get_tool = dsr2.get_tool
        self.get_tcp = dsr2.get_tcp
        self.drl_script_run = dsr2.drl_script_run
        self.get_drl_state = dsr2.get_drl_state
        self.get_robot_system = dsr2.get_robot_system
        self.set_singularity_handling = dsr2.set_singularity_handling
        self.set_velj = dsr2.set_velj
        self.set_accj = dsr2.set_accj
        self.set_velx = dsr2.set_velx
        self.set_accx = dsr2.set_accx
        self.wait = dsr2.wait
        self.posj = dsr2.posj
        self.posx = dsr2.posx

        self.DR_BASE = dsr2.DR_BASE
        self.DR_TOOL = dsr2.DR_TOOL
        self.DR_MV_MOD_ABS = dsr2.DR_MV_MOD_ABS
        self.DR_MV_MOD_REL = dsr2.DR_MV_MOD_REL
        self.DR_MV_RA_DUPLICATE = dsr2.DR_MV_RA_DUPLICATE
        self.DR_MV_RA_OVERRIDE = dsr2.DR_MV_RA_OVERRIDE
        self.DR_MVS_VEL_NONE = getattr(dsr2, "DR_MVS_VEL_NONE", 0)
        self.DR_AVOID = dsr2.DR_AVOID
        # DR_OFF = dsr2.DR_OFF  # 원본에 있었으나 DSR_ROBOT2에 DR_OFF 상수 자체가
        # 없어 AttributeError로 즉시 죽었음. set_velx() 3번째 인자로만 쓰였는데 그
        # 인자 자체가 실제 시그니처에 없어서 통째로 제거함 (set_velx() 호출부 참고).
        self.ON = getattr(dsr2, "ON", 1)
        self.OFF = getattr(dsr2, "OFF", 0)

    def guard_motion(self, grip_monitor: GripMonitor) -> None:
        """신호 단절 전후에 새 모션 명령이 이어지지 않도록 모션 API를 감싼다."""

        def guarded(function: Callable[..., Any]) -> Callable[..., Any]:
            def call(*motion_args: Any, **motion_kwargs: Any) -> Any:
                grip_monitor.raise_if_signal_lost()
                try:
                    result = function(*motion_args, **motion_kwargs)
                except BaseException:
                    # MoveStop 때문에 원래 모션 서비스가 먼저 예외를 반환하더라도
                    # 장비 오류 복구 경로가 일반 로봇 오류 화면으로 빠지지 않게 한다.
                    grip_monitor.raise_if_signal_lost()
                    raise
                grip_monitor.raise_if_signal_lost()
                return result

            return call

        self.movej = guarded(self.movej)
        self.movel = guarded(self.movel)
        self.movec = guarded(self.movec)
        self.movesx = guarded(self.movesx)
        self.amovel = guarded(self.amovel)
        self.move_periodic = guarded(self.move_periodic)
        if self.amovec is not None:
            self.amovec = guarded(self.amovec)
        if callable(self.mwait):
            self.mwait = guarded(self.mwait)


class TeachPoses:
    """Teach 포즈 (System.drvar / System(5).drvar 원본 값)."""

    def __init__(self, api: RobotApi) -> None:
        posj = api.posj
        posx = api.posx

        self.System_spoon_j = posj(-69.6, 53.27, 96.56, 25.63, 114.79, -171.03)
        self.System_spoon_l = posx(251.01, -118.46, 40.17, 87.01, 92.73, -3.35)
        self.System_grinder_j = posj(-40.58, -9.25, 131.96, 78.44, 107.42, -125.46)
        self.System_grinder_l = posx(371.82, 168.64, 360.0, 61.05, 86.52, 56.67)
        self.System_handle_j = posj(15.05, 35.75, 18.15, -1.7, 124.03, 13.38)
        self.System_handle_l = posx(532.780, 127.380, 279.83, 148.81, 178.39, 148.08)
        self.System_bottle_j_1 = posj(-19.18, 34.8, 22.24, -2.53, 124.42, -13.34)
        self.System_bottle_j_2 = posj(-43.18, 55.32, 88.81, 53.44, 114.38, -60.57)
        self.System_bottle_l = posx(401.53, 160.78, 71.68, 92.22, 89.54, 89.25)
        self.System_bottle_l2 = posx(401.53, 160.78, 76.68, 92.22, 89.54, 89.25)
        self.System_drip_j = posj(-26.64, 44.1, 73.65, 89.85, 91.27, -26.21)
        self.System_drip_l = posx(751.71, 41.51, 238.65, 60.13, 91.27, -92.13)
        self.System_home = posj(-71.33, 47.91, 97.52, 18.17, 114.49, -168.37)
        self.System_spoon_l_2 = posx(251.01, -118.46, 50.17, 87.01, 92.73, -3.35)

        # 스파이럴 드립 Sub 교시 포즈
        self.System_pot_grip = posx(*SYSTEM_POT_GRIP_VALUES)
        self.System_pot_grip_joint = posj(*SYSTEM_POT_GRIP_JOINT_VALUES)
        self.Pour_start_joint = posj(*POUR_START_JOINT_VALUES)
        self.Pickup_approach_rel = posx(*PICKUP_APPROACH_REL_VALUES)
        self.Pickup_lift_rel = posx(*PICKUP_LIFT_REL_VALUES)

        # final_drip Sub 교시 포즈 (System(5).drvar)
        self.System_fitter_j = posj(-30.99, 52.68, 70.94, 74.39, 112.41, -306.08)
        self.System_filtter_l = posx(653.6, 17.98, 181.99, 85.67, 90.12, -179.93)
        self.System_filtter_l2 = posx(349.44, 4.5, 96.59, 90.83, 90.78, 174.75)
        self.System_mug_j = posj(-32.73, 54.93, 83.19, 82.8, 93.84, -10.72)
        self.System_mug_l = posx(655.06, 48.93, 82.17, 70.19, 92.83, 123.59)
        self.System_mug_release_l = posx(648.06, 48.93, 87.17, 70.19, 92.83, 123.59)
        self.System_final_l = posx(699.92, -340.33, 332.35, 26.41, 87.7, 167.11)
        # System_final_l = posx(699.92, -400.33, 332.35, 26.41, 87.7, 167.11)
        # 원본 교시값은 전체 코드/좌표 이력 보존을 위해 남기되 이 변형에서는 실행하지 않는다.
        self.System_final_j2 = posj(-53.67, 53.18, 48.53, 96.63, 74.0, 146.0)
        self.System_final_approach_j = posj(
            -50.40, 31.33, 84.53, 98.98, 78.15, 40.42
        )


class RobotContext:
    """모션 API, 교시 포즈, 상태 발행기, 감시자를 묶은 공정 실행 컨텍스트."""

    def __init__(
        self,
        node: Any,
        api: RobotApi,
        status: StatusReporter,
        grip: GripMonitor,
        buttons: PhysicalButtonInput,
        control: CoffeeControlBridge,
    ) -> None:
        self.node = node
        self.api = api
        self.poses = TeachPoses(api)
        self.status = status
        self.grip = grip
        self.buttons = buttons
        self.control = control

    # ==================================================================
    # OnRobot RG2 그리퍼 제어 (DO 1, 2 접점 조합)
    # ==================================================================
    def grip_close(self) -> float:
        """RG2 닫기 명령 시각을 반환하고 기구 동작 안정화를 기다린다."""
        set_digital_output = self.api.set_digital_output
        wait = self.api.wait
        ON = self.api.ON
        OFF = self.api.OFF
        grip_monitor = self.grip

        grip_monitor.raise_if_signal_lost()
        wait(0.50)
        grip_monitor.raise_if_signal_lost()
        command_stamp = time.monotonic()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(1.00)
        grip_monitor.raise_if_signal_lost()
        return command_stamp

    def jar_grip_open(self) -> None:
        set_digital_output = self.api.set_digital_output
        wait = self.api.wait
        ON = self.api.ON
        OFF = self.api.OFF
        grip_monitor = self.grip

        grip_monitor.raise_if_signal_lost()
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.50)
        grip_monitor.raise_if_signal_lost()

    def handle_grip_open(self) -> None:
        set_digital_output = self.api.set_digital_output
        wait = self.api.wait
        ON = self.api.ON
        OFF = self.api.OFF
        grip_monitor = self.grip

        grip_monitor.raise_if_signal_lost()
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.50)
        grip_monitor.raise_if_signal_lost()

    def spoon_cup_grip_open(self) -> None:
        set_digital_output = self.api.set_digital_output
        wait = self.api.wait
        OFF = self.api.OFF
        grip_monitor = self.grip

        grip_monitor.raise_if_signal_lost()
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        wait(0.50)
        grip_monitor.raise_if_signal_lost()

    def close_and_verify_grip(
        self,
        stage_id: str,
        grip_task: str,
    ) -> None:
        """닫기 후 README의 우선순위로 실제 물체 파지를 확인한다."""
        node = self.node
        grip_close = self.grip_close
        grip_monitor = self.grip

        command_stamp = grip_close()
        success, diagnostics = grip_monitor.verify_grip(command_stamp)
        stage_name = TEST_STAGE_NAMES.get(stage_id, stage_id)
        diagnostics["stage_id"] = stage_id
        diagnostics["grip_task"] = grip_task
        if not success:
            node.get_logger().error(
                f"[그립 실패] 단계={stage_name}, 작업={grip_task}, "
                f"진단={json.dumps(diagnostics, ensure_ascii=False)}"
            )
            raise GripFailureError(
                stage_id,
                stage_name,
                grip_task,
                diagnostics,
            )
        node.get_logger().info(
            f"[그립 확인] 단계={stage_name}, 작업={grip_task}, "
            f"판정={diagnostics.get('decision_source', 'unknown')}"
        )

    def apply_tcp(
        self,
        name: str,
        timeout_sec: float = 6.0,
        poll_interval: float = 0.1,
    ) -> bool:
        """set_tcp()를 DRL 프로그램으로 실행해서 TCP를 바꾼다.

        set_tcp()를 ROS2 서비스로 직접 부르면 컨트롤러가 매번
        "this command can only be used in manual mode"로 거부한다 (실측: 6초
        재시도 내내 100% 거부, 반영된 적 0회). 티치펜던트 Task Writer 프로그램은
        같은 명령이 통과되는 걸로 봐서, 로봇 모드(AUTONOMOUS/MANUAL) 자체보다는
        "실행 중인 DRL 프로그램 컨텍스트에서 나온 호출이냐"가 실제 조건으로
        보인다. drl_script_run()으로 set_tcp(...) 한 줄짜리 코드를 컨트롤러에
        네이티브 프로그램처럼 로드해서 실행시켜 이 조건을 맞춘다.
        """
        get_tcp = self.api.get_tcp
        drl_script_run = self.api.drl_script_run
        get_drl_state = self.api.get_drl_state
        get_robot_system = self.api.get_robot_system
        node = self.node

        code = f'set_tcp("{name}")'
        start_ret = drl_script_run(get_robot_system(), code)
        if start_ret != 0:
            node.get_logger().error(f"drl_script_run 시작 실패: {code!r}, ret={start_ret}")
            return False

        deadline = time.monotonic() + timeout_sec
        # get_drl_state(): 0=PLAY, 1=STOP, 2=HOLD, 3=LAST. PLAY를 벗어날 때까지 대기.
        while get_drl_state() == 0:
            if time.monotonic() >= deadline:
                node.get_logger().error(
                    f"drl_script_run({code!r}) {timeout_sec:.0f}초 내에 끝나지 않음"
                )
                return False
            time.sleep(poll_interval)

        active = get_tcp()
        if active != name:
            node.get_logger().error(
                f"drl_script_run으로도 set_tcp('{name}') 반영 안 됨: 실제 활성={active}"
            )
            return False
        return True

    def read_external_force_xyz(self) -> tuple[float, float, float]:
        """현재 TCP 외력의 병진 성분 Fx, Fy, Fz를 Tool 좌표계로 읽는다."""
        get_tool_force = self.api.get_tool_force
        DR_TOOL = self.api.DR_TOOL

        value = get_tool_force(DR_TOOL)

        if not isinstance(value, (list, tuple)) or len(value) < 3:
            raise RuntimeError(
                f"get_tool_force() 반환값이 올바르지 않습니다: {value!r}"
            )

        force_xyz = tuple(float(value[index]) for index in range(3))
        if not all(math.isfinite(component) for component in force_xyz):
            raise RuntimeError(
                f"get_tool_force()에 유효하지 않은 값이 포함되어 있습니다: "
                f"{force_xyz!r}"
            )

        return force_xyz

    def wait_for_external_force(
        self,
        threshold_n: float = EXTERNAL_FORCE_TRIGGER_N,
        timeout_sec: float = EXTERNAL_FORCE_TIMEOUT_SEC,
    ) -> Optional[float]:
        """외력을 기다리되 제한시간이 지나면 ``None``으로 다음 동작을 허용한다.

        move_periodic 직후의 잔류 진동과 정적 하중을 오인하지 않도록 잠시 안정화한
        뒤 기준 힘을 평균 측정한다. 이후 3축 병진 힘 변화량의 벡터 크기가 연속
        EXTERNAL_FORCE_CONFIRM_SAMPLES회 임계값 이상이면 병을 친 것으로 판정한다.
        """
        node = self.node
        status = self.status
        read_external_force_xyz = self.read_external_force_xyz
        grip_monitor = self.grip

        node.get_logger().info(
            f"외력 감지 준비: {EXTERNAL_FORCE_SETTLE_SEC:.2f}초 안정화"
        )
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        time.sleep(EXTERNAL_FORCE_SETTLE_SEC)
        grip_monitor.raise_if_signal_lost()

        baseline_samples: list[tuple[float, float, float]] = []
        for _ in range(EXTERNAL_FORCE_BASELINE_SAMPLES):
            grip_monitor.raise_if_signal_lost()
            baseline_samples.append(read_external_force_xyz())
            time.sleep(EXTERNAL_FORCE_SAMPLE_SEC)

        baseline = tuple(
            sum(sample[axis] for sample in baseline_samples)
            / len(baseline_samples)
            for axis in range(3)
        )

        node.get_logger().info(
            "외력 기준값 설정 완료: "
            f"Fx={baseline[0]:.3f}, Fy={baseline[1]:.3f}, "
            f"Fz={baseline[2]:.3f} N, threshold={threshold_n:.1f} N"
        )

        consecutive = 0
        peak_force = 0.0
        next_ui_update = 0.0

        while rclpy.ok():
            grip_monitor.raise_if_signal_lost()
            current = read_external_force_xyz()
            delta = tuple(
                current[axis] - baseline[axis]
                for axis in range(3)
            )
            delta_norm = math.sqrt(sum(component * component for component in delta))
            peak_force = max(peak_force, delta_norm)

            now = time.monotonic()
            remaining_sec = max(0.0, deadline - now)
            if now >= next_ui_update:
                status.publish(
                    phase="WAIT_EXTERNAL_FORCE",
                    screen=5,
                    progress=70,
                    title="병 바닥을 가볍게 쳐주세요",
                    message=(
                        f"현재 외력 변화량은 {delta_norm:.1f} N입니다. "
                        f"{threshold_n:.1f} N 이상 감지되면 진행하며, "
                        f"미감지 시 {remaining_sec:.1f}초 후 자동 진행합니다."
                    ),
                    busy=True,
                    waiting_external_force=True,
                    force_delta_n=delta_norm,
                    force_peak_n=peak_force,
                    wait_remaining_sec=remaining_sec,
                )
                next_ui_update = now + EXTERNAL_FORCE_UI_UPDATE_SEC

            if delta_norm >= threshold_n:
                consecutive += 1
                if consecutive >= EXTERNAL_FORCE_CONFIRM_SAMPLES:
                    node.get_logger().info(
                        f"외력 감지 완료: {delta_norm:.3f} N "
                        f"(peak={peak_force:.3f} N)"
                    )
                    return delta_norm
            else:
                consecutive = 0

            if now >= deadline:
                node.get_logger().warning(
                    f"외력 입력 {timeout_sec:.1f}초 초과: "
                    f"peak={peak_force:.3f} N, 다음 동작을 계속합니다."
                )
                return None

            time.sleep(EXTERNAL_FORCE_SAMPLE_SEC)

        raise KeyboardInterrupt

    def ensure_tool_tcp(self) -> None:
        """각 공정/테스트 시작 전에 Tool과 기본 TCP를 확인한다."""
        set_tool = self.api.set_tool
        set_tcp = self.api.set_tcp
        get_tool = self.api.get_tool
        get_tcp = self.api.get_tcp
        node = self.node

        tool_ret = set_tool(TOOL_NAME)
        tcp_ret = set_tcp(TCP_NAME)
        active_tool = get_tool()
        active_tcp = get_tcp()
        node.get_logger().info(
            f"set_tool ret={tool_ret}, 요청={TOOL_NAME}, 실제 활성={active_tool}"
        )
        node.get_logger().info(
            f"set_tcp ret={tcp_ret}, 요청={TCP_NAME}, 실제 활성={active_tcp}"
        )
        if active_tool != TOOL_NAME or active_tcp != TCP_NAME:
            raise RuntimeError(
                "Tool/TCP가 요청한 이름으로 활성화되지 않았습니다. "
                "티치펜던트에 해당 이름이 등록되어 있는지 확인하세요."
            )

    def apply_default_motion_profile(self) -> None:
        """모든 Sub에 공통으로 쓰는 전역 속도/가속도를 적용한다."""
        self.api.set_velj(VELJ_DEFAULT)
        self.api.set_accj(ACCJ_DEFAULT)
        self.api.set_velx(VELX_LIN_DEFAULT, VELX_ROT_DEFAULT)
        self.api.set_accx(ACCX_LIN_DEFAULT, ACCX_ROT_DEFAULT)


def create_node() -> Any:
    """로봇 노드를 만들고 DR_init에 등록한다."""
    node = rclpy.create_node(NODE_NAME, namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    return node


def import_dsr2(node: Any) -> Any:
    """DSR_ROBOT2를 불러오고 컨트롤러 서비스 discovery가 끝날 때까지 기다린다."""
    try:
        import DSR_ROBOT2 as dsr2
    except ImportError as error:
        node.get_logger().error(f"DSR_ROBOT2 모듈을 불러오지 못했습니다: {error}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # DSR_ROBOT2 import 시점에 생성되는 ~100개 서비스 클라이언트가 컨트롤러와 DDS
    # discovery를 마칠 때까지 기다린다. set_singular_handling() 등 일부 API는
    # movej()와 달리 wait_for_service() 없이 곧바로 call_async()를 던지므로,
    # discovery가 끝나기 전에 첫 호출이 나가면 응답이 영영 오지 않아
    # spin_until_future_complete()가 무한 대기에 빠진다. (고정 sleep은 시스템
    # 부하에 따라 부족할 수 있어 실패 사례가 있었다 — 같은 노드로 대표 서비스
    # 하나가 실제로 매칭될 때까지 기다려서 나머지 클라이언트도 함께 뜨게 한다.)
    startup_probe = node.create_client(MoveJoint, "motion/move_joint")
    while not startup_probe.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("컨트롤러 서비스 연결 대기 중...")
    node.destroy_client(startup_probe)
    return dsr2
