"""커피 공정 디스패처와 노드 진입점."""

from __future__ import annotations

import threading
import time
from functools import partial
from typing import Any, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor

from . import recovery, stages
from .buttons import PhysicalButtonInput
from .config import (
    TEST_STAGE_NAMES, TEST_GRIP_OPEN_MODES, BEAN_BY_BUTTON,
    GRIND_BY_BUTTON, SELECTION_TIMEOUT_SEC, DEFAULT_SELECTION_BUTTON,
)
from .control_bridge import CoffeeControlBridge
from .grip_monitor import GripMonitor
from .robot import RobotApi, RobotContext, create_node, import_dsr2
from .status import StatusReporter


def select_grind_by_button(ctx) -> int:
    status = ctx.status
    buttons = ctx.buttons

    status.publish(
        phase="GRIND_SELECT",
        screen=3,
        progress=34,
        title="원하는 분쇄 굵기를 선택해 주세요",
        message=(
            "DI 13 굵게(3회전) · DI 14 중간 굵게(5회전) · "
            "DI 15 중간 곱게(7회전) · DI 16 곱게(10회전). "
            f"{SELECTION_TIMEOUT_SEC:.0f}초 동안 입력이 없으면 "
            "DI 13(굵게)을 자동 선택합니다."
        ),
        busy=True,
        waiting_physical_button=True,
        selection_timeout_sec=SELECTION_TIMEOUT_SEC,
    )
    grind_button = buttons.wait_for_button(
        timeout_sec=SELECTION_TIMEOUT_SEC,
        default_button=DEFAULT_SELECTION_BUTTON,
    )
    grind_defaulted = buttons.last_wait_defaulted()
    grind_option = GRIND_BY_BUTTON[grind_button]
    grind_name = str(grind_option["name"])
    grind_turns = int(grind_option["turns"])
    status.set_grind_selection(grind_button, grind_name, grind_turns)
    status.publish(
        phase=("GRIND_DEFAULTED" if grind_defaulted else "GRIND_SELECTED"),
        screen=3,
        progress=37,
        title=f"{grind_name}를 선택했습니다",
        message=(
            (
                f"{SELECTION_TIMEOUT_SEC:.0f}초 입력이 없어 DI "
                f"{grind_button}을 자동 선택했습니다. "
                if grind_defaulted
                else f"물리 버튼 {grind_button} 입력을 확인했습니다. "
            )
            + f"그라인더를 {grind_turns}회전합니다."
        ),
        busy=True,
        button=grind_button,
    )
    return grind_turns


def run_full_sequence(
    ctx,
    preset_grind_turns: Optional[int] = None,
) -> None:
    node = ctx.node
    status = ctx.status
    run_protected_stage = partial(recovery.run_protected_stage, ctx)
    bean_drop = partial(stages.bean_drop, ctx)
    grinder = partial(stages.grinder, ctx)
    dripper_in = partial(stages.dripper_in, ctx)
    spiral_pour = partial(stages.spiral_pour, ctx)
    final_drip = partial(stages.final_drip, ctx)

    node.get_logger().info(
        "커피 시스템 시작: bean_drop -> grinder -> dripper_in -> "
        "spiral_pour -> final_drip"
    )

    status.publish(
        phase="BEAN_LOADING",
        screen=2,
        progress=10,
        title="원두를 그라인더에 넣는 중",
        message="스푼을 집어 원두를 그라인더 투입구로 옮기고 있습니다.",
        busy=True,
    )
    run_protected_stage("bean_drop", bean_drop)

    if preset_grind_turns is None:
        grind_turns = select_grind_by_button(ctx)
    else:
        grind_turns = int(preset_grind_turns)
        status.set_grind_selection(
            None,
            f"테스트 설정 · {grind_turns}회전",
            grind_turns,
        )

    status.publish(
        phase="GRINDER_MOVE",
        screen=4,
        progress=42,
        title="그라인더로 이동하는 중",
        message=f"그라인더를 {grind_turns}회전합니다.",
        busy=True,
        grind_current_turns=0.0,
    )
    run_protected_stage("grinder", lambda: grinder(grind_turns))

    status.publish(
        phase="FILTER_LOADING",
        screen=5,
        progress=68,
        title="갈린 원두를 커피 필터에 넣는 중",
        message="분쇄 원두가 담긴 병을 집어 필터 위로 이동하고 있습니다.",
        busy=True,
    )
    run_protected_stage("dripper_in", dripper_in)

    status.publish(
        phase="SPIRAL_START",
        screen=6,
        progress=75,
        title="스파이럴 드립을 시작합니다",
        message="주전자를 집어 드리퍼 위에서 내향 스파이럴을 수행합니다.",
        busy=True,
        spiral_stage="시작",
        spiral_progress=0.0,
    )
    run_protected_stage("spiral_pour", spiral_pour)

    status.publish(
        phase="FINAL_DRIP_START",
        screen=7,
        progress=89,
        title="최종 드립 공정을 시작합니다",
        message=(
            "필터 홀더와 물컵을 배치한 뒤 mug TCP 원점을 고정해 "
            "물을 붓고 시작 자세로 복귀합니다."
        ),
        busy=True,
        final_drip_stage="시작",
        final_drip_progress=0.0,
    )
    run_protected_stage("final_drip", final_drip)


def execute_test(ctx, command: dict[str, Any]) -> None:
    node = ctx.node
    status = ctx.status
    grip_close = ctx.grip_close
    jar_grip_open = ctx.jar_grip_open
    handle_grip_open = ctx.handle_grip_open
    spoon_cup_grip_open = ctx.spoon_cup_grip_open
    ensure_tool_tcp = ctx.ensure_tool_tcp
    run_protected_stage = partial(recovery.run_protected_stage, ctx)
    bean_drop = partial(stages.bean_drop, ctx)
    grinder = partial(stages.grinder, ctx)
    dripper_in = partial(stages.dripper_in, ctx)
    spiral_pour = partial(stages.spiral_pour, ctx)
    final_drip = partial(stages.final_drip, ctx)

    stage = str(command["stage"])
    stage_name = TEST_STAGE_NAMES[stage]
    grind_turns = int(command.get("grind_turns", 3))
    open_mode = str(command.get("gripper_open_mode", "spoon_cup"))

    status.begin_cycle(
        test_mode=True,
        test_stage_id=stage,
        test_stage_name=stage_name,
    )
    ensure_tool_tcp()
    node.get_logger().warning(
        f"실제 로봇 단계 테스트 실행: {stage} ({stage_name})"
    )

    if stage == "full_sequence":
        status.set_selection(None, "단계 테스트")
        run_full_sequence(ctx, grind_turns)
    elif stage == "bean_drop":
        status.publish(
            phase="TEST_BEAN_DROP",
            screen=2,
            progress=10,
            title="원두 투입 테스트 실행 중",
            message="원두 투입 단계만 1회 실행합니다.",
            busy=True,
        )
        run_protected_stage("bean_drop", bean_drop)
    elif stage == "grinder":
        status.set_grind_selection(
            None,
            f"테스트 설정 · {grind_turns}회전",
            grind_turns,
        )
        status.publish(
            phase="TEST_GRINDER",
            screen=4,
            progress=42,
            title="그라인더 테스트 실행 중",
            message=f"그라인더 단계를 {grind_turns}회전으로 실행합니다.",
            busy=True,
            grind_current_turns=0.0,
        )
        run_protected_stage("grinder", lambda: grinder(grind_turns))
    elif stage == "dripper_in":
        status.publish(
            phase="TEST_DRIPPER_IN",
            screen=5,
            progress=68,
            title="필터 투입 테스트 실행 중",
            message="필터 투입 단계만 1회 실행합니다.",
            busy=True,
        )
        run_protected_stage("dripper_in", dripper_in)
    elif stage == "spiral_pour":
        status.publish(
            phase="TEST_SPIRAL_POUR",
            screen=6,
            progress=75,
            title="스파이럴 드립 테스트 실행 중",
            message="주전자 파지부터 반환까지 1회 실행합니다.",
            busy=True,
            spiral_stage="시작",
            spiral_progress=0.0,
        )
        run_protected_stage("spiral_pour", spiral_pour)
    elif stage == "final_drip":
        status.publish(
            phase="TEST_FINAL_DRIP",
            screen=7,
            progress=89,
            title="최종 드립 테스트 실행 중",
            message="최종 드립 단계만 1회 실행합니다.",
            busy=True,
            final_drip_stage="시작",
            final_drip_progress=0.0,
        )
        run_protected_stage("final_drip", final_drip)
    elif stage == "gripper_open":
        opener = {
            "spoon_cup": spoon_cup_grip_open,
            "jar": jar_grip_open,
            "handle": handle_grip_open,
        }[open_mode]
        run_protected_stage("gripper_open", opener)
        node.get_logger().info(
            "그리퍼 열기 테스트 완료: "
            f"mode={open_mode}, name={TEST_GRIP_OPEN_MODES[open_mode]}"
        )
    elif stage == "gripper_close":
        run_protected_stage("gripper_close", grip_close)
        node.get_logger().info("그리퍼 닫기 테스트 완료")
    else:
        raise RuntimeError(f"지원하지 않는 테스트 단계: {stage}")

    status.finish_cycle("DONE")
    status.publish(
        phase="TEST_DONE",
        screen=1,
        progress=100,
        title=f"{stage_name} 테스트 완료",
        message=(
            "테스트가 끝났습니다. 잠시 후 테스트 대기 상태로 돌아가며 "
            "다른 테스트를 다시 선택할 수 있습니다."
        ),
        busy=True,
        spiral_stage=("완료" if stage in {"full_sequence", "spiral_pour"} else None),
        spiral_progress=(100.0 if stage in {"full_sequence", "spiral_pour"} else None),
        final_drip_stage=("완료" if stage in {"full_sequence", "final_drip"} else None),
        final_drip_progress=(100.0 if stage in {"full_sequence", "final_drip"} else None),
    )
    time.sleep(0.8)


# =============================================================================
# main
# =============================================================================
def main(args=None) -> None:
    rclpy.init(args=args)

    node = create_node()
    dsr2 = import_dsr2(node)
    api = RobotApi(dsr2)

    try:
        from dsr_msgs2.srv import ChangeOperationSpeed
    except ImportError as error:
        ChangeOperationSpeed = None
        node.get_logger().warning(
            "ChangeOperationSpeed 서비스 타입을 불러오지 못했습니다: "
            f"{error}"
        )

    control = CoffeeControlBridge(ChangeOperationSpeed)
    grip_monitor = GripMonitor(control.node)
    control_executor = SingleThreadedExecutor()
    control_executor.add_node(control.node)
    control_spin_thread = threading.Thread(
        target=control_executor.spin,
        name="coffee-control-spin",
        daemon=True,
    )
    control_spin_thread.start()

    api.guard_motion(grip_monitor)

    status = StatusReporter(node, speed_getter=control.speed_percent)
    buttons = PhysicalButtonInput(node, api.get_digital_input)
    ctx = RobotContext(node, api, status, grip_monitor, buttons, control)

    # =====================================================================
    # 반복 실행 디스패처
    # =====================================================================
    api.set_singularity_handling(api.DR_AVOID)
    ctx.apply_default_motion_profile()

    ensure_tool_tcp = ctx.ensure_tool_tcp
    grip_monitor.set_signal_loss_callback(
        lambda error: recovery.notify_signal_loss_immediately(ctx, error)
    )

    try:
        while rclpy.ok():
            control.set_busy(False)
            status.clear_test_context()
            status.reset_order_state()
            status.publish(
                phase="TEST_READY",
                screen=1,
                progress=0,
                title="원두 선택 또는 단계 테스트를 시작해 주세요",
                message=(
                    "DI 13~16으로 정상 전체 공정을 시작하거나 /test 페이지에서 "
                    "원하는 테스트를 선택하십시오. 입력할 때까지 시간 제한 없이 "
                    "기다립니다."
                ),
                busy=False,
                waiting_physical_button=True,
                error="",
            )

            source, action = buttons.wait_for_button_or_command(
                control.pop_test_command,
            )
            control.set_busy(True)

            if source == "command":
                command = dict(action)
                try:
                    execute_test(ctx, command)
                except Exception as error:
                    stage = str(command.get("stage", ""))
                    status.finish_cycle("ERROR")
                    node.get_logger().error(
                        f"단계 테스트 {stage} 실행 중 오류: {error}"
                    )
                    status.publish(
                        phase="TEST_ERROR",
                        screen=9,
                        progress=0,
                        title="단계 테스트 오류",
                        message=str(error),
                        busy=True,
                        error=f"{type(error).__name__}: {error}",
                    )
                    time.sleep(1.0)
                continue

            selected_button = int(action)
            selected_bean = BEAN_BY_BUTTON[selected_button]["name"]
            status.begin_cycle(test_mode=False)
            status.set_selection(selected_button, selected_bean)
            try:
                ensure_tool_tcp()
                status.publish(
                    phase="BEAN_SELECTED",
                    screen=1,
                    progress=3,
                    title=f"{selected_bean} 선택",
                    message=(
                        f"물리 버튼 {selected_button} 입력을 확인했습니다. "
                        + "커피 시스템을 시작합니다."
                    ),
                    busy=True,
                    button=selected_button,
                )
                run_full_sequence(ctx, None)
                status.finish_cycle("DONE")
                node.get_logger().info("커피 추출 전체 시퀀스 완료.")
                status.publish(
                    phase="WAIT_NEW_ORDER",
                    screen=8,
                    progress=100,
                    title="커피 추출이 완료되었습니다",
                    message=(
                        "새 커피를 주문하려면 물리 버튼 DI 13을 눌러주세요. "
                        "이 화면은 버튼을 누를 때까지 시간 제한 없이 유지됩니다."
                    ),
                    busy=True,
                    waiting_physical_button=True,
                    spiral_stage="완료",
                    spiral_progress=100.0,
                    final_drip_stage="완료",
                    final_drip_progress=100.0,
                )
                # 새 주문 버튼은 자동 선택 없이 DI13 입력을 무기한 기다린다.
                buttons.wait_for_button(allowed_buttons={13})
                node.get_logger().info(
                    "DI 13 커피 주문하기 입력 확인: 초기 화면으로 복귀"
                )
            except Exception as error:
                status.finish_cycle("ERROR")
                node.get_logger().error(f"전체 공정 실행 중 오류: {error}")
                status.publish(
                    phase="ERROR",
                    screen=9,
                    progress=0,
                    title="로봇 작업 오류",
                    message=str(error),
                    busy=True,
                    error=f"{type(error).__name__}: {error}",
                )
                time.sleep(1.0)

    except KeyboardInterrupt:
        node.get_logger().info("사용자 종료 요청을 받았습니다.")
    finally:
        try:
            control_executor.shutdown(timeout_sec=1.0)
        except TypeError:
            control_executor.shutdown()
        control_spin_thread.join(timeout=2.0)
        control.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
