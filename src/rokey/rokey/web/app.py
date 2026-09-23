"""FastAPI 앱: 사용자 화면, 단계 테스트 페이지, 로컬 관리자 페이지."""

from __future__ import annotations

import asyncio
import json
import threading
from contextlib import asynccontextmanager
from typing import Any, Optional

import rclpy
from fastapi import (
    FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect,
)
from fastapi.responses import HTMLResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from . import pages
from .bridge import CoffeeWebBridge
from .constants import (
    CONTROL_TOPIC, DEFAULT_STATE, SPEED_MAX_PERCENT, SPEED_MIN_PERCENT,
    TEST_GRIND_TURNS, TEST_GRIP_OPEN_MODES, TEST_STAGE_NAMES,
)

try:
    from ..monitor_pjt.system_monitor import SystemMonitor
except ImportError:  # Support direct execution from the source directory.
    from monitor_pjt.system_monitor import SystemMonitor


bridge: Optional[CoffeeWebBridge] = None
monitor: Optional[SystemMonitor] = None
executor: Optional[MultiThreadedExecutor] = None
spin_thread: Optional[threading.Thread] = None


def _spin_nodes(active_executor: MultiThreadedExecutor) -> None:
    try:
        active_executor.spin()
    except ExternalShutdownException:
        pass


@asynccontextmanager
async def lifespan(_: FastAPI):
    global bridge, monitor, executor, spin_thread
    rclpy.init(args=None)
    bridge = CoffeeWebBridge()
    monitor = SystemMonitor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    executor.add_node(monitor)
    spin_thread = threading.Thread(
        target=_spin_nodes, args=(executor,), daemon=True)
    spin_thread.start()
    try:
        yield
    finally:
        active_bridge = bridge
        active_monitor = monitor
        active_executor = executor
        bridge = None
        monitor = None
        executor = None
        if active_executor is not None:
            active_executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if active_bridge is not None:
            active_bridge.destroy_node()
        if active_monitor is not None:
            active_monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


app = FastAPI(title="ROKEY Coffee System", lifespan=lifespan)


@app.get("/", response_class=HTMLResponse)
async def index() -> HTMLResponse:
    return HTMLResponse(pages.render("index"))


@app.get("/api/state")
async def api_state() -> dict[str, Any]:
    return bridge.state() if bridge is not None else dict(DEFAULT_STATE)


def _require_local(request: Request) -> None:
    client_host = request.client.host if request.client else ""
    if client_host not in {"127.0.0.1", "::1"}:
        raise HTTPException(
            status_code=403,
            detail="관리자 모드는 로봇 PC에서만 실행할 수 있습니다.",
        )


@app.get("/test", response_class=HTMLResponse)
async def test_page(request: Request) -> HTMLResponse:
    _require_local(request)
    return HTMLResponse(pages.render("test"))


@app.post("/api/test/start")
async def test_start(request: Request) -> dict[str, Any]:
    _require_local(request)
    payload = await request.json()
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400, detail="JSON 객체가 필요합니다.")

    stage = str(payload.get("stage", "")).strip()
    if stage not in TEST_STAGE_NAMES:
        raise HTTPException(status_code=400, detail="허용되지 않은 테스트 단계입니다.")

    try:
        grind_turns = int(payload.get("grind_turns", 3))
    except (TypeError, ValueError):
        raise HTTPException(status_code=400, detail="분쇄 회전 수가 올바르지 않습니다.")
    if grind_turns not in TEST_GRIND_TURNS:
        raise HTTPException(
            status_code=400,
            detail="분쇄 회전 수는 3, 5, 7, 10 중 하나여야 합니다.",
        )

    gripper_open_mode = str(
        payload.get("gripper_open_mode", "spoon_cup")
    ).strip()
    if gripper_open_mode not in TEST_GRIP_OPEN_MODES:
        raise HTTPException(
            status_code=400,
            detail="허용되지 않은 그리퍼 열기 프리셋입니다.",
        )

    if bridge is None:
        raise HTTPException(status_code=503, detail="ROS bridge가 준비되지 않았습니다.")
    if bool(bridge.state().get("busy", False)):
        raise HTTPException(
            status_code=409,
            detail="현재 테스트 또는 전체 공정이 실행 중입니다.",
        )
    if bridge.count_subscribers(CONTROL_TOPIC) <= 0:
        raise HTTPException(
            status_code=503,
            detail="커피 제어 노드가 단계 테스트 명령을 수신할 준비가 되지 않았습니다.",
        )

    command = {
        "cmd": "start_test",
        "stage": stage,
        "grind_turns": grind_turns,
        "gripper_open_mode": gripper_open_mode,
    }
    bridge.coffee_command(command)
    return {
        "ok": True,
        "stage": stage,
        "stage_name": TEST_STAGE_NAMES[stage],
        "grind_turns": grind_turns,
        "gripper_open_mode": gripper_open_mode,
        "gripper_open_mode_name": TEST_GRIP_OPEN_MODES[gripper_open_mode],
    }


@app.post("/api/control/speed")
async def set_operation_speed(request: Request) -> dict[str, Any]:
    payload = await request.json()
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400, detail="JSON 객체가 필요합니다.")
    try:
        speed_percent = int(round(float(payload.get("speed_percent"))))
    except (TypeError, ValueError):
        raise HTTPException(status_code=400, detail="속도 값이 올바르지 않습니다.")
    if not SPEED_MIN_PERCENT <= speed_percent <= SPEED_MAX_PERCENT:
        raise HTTPException(
            status_code=400,
            detail=(
                f"전체 공정 속도는 {SPEED_MIN_PERCENT}~"
                f"{SPEED_MAX_PERCENT}% 범위여야 합니다."
            ),
        )
    if bridge is None:
        raise HTTPException(status_code=503, detail="ROS bridge가 준비되지 않았습니다.")
    if bridge.count_subscribers(CONTROL_TOPIC) <= 0:
        raise HTTPException(
            status_code=503,
            detail="커피 제어 노드가 속도 명령을 수신할 준비가 되지 않았습니다.",
        )
    bridge.coffee_command({
        "cmd": "set_speed",
        "speed_percent": speed_percent,
    })
    return {"ok": True, "speed_percent": speed_percent}


@app.get("/admin", response_class=HTMLResponse)
async def admin_page(request: Request) -> HTMLResponse:
    _require_local(request)
    return HTMLResponse(pages.render("admin"))


@app.get("/api/admin/state")
async def admin_state(request: Request) -> dict[str, Any]:
    _require_local(request)
    return bridge.admin_state() if bridge is not None else {"connected": False}


@app.post("/api/admin/cmd")
async def admin_command(request: Request) -> dict[str, bool]:
    _require_local(request)
    payload = await request.json()
    command = payload.get("cmd") if isinstance(payload, dict) else None
    allowed = {
        "estop", "stop", "start", "set_control_enabled", "set_mode",
        "move_task", "move_joint6", "gripper",
    }
    if command not in allowed:
        raise HTTPException(status_code=400, detail="허용되지 않은 명령입니다.")
    if bridge is None:
        raise HTTPException(status_code=503, detail="ROS bridge가 준비되지 않았습니다.")
    bridge.admin_command(payload)
    return {"ok": True}


@app.websocket("/ws")
async def websocket_status(websocket: WebSocket) -> None:
    await websocket.accept()
    previous = ""
    try:
        while True:
            state = bridge.state() if bridge is not None else dict(DEFAULT_STATE)
            encoded = json.dumps(state, ensure_ascii=False, sort_keys=True)
            if encoded != previous:
                await websocket.send_json(state)
                previous = encoded
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        return


def main() -> None:
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


if __name__ == "__main__":
    main()
