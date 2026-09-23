"""공정 중 단계 재시작을 유발하는 예외 정의."""

from __future__ import annotations

from typing import Any


class GripFailureError(RuntimeError):
    """물체 파지가 확인되지 않아 현재 단계를 다시 시작해야 하는 오류."""

    def __init__(
        self,
        stage_id: str,
        stage_name: str,
        grip_task: str,
        diagnostics: dict[str, Any],
    ) -> None:
        self.stage_id = str(stage_id)
        self.stage_name = str(stage_name)
        self.grip_task = str(grip_task)
        self.diagnostics = dict(diagnostics)
        super().__init__(
            f"{self.stage_name}의 {self.grip_task} 그립을 확인하지 못했습니다."
        )


class GripperSignalLostError(RuntimeError):
    """그리퍼 통신 또는 안전 상태 이상으로 현재 단계를 다시 시작하는 오류."""

    def __init__(self, stage_id: str, stage_name: str, reason: str) -> None:
        self.stage_id = str(stage_id)
        self.stage_name = str(stage_name)
        self.reason = str(reason)
        super().__init__(
            f"{self.stage_name} 실행 중 그리퍼 장비 오류: {self.reason}"
        )


class ButtonWaitCancelled(RuntimeError):
    """안전 조건이 다시 나빠져 물리 버튼 대기를 취소하는 내부 예외."""
