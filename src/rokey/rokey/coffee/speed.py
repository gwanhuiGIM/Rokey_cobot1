"""작동 속도 비율을 표시용 유효값으로 환산하는 헬퍼."""

from __future__ import annotations

from .config import (
    OPERATION_SPEED_MIN_PERCENT, OPERATION_SPEED_MAX_PERCENT,
    OPERATION_SPEED_VARIABLES, OPERATION_ACCELERATION_VARIABLES,
)


def _effective_speed_value(base_value: float, percent: int) -> float:
    """작동 속도 비율을 반영한 표시용 유효 속도를 계산한다."""
    return float(base_value) * float(percent) / 100.0


def _speed_change_lines(previous_percent: int, requested_percent: int) -> list[str]:
    """터미널/UI에 공통으로 사용할 속도 변경 상세 로그를 생성한다."""
    previous = max(
        OPERATION_SPEED_MIN_PERCENT,
        min(OPERATION_SPEED_MAX_PERCENT, int(previous_percent)),
    )
    requested = max(
        OPERATION_SPEED_MIN_PERCENT,
        min(OPERATION_SPEED_MAX_PERCENT, int(requested_percent)),
    )
    lines = [
        f"작동 속도 비율: {previous}% -> {requested}%",
        "Python 상수는 유지되고 로봇 모션의 유효 속도 비율만 변경됩니다.",
    ]
    for name, base_value, unit in OPERATION_SPEED_VARIABLES:
        old_value = _effective_speed_value(base_value, previous)
        new_value = _effective_speed_value(base_value, requested)
        lines.append(
            f"{name}: {old_value:.3f} -> {new_value:.3f} {unit} "
            f"(기준값 {base_value:.3f})"
        )
    unchanged = ", ".join(
        f"{name}={value:.3f} {unit}"
        for name, value, unit in OPERATION_ACCELERATION_VARIABLES
    )
    lines.append(f"가속도 변수 변경 없음: {unchanged}")
    return lines


def _effective_speed_payload(percent: int) -> dict[str, dict[str, float | str]]:
    """웹 UI 상태에 전달할 현재 유효 속도 변수 값을 만든다."""
    return {
        name: {
            "base": round(float(base_value), 4),
            "effective": round(_effective_speed_value(base_value, percent), 4),
            "unit": unit,
        }
        for name, base_value, unit in OPERATION_SPEED_VARIABLES
    }
