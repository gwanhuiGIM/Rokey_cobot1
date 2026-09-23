"""자세/경로 계산에 쓰는 순수 수학 유틸리티."""

from __future__ import annotations

import math
from typing import Any

import numpy as np


def _pose6_from_dsr(value: Any) -> tuple[float, float, float, float, float, float]:
    """get_current_posx() 반환값에서 6축 pose를 추출한다.

    DSR_ROBOT2 버전에 따라 posx 자체 또는 (posx, solution_space) 형태로
    반환될 수 있어 두 형식을 모두 처리한다.
    """
    candidate = value

    if isinstance(value, (list, tuple)) and len(value) == 2:
        first = value[0]
        if hasattr(first, "__iter__") and not isinstance(first, (str, bytes)):
            first_values = list(first)
            if len(first_values) >= 6:
                candidate = first_values

    if hasattr(candidate, "__iter__") and not isinstance(
        candidate, (str, bytes)
    ):
        values = [float(item) for item in list(candidate)]
    else:
        raise RuntimeError(f"현재 TCP pose 형식이 올바르지 않습니다: {value!r}")

    if len(values) < 6 or not all(math.isfinite(item) for item in values[:6]):
        raise RuntimeError(f"현재 TCP pose 값이 올바르지 않습니다: {value!r}")

    return tuple(values[:6])


def _wrap_radians(angle: float) -> float:
    """각도를 [-pi, pi) 범위로 정규화한다."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _smoothstep5(progress: float) -> float:
    u = _clamp(progress, 0.0, 1.0)
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def _rot_y_deg(angle_deg: float) -> np.ndarray:
    angle = math.radians(float(angle_deg))
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]],
        dtype=float,
    )


def _rot_z_deg(angle_deg: float) -> np.ndarray:
    angle = math.radians(float(angle_deg))
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


def _zyz_to_rotm(abc_deg) -> np.ndarray:
    a_deg, b_deg, c_deg = [float(value) for value in abc_deg[:3]]
    return _rot_z_deg(a_deg) @ _rot_y_deg(b_deg) @ _rot_z_deg(c_deg)


def _project_rotation(rotation: np.ndarray) -> np.ndarray:
    u_matrix, _, vt_matrix = np.linalg.svd(
        np.asarray(rotation, dtype=float).reshape(3, 3)
    )
    output = u_matrix @ vt_matrix
    if np.linalg.det(output) < 0.0:
        u_matrix[:, -1] *= -1.0
        output = u_matrix @ vt_matrix
    return output


def _rotvec_to_rotm(rotvec_rad) -> np.ndarray:
    vector = np.asarray(rotvec_rad, dtype=float).reshape(3)
    angle = float(np.linalg.norm(vector))
    if angle < 1.0e-12:
        return np.eye(3, dtype=float)

    axis = vector / angle
    x_axis, y_axis, z_axis = axis
    skew = np.array(
        [
            [0.0, -z_axis, y_axis],
            [z_axis, 0.0, -x_axis],
            [-y_axis, x_axis, 0.0],
        ],
        dtype=float,
    )
    return (
        np.eye(3, dtype=float)
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle)) * (skew @ skew)
    )


def _rotm_to_rotvec(rotation: np.ndarray) -> np.ndarray:
    matrix = _project_rotation(rotation)
    cosine_angle = _clamp(
        (float(np.trace(matrix)) - 1.0) * 0.5,
        -1.0,
        1.0,
    )
    angle = math.acos(cosine_angle)

    if angle < 1.0e-10:
        return np.zeros(3, dtype=float)

    if abs(math.pi - angle) < 1.0e-6:
        diagonal = np.maximum((np.diag(matrix) + 1.0) * 0.5, 0.0)
        axis = np.sqrt(diagonal)
        if matrix[2, 1] - matrix[1, 2] < 0.0:
            axis[0] *= -1.0
        if matrix[0, 2] - matrix[2, 0] < 0.0:
            axis[1] *= -1.0
        if matrix[1, 0] - matrix[0, 1] < 0.0:
            axis[2] *= -1.0
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 1.0e-9:
            axis = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            axis = axis / axis_norm
        return axis * angle

    factor = angle / (2.0 * math.sin(angle))
    return factor * np.array(
        [
            matrix[2, 1] - matrix[1, 2],
            matrix[0, 2] - matrix[2, 0],
            matrix[1, 0] - matrix[0, 1],
        ],
        dtype=float,
    )


def _wrapped_delta_deg(value: float, reference: float) -> float:
    return (float(value) - float(reference) + 180.0) % 360.0 - 180.0


def _rotm_to_zyz_candidates(rotation: np.ndarray) -> list[np.ndarray]:
    matrix = _project_rotation(rotation)
    b_angle = math.acos(_clamp(float(matrix[2, 2]), -1.0, 1.0))
    sine_b = math.sin(b_angle)

    if abs(sine_b) > 1.0e-8:
        a_angle = math.atan2(float(matrix[1, 2]), float(matrix[0, 2]))
        c_angle = math.atan2(float(matrix[2, 1]), -float(matrix[2, 0]))
        first = np.degrees(
            np.array([a_angle, b_angle, c_angle], dtype=float)
        )
        second = np.array(
            [first[0] + 180.0, -first[1], first[2] + 180.0],
            dtype=float,
        )
        return [first, second]

    combined = math.degrees(
        math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    )
    b_deg = math.degrees(b_angle)
    return [
        np.array([combined, b_deg, 0.0], dtype=float),
        np.array([0.0, b_deg, combined], dtype=float),
    ]


def _rotm_to_zyz_near(
    rotation: np.ndarray,
    reference_abc_deg,
) -> np.ndarray:
    reference = np.asarray(reference_abc_deg, dtype=float).reshape(3)
    best = None
    best_cost = math.inf

    for candidate in _rotm_to_zyz_candidates(rotation):
        adjusted = candidate.copy()
        for index in (0, 2):
            adjusted[index] = reference[index] + _wrapped_delta_deg(
                adjusted[index],
                reference[index],
            )

        cost = float(
            np.linalg.norm(
                np.array(
                    [
                        _wrapped_delta_deg(adjusted[0], reference[0]),
                        adjusted[1] - reference[1],
                        _wrapped_delta_deg(adjusted[2], reference[2]),
                    ],
                    dtype=float,
                )
            )
        )
        if cost < best_cost:
            best = adjusted
            best_cost = cost

    if best is None:
        raise RuntimeError(
            "회전행렬을 Doosan Z-Y-Z Euler로 변환하지 못했습니다."
        )
    return best


def _numeric6(value: Any, *, label: str) -> np.ndarray:
    candidate = value

    if isinstance(candidate, tuple) and len(candidate) >= 1:
        first = candidate[0]
        if hasattr(first, "__iter__") and not isinstance(
            first,
            (str, bytes),
        ):
            candidate = first
    elif isinstance(candidate, list) and len(candidate) == 2:
        first = candidate[0]
        if hasattr(first, "__iter__") and not isinstance(
            first,
            (str, bytes),
        ):
            first_values = list(first)
            if len(first_values) >= 6:
                candidate = first_values

    try:
        values = np.asarray(
            [float(item) for item in candidate],
            dtype=float,
        )
    except Exception as exc:
        raise RuntimeError(f"{label} 변환 실패: {value!r}") from exc

    if values.size < 6:
        raise RuntimeError(f"{label} 길이 오류: {values.tolist()!r}")

    result = values[:6].copy()
    if not np.all(np.isfinite(result)):
        raise RuntimeError(
            f"{label}에 유효하지 않은 값이 있습니다: {result!r}"
        )
    return result
