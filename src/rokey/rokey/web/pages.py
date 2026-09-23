"""HTML 템플릿 로더.

템플릿은 ``rokey/web/templates/``에 그대로 두고 요청 시 읽어 캐시한다.
파일을 고치면 서버를 다시 시작할 때 반영된다.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"


@lru_cache(maxsize=None)
def render(name: str) -> str:
    """``templates/<name>``을 읽어 문자열로 돌려준다."""
    return (TEMPLATE_DIR / f"{name}.html").read_text(encoding="utf-8")
