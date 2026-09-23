"""Web UI 노드 진입점.

실제 구현은 ``rokey.web`` 패키지로 분리되어 있다.

- ``web.constants``  토픽 이름, 허용값, 기본 상태 스냅샷
- ``web.bridge``     상태 수집과 명령 발행을 담당하는 ROS 노드
- ``web.pages``      ``web/templates/*.html`` 로더
- ``web.app``        FastAPI 앱, 라우트, ``main()``

화면 HTML은 ``rokey/web/templates/`` 아래 index/test/admin 세 파일에 있다.
"""

from __future__ import annotations

try:
    from .web.app import app, main
except ImportError:  # Support direct execution from the source directory.
    from web.app import app, main

__all__ = ["app", "main"]


if __name__ == "__main__":
    main()
