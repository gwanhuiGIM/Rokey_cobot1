"""M0609 커피 시스템 노드 진입점.

실제 구현은 ``rokey.coffee`` 패키지로 분리되어 있다.

- ``coffee.config``          전역 설정값과 상수 테이블
- ``coffee.speed``           작동 속도 비율 환산
- ``coffee.geometry``        자세/경로 수학 유틸리티
- ``coffee.errors``          단계 재시작을 유발하는 예외
- ``coffee.grip_monitor``    RG2 그립 판정과 장비 상태 감시
- ``coffee.status``          Web UI JSON 상태 발행
- ``coffee.control_bridge``  웹 테스트/속도 명령 중계 노드
- ``coffee.buttons``         DI 13~16 물리 버튼 입력
- ``coffee.robot``           로봇 API 바인딩, 교시 포즈, 실행 컨텍스트
- ``coffee.stages``          공정 단계별 Sub 루틴
- ``coffee.recovery``        그립/장비 오류 복구 흐름
- ``coffee.app``             디스패처와 ``main()``
"""

from __future__ import annotations

from .coffee.app import main

__all__ = ["main"]


if __name__ == "__main__":
    main()
