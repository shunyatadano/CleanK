"""Damiao 系スクリプト間で共通利用するタイミング制御ユーティリティ。"""

from __future__ import annotations

import time
from typing import Callable, Optional

__all__ = ["Rate"]


class Rate:
    """``time.perf_counter`` を基準に、概ね一定周期でループを回すための補助クラス。

    1 周期で発生したジッタを次周期で吸収し、ドリフトが蓄積しないようにする。
    ``sleep()`` は経過したオーバーラン時間（秒）を返すので、遅延の監視にも使える。
    """

    def __init__(self, frequency_hz: float, *, clock: Callable[[], float] = time.perf_counter) -> None:
        if frequency_hz <= 0:
            raise ValueError("frequency_hz must be positive")
        self._period = 1.0 / frequency_hz
        self._clock = clock
        self._next_deadline = self._clock()

    def reset(self, *, start: Optional[float] = None) -> float:
        """基準時刻を ``start``（未指定なら現在時刻）に合わせ直し、その時刻を返す。"""
        if start is None:
            start = self._clock()
        self._next_deadline = start
        return start

    def sleep(self) -> float:
        """次のデッドラインまで待機し、オーバーラン時間（秒）を返す。"""
        self._next_deadline += self._period
        now = self._clock()
        remaining = self._next_deadline - now
        if remaining > 0:
            time.sleep(remaining)
            return 0.0
        # self._next_deadline = now
        return -remaining

    @property
    def period(self) -> float:
        return self._period
