"""Joint-space streaming shared by the hardware and fake models."""

from __future__ import annotations

import asyncio
from typing import Awaitable, Callable, Sequence

Update = Callable[[Sequence[float]], Awaitable[None]]
Cancelled = Callable[[], Awaitable[bool]]


async def stream_joints(
    start: Sequence[float],
    goal: Sequence[float],
    speeds: Sequence[float],
    update: Update,
    cancelled: Cancelled,
    period: float = 0.01,
) -> list[float]:
    """Move each joint in a straight line without exceeding its speed cap.

    ``period`` is the command interval in seconds. The hardware control loop
    and the fake models both use 100 Hz.
    """

    if len(start) != len(goal) or len(start) != len(speeds):
        raise ValueError("start, goal, and speeds must have the same length")
    origin = list(start)
    target = list(goal)
    duration = 0.0
    for current, desired, speed in zip(origin, target, speeds):
        distance = abs(desired - current)
        if distance == 0:
            continue
        if speed <= 0:
            raise ValueError("joint speed must be positive")
        duration = max(duration, distance / speed)
    steps = max(1, int(round(duration / period))) if duration > 0 else 1
    commanded = origin
    for step in range(1, steps + 1):
        if await cancelled():
            return commanded
        alpha = step / steps
        commanded = [current + (desired - current) * alpha for current, desired in zip(origin, target)]
        await update(commanded)
        if step != steps:
            await asyncio.sleep(period)
    return commanded
