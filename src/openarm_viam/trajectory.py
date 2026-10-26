"""Joint-space streaming shared by the hardware and fake models."""

from __future__ import annotations

import asyncio
import math
from typing import Awaitable, Callable, Sequence

Update = Callable[[Sequence[float]], Awaitable[None]]
Cancelled = Callable[[], Awaitable[bool]]

# Cosine ease-in/out peaks at pi/2 times the average rate of a linear ramp.
_EASE_PEAK = math.pi / 2


def ease_alpha(alpha: float) -> float:
    """Map a linear 0..1 fraction onto a cosine ease-in/out curve."""

    alpha = min(1.0, max(0.0, alpha))
    return 0.5 - 0.5 * math.cos(math.pi * alpha)


async def stream_joints(
    start: Sequence[float],
    goal: Sequence[float],
    speeds: Sequence[float],
    update: Update,
    cancelled: Cancelled,
    period: float = 0.01,
) -> list[float]:
    """Move each joint with a cosine ease-in/out without exceeding its speed cap.

    ``speeds`` is treated as a peak rate. The path starts and ends at zero
    velocity so the final PD hold does not have to catch a fast step.
    ``period`` is the command interval in seconds (100 Hz on hardware and fake).
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
        # Peak of the eased profile is _EASE_PEAK times the linear average rate.
        duration = max(duration, _EASE_PEAK * distance / speed)
    steps = max(1, int(round(duration / period))) if duration > 0 else 1
    commanded = origin
    for step in range(1, steps + 1):
        if await cancelled():
            return commanded
        alpha = ease_alpha(step / steps)
        commanded = [current + (desired - current) * alpha for current, desired in zip(origin, target)]
        await update(commanded)
        if step != steps:
            await asyncio.sleep(period)
    return commanded
