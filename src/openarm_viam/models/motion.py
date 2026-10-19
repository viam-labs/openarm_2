"""Shared move bookkeeping for arms and grippers."""

from __future__ import annotations

import asyncio
from typing import Awaitable, Callable, Sequence

from viam.operations import Operation

from openarm_viam.trajectory import stream_joints

Update = Callable[[Sequence[float]], Awaitable[None]]


class Motion:
    def __init__(self) -> None:
        self.generation = 0
        self.moving = False

    def cancel(self) -> None:
        self.generation += 1
        self.moving = False

    async def run(
        self,
        start: Sequence[float],
        goal: Sequence[float],
        speeds: Sequence[float],
        update: Update,
        operation: Operation,
    ) -> list[float]:
        self.generation += 1
        generation = self.generation
        self.moving = True

        async def cancelled() -> bool:
            return generation != self.generation or await operation.is_cancelled()

        try:
            return await stream_joints(start, goal, speeds, update, cancelled)
        finally:
            if generation == self.generation:
                self.moving = False


def operation_from(component: object, kwargs: dict) -> Operation:
    getter = getattr(component, "get_operation")
    return getter(kwargs)


async def wait_until(predicate: Callable[[], bool], timeout: float, message: str) -> None:
    deadline = asyncio.get_running_loop().time() + timeout
    while asyncio.get_running_loop().time() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.02)
    raise RuntimeError(message)
