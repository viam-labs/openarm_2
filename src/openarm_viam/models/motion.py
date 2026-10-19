"""Shared move bookkeeping for arms and grippers."""

from __future__ import annotations

import asyncio
from typing import Awaitable, Callable, Sequence

from viam.operations import Operation

from openarm_viam.can.bus import CanSession
from openarm_viam.can.damiao import status_name
from openarm_viam.trajectory import stream_joints

ENABLE_TIMEOUT_S = 2.0

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


def feedback_timeout(interface: str, session: CanSession) -> str:
    return (
        f"timed out waiting for feedback on {interface} ({session.diagnose()}). "
        "rx=0 with no other_ids means no motor answered. Check power, wiring, and "
        f"termination, and that the arm is on {interface}: run "
        f"'ip -details -statistics link show {interface}' and 'candump -x {interface}'"
    )


async def wait_for_enable(interface: str, session: CanSession, joints: Sequence[tuple[str, int]]) -> None:
    """Fail with each motor's reported state instead of streaming to motors that ignore it."""

    send_ids = [send_id for _name, send_id in joints]
    deadline = asyncio.get_running_loop().time() + ENABLE_TIMEOUT_S
    while asyncio.get_running_loop().time() < deadline:
        if session.commanding(send_ids):
            return
        await asyncio.sleep(0.02)
    states = []
    for name, send_id in joints:
        code = session.status(send_id)
        states.append(f"{name}={'no reply' if code is None else status_name(code)}")
    raise RuntimeError(
        f"motors on {interface} did not enable: {', '.join(states)}. "
        "Check that the estop is released and motor power is on; a fault such as "
        "undervoltage stays until power is restored."
    )


async def wait_until(predicate: Callable[[], bool], timeout: float, message: str) -> None:
    deadline = asyncio.get_running_loop().time() + timeout
    while asyncio.get_running_loop().time() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.02)
    raise RuntimeError(message)
