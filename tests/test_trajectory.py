import asyncio
import math

import pytest

from openarm_viam.trajectory import ease_alpha, stream_joints


def test_ease_alpha_starts_and_ends_gently():
    assert ease_alpha(0.0) == pytest.approx(0.0)
    assert ease_alpha(1.0) == pytest.approx(1.0)
    assert ease_alpha(0.5) == pytest.approx(0.5)
    # Early and late steps move less than a linear ramp.
    assert ease_alpha(0.1) < 0.1
    assert 1.0 - ease_alpha(0.9) < 0.1


def test_stream_joints_respects_peak_speed_with_easing():
    async def run():
        samples = []

        async def update(values):
            samples.append(list(values))

        async def never():
            return False

        await stream_joints([0.0], [0.4], [0.4], update, never, period=0.01)
        assert samples[0][0] == pytest.approx(ease_alpha(1 / len(samples)) * 0.4)
        assert samples[-1][0] == pytest.approx(0.4)
        # Peak rate ~0.4 rad/s; ease stretches duration by pi/2.
        assert len(samples) == pytest.approx(int(round((math.pi / 2) * 0.4 / 0.4 / 0.01)), abs=1)

    asyncio.run(run())
