import pytest

from openarm_viam.can.damiao import mit_packet, parse_state


def test_zero_mit_packet_uses_the_midpoint_encoding():
    packet = mit_packet("DM8009", 0, 0, 0, 0, 0)
    assert packet == bytes((127, 255, 127, 240, 0, 0, 7, 255))


def test_feedback_midpoint_decodes_near_zero():
    # 16-bit midpoint is 32767. 12-bit midpoint is 2047.
    data = bytes((0x11, 127, 255, 127, 247, 255, 30, 31))
    state = parse_state("DM4310", data)
    assert state is not None
    assert state.position == pytest.approx(0, abs=1e-3)
    assert state.velocity == pytest.approx(0, abs=0.02)
    assert state.torque == pytest.approx(0, abs=0.01)
    assert state.temperature_mos == 30
    assert state.temperature_rotor == 31
    assert state.error == 1
