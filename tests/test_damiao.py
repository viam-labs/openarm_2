import struct

import pytest

from openarm_viam.can.bus import CANFD_BRS, ENABLE_CYCLES, Motor, commands_for, pack_canfd, pack_classic, unpack_frame
from openarm_viam.can.damiao import (
    CLEAR_ERROR,
    ENABLE,
    MODE_POS_FORCE,
    STATUS_DISABLED,
    STATUS_ENABLED,
    command_packet,
    control_mode_packet,
    is_param_reply,
    mit_packet,
    parse_state,
    posforce_packet,
)


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


def test_canfd_transmit_frame_sets_bitrate_switch_only():
    frame = pack_canfd(0x01, bytes(8))
    assert len(frame) == 72
    can_id, length, flags = struct.unpack_from("<IBB", frame)
    assert can_id == 0x01
    assert length == 8
    assert flags == CANFD_BRS
    unpacked = unpack_frame(frame)
    assert unpacked is not None
    assert unpacked[0] == 0x01
    assert unpacked[1] == bytes(8)
    assert unpacked[2] is False


def test_classic_frame_uses_the_16_byte_layout():
    frame = pack_classic(0x7FF, bytes((1, 0, 0xCC, 0, 0, 0, 0, 0)))
    assert len(frame) == 16
    unpacked = unpack_frame(frame)
    assert unpacked == (0x7FF, bytes((1, 0, 0xCC, 0, 0, 0, 0, 0)), False)


def test_control_mode_and_posforce_frames_match_the_openarm_layout():
    assert control_mode_packet(0x08, MODE_POS_FORCE) == (0x7FF, bytes((0x08, 0x00, 0x55, 10, MODE_POS_FORCE, 0, 0, 0)))
    position, speed, torque = struct.unpack("<fHH", posforce_packet(1.25, 50.0, 1.0 / 4.5))
    assert position == pytest.approx(1.25)
    assert speed == 5000
    assert torque == int((1.0 / 4.5) * 10000)


def test_arm_motor_clears_faults_then_holds_measured_position_once_enabled():
    motor = Motor("DM4310", 0x01, 0x11, mode="configure")
    for _ in range(3):
        assert commands_for(motor) == [(0x01, command_packet(CLEAR_ERROR))]
    assert motor.mode == "enable"
    for _ in range(ENABLE_CYCLES):
        assert commands_for(motor) == [(0x01, command_packet(ENABLE))]
    motor.measured = 0.4
    motor.error = STATUS_DISABLED
    commands_for(motor)
    assert motor.mode == "enable"
    motor.error = STATUS_ENABLED
    assert commands_for(motor) == [(0x01, command_packet(ENABLE))]
    assert motor.mode == "mit"
    assert motor.target == pytest.approx(0.4)
    assert commands_for(motor) == [(0x01, mit_packet("DM4310", 0.4, 0.0, 0.0, 0.0, 0.0))]


def test_gripper_sets_position_force_mode_before_enabling():
    motor = Motor("DM3507", 0x08, 0x18, mode="configure", control="posforce")
    assert commands_for(motor) == [
        (0x08, command_packet(CLEAR_ERROR)),
        control_mode_packet(0x08, MODE_POS_FORCE),
    ]


def test_gripper_runs_in_position_force_mode():
    motor = Motor("DM3507", 0x08, 0x18, mode="mit", control="posforce", has_target=True, target=0.2, speed=50, torque_pu=0.2)
    arbitration_id, _data = commands_for(motor)[0]
    assert arbitration_id == 0x308


def test_register_replies_are_not_read_as_positions():
    reply = bytes((0x08, 0x00, 0x55, 10, MODE_POS_FORCE, 0, 0, 0))
    assert is_param_reply(reply)
    assert parse_state("DM3507", reply) is None
