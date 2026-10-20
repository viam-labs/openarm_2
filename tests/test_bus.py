import time

from openarm_viam.can.bus import FEEDBACK_TIMEOUT_S, CanSession, Motor, commands_for, reset_motor
from openarm_viam.can.damiao import STATUS_DISABLED, STATUS_ENABLED, mit_packet, refresh_packet


def test_repeated_targets_do_not_restart_enable():
    session = CanSession("can0")
    session.motors[0x01] = Motor("DM4310", 0x01, 0x11, mode="enable", enable_cycles=3)
    session.set_target(0x01, 0.2, 10.0, 1.0)
    motor = session.motors[0x01]
    assert motor.mode == "enable"
    assert motor.enable_cycles == 3
    assert motor.target == 0.2


def test_hold_servos_to_the_measured_position():
    session = CanSession("can0")
    session.motors[0x01] = Motor("DM4310", 0x01, 0x11, mode="mit", target=1.5, measured=0.4)
    session.hold([0x01])
    assert session.motors[0x01].target == 0.4


def test_first_target_enables_a_refreshed_motor():
    session = CanSession("can0")
    session.motors[0x01] = Motor("DM4310", 0x01, 0x11)
    session.set_target(0x01, 0.1, 10.0, 1.0)
    assert session.motors[0x01].mode == "configure"
    assert session.motors[0x01].configure_cycles == 0
    assert session.motors[0x01].has_target


def test_prepare_does_not_interrupt_holding_motor_with_stale_rx():
    session = CanSession("can0")
    motor = Motor(
        "DM4310",
        0x01,
        0x11,
        mode="mit",
        measured=0.4,
        error=STATUS_ENABLED,
        last_rx=time.monotonic() - FEEDBACK_TIMEOUT_S - 0.1,
        has_target=True,
        target=0.4,
        kp=10.0,
        kd=1.0,
    )
    session.motors[0x01] = motor
    session.prepare([0x01])
    assert motor.mode == "mit"
    assert motor.has_target is True
    assert motor.target == 0.4


def test_prepare_reenables_disabled_holding_motor():
    session = CanSession("can0")
    motor = Motor(
        "DM4310",
        0x01,
        0x11,
        mode="mit",
        measured=0.4,
        error=STATUS_DISABLED,
        has_target=True,
        target=0.4,
        kp=10.0,
        kd=1.0,
    )
    session.motors[0x01] = motor
    session.prepare([0x01])
    assert motor.mode == "enable"
    assert motor.has_target is True
    assert motor.target == 0.4


def test_prepare_leaves_a_live_enabled_motor_alone():
    session = CanSession("can0")
    motor = Motor(
        "DM4310",
        0x01,
        0x11,
        mode="mit",
        measured=0.4,
        error=STATUS_ENABLED,
        last_rx=time.monotonic(),
        has_target=True,
        target=0.4,
    )
    session.motors[0x01] = motor
    session.prepare([0x01])
    assert motor.mode == "mit"
    assert motor.measured == 0.4


def test_reset_motor_keeps_hold_target_for_reenable():
    motor = Motor(
        "DM4310",
        0x01,
        0x11,
        mode="mit",
        measured=1.0,
        error=STATUS_DISABLED,
        has_target=True,
        target=1.0,
        kp=12.0,
        kd=1.5,
    )
    reset_motor(motor)
    assert motor.mode == "configure"
    assert motor.measured is None
    assert motor.has_target is True
    assert motor.target == 1.0
    assert motor.kp == 12.0


def test_reset_motor_without_target_returns_to_refresh():
    motor = Motor("DM4310", 0x01, 0x11, mode="mit", measured=1.0, error=STATUS_DISABLED)
    reset_motor(motor)
    assert motor.mode == "refresh"
    assert motor.has_target is False


def test_configure_without_fault_does_not_spam_clear_error():
    motor = Motor(
        "DM4310",
        0x01,
        0x11,
        mode="configure",
        has_target=True,
        target=0.3,
        kp=20.0,
        kd=2.0,
    )
    hold = mit_packet("DM4310", 0.3, 0.0, 20.0, 2.0, 0.0)
    frames = commands_for(motor)
    assert frames == [refresh_packet(0x01), (0x01, hold)]
    assert (0x01, bytes((0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB))) not in frames


def test_configure_and_enable_keep_streaming_mit_holds():
    motor = Motor(
        "DM4310",
        0x01,
        0x11,
        mode="configure",
        has_target=True,
        target=0.3,
        kp=20.0,
        kd=2.0,
    )
    hold = mit_packet("DM4310", 0.3, 0.0, 20.0, 2.0, 0.0)
    for _ in range(3):
        frames = commands_for(motor)
        assert (0x01, hold) in frames
    assert motor.mode == "enable"

    frames = commands_for(motor)
    assert (0x01, hold) in frames
    assert any(packet.endswith(bytes([0xFC])) for _can_id, packet in frames)


def test_drop_stale_feedback_keeps_mit_hold():
    session = CanSession("can0")
    motor = Motor(
        "DM4310",
        0x01,
        0x11,
        mode="mit",
        measured=0.4,
        error=STATUS_ENABLED,
        last_rx=time.monotonic() - 2.0,
        has_target=True,
        target=0.4,
        kp=10.0,
        kd=1.0,
    )
    session.motors[0x01] = motor
    session._drop_stale_feedback()
    assert motor.mode == "mit"
    assert motor.has_target is True
    assert motor.target == 0.4
