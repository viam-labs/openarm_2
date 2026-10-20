import time

from openarm_viam.can.bus import FEEDBACK_TIMEOUT_S, CanSession, Motor, reset_motor
from openarm_viam.can.damiao import STATUS_DISABLED, STATUS_ENABLED


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


def test_prepare_reenables_after_unplug():
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
    )
    session.motors[0x01] = motor
    session.prepare([0x01])
    assert motor.mode == "configure"
    assert motor.measured is None
    assert motor.error is None
    assert motor.configure_cycles == 0


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
    )
    session.motors[0x01] = motor
    session.prepare([0x01])
    assert motor.mode == "mit"
    assert motor.measured == 0.4


def test_reset_motor_forgets_stale_state():
    motor = Motor("DM4310", 0x01, 0x11, mode="mit", measured=1.0, error=STATUS_DISABLED, has_target=True)
    reset_motor(motor)
    assert motor.mode == "refresh"
    assert motor.measured is None
    assert motor.has_target is False
