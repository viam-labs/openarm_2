from openarm_viam.can.bus import CanSession, Motor


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
    assert session.motors[0x01].mode == "enable"
    assert session.motors[0x01].enable_cycles == 0
