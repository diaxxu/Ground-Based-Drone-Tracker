#!/usr/bin/env python3
"""
odrive_setup.py

First-time configuration script for the ODrive S1 controlling two
iPower GBM5208-200T brushless motors with AMT21 absolute encoders.

Run this script ONCE after mechanical assembly, before launching the
tracker. It writes PID gains, motor parameters, and encoder settings
to the ODrive's non-volatile configuration.

Usage:
    python3 tools/odrive_setup.py

The script will:
    1. Connect to the ODrive
    2. Configure motor parameters for GBM5208-200T
    3. Configure AMT21 SPI encoders
    4. Run motor calibration sequence
    5. Tune PID gains
    6. Save configuration to ODrive NVM
    7. Run a basic movement test to verify setup
"""

import sys
import time

try:
    import odrive
    from odrive.enums import (
        AxisState, ControlMode, InputMode,
        MotorType, EncoderMode,
    )
except ImportError:
    print("ERROR: odrive package not installed.")
    print("Install with:  pip3 install odrive")
    sys.exit(1)


# GBM5208-200T motor parameters
MOTOR_POLE_PAIRS        = 11     # 22 poles / 2
MOTOR_KV                = 200    # RPM/V
MOTOR_RESISTANCE_OHM    = 0.30   # measured phase-to-phase / 2
MOTOR_INDUCTANCE_H      = 0.00012  # measured phase-to-phase / 2
MOTOR_CURRENT_LIM_A     = 20.0   # peak current limit (motor rated ~15A continuous)
MOTOR_CURRENT_SOFT_MAX  = 15.0

# PID gains — initial values, tune per docs/08_tuning.md
PAN_POS_GAIN    = 40.0
PAN_VEL_P       = 0.02
PAN_VEL_I       = 0.005
PAN_VEL_LIM     = 120.0  # turns/s peak slew rate (120 deg/s at 1:1)

TILT_POS_GAIN   = 40.0
TILT_VEL_P      = 0.02
TILT_VEL_I      = 0.005
TILT_VEL_LIM    = 60.0   # tilt needs less range


def configure_axis(axis, label: str, vel_limit: float, pos_gain: float, vel_p: float, vel_i: float):
    print(f"\n  Configuring {label} axis...")

    m = axis.motor
    m.config.motor_type         = MotorType.HIGH_CURRENT
    m.config.pole_pairs         = MOTOR_POLE_PAIRS
    m.config.resistance_calib_max_voltage = 4.0
    m.config.requested_current_range      = MOTOR_CURRENT_LIM_A
    m.config.current_lim                  = MOTOR_CURRENT_LIM_A
    m.config.current_lim_margin           = 5.0

    e = axis.encoder
    e.config.mode       = EncoderMode.SPI_ABS_AMT21
    e.config.cpr        = 16384   # 2^14 for AMT21 16-bit
    e.config.bandwidth  = 1000.0  # Hz, faster response

    c = axis.controller
    c.config.control_mode = ControlMode.POSITION_CONTROL
    c.config.input_mode   = InputMode.POS_FILTER
    c.config.input_filter_bandwidth = 200.0

    c.config.pos_gain = pos_gain
    c.config.vel_gain = vel_p
    c.config.vel_integrator_gain = vel_i
    c.config.vel_limit  = vel_limit
    c.config.vel_ramp_rate = 500.0  # turns/s^2

    print(f"  {label}: motor + encoder + PID configured")


def calibrate_axis(axis, label: str):
    print(f"\n  Running full calibration sequence for {label}...")
    print("  WARNING: motor will move during calibration. Ensure clear range of motion.")
    input("  Press ENTER to begin calibration (or Ctrl+C to abort)...")

    axis.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    while axis.current_state != AxisState.IDLE:
        time.sleep(0.5)
        print(f"  {label} state: {axis.current_state}", end='\r')

    if axis.motor.is_calibrated and axis.encoder.is_ready:
        print(f"\n  {label}: calibration complete")
    else:
        print(f"\n  {label}: calibration FAILED")
        print(f"         motor.is_calibrated = {axis.motor.is_calibrated}")
        print(f"         encoder.is_ready    = {axis.encoder.is_ready}")
        raise RuntimeError(f"{label} calibration failed — check wiring and retry")


def test_movement(axis, label: str, amplitude_turns: float = 0.05):
    print(f"\n  Movement test for {label} (amplitude: {amplitude_turns} turns)...")
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL

    start = axis.encoder.pos_estimate
    axis.controller.input_pos = start + amplitude_turns
    time.sleep(0.5)
    pos = axis.encoder.pos_estimate
    error = abs(pos - (start + amplitude_turns))
    print(f"  Target: {start + amplitude_turns:.4f}  Actual: {pos:.4f}  Error: {error:.4f} turns")

    axis.controller.input_pos = start
    time.sleep(0.5)
    axis.requested_state = AxisState.IDLE

    if error > 0.005:  # > 0.005 turns = 1.8 degrees
        print(f"  WARNING: position error larger than expected ({error:.4f} turns)")
    else:
        print(f"  {label}: movement test passed")


def main():
    print("ODrive S1 Setup Script")
    print("======================")
    print("Connecting to ODrive S1...")

    try:
        odrv = odrive.find_any(timeout=15)
    except Exception as e:
        print(f"ERROR: Could not find ODrive: {e}")
        print("Check USB cable and that the ODrive is powered (24V).")
        sys.exit(1)

    print(f"Connected to ODrive: serial number {odrv.serial_number}")
    print(f"Hardware version: {odrv.hw_version_major}.{odrv.hw_version_minor}")
    print(f"Firmware version: {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")

    if odrv.vbus_voltage < 20.0:
        print(f"WARNING: VBus voltage is {odrv.vbus_voltage:.1f}V. Expect 24V. Check power supply.")

    print("\nStep 1: Erasing old configuration...")
    odrv.erase_configuration()
    time.sleep(1.0)

    print("Step 2: Configuring power parameters...")
    odrv.config.dc_bus_undervoltage_trip_level = 18.0
    odrv.config.dc_bus_overvoltage_trip_level  = 30.0
    odrv.config.dc_max_positive_current        = 30.0
    odrv.config.dc_max_negative_current        = -5.0
    odrv.config.brake_resistance               = 0.0

    print("Step 3: Configuring pan axis (axis0)...")
    configure_axis(odrv.axis0, "Pan", PAN_VEL_LIM, PAN_POS_GAIN, PAN_VEL_P, PAN_VEL_I)

    print("Step 4: Configuring tilt axis (axis1)...")
    configure_axis(odrv.axis1, "Tilt", TILT_VEL_LIM, TILT_POS_GAIN, TILT_VEL_P, TILT_VEL_I)

    print("\nStep 5: Saving pre-calibration configuration...")
    odrv.save_configuration()
    time.sleep(2.0)

    print("\nStep 6: Motor calibration (pan axis)...")
    calibrate_axis(odrv.axis0, "Pan")

    print("\nStep 7: Motor calibration (tilt axis)...")
    calibrate_axis(odrv.axis1, "Tilt")

    print("\nStep 8: Setting startup state to closed loop...")
    odrv.axis0.config.startup_closed_loop_control = True
    odrv.axis1.config.startup_closed_loop_control = True

    print("\nStep 9: Saving final configuration...")
    odrv.save_configuration()
    time.sleep(2.0)

    print("\nStep 10: Running movement tests...")
    test_movement(odrv.axis0, "Pan",  amplitude_turns=0.05)
    test_movement(odrv.axis1, "Tilt", amplitude_turns=0.02)

    print("\n======================")
    print("ODrive setup complete.")
    print("Next step: python3 tools/calibrate_extrinsics.py")


if __name__ == '__main__':
    main()
