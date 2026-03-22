# Gimbal Setup

## Motor Mounting

See `hardware/mechanical_assembly.md` for full frame construction details. This document covers the electrical and firmware aspects.

## First Connection

1. Power the ODrive S1 from the 24V supply via XT60 connectors.
2. Connect the ODrive to the Jetson via USB-C.
3. Verify connection: `python3 -c "import odrive; o = odrive.find_any(timeout=10); print(o.serial_number)"`

## Running Setup Script

```bash
python3 tools/odrive_setup.py
```

The script configures motor parameters, encoder settings, and initial PID gains. It then runs a calibration sequence (motor resistance/inductance measurement and encoder offset calibration) and a movement test.

Calibration takes approximately 2 minutes per axis. The motor will make a brief buzzing sound during resistance calibration and will rotate slowly during encoder offset calibration. This is normal.

## Verifying Closed-Loop Control

After setup, test closed-loop positioning:

```python
import odrive
odrv = odrive.find_any()
ax = odrv.axis0
ax.requested_state = 8  # CLOSED_LOOP_CONTROL
# Move 90 degrees (0.25 turns at direct drive)
ax.controller.input_pos = 0.25
import time; time.sleep(1.0)
print(f"Position: {ax.encoder.pos_estimate:.4f} turns (target: 0.25)")
ax.controller.input_pos = 0.0
time.sleep(1.0)
ax.requested_state = 1  # IDLE
```

Expected position error at rest: < 0.002 turns (0.72 degrees).

## PID Tuning

See `docs/08_tuning.md` for the complete gain tuning procedure.
