# Gimbal PID Tuning Guide

This guide walks through tuning the cascaded PID controller on the ODrive S1 for both the pan and tilt axes. The default gains in `params.yaml` and `tools/odrive_setup.py` are a reasonable starting point for the GBM5208-200T at 24V but will need adjustment for your specific mechanical assembly.

The goal is to achieve the fastest step response with no overshoot and no sustained oscillation.

---

## Controller Architecture

The ODrive S1 runs three nested loops:

```
Position setpoint (200 Hz from Jetson)
    |
    v
[ Position P controller ]  ---> velocity setpoint
    |
    v
[ Velocity PI controller ] (1 kHz)  ---> current setpoint + IMU feedforward
    |
    v
[ Current/torque controller ] (8 kHz, hardware, not user-tunable)
    |
    v
Motor PWM
```

Tune from the inside out: current loop is fixed. Tune velocity loop first. Then tune position loop.

---

## Step 1 — Verify motor and encoder are working

Before any tuning, confirm the motor and encoder are operational:

```python
import odrive
odrv = odrive.find_any()
ax = odrv.axis0  # pan axis

# Put in closed loop
ax.requested_state = 8  # CLOSED_LOOP_CONTROL

# Command a small move
ax.controller.input_pos = ax.encoder.pos_estimate + 0.1  # 0.1 turns = 36 degrees
import time; time.sleep(1)
print("Position error:", ax.controller.pos_setpoint - ax.encoder.pos_estimate)
```

Expected: position error < 0.002 turns (0.72 degrees) at rest.

---

## Step 2 — Tune the velocity loop

Set the controller to velocity mode temporarily:

```python
from odrive.enums import ControlMode
ax.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
ax.controller.config.vel_limit = 30.0  # turns/s
```

Apply a square-wave velocity command and observe the response. You want the velocity to reach setpoint within 2-3 control cycles (2-3 ms) with no overshoot.

**vel_gain (P):** Start at 0.02. If response is slow, increase by 0.005 at a time. If oscillation appears, reduce by 0.005.

**vel_integrator_gain (I):** Start at 0.005. This eliminates steady-state velocity error. If low-frequency oscillation appears at rest, reduce this value. Do not set above vel_gain * 10.

Typical final values for GBM5208 at 24V direct drive:

| Gain | Range | Typical |
|---|---|---|
| vel_gain | 0.01 - 0.05 | 0.022 |
| vel_integrator_gain | 0.001 - 0.02 | 0.006 |

---

## Step 3 — Tune the position loop

Return to position control mode:

```python
ax.controller.config.control_mode = ControlMode.POSITION_CONTROL
```

The position P gain controls how aggressively position error is converted to velocity setpoint. Too low: slow, sluggish tracking. Too high: oscillation.

Apply a step input of 0.1 turns and measure the response time and overshoot:

```python
import time
start = ax.encoder.pos_estimate
ax.controller.input_pos = start + 0.1
time.sleep(0.5)
final = ax.encoder.pos_estimate
print(f"Target: {start+0.1:.4f}  Actual: {final:.4f}  Error: {final-(start+0.1):.4f}")
```

**pos_gain:** Start at 40.0. Increase by 5 until overshoot appears, then back off by 5.

Typical final values:

| Gain | Range | Typical |
|---|---|---|
| pos_gain | 20 - 60 | 40 |

---

## Step 4 — Tune the input filter

The ODrive `INPUT_MODE_POS_FILTER` applies a second-order position filter that smooths the commanded input and prevents step discontinuities from exciting mechanical resonance.

```python
from odrive.enums import InputMode
ax.controller.config.input_mode = InputMode.POS_FILTER
ax.controller.config.input_filter_bandwidth = 200.0  # Hz
```

Setting this to 200 Hz means the filter passes setpoint changes up to 200 Hz, which matches the 200 Hz command rate from the Jetson. Do not set this above the command rate. Do not set below 50 Hz or tracking will feel sluggish at fast target speeds.

---

## Step 5 — IMU feedforward gain

The IMU feedforward is applied in `gimbal_controller.py` as an additive velocity term. The gain `imu_feedforward_gain` in `params.yaml` scales this term.

To tune:
1. Mount the tracker on your intended deployment surface.
2. Tap the base firmly. Watch the camera output for jitter.
3. Increase `imu_feedforward_gain` from 0.0 to 0.8 in steps of 0.2, relaunching between each.
4. Stop increasing when the jitter stops decreasing or oscillation appears.

If the mount is very stiff (bolted to concrete), the IMU feedforward may not be needed. Set to 0.0 in that case.

---

## Step 6 — End-to-end step response test

With all loops tuned, perform a realistic tracking test:

1. Launch the full tracker.
2. Place a target at 30 m in a known position.
3. Move the target laterally at a constant speed (walk at ~2 m/s).
4. Observe whether the tracker holds the target centered within ±10 pixels at the camera's field of view.

If the tracker lags behind:
- Check that the EKF `system_latency_s` parameter matches your measured end-to-end latency.
- Increase `pos_gain` by 5.
- Increase `input_filter_bandwidth` to 300 Hz.

If the tracker oscillates ahead of the target:
- Reduce `pos_gain` by 5.
- Reduce `imu_feedforward_gain` by 0.1.

---

## Saving Tuned Gains

After tuning, save both to the ODrive NVM and to `params.yaml`:

```python
odrv.save_configuration()
```

And update `params.yaml`:
```yaml
gimbal_controller:
  ros__parameters:
    imu_feedforward_gain: 0.75   # your tuned value
```

Then update the gains in `tools/odrive_setup.py` so they are applied on the next full setup run:
```python
PAN_POS_GAIN  = 45.0   # your tuned value
PAN_VEL_P     = 0.022
PAN_VEL_I     = 0.006
```
