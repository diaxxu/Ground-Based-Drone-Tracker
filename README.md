# Ground Tracker — Radar + Camera Drone Tracker
<img width="1113" height="620" alt="image" src="https://github.com/user-attachments/assets/6fb8c495-16a3-4e29-939f-7f8b4d0dc430" />


> **Under $1,300 in off-the-shelf parts. 12 ms end-to-end latency. 120 deg/s tracking rate.**

A ground-based pan-tilt tracker that locks onto aerial targets using a 77 GHz FMCW radar for detection and a global-shutter camera for precision centroid lock. An Extended Kalman Filter fuses both sensors into a predictive 3D track. A direct-drive brushless gimbal points the payload at where the target will be — not where it was.


---

## Table of Contents

- [How It Works](#how-it-works)
- [System Architecture](#system-architecture)
- [Bill of Materials](#bill-of-materials)
- [Step-by-Step Build Guide](#step-by-step-build-guide)
- [Software Installation](#software-installation)
- [Calibration](#calibration)
- [Tuning](#tuning)
- [Running the Tracker](#running-the-tracker)
- [Performance](#performance)
- [Troubleshooting](#troubleshooting)
- [FAQ](#faq)

---

## How It Works

```
[AWR1843 radar]            [OV2710 camera]
      |                          |
      | UART 921600              | USB 3.0
      v                          v
[radar_driver node]     [camera_tracker node]
      |                          |
      | /radar/detections        | /camera/target
      |   50 Hz                  |   120 Hz
      v                          v
             [ekf_controller node]
              — fuses both sensors
              — predicts position 12.5 ms ahead
              — outputs az/el at 200 Hz
                       |
                       | /ekf/target_pose (200 Hz)
                       v
             [gimbal_controller node]
                       |
                       | ODrive S1 via USB-C (200 Hz)
                       v
             [GBM5208 pan + tilt motors]
              — position loop:  200 Hz
              — velocity loop:  1 kHz
              — current loop:   8 kHz
```

1. The radar detects a moving target and reports its 3D position (range, angle, radial velocity) at 50 Hz
2. The camera runs YOLOv8-nano on a TensorRT engine to detect the target, then switches to CSRT for sub-frame centroid tracking at 120 fps
3. The EKF merges both measurements into a smooth 9-state track (position, velocity, acceleration)
4. The EKF predicts where the target will be 12.5 ms into the future to compensate for system latency
5. The gimbal controller sends predicted azimuth/elevation to the ODrive at 200 Hz
6. The ODrive runs a cascaded FOC loop driving two direct-drive brushless motors with sub-0.05 degree accuracy

**All motor control runs inside the ODrive firmware. You write zero low-level motor code.**

---

## System Architecture

```
+-----------------------------------------------------------------------+
|                          JETSON ORIN NANO                              |
|                                                                        |
|  +--------------+   +-------------------+   +---------------------+  |
|  | radar_driver |   |  camera_tracker   |   |   ekf_controller    |  |
|  |              |   |                   |   |                     |  |
|  | UART parse   |   | YOLOv8n TensorRT  |   | 9-state EKF         |  |
|  | CFAR detect  |   | CSRT tracker      |   | 1 kHz predict step  |  |
|  | 50 Hz output |   | 120 Hz output     |   | 200 Hz az/el output |  |
|  +------+-------+   +--------+----------+   +---------+-----------+  |
|         |                    |                         |              |
|         +--------------------+-------------------------+              |
|                              | /ekf/target_pose                       |
|                    +---------+---------+                              |
|                    | gimbal_controller |                              |
|                    |  200 Hz setpoints |                              |
|                    |  IMU feedforward  |                              |
|                    +---------+---------+                              |
+------------------------------|-----------------------------------------+
                                | USB-C (ODrive protocol)
                                v
              +----------------------------------+
              |           ODrive S1              |
              |  position loop     200 Hz        |
              |  velocity loop     1 kHz         |
              |  current loop (FOC) 8 kHz        |
              +--------+-----------+-------------+
                       | 3-phase   | 3-phase
                       | PWM       | PWM
              +--------+-----+ +---+--------+
              | GBM5208-200T | | GBM5208-200T|
              |  Pan motor   | |  Tilt motor |
              | direct drive | | direct drive|
              +--------+-----+ +---+--------+
                       |               |
              +--------+-----+ +---+--------+
              | AMT21 16-bit | | AMT21 16-bit|
              |   encoder    | |   encoder   |
              |   SPI        | |   SPI       |
              +--------------+ +------------+
                       ^
                       | I2C 400 Hz
              +--------+-----+
              |  BMI088 IMU  |
              | gyro feedfwd |
              +--------------+
```

---

## Bill of Materials

Total cost: **$1,293 USD** 

### Sensing

| Part | Part Number | Supplier | Unit Cost | Qty | Subtotal |
|------|------------|---------|-----------|-----|----------|
| TI AWR1843BOOST 77 GHz radar | AWR1843BOOST | Mouser | $99 | 1 | $99 |
| ELP OV2710 global-shutter USB camera | ELP-USB100W05MT-DL36 | Amazon | $45 | 1 | $45 |
| 8-50 mm CS-mount varifocal lens | MVL8M23 | AliExpress | $35 | 1 | $35 |

> **Global shutter is not optional.** A rolling-shutter camera smears a fast-moving drone into a diagonal streak. The centroid computed from that streak is wrong by 10-30 pixels. Do not substitute the OV2710 with any rolling-shutter sensor.

### Compute

| Part | Part Number | Supplier | Unit Cost | Qty | Subtotal |
|------|------------|---------|-----------|-----|----------|
| NVIDIA Jetson Orin Nano 8 GB Dev Kit | 945-13766-0000-000 | Arrow | $499 | 1 | $499 |
| 256 GB NVMe SSD M.2 2280 | Any | Amazon | $30 | 1 | $30 |

> Do not use the 4 GB Jetson variant. TensorRT YOLOv8 model loading requires more than 5 GB during initialization.

### Actuation

| Part | Part Number | Supplier | Unit Cost | Qty | Subtotal |
|------|------------|---------|-----------|-----|----------|
| iPower GBM5208-200T brushless motor | GBM5208-200T | AliExpress | $55 | 2 | $110 |
| ODrive S1 motor controller | ODrive S1 | ODrive Robotics | $139 | 1 | $139 |
| AMT21 absolute magnetic encoder 16-bit | AMT21A-V | DigiKey | $55 | 2 | $110 |
| Bosch BMI088 IMU breakout | BMI088 | Adafruit | $18 | 1 | $18 |

### Frame and Power

| Part | Supplier | Unit Cost | Qty | Subtotal |
|------|---------|-----------|-----|----------|
| 30x30 mm aluminum extrusion 400 mm | Misumi | $12 | 4 | $48 |
| 4 mm aluminum plate 200x200 mm | Online Metals | $22 | 1 | $22 |
| 6804-2RS thin section bearing | Amazon | $14 | 2 | $28 |
| Mean Well S-240-24 PSU (24V 10A) | Amazon | $35 | 1 | $35 |
| Pololu D24V50F5 5V buck converter | Pololu | $12 | 1 | $12 |
| Wiring, connectors, fasteners, misc hardware | Various | — | — | $61 |

| | | | **Total** | **$1,293** |

Full part numbers with supplier links are in `hardware/bom.csv`.

### Tools Required

| Tool | Purpose |
|------|---------|
| Multimeter | Voltage verification before first power-on |
| Drill press or hand drill | Frame plate holes |
| Angle grinder or metal saw | Cutting extrusion to length |
| Hex key set (M3, M5) | Frame and motor assembly |
| Laptop (any OS) | Jetson setup, ODrive configuration |
| Caliper | Encoder gap measurement — must be 0.5-1.5 mm |

---

## Step-by-Step Build Guide

### Phase 1 — Mechanical Assembly

#### Step 1 — Cut and assemble the base frame

Cut the 30x30 aluminum extrusion to the following lengths:

| Piece | Length | Qty |
|-------|--------|-----|
| Base legs | 150 mm | 4 |
| Base crossmembers | 200 mm | 2 |
| Tilt arms | 120 mm | 2 |

Assemble into a rectangular base using M5 T-slot nuts and M5x12 bolts. Measure both diagonals — they must be equal within 1 mm before final tightening. Torque to 3 Nm.

#### Step 2 — Mount the pan motor

Mount the GBM5208 pan motor stator to the center of the base plate with M3x10 bolts through the motor's 4-hole bolt circle (58 mm diameter). Press one 6804-2RS bearing onto the motor shaft. The rotating upper plate rests on this bearing's outer race.

#### Step 3 — Install pan encoder

Mount the AMT21 encoder body to the underside of the base plate, centered on the motor shaft. Press the 6 mm magnet onto the motor shaft back face with a set-screw collar. **Set the gap between magnet and encoder face to 0.5-1.5 mm using a 1 mm shim.** Outside this range, the encoder will lose position.

#### Step 4 — Mount the tilt motor and encoder

Repeat the motor and encoder procedure for the tilt axis. The tilt motor shaft runs horizontally, perpendicular to the look direction. Balance the tilt arm by sliding the payload fore and aft until it holds any angle without motor power — an unbalanced tilt axis wastes torque and slows step response.

#### Step 5 — Co-boresight camera and radar

Mount the camera and radar on the tilt arm facing the same direction. Center-to-center offset must be less than 30 mm horizontally and 30 mm vertically. **Record the exact offset in mm** — you will enter it during extrinsic calibration.

Full mechanical details with hole patterns and weight budget are in `hardware/mechanical_assembly.md`.

---

### Phase 2 — Wiring

#### Step 6 — Wire the power rail

```
Mean Well PSU (24V 10A)
    |
    +-- [XT60] ----------------------------------> ODrive S1 VBus terminals (14 AWG)
    |
    +-- [D24V50F5 buck] --> 5V rail -----------> Jetson Orin Nano USB-C
    |
    +-- [3A fuse] --> 3.3V (from Jetson pin 1) -> BMI088 VIN
```

All GND references must return to the PSU negative terminal at a single star point. Do not create ground loops between the ODrive GND and the Jetson GND through multiple paths.

#### Step 7 — Wire motors to ODrive

Connect each motor's three phase wires to the corresponding ODrive motor output terminals (M0 for pan, M1 for tilt). Phase wire order does not matter for initial wiring — if a motor spins the wrong direction during calibration, swap any two of its three phase wires.

#### Step 8 — Wire encoders to ODrive

| AMT21 pin | ODrive ENC header pin |
|-----------|----------------------|
| 1 — GND | GND |
| 2 — 5V | 5V |
| 3 — MOSI | MOSI |
| 4 — MISO | MISO |
| 5 — CLK | CLK |
| 6 — CS | CS |

#### Step 9 — Wire IMU to Jetson

| BMI088 pin | Jetson 40-pin header |
|-----------|---------------------|
| VIN | 3.3V — pin 1 (do not use 5V) |
| GND | GND — pin 6 |
| SDA | I2C SDA — pin 3 |
| SCL | I2C SCL — pin 5 |

Add 4.7k ohm pullup resistors from SDA and SCL to 3.3V.

#### Step 10 — Connect radar and camera

- AWR1843BOOST: USB Micro-B to any Jetson USB port. This powers the radar and creates two UART devices.
- OV2710 camera: USB 3.0 cable directly into the blue USB port on the Jetson.

Full connector-level tables and a pre-power checklist are in `hardware/wiring_diagram.md`.

---

### Phase 3 — First Power-On

#### Step 11 — Verify voltages before connecting anything

```
[ ] PSU output at ODrive terminals: 24.0V +/- 0.5V
[ ] 5V rail at Jetson input: 5.0V +/- 0.2V
[ ] 3.3V rail at BMI088 VIN: 3.28-3.35V
[ ] Motor phase wires confirmed not touching each other
[ ] AMT21 encoder gap measured at 0.5-1.5 mm on both axes
```

Do not proceed until all five checks pass. Shorting the motor phases before ODrive calibration can destroy the gate drivers.

---

## Software Installation

Run on a Jetson Orin Nano with JetPack 6.0 (Ubuntu 22.04) and internet access:

```bash
git clone https://github.com/your-org/ground-tracker.git
cd ground-tracker
chmod +x scripts/install.sh
./scripts/install.sh
```

The installer handles everything:
- ROS 2 Humble
- Python dependencies (numpy, scipy, ultralytics, odrive, smbus2, opencv)
- YOLOv8-nano TensorRT FP16 export — takes 5-8 minutes on first run
- udev rules for ODrive USB and radar UART ports
- ROS 2 workspace build in Release mode

After the installer finishes, **log out and back in** so the `dialout` group change takes effect. Then verify:

```bash
./scripts/check_hardware.sh
```

Expected output with all hardware connected:

```
[ Camera ]
  PASS  Camera device found: /dev/video0
  PASS  Global shutter confirmed
  PASS  1280x720 resolution available

[ Radar ]
  PASS  Radar config port: /dev/ttyRadarConfig
  PASS  Radar data port:   /dev/ttyRadarData

[ ODrive S1 ]
  PASS  ODrive S1 detected on USB
  PASS  ODrive responsive: serial=3874209432
  PASS  VBus voltage: 24.0V (nominal 24V)

[ BMI088 IMU ]
  PASS  BMI088 accelerometer found at I2C 0x18
  PASS  BMI088 gyroscope found at I2C 0x68

[ Software ]
  PASS  ROS 2 available
  PASS  Ultralytics YOLOv8 importable
  PASS  TensorRT importable
  PASS  odrive package importable
  PASS  TensorRT engine file exists

Results: 14 passed  0 failed  0 warnings
System is ready to launch.
```

Full step-by-step installation is in `docs/06_software_install.md`.

---

## Calibration

Two calibration steps are required before first use. Both must be re-run if the camera or radar is physically moved relative to each other.

### Step 12 — Configure the ODrive

```bash
python3 tools/odrive_setup.py
```

The script configures motor parameters, encoder type, and initial PID gains for both axes. It then runs a full calibration sequence (resistance measurement, encoder offset calibration) and a movement test. Follow the on-screen prompts. The motors will move during calibration — ensure clear range of motion on both axes before starting.

### Step 13 — Camera intrinsic calibration

Print a 9x6 checkerboard with 25 mm squares on A3 paper and mount it flat on a rigid board.

```bash
python3 tools/calibrate_extrinsics.py
```

Hold the board at various distances (0.3-1.5 m) and orientations. Press SPACE to capture a frame when the corners are detected. Capture at least 20 frames across varied poses, then press R to compute calibration.

**Acceptable reprojection error: under 0.8 pixels.** Results are written to `config/params.yaml` automatically.

### Step 14 — Radar-camera extrinsic calibration

Place a corner reflector or 300x300 mm metal plate at 6-8 known positions in front of the tracker. For each position, measure the 3D coordinates with a tape measure, click the target center in the camera window, and press ENTER. Collect at least 8 points across different distances and angles.

Press R to solve. **Acceptable reprojection error: under 3 pixels.** Results are saved to `config/params.yaml`.

Full calibration procedures with equipment lists and expected output values are in `docs/07_calibration.md`.

---

## Tuning

Default PID gains in `tools/odrive_setup.py` are a reasonable starting point for the GBM5208 at 24V direct drive. After mechanical assembly, tune in this order — always inner loop first.

**1. Velocity loop (inner)**

Apply a square-wave velocity command and observe the response. Tune `vel_gain` until the velocity reaches setpoint in 2-3 ms with no overshoot. Start at 0.02 and increment by 0.005.

**2. Position loop (outer)**

Apply a 0.1-turn step. Tune `pos_gain` until the step response is fast with no overshoot. Start at 40, increment by 5 until overshoot appears, back off by 5.

**3. IMU feedforward**

Tap the mount firmly while watching the camera image. Increase `imu_feedforward_gain` in `params.yaml` from 0.0 to 0.8 in steps of 0.2 until vibration in the image stops improving.

| Gain | Starting value | If tracking lags | If oscillating |
|------|---------------|-----------------|----------------|
| pos_gain | 40 | Increase by 5 | Decrease by 5 |
| vel_gain | 0.022 | Increase by 0.005 | Decrease by 0.005 |
| vel_integrator_gain | 0.006 | Increase by 0.001 | Decrease by 0.001 |
| imu_feedforward_gain | 0.8 | Increase toward 1.0 | Decrease toward 0.5 |

Full tuning procedure with verification steps is in `docs/08_tuning.md`.

---

## Running the Tracker

### Pre-launch checklist

```
[ ] Jetson booted and logged in
[ ] ./scripts/check_hardware.sh shows 0 failures
[ ] ODrive powered — 24V LED solid green
[ ] Gimbal pointing to mechanical center and zeroed
[ ] Camera image live and in focus at target distance
[ ] Radar green LEDs solid (powered and configured)
[ ] params.yaml has calibrated intrinsics and extrinsics — not defaults
[ ] TensorRT engine present at config/yolov8n.engine
```

### Launch

```bash
cd software/ros2_ws
source install/setup.bash
ros2 launch tracker tracker.launch.py
```

All four nodes start: `radar_driver`, `camera_tracker`, `ekf_controller`, `gimbal_controller`.

### Monitor in real time

```bash
# EKF state — shows confidence, target speed, sensor update counts
ros2 topic echo /ekf/status

# Verify all topics are publishing at correct rates
ros2 topic hz /radar/detections     # expect ~50 Hz
ros2 topic hz /camera/target        # expect ~120 Hz
ros2 topic hz /ekf/target_pose      # expect ~200 Hz
```

### Stop

`Ctrl+C` — the gimbal controller sets both motors to IDLE before exiting.

---

## Performance

| Metric | Value |
|--------|-------|
| End-to-end latency | ~12.5 ms |
| Max sustained tracking rate | 120 deg/s |
| Peak slew rate | 300 deg/s |
| Pointing accuracy at lock | < 0.05 deg RMS |
| Radar detection range (0.01 m2 RCS) | up to 150 m |
| Camera update rate | 120 fps |
| EKF predict rate | 1 kHz |
| Gimbal position loop rate | 200 Hz |
| Total BOM cost | $1,293 USD |

### Latency breakdown

```
Radar chirp -> CFAR detection:      2.0 ms
UART transfer to Jetson:            1.0 ms
radar_driver node parse:            0.5 ms
Camera capture -> CSRT centroid:    6.0 ms
EKF predict + update:               0.5 ms
Setpoint -> ODrive velocity loop:   1.0 ms
Motor electrical step response:     2.0 ms
----------------------------------------------
Total:                            ~12.5 ms
```

The EKF prediction step compensates for this total latency by commanding the gimbal to point at where the target will be in 12.5 ms. For constant-velocity targets this makes the system effectively zero-lag in the pointing direction.

---

## Troubleshooting

### "No detections on /radar/detections"

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Frame count stuck at 0 | UART ports swapped | Swap USB-UART adapters between config and data ports |
| Frames incrementing, 0 targets | Target below threshold | Point at a car at 5-10 m to verify basic detection |
| Config errors in log | Wrong baud rate | Config port must be 115200, data port must be 921600 |
| Stationary clutter not removed | Clutter removal off | Verify `clutterRemoval -1 1` in firmware/awr1843_cfg.cfg |

### "Camera not opening / black frames"

1. Confirm device exists: `ls /dev/video*`
2. Confirm OV2710 chip ID: `v4l2-ctl --device /dev/video0 --info`
3. Verify raw stream: `ffplay /dev/video0`
4. Black frames in bright light: `v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1`

### "ODrive not found on USB"

1. Check USB cable — use a data cable, not a charge-only cable
2. Run `lsusb | grep 1209:0d32` — ODrive must appear here before proceeding
3. Check dialout group: `groups $USER` — if missing, run `sudo usermod -aG dialout $USER` and log out/in
4. Power-cycle the ODrive — it may not have initialized on first connect

### "Motor oscillates or makes grinding noise"

Velocity loop P gain is too high. In Python: `odrv.axis0.controller.config.vel_gain -= 0.005` and repeat until stable. Do not run the motor in this state — it will overheat the windings.

### "Gimbal lags behind the target"

1. Verify `system_latency_s` in `params.yaml` matches your measured round-trip latency
2. Increase `pos_gain` by 5 in the ODrive
3. Check Jetson is not thermal throttling: `tegrastats` should show maximum clocks
4. Confirm TensorRT engine is loading — look for `TensorRT engine loaded` in the camera_tracker startup log

### "EKF never initializes"

The EKF initializes from the first radar detection. If `/radar/detections` is empty, the EKF will never start. Fix the radar issue first — once detections appear, the EKF initializes automatically.

### "Track diverges or jumps suddenly"

1. Verify extrinsic calibration reprojection error was under 3 pixels — redo if needed
2. Increase `process_noise_acc` in `params.yaml` from 1.0 to 5.0 for more maneuverable targets
3. Run `ros2 topic echo /ekf/status` and confirm both radar and camera update counts are incrementing

Full troubleshooting reference for every known failure mode is in `docs/09_troubleshooting.md`.

---

## FAQ

**Q: Why use both radar and camera — can I use just one?**

Radar alone gives 3D position but its angular resolution is approximately 1.5 degrees, which is 2.6 m lateral uncertainty at 100 m. Camera alone gives sub-pixel angular accuracy but no range information and fails in low light. Together the radar provides the 3D anchor and the camera refines pointing accuracy at 120 fps. Neither sensor alone achieves the performance this system reaches.

**Q: Can I use a cheaper camera?**

Only if it is a global shutter sensor. The OV9281, AR0144, and IMX296 are acceptable alternatives. Rolling shutter sensors are unusable for fast aerial tracking — the electronic exposure smear corrupts the centroid measurement at any speed above about 20 deg/s. Adjust `frame_width`, `frame_height`, and `fps` in `params.yaml` for your chosen sensor.

**Q: Do I need a custom-trained drone detector?**

No. The default YOLOv8-nano COCO model works for initial system testing using a person or vehicle as the target. For operational drone tracking, fine-tune on a drone dataset such as Roboflow Universe "drone detection" or DUT Anti-UAV. The fine-tuned model exports to TensorRT using the same procedure in `scripts/install.sh`.

**Q: Can I add a roll stabilization axis?**

Yes. Add a third GBM5208 on the roll axis, wire it to the ODrive M1 output (move tilt to a second ODrive), and add a third axis to `gimbal_controller.py`. Roll stabilization is primarily useful if the tracker is mounted on a vehicle or vessel that pitches and rolls.

**Q: What happens when the target goes out of view?**

The EKF confidence score decreases at 0.3 per frame with no measurement. When confidence drops below 0.2 the track is dropped and the gimbal holds the last known position. The tracker re-acquires on the next valid radar or camera detection. To extend tracking through brief occlusions, increase `process_noise_acc` in `params.yaml` to allow the EKF to predict for longer without updates.

**Q: Can I run this on a Raspberry Pi instead of a Jetson?**

Not at the designed performance level. YOLOv8-nano in PyTorch on a Pi 4 runs at approximately 3-4 fps — far below the 120 fps required. The Jetson is needed for TensorRT FP16 acceleration. A Hailo-8 accelerator on a Pi 5 is a possible lower-cost alternative but requires porting the inference pipeline to the Hailo runtime.

**Q: What is the maximum detection range?**

The AWR1843 configured at 4 GHz bandwidth detects a target with 0.01 m2 radar cross-section (roughly the size of a DJI Mini) at up to approximately 150 m. A larger drone (0.1 m2 RCS) is detectable at up to 300 m. Camera lock at 150 m requires the varifocal lens set to the 50 mm position with the target subtending at least 4 pixels — approximately 0.5 m wingspan or larger.

**Q: What if I need to track indoors or in a GPS-denied environment?**

The tracker is entirely self-contained — it uses no GPS. It works indoors provided the radar has a clear line of sight to the target and the camera can see it. Clutter removal (`clutterRemoval -1 1` in the radar config) may need to be disabled indoors if wall reflections are being suppressed along with the target.

---

## Repository Structure

```
ground-tracker/
├── README.md
├── LICENSE
├── hardware/
│   ├── bom.csv                    complete bill of materials with part numbers
│   ├── wiring_diagram.md          connector-level wiring tables
│   └── mechanical_assembly.md     frame build, motor mounting, weight budget
├── docs/
│   ├── 01_system_overview.md      data flow, coordinate frames, latency budget
│   ├── 02_radar_setup.md          AWR1843 hardware and chirp configuration
│   ├── 03_camera_setup.md         OV2710 setup, focus, and frame rate
│   ├── 04_gimbal_setup.md         ODrive first connection and verification
│   ├── 05_ekf_theory.md           EKF math, state vector, Jacobian derivations
│   ├── 06_software_install.md     full installation walkthrough
│   ├── 07_calibration.md          intrinsic and extrinsic calibration
│   ├── 08_tuning.md               PID tuning procedure step by step
│   └── 09_troubleshooting.md      failure modes and fixes
├── firmware/
│   └── awr1843_cfg.cfg            production radar chirp configuration
├── software/
│   └── ros2_ws/src/tracker/
│       ├── tracker/
│       │   ├── radar_driver.py    AWR1843 UART frame parser -> ROS topic
│       │   ├── camera_tracker.py  YOLOv8 TensorRT + CSRT -> centroid topic
│       │   ├── ekf.py             9-state EKF with analytical Jacobians
│       │   ├── ekf_controller.py  ROS node wiring sensors into EKF
│       │   └── gimbal_controller.py ODrive setpoint publisher with IMU FF
│       ├── launch/
│       │   └── tracker.launch.py  single launch file for the full stack
│       └── config/
│           └── params.yaml        every tunable parameter documented
├── tools/
│   ├── odrive_setup.py            automated ODrive configuration and calibration
│   └── calibrate_extrinsics.py   interactive radar-camera calibration tool
└── scripts/
    ├── install.sh                 full dependency installer with verification
    └── check_hardware.sh          pre-launch hardware check with pass/fail output
```

---

## Contributing

Pull requests are welcome. If you build this, open an issue with photos and your measured performance numbers — environment, mounting surface, target type, and observed latency. It helps everyone calibrate expectations for different deployment conditions.

## License

MIT License — see `LICENSE`. Build freely.
