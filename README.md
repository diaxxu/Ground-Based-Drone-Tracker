# Ground-Based Drone Tracker

A sub-$1,300 open-source ground-based tracking system that uses radar and a global-shutter camera to detect and follow aerial targets with sub-50 ms end-to-end latency. Built around a TI AWR1843 77 GHz radar, an OV2710 global-shutter camera, a Jetson Orin Nano compute module, and a direct-drive brushless gimbal controlled by an ODrive S1.

This repository contains everything required to reproduce the system from scratch: bill of materials, mechanical drawings, wiring diagrams, ROS 2 software stack, calibration procedures, and tuning guides.

---

## Performance Targets

| Metric | Value |
|---|---|
| End-to-end latency | ~12.5 ms |
| Max sustained tracking rate | 120 deg/s |
| Peak slew rate | 300 deg/s |
| Pointing accuracy at lock | < 0.05 deg RMS |
| Detection range (0.01 m2 RCS) | up to 150 m |
| Camera update rate | 120 fps |
| Radar update rate | 50 Hz |
| EKF predict rate | 1 kHz |
| Gimbal position loop | 200 Hz |
| Gimbal velocity loop | 1 kHz |
| Total BOM cost | ~$1,293 USD |

---

## Repository Structure

```
ground-tracker/
├── README.md
├── LICENSE
├── hardware/
│   ├── bom.csv                  # Full bill of materials with part numbers
│   ├── wiring_diagram.md        # Connector-level wiring instructions
│   └── mechanical_assembly.md   # Frame build and motor mounting
├── docs/
│   ├── 01_system_overview.md
│   ├── 02_radar_setup.md
│   ├── 03_camera_setup.md
│   ├── 04_gimbal_setup.md
│   ├── 05_ekf_theory.md
│   ├── 06_software_install.md
│   ├── 07_calibration.md
│   ├── 08_tuning.md
│   └── 09_troubleshooting.md
├── software/
│   └── ros2_ws/
│       └── src/
│           └── tracker/
│               ├── package.xml
│               ├── setup.py
│               ├── tracker/
│               │   ├── __init__.py
│               │   ├── radar_driver.py
│               │   ├── camera_tracker.py
│               │   ├── ekf.py
│               │   └── gimbal_controller.py
│               ├── launch/
│               │   └── tracker.launch.py
│               └── config/
│                   └── params.yaml
├── firmware/
│   └── awr1843_cfg.cfg          # TI mmWave radar chirp configuration
├── tools/
│   ├── calibrate_extrinsics.py  # Radar-camera extrinsic calibration
│   └── odrive_setup.py          # ODrive S1 first-time configuration
└── scripts/
    ├── install.sh               # Full dependency installer
    └── check_hardware.sh        # Pre-flight hardware check
```

---

## Quick Start

If you have all hardware assembled and wired, the following five steps bring the system to a running state. Detailed instructions for each step are in the `docs/` folder.

**Step 1 — Clone the repository**
```bash
git clone https://github.com/your-org/ground-tracker.git
cd ground-tracker
```

**Step 2 — Run the installer**
```bash
chmod +x scripts/install.sh
./scripts/install.sh
```
This installs ROS 2 Humble, Python dependencies, TensorRT, and the ODrive Python package. Estimated time: 20 minutes on a fresh Jetson Orin Nano.

**Step 3 — Configure the ODrive**
```bash
python3 tools/odrive_setup.py
```
Follow the prompts. The script sets motor parameters, encoder type, and PID gains for both axes.

**Step 4 — Run extrinsic calibration**
```bash
python3 tools/calibrate_extrinsics.py
```
Point the system at a calibration target (a corner reflector or flat plate at a known range). The script outputs a 4x4 transform matrix and writes it to `software/ros2_ws/src/tracker/config/params.yaml`.

**Step 5 — Launch the tracker**
```bash
cd software/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch tracker tracker.launch.py
```

---

## Hardware Required

See `hardware/bom.csv` for the full list with supplier links and part numbers. Key components:

- TI AWR1843BOOST radar evaluation board
- ELP OV2710 1080p global-shutter USB camera
- NVIDIA Jetson Orin Nano 8 GB Developer Kit
- iPower GBM5208-200T brushless motors x2
- ODrive S1 motor controller
- AMT21 absolute magnetic encoders x2
- Bosch BMI088 IMU breakout board
- 24 V 10 A switching power supply
- Aluminum extrusion frame (30x30 mm profile)

---

## License

MIT License. See `LICENSE` for details.

---

## Contributing

Pull requests welcome. Please open an issue before starting significant work so efforts are not duplicated.
