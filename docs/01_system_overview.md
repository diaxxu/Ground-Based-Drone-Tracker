# System Overview

## Purpose

This system is an open-source, reproducible ground-based tracker for aerial targets (drones, balloons, aircraft). It uses a 77 GHz FMCW radar for wide-angle detection and initial localization, a global-shutter camera with a machine-learning detector for precision centroid lock, an Extended Kalman Filter to fuse both sensors into a smooth predictive track, and a direct-drive brushless gimbal to physically aim the camera-radar payload at the target.

The design prioritizes:
- Minimum end-to-end latency (~12 ms, sensor to motor)
- Maximum tracking rate (up to 120 deg/s sustained)
- Sub-$2,000 off-the-shelf bill of materials
- Full reproducibility from this repository alone

---

## Sensor Roles

The radar and camera play complementary roles and neither is sufficient alone.

The radar provides 3D position (range, azimuth, elevation) and radial velocity at 50 Hz. Its angular resolution is approximately 1.5 degrees, which at 100 m range translates to 2.6 m lateral position uncertainty. This is sufficient for acquisition and initial pointing, but not for precision lock. The radar works in all lighting conditions and is unaffected by optical obscuration.

The camera provides a 2D pixel centroid of the detected target at up to 120 Hz with sub-pixel accuracy. At 100 m range with a 25 mm lens, one pixel subtends approximately 0.016 degrees (2.8 cm at target). The camera is sensitive to lighting and loses the target when obscured. It has no direct range information.

The EKF fuses both measurements. The radar anchors the 3D position estimate. The camera refines the angular accuracy and updates at higher rate. Together they produce a smooth, high-rate 3D track that is more accurate than either sensor alone.

---

## Coordinate Frames

Three coordinate frames are used in the system:

**Radar frame:** Origin at the radar phase center. x = forward (boresight), y = left, z = up. All radar measurements are expressed in this frame.

**Camera frame:** Origin at the camera optical center. x = right, y = down, z = forward. Pixel measurements are related to world position through the intrinsic matrix K and the extrinsic transform R, t.

**World frame:** Fixed to the ground at the tracker base. Aligned with the radar frame at system startup. The EKF state vector is expressed in this frame.

The extrinsic calibration (step 2 of calibration) provides the transform from radar frame to camera frame. The EKF uses this to project its state estimate (in world/radar frame) into camera pixel coordinates for the camera measurement update.

---

## Data Flow

```
AWR1843 radar
    |  UART 921600 baud
    v
radar_driver node
    |  /radar/detections  (PoseArray, 50 Hz)
    v
ekf_controller node  <---  /camera/target  (PointStamped, 120 Hz)
    |                           ^
    |                           |
    |                    camera_tracker node
    |                           ^
    |                           |
    |                    OV2710 camera (USB 3.0)
    |
    |  /ekf/target_pose  (PointStamped, az/el, 200 Hz)
    v
gimbal_controller node
    |  USB-C (ODrive protocol, 200 Hz)
    v
ODrive S1
    |  3-phase FOC PWM
    v
GBM5208 motors (pan + tilt)
    |  AMT21 encoder feedback (SPI, 16-bit, continuous)
    |  BMI088 IMU feedforward (I2C, 400 Hz)
    v
Gimbal (points camera + radar at target)
```

---

## Latency Budget

The end-to-end latency from a target moving to the gimbal beginning to respond is approximately 12.5 ms, broken down as:

| Stage | Latency | Notes |
|---|---|---|
| Radar frame acquisition | 20 ms period | CFAR runs on-chip DSP |
| UART transfer to Jetson | 1 ms | 921600 baud, ~2 KB frame |
| radar_driver parse | 0.5 ms | Struct unpack in Python |
| Camera frame capture | 8.3 ms period | 120 fps = 8.3 ms/frame |
| YOLOv8-nano TensorRT | 5.5 ms | On Orin Nano, FP16 |
| CSRT tracker | 0.8 ms | OpenCV C++ backend |
| EKF predict step | 0.1 ms | 9x9 matrix ops in NumPy |
| EKF update step | 0.5 ms | Both sensors async |
| Setpoint publish | 0.1 ms | ROS 2 shared memory |
| ODrive position loop | 1 ms | 1 kHz inner loop |
| Motor electrical step response | 2 ms | L/R time constant |
| **Total** | **~12.5 ms** | |

The EKF prediction step compensates for this latency by forecasting the target position 12.5 ms into the future before computing the gimbal setpoint. This makes the system effectively zero-lag in the pointing direction for constant-velocity targets.

---

## Dataset References for Custom Drone Detector

The default YOLOv8-nano model is trained on COCO and detects people, vehicles, and animals, which is useful for initial system testing. For deployment against drones, fine-tune on a drone-specific dataset:

- Roboflow Universe: search "drone detection" — multiple datasets with 5,000+ annotated drone images in various backgrounds.
- DUT Anti-UAV: publicly released by Dalian University of Technology, contains infrared and RGB drone footage with bounding box annotations.
- MAV-VID: multi-altitude drone video dataset from ETH Zurich.

Export the fine-tuned model to TensorRT following the same procedure as in `scripts/install.sh`. The engine file path is set in `config/params.yaml` under `engine_path`.
