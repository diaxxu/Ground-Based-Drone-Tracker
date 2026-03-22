# Calibration Procedures

Two calibration steps are required before first use. Both must be re-run if the camera or radar is physically moved relative to each other.

---

## 1. Camera Intrinsic Calibration

Camera intrinsic calibration determines the focal length and principal point of the camera. These values are used by the EKF camera measurement model to project 3D world coordinates into pixel coordinates.

### Equipment needed

- Printed checkerboard calibration pattern: 9x6 inner corners, 25 mm square size. Print on A3 paper, mount flat on a rigid board.
- The connected camera, running via the camera_tracker node or directly via `calibrate_extrinsics.py`.

### Procedure

Run the calibration tool:
```bash
python3 tools/calibrate_extrinsics.py
```

When the intrinsic calibration window opens:
1. Hold the checkerboard at various orientations and distances (0.3 m to 1.5 m).
2. Tilt the board left, right, up, down, and diagonally. The more varied the poses the better.
3. When the checkerboard is detected (corners drawn in color), press SPACE to capture.
4. Capture at least 20 frames across varied poses.
5. Press R to run calibration.

Acceptable reprojection error: under 0.8 pixels. If error exceeds 1.0 pixels, discard frames with the highest individual errors and recalibrate.

The calibrated K matrix and distortion coefficients are automatically written to `config/params.yaml`.

### Focal length sanity check

For the 25 mm varifocal lens set at approximately 25 mm, the expected focal length in pixels for a 1280x720 sensor is:

```
fx = focal_length_mm / pixel_size_um * 1000
   = 25 / 2.8 * 1000
   ≈ 893 px
```

If your calibrated `fx` is less than 700 or more than 1200, check that the lens is not set to a different zoom position and that the correct camera is selected.

---

## 2. Radar-Camera Extrinsic Calibration

Extrinsic calibration determines the rigid-body transform (rotation R and translation t) from the radar coordinate frame to the camera coordinate frame. Without this, the EKF cannot project radar detections into the camera image for the camera measurement update.

### Equipment needed

- A corner reflector target or a flat metal plate at least 300x300 mm (both give strong radar return).
- A visible center marker on the target: a 50 mm colored dot or a printed ArUco marker centered on the reflector.
- A tape measure.
- A helper to move the target, or a tripod to position it.

### Procedure

**Step 1: Launch radar and camera nodes only**

Temporarily comment out `ekf` and `gimbal` from the launch file, then:
```bash
ros2 launch tracker tracker.launch.py
```

**Step 2: Run the calibration tool**
```bash
python3 tools/calibrate_extrinsics.py
```

Press Q on the intrinsic calibration window to skip it (assuming intrinsics are already calibrated).

**Step 3: Collect calibration points**

For each calibration point:
1. Place the corner reflector at a known position relative to the radar center. Measure x (forward), y (left), z (up) in meters with a tape measure. Typical positions: 5m straight ahead, 8m at 30 degrees left, 6m at 20 degrees right, 4m elevated on a 1m stand, and so on. Aim for at least 8 points covering the central tracking volume.
2. Verify the radar is detecting the target: `ros2 topic echo /radar/detections` should show a point near your measured position.
3. Left-click the center of the visible marker in the calibration window.
4. Press ENTER and type the three measured coordinates (x y z in meters).

**Step 4: Solve and save**

After at least 6 points (8+ recommended), press R. The tool runs solvePnPRansac and reports:
- Reprojection error: should be under 3 pixels. Above 5 pixels indicates a measurement error or a bad calibration point.
- The solved R (rotation matrix) and t (translation vector).

Results are written automatically to `config/params.yaml`.

### Expected transform

For a co-boresighted camera and radar (mounted within 30mm of each other), the expected translation vector is small:
```
t ≈ [0.02, -0.03, 0.01]   # 2 cm forward, 3 cm right, 1 cm up (example)
```

The rotation matrix R should be close to identity if the camera and radar are aligned to the same boresight. Large off-diagonal values indicate physical misalignment and the mounts should be adjusted.

---

## 3. Gimbal Zero Position

After ODrive setup and before first tracking use, set the gimbal to its mechanical center and zero the encoder reference:

```bash
python3 - <<'EOF'
import odrive
odrv = odrive.find_any()
# Set home position to current encoder position
odrv.axis0.encoder.set_linear_count(0)
odrv.axis1.encoder.set_linear_count(0)
odrv.save_configuration()
print("Gimbal zeroed at current position")
EOF
```

Ensure the gimbal is pointing exactly forward (boresighted with the radar and camera) before running this command.

---

## Re-calibration Schedule

| Trigger | Intrinsics | Extrinsics | Gimbal zero |
|---|---|---|---|
| First build | Required | Required | Required |
| Camera physically moved | Optional | Required | Not needed |
| Lens zoom changed | Required | Required | Not needed |
| Radar physically moved | Not needed | Required | Not needed |
| Camera or radar replaced | Required | Required | Not needed |
| Gimbal motor or encoder replaced | Not needed | Not needed | Required |
| After transport or shipping | Optional | Recommended | Recommended |
