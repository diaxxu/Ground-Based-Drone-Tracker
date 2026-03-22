# Troubleshooting

---

## Radar

**No detections on /radar/detections**

1. Check both UART ports are connected and at the correct baud rates (config: 115200, data: 921600).
2. Verify the radar is powered — the green LED on the AWR1843BOOST should be solid.
3. Confirm the config file was sent successfully. Look for `Done` responses in the config UART terminal.
4. Run `ros2 topic echo /radar/status` — if frame count is incrementing but detections are zero, the radar is running but no targets are being detected. Point the radar at a large metal object (car, wall) at 5-10 m distance to verify detection.
5. If the frame count is zero, the data port is not receiving frames. Swap the USB-UART adapters — the config and data ports may be reversed.

**Radar detects stationary objects instead of moving drone**

The chirp configuration has clutter removal enabled (`clutterRemoval -1 1`). If stationary clutter is still appearing, verify the command was accepted by the radar. Check `ros2 topic echo /radar/status` — the status string should not show errors.

**Radar detections are offset from the true target position**

Run `measureRangeBiasAndRxChanPhase` with a corner reflector at a known distance to calibrate the range bias. The output goes into the `compRangeBiasAndRxChanPhase` line in `firmware/awr1843_cfg.cfg`. Update this line and restart the radar driver node.

---

## Camera

**Camera not opening / black frames**

1. Verify the camera appears in `ls /dev/video*`.
2. Run `v4l2-ctl --device /dev/video0 --info` and check that the OV2710 chip ID appears.
3. Try `ffplay /dev/video0` to confirm the raw video stream is working outside of ROS.
4. If frames are black only in bright light, the auto-exposure is likely not converging. Set manual exposure: `v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1 --set-ctrl=exposure_absolute=500`.

**YOLO finds no detections**

The default model is trained on COCO and was not specifically trained on drones. For testing, set `target_class_ids: ""` in `params.yaml` to detect all classes and wave your hand in front of the camera. If that works, the pipeline is functional and you need a drone-specific model. Fine-tune YOLOv8-nano on a drone dataset (see `docs/01_system_overview.md` for dataset links).

**CSRT tracker loses target and does not recover**

CSRT is initialized from YOLO and recovers only when YOLO re-detects the target. If the target disappears from the frame entirely, re-detection depends on the YOLO confidence threshold. Lower `confidence_threshold` from 0.35 to 0.25 to improve re-acquisition sensitivity at the cost of more false positives.

**High latency on camera frames**

The camera buffer is set to 1 frame (`CAP_PROP_BUFFERSIZE = 1`). If the OS is buffering more frames, force the V4L2 backend: confirm `cv2.CAP_V4L2` is being passed to `VideoCapture`. On the Jetson, also confirm that the Jetson multimedia API (nvarguscamerasrc) is not intercepting the camera; USB cameras should always appear at `/dev/video*` and use the V4L2 path.

---

## ODrive / Gimbal

**ODrive not found on USB**

1. Check USB cable (some cables are charge-only — use a data cable).
2. Run `lsusb | grep 1209:0d32`. If not found, the ODrive is not powered or the USB is not initialised. Cycle power on the ODrive.
3. Check that the user is in the `dialout` group: `groups $USER`. If not, run `sudo usermod -aG dialout $USER` and log out/in.

**Motor does not move during calibration**

1. Check that 24V is present on the VBus terminals with a multimeter.
2. Check that the motor phase wires are connected and none are broken.
3. Run `odrv.axis0.motor.error` in Python. Common errors: `PHASE_RESISTANCE_OUT_OF_RANGE` means the resistance calibration failed — check phase wiring. `CURRENT_LIMIT_VIOLATION` means current spiked above limit — the phase wires may be shorted.

**Motor oscillates or makes grinding noise**

This is a PID gain too high. Reduce `vel_gain` by 0.005 increments in the ODrive Python interface until oscillation stops. See `docs/08_tuning.md` for the full tuning procedure.

**Gimbal moves to wrong position / drifts**

1. Verify the AMT21 encoder magnets are within 1.5 mm of the encoder face. Excessive gap causes missed counts.
2. Run `odrv.axis0.encoder.pos_estimate` and manually rotate the motor by one full turn. The estimate should change by exactly 1.0 (one turn = 1.0 in ODrive units). If not, the CPR setting is wrong or the encoder is not communicating — check SPI wiring.
3. Check that the gimbal was zeroed at mechanical center before the first run.

**Gimbal lags behind the target**

1. Increase `pos_gain` in ODrive.
2. Increase `input_filter_bandwidth` to 300 Hz.
3. Verify `system_latency_s` in `params.yaml` is set to the measured round-trip latency. Run `python3 tools/measure_latency.py` (if implemented) or use an oscilloscope.
4. Ensure the Jetson is not thermally throttling: `tegrastats` should show clock speeds at maximum.

---

## EKF

**Track diverges or jumps suddenly**

1. Check that the extrinsic calibration reprojection error was under 3 pixels.
2. If the radar and camera detections are contradicting each other (radar says target is at 50 m, camera says it is at 10 m), the extrinsic calibration needs to be redone.
3. Increase `process_noise_acc` in `params.yaml` (try 5.0) to allow the filter to track more aggressive maneuvers.

**EKF never initializes (confidence stays at 0)**

The EKF initializes from the first radar detection. If `/radar/detections` is empty, see the radar section above. Run `ros2 topic echo /ekf/status` to see the current state.

**Track is very noisy / jittery**

Reduce `process_noise_acc` (try 0.1) to smooth the state estimate. This will make the filter less responsive to fast maneuvers but reduce noise at steady state. Alternatively, increase `radar_noise_range` and `radar_noise_angle` to tell the filter to trust radar measurements less.

---

## Jetson / System

**System runs slow / high CPU**

1. Enable maximum performance mode: `sudo nvpmodel -m 0 && sudo jetson_clocks`.
2. Confirm TensorRT engine is being used (not PyTorch fallback): look for `TensorRT engine loaded` in the camera_tracker node output.
3. Reduce `yolo_every_n_frames` to 10 if needed — CSRT is much lighter than YOLO.

**ROS 2 nodes crash on startup**

Check that the workspace was built in Release mode: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`. Debug builds are significantly slower and may miss real-time deadlines.

**Out of memory during TensorRT export**

The YOLOv8-nano TensorRT export needs approximately 5 GB of RAM. Close all other applications during export. If the Jetson only has 8 GB total and the OS uses 3 GB, headroom is tight — set a swap file if needed:
```bash
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```
