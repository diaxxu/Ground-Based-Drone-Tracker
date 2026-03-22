# Camera Setup

## Hardware Preparation

1. Attach the CS-mount varifocal lens to the camera body. Thread clockwise until finger-tight.
2. Set the zoom ring to approximately the 25 mm position (the wider end of the zoom range).
3. Connect via USB 3.0 (blue port) to the Jetson Orin Nano.

## Focus and Zoom

Set focus and zoom for your expected tracking distance:

- For targets at 30-100 m: set zoom to 25 mm. Focus at 50 m.
- For targets at 50-200 m: set zoom to 35-50 mm. Focus at 100 m.

Focus by pointing at a target at the desired distance, opening the camera with `ffplay /dev/video0`, and adjusting the focus ring until the image is sharp. Tighten the lock screws on both rings once set.

## Verify Global Shutter

This is critical. Confirm the camera is the OV2710 (global shutter) and not an OV2735 or similar rolling-shutter substitute:

```bash
v4l2-ctl --device /dev/video0 --info | grep -i "driver\|card\|chip"
```

A rolling shutter will cause fast-moving targets to appear diagonally smeared, which makes centroid detection unreliable at high speeds. If in doubt, wave a fast-moving object (e.g. a spinning propeller) in front of the camera and check for diagonal tearing in the image.

## Frame Rate

Verify 120 fps is available:

```bash
v4l2-ctl --device /dev/video0 --list-formats-ext | grep -A5 "YUYV\|MJPG"
```

At 1280x720, the OV2710 supports 120 fps in YUYV format. At 1920x1080, maximum is 60 fps. The tracker uses 1280x720/120fps by default for minimum latency.
