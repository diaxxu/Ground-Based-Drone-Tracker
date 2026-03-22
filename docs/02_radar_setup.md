# Radar Setup

## Hardware Preparation

The AWR1843BOOST is a plug-and-play evaluation board. No soldering is required.

1. Connect the USB Micro-B cable from the AWR1843BOOST to the Jetson. This powers the board and exposes the two UART ports.
2. Verify two UART devices appear: `ls /dev/ttyUSB*` should show `/dev/ttyUSB0` and `/dev/ttyUSB1`.
3. If udev rules are installed (from `scripts/install.sh`), symlinks `/dev/ttyRadarConfig` and `/dev/ttyRadarData` will also appear.

## Antenna Orientation

Mount the AWR1843BOOST with the antenna array facing the target area. The board silkscreen shows the transmit and receive antenna arrays. The boresight (maximum gain direction) is perpendicular to the flat face of the board. Mount it level — the elevation measurement assumes the array is horizontal.

## Chirp Configuration

The file `firmware/awr1843_cfg.cfg` contains the full chirp configuration. Key parameters:

| Parameter | Value | Effect |
|---|---|---|
| Start frequency | 77 GHz | Regulatory band |
| Bandwidth | 4 GHz | 4 cm range resolution |
| Frame rate | 50 Hz | 20 ms update period |
| Max range | ~200 m | For 0.01 m2 RCS target |
| Max velocity | +/-28 m/s | ~100 km/h |
| FOV (azimuth) | +/-60 degrees | Hardware limited |
| FOV (elevation) | +/-15 degrees | Hardware limited |

The clutter removal command (`clutterRemoval -1 1`) suppresses stationary returns. Disable this (change to 0) only if tracking a hovering drone directly in front of the radar — a stationary target will be filtered out otherwise.

## Verifying Operation

After launching the radar driver node:

```bash
ros2 topic echo /radar/detections
ros2 topic hz /radar/detections
```

Expected topic rate: 50 Hz. Walk in front of the radar at 5 m distance — you should see detections appear on `/radar/detections`.
