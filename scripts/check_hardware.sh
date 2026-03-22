#!/usr/bin/env bash
# check_hardware.sh
# Pre-flight hardware verification script.
# Run before every session to confirm all devices are present and responsive.

set -uo pipefail

PASS=0; FAIL=0; WARN=0

green="\033[0;32m"; red="\033[0;31m"; yellow="\033[0;33m"; reset="\033[0m"
pass()  { echo -e "${green}  PASS${reset}  $*"; ((PASS++)); }
fail()  { echo -e "${red}  FAIL${reset}  $*"; ((FAIL++)); }
warn()  { echo -e "${yellow}  WARN${reset}  $*"; ((WARN++)); }

echo ""
echo "Ground Tracker — Hardware Check"
echo "================================"
echo ""

# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
echo "[ Camera ]"
if ls /dev/video* &>/dev/null; then
    DEV=$(ls /dev/video* | head -1)
    pass "Camera device found: $DEV"
    # Check that global shutter flag is readable (best-effort)
    if v4l2-ctl -d "$DEV" --info 2>/dev/null | grep -qi "global"; then
        pass "Global shutter confirmed"
    else
        warn "Could not confirm global shutter mode — verify OV2710 camera is connected (not a rolling-shutter substitute)"
    fi
    # Check resolution
    if v4l2-ctl -d "$DEV" --list-formats-ext 2>/dev/null | grep -q "1280x720"; then
        pass "1280x720 resolution available"
    else
        warn "1280x720 not listed — check camera capabilities"
    fi
else
    fail "No camera device found at /dev/video*"
fi

echo ""

# ---------------------------------------------------------------------------
# Radar (UART ports)
# ---------------------------------------------------------------------------
echo "[ Radar ]"
if ls /dev/ttyRadarConfig &>/dev/null; then
    pass "Radar config port: /dev/ttyRadarConfig"
else
    if ls /dev/ttyUSB0 &>/dev/null; then
        warn "Radar udev symlink not found but /dev/ttyUSB0 exists — udev rules may need reload"
        warn "Run: sudo udevadm control --reload-rules && sudo udevadm trigger"
    else
        fail "Radar config port not found (/dev/ttyRadarConfig or /dev/ttyUSB0)"
    fi
fi

if ls /dev/ttyRadarData &>/dev/null; then
    pass "Radar data port: /dev/ttyRadarData"
else
    if ls /dev/ttyUSB1 &>/dev/null; then
        warn "Radar udev symlink not found but /dev/ttyUSB1 exists"
    else
        fail "Radar data port not found (/dev/ttyRadarData or /dev/ttyUSB1)"
    fi
fi

echo ""

# ---------------------------------------------------------------------------
# ODrive (USB)
# ---------------------------------------------------------------------------
echo "[ ODrive S1 ]"
if lsusb 2>/dev/null | grep -q "1209:0d32"; then
    pass "ODrive S1 detected on USB"
    if python3 -c "import odrive; o = odrive.find_any(timeout=5); print(o.serial_number)" &>/dev/null; then
        SERIAL=$(python3 -c "import odrive; o = odrive.find_any(timeout=5); print(o.serial_number)" 2>/dev/null)
        pass "ODrive responsive: serial=$SERIAL"
        VBUS=$(python3 -c "import odrive; o = odrive.find_any(timeout=5); print(f'{o.vbus_voltage:.1f}')" 2>/dev/null)
        if python3 -c "import odrive; o = odrive.find_any(timeout=5); v=o.vbus_voltage; exit(0 if 22<v<26 else 1)" &>/dev/null; then
            pass "VBus voltage: ${VBUS}V (nominal 24V)"
        else
            warn "VBus voltage: ${VBUS}V — expected 22-26V. Check power supply."
        fi
    else
        fail "ODrive found on USB but not responding — check permissions or reconnect"
    fi
else
    fail "ODrive S1 not found on USB (VID:PID 1209:0d32)"
fi

echo ""

# ---------------------------------------------------------------------------
# IMU (I2C)
# ---------------------------------------------------------------------------
echo "[ BMI088 IMU ]"
if command -v i2cdetect &>/dev/null; then
    ACCEL_ADDR="18"
    GYRO_ADDR="68"
    if i2cdetect -y -r 1 2>/dev/null | grep -q "$ACCEL_ADDR"; then
        pass "BMI088 accelerometer found at I2C 0x${ACCEL_ADDR}"
    else
        fail "BMI088 accelerometer NOT found at 0x${ACCEL_ADDR} on I2C bus 1"
    fi
    if i2cdetect -y -r 1 2>/dev/null | grep -q "$GYRO_ADDR"; then
        pass "BMI088 gyroscope found at I2C 0x${GYRO_ADDR}"
    else
        fail "BMI088 gyroscope NOT found at 0x${GYRO_ADDR} on I2C bus 1"
    fi
else
    warn "i2cdetect not installed — skipping IMU check (run: sudo apt install i2c-tools)"
fi

echo ""

# ---------------------------------------------------------------------------
# Software environment
# ---------------------------------------------------------------------------
echo "[ Software ]"
command -v ros2 &>/dev/null && pass "ROS 2 available" || fail "ROS 2 not found — run scripts/install.sh"
python3 -c "from ultralytics import YOLO" &>/dev/null && pass "Ultralytics YOLOv8 importable" || fail "Ultralytics not installed"
python3 -c "import tensorrt" &>/dev/null && pass "TensorRT importable" || warn "TensorRT not importable — inference will fall back to PyTorch (slower)"
python3 -c "import odrive" &>/dev/null && pass "odrive package importable" || fail "odrive package not installed"
python3 -c "import cv2" &>/dev/null && pass "OpenCV importable" || fail "OpenCV not installed"

ENGINE="$(dirname "$0")/../software/ros2_ws/src/tracker/config/yolov8n.engine"
[[ -f "$ENGINE" ]] && pass "TensorRT engine file exists" || warn "TensorRT engine not found at config/yolov8n.engine — run scripts/install.sh"

PARAMS="$(dirname "$0")/../software/ros2_ws/src/tracker/config/params.yaml"
[[ -f "$PARAMS" ]] && pass "params.yaml exists" || fail "params.yaml missing"

if [[ -f "$PARAMS" ]]; then
    python3 - <<PYEOF
import yaml, sys
with open("$PARAMS") as f:
    p = yaml.safe_load(f)
ext = p.get("extrinsics", {})
R = ext.get("R_radar_to_camera", [1,0,0,0,1,0,0,0,1])
t = ext.get("t_radar_to_camera", [0,0,0])
is_identity_R = (R == [1,0,0,0,1,0,0,0,1])
is_zero_t = (t == [0,0,0])
if is_identity_R and is_zero_t:
    print("  WARN  Extrinsics appear to be defaults — run tools/calibrate_extrinsics.py")
    sys.exit(2)
else:
    print("  PASS  Extrinsics calibrated")
PYEOF
fi

echo ""

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
TOTAL=$((PASS + FAIL + WARN))
echo "================================"
echo -e "Results: ${green}$PASS passed${reset}  ${red}$FAIL failed${reset}  ${yellow}$WARN warnings${reset}  (${TOTAL} checks)"
echo ""

if [[ $FAIL -gt 0 ]]; then
    echo "System is NOT ready. Fix failures before launching."
    exit 1
elif [[ $WARN -gt 0 ]]; then
    echo "System has warnings. Review before field deployment."
    exit 0
else
    echo "All checks passed. System is ready to launch."
    echo "Run: ros2 launch tracker tracker.launch.py"
    exit 0
fi
