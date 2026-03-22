#!/usr/bin/env bash
# install.sh
# Full dependency installer for the ground tracker on Jetson Orin Nano (JetPack 6 / Ubuntu 22.04)
# Run as a non-root user with sudo access.
# Estimated time: 20-30 minutes on a fresh system.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_FILE="/tmp/tracker_install.log"

log() { echo "[$(date '+%H:%M:%S')] $*" | tee -a "$LOG_FILE"; }
die() { echo "FATAL: $*" >&2; exit 1; }

log "Starting ground tracker installation"
log "Repo root: $REPO_ROOT"
log "Log file:  $LOG_FILE"

# ---------------------------------------------------------------------------
# 0. Sanity checks
# ---------------------------------------------------------------------------
[[ "$(uname -m)" == "aarch64" ]] || log "WARNING: not running on aarch64 — Jetson-specific steps may fail"

if ! python3 -c "import tensorrt" &>/dev/null; then
    log "WARNING: TensorRT not found. Ensure JetPack 6 is correctly installed."
    log "         Run: sudo apt install tensorrt"
fi

# ---------------------------------------------------------------------------
# 1. System packages
# ---------------------------------------------------------------------------
log "Step 1/7 — System packages"
sudo apt-get update -qq
sudo apt-get install -y \
    curl git cmake build-essential \
    python3-pip python3-dev python3-venv \
    libopencv-dev python3-opencv \
    i2c-tools libi2c-dev \
    libserial-dev \
    v4l-utils \
    ros-dev-tools \
    2>>"$LOG_FILE"

# ---------------------------------------------------------------------------
# 2. ROS 2 Humble
# ---------------------------------------------------------------------------
log "Step 2/7 — ROS 2 Humble"
if ! command -v ros2 &>/dev/null; then
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt-get update -qq
    sudo apt-get install -y ros-humble-desktop python3-colcon-common-extensions
    log "ROS 2 Humble installed"
else
    log "ROS 2 already installed — skipping"
fi

# Add ROS 2 to .bashrc if not present
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

# ---------------------------------------------------------------------------
# 3. Python dependencies
# ---------------------------------------------------------------------------
log "Step 3/7 — Python packages"
pip3 install --upgrade pip --quiet
pip3 install \
    numpy \
    scipy \
    opencv-python \
    pyserial \
    odrive \
    smbus2 \
    transforms3d \
    matplotlib \
    pyyaml \
    cv-bridge \
    2>>"$LOG_FILE"

# ---------------------------------------------------------------------------
# 4. Ultralytics YOLOv8
# ---------------------------------------------------------------------------
log "Step 4/7 — Ultralytics YOLOv8"
pip3 install ultralytics --quiet 2>>"$LOG_FILE"

# Export to TensorRT FP16 if engine does not already exist
ENGINE_PATH="$REPO_ROOT/software/ros2_ws/src/tracker/config/yolov8n.engine"
if [[ ! -f "$ENGINE_PATH" ]]; then
    log "Exporting YOLOv8-nano to TensorRT FP16 (this takes 5-8 minutes)..."
    python3 - <<'PYEOF'
import os, sys
from ultralytics import YOLO
engine_path = os.environ.get("ENGINE_PATH", "yolov8n.engine")
model = YOLO("yolov8n.pt")
model.export(format="engine", half=True, imgsz=640, device=0)
import shutil
if os.path.exists("yolov8n.engine"):
    shutil.move("yolov8n.engine", engine_path)
    print(f"Engine saved to {engine_path}")
PYEOF
    export ENGINE_PATH
else
    log "TensorRT engine already exists at $ENGINE_PATH — skipping export"
fi

# ---------------------------------------------------------------------------
# 5. udev rules
# ---------------------------------------------------------------------------
log "Step 5/7 — udev rules"

# ODrive S1
cat <<'UDEV' | sudo tee /etc/udev/rules.d/91-odrive.rules >/dev/null
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d32", MODE="0666", GROUP="dialout"
UDEV

# AWR1843 UART ports
cat <<'UDEV' | sudo tee /etc/udev/rules.d/99-radar.rules >/dev/null
SUBSYSTEM=="tty", ATTRS{idVendor}=="0451", ATTRS{idProduct}=="bef3", SYMLINK+="ttyRadarConfig", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0451", ATTRS{idProduct}=="bef4", SYMLINK+="ttyRadarData",   MODE="0666"
UDEV

sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
log "udev rules installed. You must log out and back in for group changes to take effect."

# ---------------------------------------------------------------------------
# 6. Build ROS 2 workspace
# ---------------------------------------------------------------------------
log "Step 6/7 — Building ROS 2 workspace"
cd "$REPO_ROOT/software/ros2_ws"
rosdep init 2>/dev/null || true
rosdep update --quiet 2>>"$LOG_FILE"
rosdep install --from-paths src --ignore-src -r -y --quiet 2>>"$LOG_FILE"
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>>"$LOG_FILE"

# Add workspace to .bashrc
WS_SOURCE="source $REPO_ROOT/software/ros2_ws/install/setup.bash"
if ! grep -qF "$WS_SOURCE" ~/.bashrc; then
    echo "$WS_SOURCE" >> ~/.bashrc
fi

# ---------------------------------------------------------------------------
# 7. Verify installation
# ---------------------------------------------------------------------------
log "Step 7/7 — Verification"
source "$REPO_ROOT/software/ros2_ws/install/setup.bash"

PASS=0; FAIL=0
check() {
    if eval "$2" &>/dev/null; then
        log "  PASS  $1"
        ((PASS++))
    else
        log "  FAIL  $1"
        ((FAIL++))
    fi
}

check "ros2 available"           "command -v ros2"
check "tracker package built"    "ros2 pkg list | grep -q tracker"
check "odrive pip package"       "python3 -c 'import odrive'"
check "ultralytics"              "python3 -c 'from ultralytics import YOLO'"
check "opencv"                   "python3 -c 'import cv2'"
check "numpy"                    "python3 -c 'import numpy'"
check "scipy"                    "python3 -c 'import scipy'"
check "pyserial"                 "python3 -c 'import serial'"
check "smbus2"                   "python3 -c 'import smbus2'"
check "TensorRT engine exists"   "[[ -f '$ENGINE_PATH' ]]"

log ""
log "Installation complete: $PASS passed, $FAIL failed"
if [[ $FAIL -gt 0 ]]; then
    log "Review $LOG_FILE for details on failures"
    exit 1
fi

log ""
log "Next steps:"
log "  1. Log out and back in (for dialout group to take effect)"
log "  2. Power on the ODrive and run:  python3 tools/odrive_setup.py"
log "  3. Run extrinsic calibration:    python3 tools/calibrate_extrinsics.py"
log "  4. Launch the tracker:           ros2 launch tracker tracker.launch.py"
