# Software Installation

This guide installs all required software on a fresh Jetson Orin Nano running JetPack 6.0. All commands run as a non-root user with sudo access.

---

## Prerequisites

- Jetson Orin Nano 8GB flashed with JetPack 6.0 (Ubuntu 22.04)
- Internet connection (ethernet recommended during install)
- At least 20 GB of free disk space on the NVMe SSD
- The NVMe SSD installed and set as the boot device via BIOS/UEFI

Verify JetPack version:
```bash
cat /etc/nv_tegra_release
```
Expected output contains `R36` (JetPack 6.x).

---

## Step 1 — System Update

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl git build-essential cmake python3-pip python3-venv
```

---

## Step 2 — Install ROS 2 Humble

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

Add to `.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:
```bash
ros2 --version
```

---

## Step 3 — Install Python Dependencies

```bash
pip3 install --upgrade pip
pip3 install \
    numpy \
    scipy \
    opencv-python \
    pyserial \
    odrive \
    smbus2 \
    transforms3d \
    matplotlib \
    pyyaml
```

---

## Step 4 — Install TensorRT and YOLOv8

TensorRT is pre-installed with JetPack. Verify:
```bash
python3 -c "import tensorrt; print(tensorrt.__version__)"
```

Install Ultralytics:
```bash
pip3 install ultralytics
```

Download YOLOv8-nano and export to TensorRT FP16:
```bash
cd ~
python3 - <<'EOF'
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.export(format='engine', half=True, imgsz=640, device=0)
print("TensorRT engine saved to yolov8n.engine")
EOF
```

This takes approximately 5-8 minutes on the Orin Nano. The resulting `yolov8n.engine` file is placed in the current directory. Copy it to the config folder:
```bash
cp ~/yolov8n.engine ~/ground-tracker/software/ros2_ws/src/tracker/config/
```

---

## Step 5 — Install ODrive Tools

```bash
pip3 install odrive
```

Verify USB connection to ODrive S1:
```bash
python3 -c "import odrive; odrv = odrive.find_any(); print('ODrive found:', odrv.serial_number)"
```

If the ODrive is not found, add the USB udev rule:
```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d32", MODE="0666"' \
    | sudo tee /etc/udev/rules.d/91-odrive.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## Step 6 — Configure UART for Radar

The AWR1843BOOST uses two UART ports. Add user to dialout group:
```bash
sudo usermod -aG dialout $USER
```

Log out and back in, then verify:
```bash
ls /dev/ttyUSB*
```
You should see `/dev/ttyUSB0` and `/dev/ttyUSB1`. The lower-numbered port is typically the config port (115200 baud) and the higher is the data port (921600 baud). Verify by checking which port accepts the radar configuration file — the config port will respond with `Done` after each command.

Set udev rules for consistent naming:
```bash
cat <<'EOF' | sudo tee /etc/udev/rules.d/99-radar.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="0451", ATTRS{idProduct}=="bef3", SYMLINK+="ttyRadarConfig", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0451", ATTRS{idProduct}=="bef4", SYMLINK+="ttyRadarData", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## Step 7 — Configure I2C for IMU

Enable I2C bus 1 (Jetson 40-pin header):
```bash
sudo apt install -y i2c-tools
sudo i2cdetect -y -r 1
```

The BMI088 should appear at address `0x18` (accelerometer) and `0x68` (gyroscope). If not, check wiring and pullup resistors.

---

## Step 8 — Build the ROS 2 Workspace

```bash
cd ~/ground-tracker/software/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Add the workspace source to `.bashrc`:
```bash
echo "source ~/ground-tracker/software/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Step 9 — Verify All Nodes Start

```bash
ros2 launch tracker tracker.launch.py
```

Check that all four nodes appear in the node list:
```bash
ros2 node list
```

Expected output:
```
/radar_driver
/camera_tracker
/ekf_controller
/gimbal_controller
```

Check topics are publishing:
```bash
ros2 topic hz /radar/detections
ros2 topic hz /camera/target
ros2 topic hz /ekf/target_pose
ros2 topic hz /gimbal/setpoint
```
