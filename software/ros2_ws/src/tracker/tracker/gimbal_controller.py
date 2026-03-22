#!/usr/bin/env python3
"""
gimbal_controller.py

ROS 2 node that subscribes to the EKF target pose and commands the ODrive S1
to point the gimbal at the predicted target position.

Control architecture:
    - Position setpoints computed from EKF azimuth/elevation output
    - Sent to ODrive at 200 Hz via USB serial using odrive Python library
    - ODrive runs inner velocity loop at 1 kHz internally
    - IMU angular velocity from /imu/data used as velocity feedforward
      to pre-cancel base vibrations before they cause pointing error

Topics subscribed:
    /ekf/target_pose      (geometry_msgs/PointStamped) -- az/el in radians (x=az, y=el)
    /imu/data             (sensor_msgs/Imu)            -- raw IMU at 400 Hz

Parameters:
    odrive_serial         : ODrive serial number string, or '' for first found
    pan_axis              : ODrive axis index for pan (default 0)
    tilt_axis             : ODrive axis index for tilt (default 1)
    pan_gear_ratio        : 1.0 for direct drive
    tilt_gear_ratio       : 1.0 for direct drive
    pan_limit_deg         : soft limit each side of center (default 175)
    tilt_limit_up_deg     : soft upper tilt limit (default 90)
    tilt_limit_down_deg   : soft lower tilt limit (default -20)
    command_rate_hz       : setpoint send rate (default 200)
    imu_feedforward_gain  : scale applied to IMU angular velocity feedforward (default 0.8)
"""

import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu

try:
    import odrive
    from odrive.enums import AxisState, ControlMode, InputMode
    ODRIVE_AVAILABLE = True
except ImportError:
    ODRIVE_AVAILABLE = False


class GimbalController(Node):
    def __init__(self):
        super().__init__('gimbal_controller')

        self.declare_parameter('odrive_serial',        '')
        self.declare_parameter('pan_axis',             0)
        self.declare_parameter('tilt_axis',            1)
        self.declare_parameter('pan_gear_ratio',       1.0)
        self.declare_parameter('tilt_gear_ratio',      1.0)
        self.declare_parameter('pan_limit_deg',        175.0)
        self.declare_parameter('tilt_limit_up_deg',    90.0)
        self.declare_parameter('tilt_limit_down_deg', -20.0)
        self.declare_parameter('command_rate_hz',      200)
        self.declare_parameter('imu_feedforward_gain', 0.8)

        serial_num      = self.get_parameter('odrive_serial').value
        self.pan_idx    = self.get_parameter('pan_axis').value
        self.tilt_idx   = self.get_parameter('tilt_axis').value
        self.pan_ratio  = self.get_parameter('pan_gear_ratio').value
        self.tilt_ratio = self.get_parameter('tilt_gear_ratio').value
        pan_lim         = self.get_parameter('pan_limit_deg').value
        tilt_up         = self.get_parameter('tilt_limit_up_deg').value
        tilt_down       = self.get_parameter('tilt_limit_down_deg').value
        rate_hz         = self.get_parameter('command_rate_hz').value
        self.ff_gain    = self.get_parameter('imu_feedforward_gain').value

        self.pan_lim_rad   = math.radians(pan_lim)
        self.tilt_up_rad   = math.radians(tilt_up)
        self.tilt_down_rad = math.radians(tilt_down)

        # Current setpoints (radians from center)
        self.az_cmd = 0.0
        self.el_cmd = 0.0

        # Latest IMU angular velocity (rad/s) in body frame [x, y, z]
        self.imu_omega = np.zeros(3)
        self.imu_stamp = 0.0

        # Connect to ODrive
        self.odrv = None
        self.pan_axis_obj  = None
        self.tilt_axis_obj = None

        if ODRIVE_AVAILABLE:
            self._connect_odrive(serial_num)
        else:
            self.get_logger().warn('odrive package not installed — running in simulation mode')

        # Subscriptions
        self.create_subscription(PointStamped, '/ekf/target_pose', self._on_ekf_target, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)

        # Command loop
        period = 1.0 / rate_hz
        self.create_timer(period, self._command_loop)

        self.get_logger().info(f'Gimbal controller running at {rate_hz} Hz')

    def _connect_odrive(self, serial_num: str) -> None:
        self.get_logger().info('Connecting to ODrive S1...')
        try:
            if serial_num:
                self.odrv = odrive.find_any(serial_number=serial_num, timeout=10)
            else:
                self.odrv = odrive.find_any(timeout=10)
        except Exception as e:
            self.get_logger().error(f'ODrive connection failed: {e}')
            return

        self.pan_axis_obj  = getattr(self.odrv, f'axis{self.pan_idx}')
        self.tilt_axis_obj = getattr(self.odrv, f'axis{self.tilt_idx}')

        # Enable closed loop control
        for ax in [self.pan_axis_obj, self.tilt_axis_obj]:
            ax.requested_state = AxisState.CLOSED_LOOP_CONTROL
            ax.controller.config.control_mode = ControlMode.POSITION_CONTROL
            ax.controller.config.input_mode    = InputMode.POS_FILTER

        self.get_logger().info(
            f'ODrive connected: serial={self.odrv.serial_number}'
        )

    def _on_ekf_target(self, msg: PointStamped) -> None:
        """Receive predicted azimuth/elevation from EKF node."""
        az = float(msg.point.x)
        el = float(msg.point.y)

        # Apply soft limits
        az = float(np.clip(az, -self.pan_lim_rad,   self.pan_lim_rad))
        el = float(np.clip(el,  self.tilt_down_rad, self.tilt_up_rad))

        self.az_cmd = az
        self.el_cmd = el

    def _on_imu(self, msg: Imu) -> None:
        """Cache latest IMU angular velocity for feedforward."""
        self.imu_omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        self.imu_stamp = time.time()

    def _command_loop(self) -> None:
        """Send position setpoints to ODrive with IMU velocity feedforward."""
        if self.pan_axis_obj is None or self.tilt_axis_obj is None:
            return

        # Convert radians to turns (ODrive native unit)
        pan_turns  = self.az_cmd / (2.0 * math.pi) * self.pan_ratio
        tilt_turns = self.el_cmd / (2.0 * math.pi) * self.tilt_ratio

        # IMU feedforward: gyro_z counters pan vibration, gyro_x counters tilt vibration
        imu_age = time.time() - self.imu_stamp
        if imu_age < 0.05:  # only use IMU data if fresh (<50 ms old)
            pan_vel_ff  = -self.imu_omega[2] / (2.0 * math.pi) * self.ff_gain * self.pan_ratio
            tilt_vel_ff = -self.imu_omega[0] / (2.0 * math.pi) * self.ff_gain * self.tilt_ratio
        else:
            pan_vel_ff  = 0.0
            tilt_vel_ff = 0.0

        try:
            self.pan_axis_obj.controller.input_pos  = pan_turns
            self.tilt_axis_obj.controller.input_pos = tilt_turns
            self.pan_axis_obj.controller.input_vel  = pan_vel_ff
            self.tilt_axis_obj.controller.input_vel = tilt_vel_ff
        except Exception as e:
            self.get_logger().warn(f'ODrive write error: {e}')

    def _safe_stop(self) -> None:
        """Put motors into idle state on shutdown."""
        if self.pan_axis_obj is None:
            return
        try:
            for ax in [self.pan_axis_obj, self.tilt_axis_obj]:
                ax.requested_state = AxisState.IDLE
            self.get_logger().info('Motors set to IDLE')
        except Exception:
            pass

    def destroy_node(self):
        self._safe_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
