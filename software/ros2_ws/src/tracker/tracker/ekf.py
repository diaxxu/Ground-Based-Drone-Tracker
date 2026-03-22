#!/usr/bin/env python3
"""
ekf.py

Extended Kalman Filter for fusing radar and camera measurements into a
smooth, predictive 3D track of an aerial target.

State vector (9-dimensional, world frame):
    x = [px, py, pz, vx, vy, vz, ax, ay, az]

Radar measurement (4-dimensional, spherical):
    z_r = [range, azimuth, elevation, range_rate]

Camera measurement (2-dimensional, pixel):
    z_c = [u_pixels, v_pixels]

All angles in radians. All distances in meters. All times in seconds.

See docs/05_ekf_theory.md for derivation of all Jacobians.
"""

import numpy as np
from typing import Optional, Tuple


class EKF:
    """
    Asynchronous multi-sensor Extended Kalman Filter.
    Predict step runs at a caller-defined rate (nominally 1 kHz).
    Update steps run on sensor callbacks at their native rates.
    """

    N = 9  # state dimension

    def __init__(
        self,
        process_noise_pos: float = 1e-4,
        process_noise_vel: float = 1e-2,
        process_noise_acc: float = 1.0,
        radar_noise_range: float = 0.04,
        radar_noise_angle: float = 0.02,
        radar_noise_rdot: float = 0.10,
        camera_noise_px: float = 2.0,
        system_latency_s: float = 0.0125,
    ):
        """
        Parameters
        ----------
        process_noise_pos     : position process noise std dev (m)
        process_noise_vel     : velocity process noise std dev (m/s)
        process_noise_acc     : acceleration process noise std dev (m/s^2)
        radar_noise_range     : radar range measurement std dev (m)
        radar_noise_angle     : radar angle measurement std dev (rad)
        radar_noise_rdot      : radar radial velocity std dev (m/s)
        camera_noise_px       : camera centroid noise std dev (pixels)
        system_latency_s      : total system latency for predictive pointing (s)
        """
        self.system_latency = system_latency_s

        # Process noise covariance Q
        q = np.array([
            process_noise_pos,
            process_noise_pos,
            process_noise_pos,
            process_noise_vel,
            process_noise_vel,
            process_noise_vel,
            process_noise_acc,
            process_noise_acc,
            process_noise_acc,
        ]) ** 2
        self.Q = np.diag(q)

        # Radar measurement noise covariance R_r (4x4)
        self.R_radar = np.diag([
            radar_noise_range ** 2,
            radar_noise_angle ** 2,
            radar_noise_angle ** 2,
            radar_noise_rdot ** 2,
        ])

        # Camera measurement noise covariance R_c (2x2)
        self.R_camera = np.diag([camera_noise_px ** 2, camera_noise_px ** 2])

        # State and covariance (uninitialised)
        self.x: Optional[np.ndarray] = None
        self.P: Optional[np.ndarray] = None
        self.initialized = False
        self.confidence: float = 0.0
        self._last_predict_time: Optional[float] = None

        # Camera intrinsics and extrinsics (set via set_camera_params)
        self.K: Optional[np.ndarray] = None   # 3x3 intrinsic matrix
        self.R_cam: Optional[np.ndarray] = None   # 3x3 rotation world->camera
        self.t_cam: Optional[np.ndarray] = None   # 3x1 translation world->camera

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def set_camera_params(self, K: np.ndarray, R_cam: np.ndarray, t_cam: np.ndarray) -> None:
        """Set camera intrinsic and extrinsic parameters from calibration."""
        self.K = K.copy()
        self.R_cam = R_cam.copy()
        self.t_cam = t_cam.reshape(3, 1).copy()

    def initialize_from_radar(self, r: float, az: float, el: float, rdot: float) -> None:
        """Initialise track from a single radar detection (spherical coords)."""
        px = r * np.cos(el) * np.cos(az)
        py = r * np.cos(el) * np.sin(az)
        pz = r * np.sin(el)

        self.x = np.zeros(self.N)
        self.x[:3] = [px, py, pz]

        # High initial uncertainty on velocity and acceleration
        self.P = np.diag([100.0, 100.0, 100.0,
                          25.0,  25.0,  25.0,
                           4.0,   4.0,   4.0])
        self.initialized = True
        self.confidence = 0.1
        self._last_predict_time = None

    # ------------------------------------------------------------------
    # Predict
    # ------------------------------------------------------------------

    def predict(self, dt: float) -> None:
        """Propagate state forward by dt seconds using constant-acceleration model."""
        if not self.initialized:
            return

        F = self._state_transition(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

        self.confidence = max(0.0, self.confidence - 0.005)

    def _state_transition(self, dt: float) -> np.ndarray:
        dt2 = 0.5 * dt * dt
        F = np.eye(self.N)
        # Position from velocity
        F[0, 3] = dt;  F[1, 4] = dt;  F[2, 5] = dt
        # Position from acceleration
        F[0, 6] = dt2; F[1, 7] = dt2; F[2, 8] = dt2
        # Velocity from acceleration
        F[3, 6] = dt;  F[4, 7] = dt;  F[5, 8] = dt
        return F

    # ------------------------------------------------------------------
    # Radar update
    # ------------------------------------------------------------------

    def update_radar(self, r: float, az: float, el: float, rdot: float) -> None:
        """Update state with a radar measurement."""
        if not self.initialized:
            self.initialize_from_radar(r, az, el, rdot)
            return

        z = np.array([r, az, el, rdot])
        h, H = self._radar_measurement_and_jacobian()

        self._update(z, h, H, self.R_radar)
        self.confidence = min(1.0, self.confidence + 0.1)

    def _radar_measurement_and_jacobian(self) -> Tuple[np.ndarray, np.ndarray]:
        px, py, pz = self.x[0], self.x[1], self.x[2]
        vx, vy, vz = self.x[3], self.x[4], self.x[5]

        rxy = np.sqrt(px**2 + py**2)
        r   = np.sqrt(px**2 + py**2 + pz**2)
        r   = max(r, 0.001)  # avoid divide by zero

        az    = np.arctan2(py, px)
        el    = np.arctan2(pz, rxy)
        rdot  = (px*vx + py*vy + pz*vz) / r

        h = np.array([r, az, el, rdot])

        # Jacobian H (4 x 9) -- partial derivatives w.r.t. state
        H = np.zeros((4, self.N))

        # d(range)/d(pos)
        H[0, 0] = px / r
        H[0, 1] = py / r
        H[0, 2] = pz / r

        # d(azimuth)/d(pos)
        rxy2 = max(rxy**2, 1e-6)
        H[1, 0] = -py / rxy2
        H[1, 1] =  px / rxy2

        # d(elevation)/d(pos)
        r2 = max(r**2, 1e-6)
        H[2, 0] = -(px * pz) / (r2 * max(rxy, 1e-6))
        H[2, 1] = -(py * pz) / (r2 * max(rxy, 1e-6))
        H[2, 2] =  rxy / r2

        # d(rdot)/d(pos) and d(rdot)/d(vel)
        H[3, 0] = (vx*r - px*rdot) / r2
        H[3, 1] = (vy*r - py*rdot) / r2
        H[3, 2] = (vz*r - pz*rdot) / r2
        H[3, 3] = px / r
        H[3, 4] = py / r
        H[3, 5] = pz / r

        return h, H

    # ------------------------------------------------------------------
    # Camera update
    # ------------------------------------------------------------------

    def update_camera(self, u: float, v: float) -> None:
        """Update state with a camera pixel centroid measurement."""
        if not self.initialized or self.K is None:
            return

        z = np.array([u, v])
        h, H = self._camera_measurement_and_jacobian()

        if h is None:
            return  # target behind camera

        self._update(z, h, H, self.R_camera)
        self.confidence = min(1.0, self.confidence + 0.05)

    def _camera_measurement_and_jacobian(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        p_world = self.x[:3].reshape(3, 1)
        p_cam   = self.R_cam @ p_world + self.t_cam

        if p_cam[2, 0] <= 0:
            return None, None  # behind camera

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        Xc, Yc, Zc = p_cam[0, 0], p_cam[1, 0], p_cam[2, 0]

        u_hat = fx * Xc / Zc + cx
        v_hat = fy * Yc / Zc + cy

        h = np.array([u_hat, v_hat])

        # Jacobian of projection w.r.t. camera-frame point
        J_proj = np.array([
            [fx / Zc,       0, -fx * Xc / (Zc**2)],
            [      0, fy / Zc, -fy * Yc / (Zc**2)],
        ])

        # Chain rule: d(pixel)/d(world_pos) = J_proj @ R_cam
        dp_dstate = np.zeros((3, self.N))
        dp_dstate[:, :3] = self.R_cam

        H = J_proj @ dp_dstate  # 2 x 9

        return h, H

    # ------------------------------------------------------------------
    # Common update step
    # ------------------------------------------------------------------

    def _update(self, z: np.ndarray, h: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        """Standard Kalman update step."""
        y = z - h                           # innovation
        S = H @ self.P @ H.T + R           # innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain

        self.x = self.x + K @ y
        I_KH   = np.eye(self.N) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T  # Joseph form (numerically stable)

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------

    def get_predicted_position(self, extra_dt: Optional[float] = None) -> Optional[np.ndarray]:
        """
        Return the predicted 3D position at time (now + system_latency + extra_dt).
        Used for predictive gimbal pointing.
        """
        if not self.initialized:
            return None

        dt = self.system_latency + (extra_dt or 0.0)
        px = self.x[0] + self.x[3] * dt + 0.5 * self.x[6] * dt**2
        py = self.x[1] + self.x[4] * dt + 0.5 * self.x[7] * dt**2
        pz = self.x[2] + self.x[5] * dt + 0.5 * self.x[8] * dt**2
        return np.array([px, py, pz])

    def get_azimuth_elevation(self) -> Optional[Tuple[float, float]]:
        """
        Return (azimuth_rad, elevation_rad) pointing angles to the predicted position.
        azimuth: 0 = forward (+x), positive = left (+y)
        elevation: 0 = horizontal, positive = up (+z)
        """
        p = self.get_predicted_position()
        if p is None:
            return None

        az = float(np.arctan2(p[1], p[0]))
        el = float(np.arctan2(p[2], np.sqrt(p[0]**2 + p[1]**2)))
        return az, el

    @property
    def position(self) -> Optional[np.ndarray]:
        return self.x[:3].copy() if self.initialized else None

    @property
    def velocity(self) -> Optional[np.ndarray]:
        return self.x[3:6].copy() if self.initialized else None

    @property
    def speed(self) -> float:
        if not self.initialized:
            return 0.0
        return float(np.linalg.norm(self.x[3:6]))
