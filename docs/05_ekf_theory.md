# EKF Theory and Implementation

The Extended Kalman Filter (EKF) is the core algorithm that fuses radar and camera measurements into a smooth, predictive track. This document explains the mathematics, the state vector design, and how the implementation maps to the code.

---

## Why an EKF

A regular Kalman filter assumes linear measurement models — the measurement is a linear function of the state. Neither radar nor camera provides that. Radar gives range and angle (nonlinear in Cartesian position). Camera gives pixel coordinates (nonlinear in 3D world position). The Extended Kalman filter linearizes these measurement functions around the current state estimate using first-order Taylor expansion (the Jacobian), making the standard Kalman update applicable.

---

## State Vector

The state vector x contains 9 elements in world-frame Cartesian coordinates:

```
x = [px, py, pz, vx, vy, vz, ax, ay, az]^T
```

where p = position (meters), v = velocity (m/s), a = acceleration (m/s^2), all in a fixed world frame with origin at the tracker base.

The acceleration states model slow-changing maneuver dynamics. They are driven by process noise and allow the filter to track turning or accelerating targets without prediction divergence.

---

## Process Model (Predict Step)

The state transition matrix F for a constant-acceleration model with timestep dt:

```
        [ 1  0  0  dt  0  0  dt^2/2    0       0   ]
        [ 0  1  0   0 dt  0    0     dt^2/2    0   ]
        [ 0  0  1   0  0 dt    0       0     dt^2/2]
F(dt) = [ 0  0  0   1  0  0   dt       0       0   ]
        [ 0  0  0   0  1  0    0      dt       0   ]
        [ 0  0  0   0  0  1    0       0      dt   ]
        [ 0  0  0   0  0  0    1       0       0   ]
        [ 0  0  0   0  0  0    0       1       0   ]
        [ 0  0  0   0  0  0    0       0       1   ]
```

Predicted state:   x_pred = F * x
Predicted covariance: P_pred = F * P * F^T + Q

Q is the process noise covariance. Tune diagonal entries for position (1e-4), velocity (1e-2), and acceleration (1.0). Higher acceleration noise Q_a allows the filter to track faster maneuvers at the cost of noisier velocity estimates.

---

## Radar Measurement Model

Radar provides: range r, azimuth angle theta, elevation angle phi, and radial velocity r_dot.

The measurement function h_r(x) maps state to expected radar measurement:

```
r       = sqrt(px^2 + py^2 + pz^2)
theta   = atan2(py, px)
phi     = atan2(pz, sqrt(px^2 + py^2))
r_dot   = (px*vx + py*vy + pz*vz) / r
```

The Jacobian H_r = d(h_r)/dx is the 4x9 matrix of partial derivatives evaluated at the current state estimate. This is computed analytically in `ekf.py`.

Radar measurement noise covariance R_r (diagonal):

```
sigma_r     = 0.04 m   (range noise, 4 cm 1-sigma)
sigma_theta = 0.02 rad (azimuth noise)
sigma_phi   = 0.02 rad (elevation noise)
sigma_rdot  = 0.1 m/s  (radial velocity noise)
```

---

## Camera Measurement Model

The camera provides pixel centroid [u, v] of the detected target. The measurement function h_c(x) projects the 3D world position through the camera model:

```
[u]   = K * [R | t] * [px, py, pz, 1]^T  (pinhole projection)
[v]
```

where K is the 3x3 camera intrinsic matrix (calibrated), and [R|t] is the 3x4 extrinsic transform from world frame to camera frame (calibrated by `calibrate_extrinsics.py`).

The Jacobian H_c = d(h_c)/dx is a 2x9 matrix. Only the first three columns (position) are nonzero since pixel position does not depend directly on velocity or acceleration states.

Camera measurement noise covariance R_c:

```
sigma_u = 2.0 px
sigma_v = 2.0 px
```

This value assumes the centroid is computed from a bounding box center. If using subpixel interpolation from CSRT, reduce to 0.5 px.

---

## Update Step

For each measurement z from either sensor:

```
Innovation:    y = z - h(x_pred)
Innovation cov: S = H * P_pred * H^T + R
Kalman gain:   K = P_pred * H^T * S^-1
Updated state: x = x_pred + K * y
Updated cov:   P = (I - K * H) * P_pred
```

Both sensors update asynchronously. The radar update runs at 50 Hz, the camera update at 120 Hz. Between updates, the predict step runs at 1 kHz to keep the state estimate current.

---

## Predictive Pointing

The key to zero-lag tracking is commanding the gimbal to point at where the target will be when the motor responds, not where the target is now.

The total system latency from measurement to motor response is approximately 12.5 ms. The predicted position at time t + 12.5ms:

```
p_predicted = p + v * 0.0125 + 0.5 * a * 0.0125^2
```

Convert to azimuth and elevation:

```
az_cmd = atan2(p_predicted[1], p_predicted[0])   (radians)
el_cmd = atan2(p_predicted[2], norm(p_predicted[:2]))
```

These become the gimbal setpoint commands sent to the ODrive at 200 Hz.

---

## Initialization

When a new target is first detected (radar or camera, no prior track), initialize the state vector:

- Position: from radar measurement (convert spherical to Cartesian)
- Velocity: [0, 0, 0] initially (no velocity measurement at first frame)
- Acceleration: [0, 0, 0]
- Covariance P: diagonal [100, 100, 100, 25, 25, 25, 4, 4, 4] (high initial uncertainty)

The filter converges to an accurate velocity estimate within approximately 5 radar frames (100 ms) for targets moving faster than 2 m/s.

---

## Track Management

The implementation maintains a single track (highest-confidence detection). Multi-target track management (multiple hypothesis tracking) is not implemented in this version.

Track confidence is a scalar in [0, 1]:
- Increases by 0.1 per frame with a successful measurement update
- Decreases by 0.3 per frame with no measurement (occlusion or miss)
- Track is dropped when confidence falls below 0.2
- Gimbal holds last known azimuth/elevation when track is dropped
