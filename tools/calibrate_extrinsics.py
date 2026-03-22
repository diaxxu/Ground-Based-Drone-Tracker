#!/usr/bin/env python3
"""
calibrate_extrinsics.py

Calibrates the rigid-body transform between the radar coordinate frame and
the camera coordinate frame. This transform is required by the EKF to project
radar-measured 3D positions into camera pixel coordinates for the camera
measurement model.

Method:
    A corner reflector or flat plate is placed at a known position in front
    of the tracker. The radar reports the 3D position in radar frame. The
    camera reports the pixel centroid. Using the camera intrinsic matrix K
    (either pre-calibrated or estimated), the script solves for the 4x4
    extrinsic transform [R | t] via least-squares over multiple calibration
    points at different distances and angles.

Requirements:
    - Camera must be connected and returning frames
    - Radar must be running and publishing /radar/detections
    - A calibration target: ideally a TI corner reflector (strong radar return)
      with a visible marker (colored circle or ArUco marker) at the same center

Usage:
    1. Launch the radar and camera nodes:
           ros2 launch tracker tracker.launch.py
       (comment out ekf_controller and gimbal_controller in the launch file)
    2. Run this script:
           python3 tools/calibrate_extrinsics.py
    3. Follow the prompts to collect calibration points
    4. The script writes results to config/params.yaml

Output:
    Updates the extrinsics section of config/params.yaml with:
        R_radar_to_camera: [9 floats, row-major 3x3]
        t_radar_to_camera: [3 floats, meters]
    Also updates camera intrinsics if measured with a checkerboard.
"""

import sys
import os
import time
import cv2
import numpy as np
import yaml

PARAMS_FILE = 'software/ros2_ws/src/tracker/config/params.yaml'
CAMERA_INDEX = 0
CHECKERBOARD = (9, 6)         # inner corners of calibration checkerboard
CHECKERBOARD_SQUARE_MM = 25.0  # physical square size in mm


def calibrate_camera_intrinsics(cap: cv2.VideoCapture) -> tuple:
    """
    Run a checkerboard intrinsic calibration session.
    Returns (K, dist) where K is 3x3 and dist is the distortion vector.
    """
    print("\nCamera intrinsic calibration")
    print("Hold a printed checkerboard in front of the camera.")
    print("Press SPACE to capture a frame, R to run calibration, Q to quit.")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= CHECKERBOARD_SQUARE_MM

    obj_points = []
    img_points = []

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        display = frame.copy()
        if found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(display, CHECKERBOARD, corners2, found)

        cv2.putText(display, f'Frames captured: {len(obj_points)}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('Intrinsic Calibration', display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' ') and found:
            obj_points.append(objp)
            img_points.append(corners2)
            print(f'  Captured frame {len(obj_points)}')
        elif key == ord('r') and len(obj_points) >= 10:
            break
        elif key == ord('q'):
            cv2.destroyAllWindows()
            return None, None

    cv2.destroyAllWindows()

    h, w = frame.shape[:2]
    print(f'\nCalibrating with {len(obj_points)} frames at {w}x{h}...')
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
    print(f'Reprojection error: {rms:.4f} pixels')
    print(f'Focal length: fx={K[0,0]:.1f}, fy={K[1,1]:.1f}')
    print(f'Principal point: cx={K[0,2]:.1f}, cy={K[1,2]:.1f}')
    return K, dist


def collect_calibration_points(cap: cv2.VideoCapture) -> list:
    """
    Interactively collect paired (radar_pos, pixel_centroid) observations.
    Returns list of dicts with keys 'radar' (3D float array) and 'pixel' (2D float array).
    """
    print("\nExtrinsic calibration — data collection")
    print("Place the corner reflector at various positions in front of the tracker.")
    print("For each position, enter the radar-reported 3D coordinates,")
    print("then click the center of the target in the camera image.")
    print()
    print("Commands: ENTER to record point, D to delete last, R to run calibration, Q to quit")

    points = []
    click_point = [None]

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_point[0] = (x, y)

    cv2.namedWindow('Extrinsic Calibration')
    cv2.setMouseCallback('Extrinsic Calibration', mouse_callback)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        display = frame.copy()
        if click_point[0]:
            cv2.circle(display, click_point[0], 6, (0, 0, 255), -1)

        for i, pt in enumerate(points):
            px = tuple(int(v) for v in pt['pixel'])
            cv2.circle(display, px, 5, (0, 255, 0), -1)
            cv2.putText(display, str(i+1), (px[0]+6, px[1]-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.putText(display, f'Points: {len(points)} (need >= 6)',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow('Extrinsic Calibration', display)
        key = cv2.waitKey(1) & 0xFF

        if key == 13:  # ENTER
            if click_point[0] is None:
                print('  Click the target center in the image first')
                continue
            print('  Enter radar 3D position (x y z in meters, space-separated): ', end='')
            try:
                coords = [float(v) for v in input().split()]
                if len(coords) != 3:
                    raise ValueError
            except ValueError:
                print('  Invalid input — enter three numbers')
                continue
            points.append({
                'radar': np.array(coords),
                'pixel': np.array(click_point[0], dtype=float),
            })
            click_point[0] = None
            print(f'  Recorded point {len(points)}: radar={coords}, pixel={points[-1]["pixel"].tolist()}')

        elif key == ord('d') and points:
            points.pop()
            print(f'  Deleted last point. {len(points)} remaining.')

        elif key == ord('r') and len(points) >= 6:
            break

        elif key == ord('q'):
            break

    cv2.destroyAllWindows()
    return points


def solve_extrinsics(points: list, K: np.ndarray, dist: np.ndarray) -> tuple:
    """Solve for R, t using solvePnP over collected calibration points."""
    obj_pts = np.array([p['radar'] for p in points], dtype=np.float32)
    img_pts = np.array([p['pixel'] for p in points], dtype=np.float32)

    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        obj_pts, img_pts, K, dist,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )

    if not success:
        raise RuntimeError('solvePnPRansac failed — add more calibration points')

    R, _ = cv2.Rodrigues(rvec)
    t    = tvec.flatten()

    # Verify reprojection error
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    errors  = np.linalg.norm(img_pts - proj.reshape(-1, 2), axis=1)
    print(f'Reprojection error: mean={errors.mean():.2f}px  max={errors.max():.2f}px')

    return R, t


def save_params(K: np.ndarray, dist: np.ndarray, R: np.ndarray, t: np.ndarray) -> None:
    if os.path.exists(PARAMS_FILE):
        with open(PARAMS_FILE, 'r') as f:
            params = yaml.safe_load(f)
    else:
        params = {}

    params.setdefault('camera', {})
    params['camera']['intrinsics'] = K.flatten().tolist()
    params['camera']['distortion'] = dist.flatten().tolist()

    params['extrinsics'] = {
        'R_radar_to_camera': R.flatten().tolist(),
        't_radar_to_camera': t.tolist(),
    }

    with open(PARAMS_FILE, 'w') as f:
        yaml.dump(params, f, default_flow_style=False)

    print(f'\nParameters saved to {PARAMS_FILE}')


def main():
    print("Extrinsic Calibration Tool")
    print("==========================")

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

    if not cap.isOpened():
        print(f'ERROR: Cannot open camera {CAMERA_INDEX}')
        sys.exit(1)

    print('\nStep 1: Camera intrinsic calibration')
    print('If you already have intrinsics (from a prior run), press Q to skip.')
    K, dist = calibrate_camera_intrinsics(cap)

    if K is None:
        print('Loading intrinsics from params.yaml...')
        with open(PARAMS_FILE, 'r') as f:
            p = yaml.safe_load(f)
        K    = np.array(p['camera']['intrinsics']).reshape(3, 3)
        dist = np.array(p['camera']['distortion'])
        print(f'Loaded K:\n{K}')

    print('\nStep 2: Collect extrinsic calibration points')
    points = collect_calibration_points(cap)

    if len(points) < 6:
        print('Not enough points collected. Need at least 6. Exiting.')
        cap.release()
        sys.exit(1)

    print('\nStep 3: Solving for extrinsic transform...')
    R, t = solve_extrinsics(points, K, dist)
    print(f'R =\n{R}')
    print(f't = {t}')

    print('\nStep 4: Saving to params.yaml...')
    save_params(K, dist, R, t)

    cap.release()
    print('\nCalibration complete. You can now launch the full tracker.')


if __name__ == '__main__':
    main()
