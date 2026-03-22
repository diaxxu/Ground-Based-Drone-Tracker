#!/usr/bin/env python3
"""
ekf_controller.py

ROS 2 node that runs the Extended Kalman Filter, subscribing to radar
detections and camera centroids and publishing the fused predictive
target pose to the gimbal controller.

Subscriptions:
    /radar/detections    (geometry_msgs/PoseArray)   -- radar CFAR detections
    /camera/target       (geometry_msgs/PointStamped) -- camera pixel centroid

Publications:
    /ekf/target_pose     (geometry_msgs/PointStamped) -- az/el setpoint (x=az, y=el, radians)
    /ekf/status          (std_msgs/String)             -- filter state string

The predict step is driven by a 1 kHz timer independent of sensor callbacks.
Camera and radar updates are applied asynchronously on receipt.
"""

import time
import math
import numpy as np
import yaml
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PointStamped, Point
from std_msgs.msg import String, Header

from .ekf import EKF


class EKFController(Node):
    def __init__(self):
        super().__init__('ekf_controller')

        self.declare_parameter('params_file', 'config/params.yaml')
        self.declare_parameter('predict_rate_hz', 1000)
        self.declare_parameter('output_rate_hz',  200)
        self.declare_parameter('system_latency_s', 0.0125)

        params_file  = self.get_parameter('params_file').value
        predict_rate = self.get_parameter('predict_rate_hz').value
        output_rate  = self.get_parameter('output_rate_hz').value
        latency      = self.get_parameter('system_latency_s').value

        self.ekf = EKF(system_latency_s=latency)
        self._load_camera_params(params_file)

        self.pub_pose   = self.create_publisher(PointStamped, '/ekf/target_pose', 10)
        self.pub_status = self.create_publisher(String, '/ekf/status', 10)

        self.create_subscription(PoseArray,    '/radar/detections', self._on_radar,  10)
        self.create_subscription(PointStamped, '/camera/target',    self._on_camera, 10)

        self._last_predict = time.monotonic()
        self.create_timer(1.0 / predict_rate, self._predict_step)
        self.create_timer(1.0 / output_rate,  self._publish_setpoint)

        self._radar_count  = 0
        self._camera_count = 0

        self.get_logger().info('EKF controller started')

    def _load_camera_params(self, params_file: str) -> None:
        if not os.path.exists(params_file):
            self.get_logger().warn(f'Params file not found: {params_file} -- camera update disabled')
            return

        with open(params_file, 'r') as f:
            p = yaml.safe_load(f)

        try:
            K = np.array(p['camera']['intrinsics']).reshape(3, 3)
            R = np.array(p['extrinsics']['R_radar_to_camera']).reshape(3, 3)
            t = np.array(p['extrinsics']['t_radar_to_camera']).reshape(3, 1)
            self.ekf.set_camera_params(K, R, t)
            self.get_logger().info('Camera parameters loaded successfully')
        except (KeyError, ValueError) as e:
            self.get_logger().warn(f'Could not parse camera params: {e}')

    def _predict_step(self) -> None:
        now = time.monotonic()
        dt = now - self._last_predict
        self._last_predict = now

        if dt > 0.01:
            dt = 0.01  # cap dt to avoid divergence on startup

        self.ekf.predict(dt)

    def _on_radar(self, msg: PoseArray) -> None:
        if not msg.poses:
            return

        # Select the detection closest to current track, or nearest overall
        if self.ekf.initialized:
            p_est = self.ekf.position
            best_dist = float('inf')
            best_pose = None
            for pose in msg.poses:
                d = math.sqrt(
                    (pose.position.x - p_est[0])**2 +
                    (pose.position.y - p_est[1])**2 +
                    (pose.position.z - p_est[2])**2
                )
                if d < best_dist:
                    best_dist = d
                    best_pose = pose
        else:
            # Pick the detection with highest elevation (most likely aerial)
            best_pose = max(msg.poses, key=lambda p: p.position.z)

        if best_pose is None:
            return

        px = best_pose.position.x
        py = best_pose.position.y
        pz = best_pose.position.z
        rdot = best_pose.orientation.w  # packed into w by radar_driver

        r     = math.sqrt(px**2 + py**2 + pz**2)
        az    = math.atan2(py, px)
        el    = math.atan2(pz, math.sqrt(px**2 + py**2))

        self.ekf.update_radar(r, az, el, rdot)
        self._radar_count += 1

    def _on_camera(self, msg: PointStamped) -> None:
        u = msg.point.x
        v = msg.point.y
        if u == 0.0 and v == 0.0:
            return
        self.ekf.update_camera(u, v)
        self._camera_count += 1

    def _publish_setpoint(self) -> None:
        result = self.ekf.get_azimuth_elevation()
        stamp  = self.get_clock().now().to_msg()

        if result is not None:
            az, el = result
            msg = PointStamped()
            msg.header = Header()
            msg.header.stamp    = stamp
            msg.header.frame_id = 'world'
            msg.point           = Point(x=az, y=el, z=float(self.ekf.confidence))
            self.pub_pose.publish(msg)

        status = String()
        status.data = (
            f'init={self.ekf.initialized} '
            f'conf={self.ekf.confidence:.2f} '
            f'speed={self.ekf.speed:.1f}m/s '
            f'radar_updates={self._radar_count} '
            f'camera_updates={self._camera_count}'
        )
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = EKFController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
