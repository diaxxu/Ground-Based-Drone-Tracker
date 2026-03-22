#!/usr/bin/env python3
"""
camera_tracker.py

ROS 2 node that captures frames from the OV2710 global-shutter camera,
runs YOLOv8-nano (TensorRT FP16) for target detection, then switches to
OpenCV CSRT tracker for sub-frame-level centroid estimation between YOLO
confirmations.

Publishes pixel centroid of the highest-confidence aerial target at the
full camera frame rate (120 fps).

Topics published:
    /camera/target         (geometry_msgs/PointStamped)  -- pixel centroid [u, v, confidence]
    /camera/detection_image (sensor_msgs/Image)          -- annotated debug image (10 Hz)

Parameters:
    camera_index        : V4L2 device index (default 0)
    frame_width         : capture width (default 1280)
    frame_height        : capture height (default 720)
    fps                 : capture frame rate (default 120)
    engine_path         : path to YOLOv8 TensorRT engine file
    yolo_every_n_frames : run YOLO every N frames, CSRT in between (default 6)
    confidence_threshold: minimum YOLO confidence to start a track (default 0.35)
    target_class_ids    : COCO class IDs to track, comma-separated (default '0' = person for testing; use '' for all)
"""

import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


class CameraTracker(Node):
    def __init__(self):
        super().__init__('camera_tracker')

        self.declare_parameter('camera_index',        0)
        self.declare_parameter('frame_width',         1280)
        self.declare_parameter('frame_height',        720)
        self.declare_parameter('fps',                 120)
        self.declare_parameter('engine_path',         'config/yolov8n.engine')
        self.declare_parameter('yolo_every_n_frames', 6)
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('target_class_ids',    '0')

        cam_idx    = self.get_parameter('camera_index').value
        width      = self.get_parameter('frame_width').value
        height     = self.get_parameter('frame_height').value
        fps        = self.get_parameter('fps').value
        engine     = self.get_parameter('engine_path').value
        self.yolo_period   = self.get_parameter('yolo_every_n_frames').value
        self.conf_thresh   = self.get_parameter('confidence_threshold').value
        class_str          = self.get_parameter('target_class_ids').value

        self.target_classes = [int(c) for c in class_str.split(',') if c.strip()] \
            if class_str.strip() else None

        self.pub_target = self.create_publisher(PointStamped, '/camera/target', 10)
        self.pub_debug  = self.create_publisher(Image, '/camera/detection_image', 10)
        self.bridge     = CvBridge()

        # Open camera
        self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)  # minimize latency

        if not self.cap.isOpened():
            self.get_logger().fatal(f'Cannot open camera at index {cam_idx}')
            raise RuntimeError('Camera not available')

        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f'Camera opened at {width}x{height} @ {actual_fps} fps')

        # Load YOLO model
        if ULTRALYTICS_AVAILABLE:
            try:
                self.model = YOLO(engine, task='detect')
                self.get_logger().info(f'YOLOv8 TensorRT engine loaded from {engine}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load TensorRT engine ({e}), using PyTorch fallback')
                self.model = YOLO('yolov8n.pt')
        else:
            self.model = None
            self.get_logger().warn('Ultralytics not available — detection disabled, publishing zeros')

        # CSRT tracker
        self.tracker: Optional[cv2.Tracker] = None
        self.tracker_initialized = False
        self.last_bbox = None  # (x, y, w, h)

        self._frame_count = 0
        self._debug_count = 0

        # Main loop timer at camera fps
        period = 1.0 / fps
        self.create_timer(period, self._capture_and_track)

    def _capture_and_track(self) -> None:
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame capture failed')
            return

        self._frame_count += 1
        stamp = self.get_clock().now().to_msg()

        run_yolo = (self._frame_count % self.yolo_period == 0) or not self.tracker_initialized

        if run_yolo and self.model is not None:
            bbox, conf = self._run_yolo(frame)
            if bbox is not None:
                self._init_csrt(frame, bbox)
                self.last_bbox = bbox
        elif self.tracker_initialized:
            ok, bbox_raw = self.tracker.update(frame)
            if ok:
                self.last_bbox = (
                    int(bbox_raw[0]), int(bbox_raw[1]),
                    int(bbox_raw[2]), int(bbox_raw[3])
                )
                conf = 0.8  # CSRT does not provide confidence — assume good track
            else:
                self.tracker_initialized = False
                self.last_bbox = None
                conf = 0.0

        if self.last_bbox is not None:
            u = float(self.last_bbox[0] + self.last_bbox[2] / 2.0)
            v = float(self.last_bbox[1] + self.last_bbox[3] / 2.0)
            self._publish_centroid(u, v, stamp)

        # Debug image at 10 Hz
        self._debug_count += 1
        if self._debug_count % int(max(1, self._frame_count / max(1, self._debug_count) / 10)) == 0:
            self._publish_debug(frame, stamp)

    def _run_yolo(self, frame: np.ndarray):
        """Run YOLOv8 inference, return best matching bounding box and confidence."""
        results = self.model(frame, verbose=False, conf=self.conf_thresh)
        best_conf = 0.0
        best_box  = None

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                if self.target_classes and cls_id not in self.target_classes:
                    continue
                conf = float(box.conf[0])
                if conf > best_conf:
                    best_conf = conf
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    best_box = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))

        return best_box, best_conf

    def _init_csrt(self, frame: np.ndarray, bbox: tuple) -> None:
        """Initialise CSRT tracker with a new bounding box."""
        self.tracker = cv2.TrackerCSRT_create()
        self.tracker.init(frame, bbox)
        self.tracker_initialized = True

    def _publish_centroid(self, u: float, v: float, stamp) -> None:
        msg = PointStamped()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera'
        msg.point = Point(x=u, y=v, z=0.0)
        self.pub_target.publish(msg)

    def _publish_debug(self, frame: np.ndarray, stamp) -> None:
        if self.last_bbox is not None:
            x, y, w, h = self.last_bbox
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            u = x + w // 2
            v = y + h // 2
            cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = stamp
        self.pub_debug.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
