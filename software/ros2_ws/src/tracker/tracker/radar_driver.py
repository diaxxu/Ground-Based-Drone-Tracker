#!/usr/bin/env python3
"""
radar_driver.py

ROS 2 node that reads CFAR point cloud detections from the TI AWR1843BOOST
over UART and publishes them as geometry_msgs/PoseArray on /radar/detections.

The AWR1843 is configured via a separate config UART port using the chirp
configuration file in firmware/awr1843_cfg.cfg. This node handles only the
data port.

Topic published:
    /radar/detections  (geometry_msgs/PoseArray)  -- detected targets in radar frame
    /radar/status      (std_msgs/String)            -- driver status string

Coordinate frame:
    x -- forward (boresight direction)
    y -- left
    z -- up
    All in meters. Origin at radar phase center.
"""

import struct
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import String, Header


MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'
MAX_TARGETS = 32

# TLV type IDs from TI mmWave SDK
TLV_DETECTED_POINTS = 1
TLV_RANGE_PROFILE   = 2
TLV_STATS           = 6


class RadarDriver(Node):
    def __init__(self):
        super().__init__('radar_driver')

        self.declare_parameter('data_port',   '/dev/ttyUSB1')
        self.declare_parameter('config_port', '/dev/ttyUSB0')
        self.declare_parameter('config_file', 'firmware/awr1843_cfg.cfg')
        self.declare_parameter('frame_id',    'radar')

        data_port   = self.get_parameter('data_port').value
        config_port = self.get_parameter('config_port').value
        config_file = self.get_parameter('config_file').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub_detections = self.create_publisher(PoseArray, '/radar/detections', 10)
        self.pub_status     = self.create_publisher(String, '/radar/status', 10)

        self._configure_radar(config_port, config_file)

        try:
            self.serial = serial.Serial(data_port, baudrate=921600, timeout=0.05)
            self.get_logger().info(f'Opened data port {data_port}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open data port {data_port}: {e}')
            raise

        self.create_timer(0.001, self._read_frame)  # poll at 1 kHz, frames arrive at 50 Hz
        self._buf = b''
        self._frame_count = 0

    def _configure_radar(self, config_port: str, config_file: str) -> None:
        try:
            cfg_serial = serial.Serial(config_port, baudrate=115200, timeout=1.0)
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open config port {config_port}: {e}')
            return

        try:
            with open(config_file, 'r') as f:
                commands = [line.strip() for line in f if line.strip() and not line.startswith('%')]
        except FileNotFoundError:
            self.get_logger().error(f'Config file not found: {config_file}')
            cfg_serial.close()
            return

        self.get_logger().info(f'Sending {len(commands)} commands to radar')
        time.sleep(0.5)
        for cmd in commands:
            cfg_serial.write((cmd + '\n').encode())
            time.sleep(0.05)
            resp = cfg_serial.read(512).decode(errors='ignore')
            if 'Error' in resp:
                self.get_logger().warn(f'Radar config warning on: {cmd}')

        cfg_serial.close()
        self.get_logger().info('Radar configuration complete')

    def _read_frame(self) -> None:
        if not self.serial.in_waiting:
            return

        self._buf += self.serial.read(self.serial.in_waiting)

        # Locate magic word
        idx = self._buf.find(MAGIC_WORD)
        if idx < 0:
            # Keep last 7 bytes in case magic word straddles reads
            self._buf = self._buf[-7:]
            return
        if idx > 0:
            self._buf = self._buf[idx:]

        # Need at least header (40 bytes) after magic word
        if len(self._buf) < 40:
            return

        # Parse header
        magic, version, total_len, platform, frame_num, cpu_cycles, \
            num_obj, num_tlv, subframe = struct.unpack_from('<8sIIIIIIII', self._buf, 0)

        if len(self._buf) < total_len:
            return  # Wait for full frame

        frame_data = self._buf[:total_len]
        self._buf = self._buf[total_len:]

        detections = self._parse_tlvs(frame_data[40:], num_tlv, num_obj)
        self._publish(detections, frame_num)
        self._frame_count += 1

    def _parse_tlvs(self, data: bytes, num_tlv: int, num_obj: int) -> list:
        detections = []
        offset = 0
        for _ in range(num_tlv):
            if offset + 8 > len(data):
                break
            tlv_type, tlv_len = struct.unpack_from('<II', data, offset)
            offset += 8
            tlv_data = data[offset:offset + tlv_len]
            offset += tlv_len

            if tlv_type == TLV_DETECTED_POINTS:
                detections = self._parse_detected_points(tlv_data, num_obj)

        return detections

    def _parse_detected_points(self, data: bytes, num_obj: int) -> list:
        """
        Each detected point is 16 bytes:
            float x, float y, float z, float doppler
        In radar frame: x=forward, y=left, z=up
        """
        points = []
        point_size = 16
        for i in range(min(num_obj, MAX_TARGETS)):
            if (i + 1) * point_size > len(data):
                break
            x, y, z, doppler = struct.unpack_from('<ffff', data, i * point_size)
            points.append({'x': x, 'y': y, 'z': z, 'doppler': doppler})
        return points

    def _publish(self, detections: list, frame_num: int) -> None:
        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        for det in detections:
            pose = Pose()
            pose.position = Point(x=det['x'], y=det['y'], z=det['z'])
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=float(det['doppler']))
            msg.poses.append(pose)

        self.pub_detections.publish(msg)

        status = String()
        status.data = f'frame={frame_num} targets={len(detections)}'
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = RadarDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
