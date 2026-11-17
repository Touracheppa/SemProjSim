#!/usr/bin/env python3
"""Validate camera data on a running simulation without moving the robot."""
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def main():
    rclpy.init()
    node = rclpy.create_node('sieglinde_camera_smoke_test')
    latest, stamps = {}, {}
    topics = {'color/image_raw': Image, 'depth/image_raw': Image,
              'color/camera_info': CameraInfo, 'depth/camera_info': CameraInfo,
              'depth/points': PointCloud2}
    def receive(key, msg):
        latest[key] = msg
        stamps.setdefault(key, set()).add((msg.header.stamp.sec, msg.header.stamp.nanosec))
    subscriptions = [node.create_subscription(cls, '/camera/' + topic,
                     lambda msg, key=topic: receive(key, msg), qos_profile_sensor_data)
                     for topic, cls in topics.items()]
    try:
        end = time.monotonic() + 60
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(latest) == len(topics) and all(len(s) >= 3 for s in stamps.values()):
                break
        assert len(latest) == len(topics), f'Missing camera topics: {topics.keys() - latest.keys()}'
        assert all(len(s) >= 3 for s in stamps.values()), 'Camera timestamps are not advancing'
        for msg in latest.values():
            assert msg.header.frame_id == 'camera_optical_frame', msg.header.frame_id
            assert (msg.width, msg.height) == (640, 480)
        rgb = latest['color/image_raw']
        assert rgb.encoding == 'rgb8', rgb.encoding
        assert len(rgb.data) == rgb.step * rgb.height
        assert np.ptp(np.frombuffer(rgb.data, dtype=np.uint8)) > 10, 'RGB image is blank'
        depth = latest['depth/image_raw']
        assert depth.encoding == '32FC1', depth.encoding
        values = np.ndarray((depth.height, depth.width), dtype='>f4' if depth.is_bigendian else '<f4', buffer=depth.data, strides=(depth.step, 4))
        finite = values[np.isfinite(values)]
        assert finite.size > values.size * 0.05, 'Depth image has too few valid pixels'
        assert finite.min() >= 0.24 and finite.max() <= 9.01
        for key in ['color/camera_info', 'depth/camera_info']:
            info = latest[key]
            assert info.k[0] > 0 and info.k[4] > 0 and info.k[8] == 1
            assert 0 < info.k[2] < 640 and 0 < info.k[5] < 480
        assert np.array_equal(latest['color/camera_info'].k, latest['depth/camera_info'].k)
        cloud = latest['depth/points']
        assert len(cloud.data) == cloud.row_step * cloud.height
        assert {'x', 'y', 'z', 'rgb'} <= {field.name for field in cloud.fields}
        print(f'PASS: RGB, metric depth, both camera infos, colored cloud; advancing optical-frame data; depth {finite.min():.2f}–{finite.max():.2f} m')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
