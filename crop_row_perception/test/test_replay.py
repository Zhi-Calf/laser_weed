"""Real ROS image transport and OpenCV file decode, without a robot or SDK."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import time
import unittest

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

spec = importlib.util.spec_from_file_location('replay_images', sys.argv.pop(1))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReplayTest(unittest.TestCase):
    def run_replay(self, path, expected):
        rclpy.init(args=['--ros-args', '-p', f'path:={path}', '-p', 'fps:=20.0',
                        '-p', 'image_topic:=/crop_row_test/replay_image'])
        source = None
        client = None
        executor = SingleThreadedExecutor()
        received = []
        try:
            source = module.ReplayImages()
            client = Node('replay_test_client')
            subscription = client.create_subscription(
                Image, '/crop_row_test/replay_image', received.append, qos_profile_sensor_data)
            executor.add_node(source)
            executor.add_node(client)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
                if len(received) >= expected and source.timer.is_canceled():
                    break
            self.assertEqual(len(received), expected)
            self.assertTrue(source.timer.is_canceled())
            for msg in received:
                self.assertEqual(msg.encoding, 'bgr8')
                self.assertEqual((msg.width, msg.height, msg.step), (32, 24, 96))
                self.assertEqual(len(msg.data), 32 * 24 * 3)
                self.assertEqual(msg.header.frame_id, 'camera_optical_frame')
                self.assertGreater(msg.header.stamp.sec, 0)
            stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in received]
            self.assertTrue(all(a < b for a, b in zip(stamps, stamps[1:])))
            client.destroy_subscription(subscription)
        finally:
            executor.shutdown()
            if source is not None:
                source.destroy_node()
            if client is not None:
                client.destroy_node()
            rclpy.shutdown()

    def test_single_image_survives_discovery(self):
        with tempfile.TemporaryDirectory(prefix='crop-row-replay-') as folder:
            image = Path(folder) / 'frame.png'
            self.assertTrue(cv2.imwrite(str(image), np.zeros((24, 32, 3), dtype=np.uint8)))
            self.run_replay(image, 1)

    def test_video_stops_at_end(self):
        with tempfile.TemporaryDirectory(prefix='crop-row-replay-') as folder:
            video = Path(folder) / 'clip.avi'
            writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'MJPG'), 20, (32, 24))
            self.assertTrue(writer.isOpened())
            for value in (30, 60, 90, 120):
                writer.write(np.full((24, 32, 3), value, dtype=np.uint8))
            writer.release()
            self.run_replay(video, 4)


if __name__ == '__main__':
    unittest.main()
