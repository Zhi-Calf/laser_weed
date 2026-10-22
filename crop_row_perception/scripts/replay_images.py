#!/usr/bin/env python3
"""Wall-paced image/video replay for observation tests, not live camera capture."""
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ReplayImages(Node):
    def __init__(self):
        super().__init__('crop_row_replay')
        path = self.declare_parameter('path', '').value
        fps = float(self.declare_parameter('fps', 15.0).value)
        self.frame_id = self.declare_parameter('camera_frame', 'camera_optical_frame').value
        topic = self.declare_parameter('image_topic', '/crop_row/replay/image').value
        self.repeat = self.declare_parameter('repeat', False).value
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Replay requires wall time; simulated clock is unsupported')
        if not Path(path).is_file() or not 0.1 <= fps <= 120:
            raise ValueError('Provide a readable image/video file and fps in [0.1, 120]')
        self.image = None
        self.video = None
        if Path(path).suffix.lower() in {'.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'}:
            self.image = cv2.imread(path, cv2.IMREAD_COLOR)
            if self.image is None:
                raise ValueError('Image decode failed')
        else:
            self.video = cv2.VideoCapture(path)
            if not self.video.isOpened():
                raise ValueError('Video open failed')
        self.pub = self.create_publisher(Image, topic, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info('REPLAY timestamps are playback time, not original acquisition time')

    def tick(self):
        # A single image must not disappear before DDS discovery completes.
        if self.pub.get_subscription_count() == 0:
            return
        frame = self.image
        if self.video is not None:
            ok, frame = self.video.read()
            if not ok and self.repeat:
                self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.video.read()
            if not ok:
                self.timer.cancel()
                self.get_logger().info('End of replay; observer should subsequently report STALE')
                return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)
        if self.image is not None and not self.repeat:
            self.timer.cancel()

    def destroy_node(self):
        if self.video is not None:
            self.video.release()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = ReplayImages()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
