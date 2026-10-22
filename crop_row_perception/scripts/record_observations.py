#!/usr/bin/env python3
"""Record and audit observations without publishing robot commands."""
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from weeder_msgs.msg import CropRowObservation

from observation_contract import ObservationAudit, json_safe


class ObservationRecorder(Node):
    def __init__(self):
        super().__init__('crop_row_recorder')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Recorder requires wall time matching the image producer')
        topic = self.declare_parameter('observation_topic', '/crop_row/observation').value
        self.observation_topic = topic
        output = self.declare_parameter('output_path', '').value
        age = self.declare_parameter('max_frame_age_ms', 250.0).value
        if not output:
            raise ValueError('Provide output_path to a new JSONL file')
        self.audit = ObservationAudit(age)
        path = Path(output).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = path.open('x', encoding='utf-8')
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(CropRowObservation, topic, self.receive, qos)
        self.timer = self.create_timer(5.0, self.report)
        self.get_logger().info(f'Read-only recording to {path}; no control publishers')

    def write(self, value):
        self.stream.write(json.dumps(json_safe(value), ensure_ascii=False, allow_nan=False) + '\n')
        self.stream.flush()

    def receive(self, msg):
        receive_ns = self.get_clock().now().nanoseconds
        record = {key: getattr(msg, key) for key in (
            'session_id', 'sequence', 'has_capture_time', 'replay', 'status', 'sdk_return_code',
            'detail', 'lateral_error_m', 'heading_error_rad', 'lookahead_m', 'confidence',
            'processing_ms', 'frame_age_ms', 'dropped_frames', 'sdk_version', 'calibration_id')}
        record.update(capture_time_ns=msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                      vehicle_frame=msg.header.frame_id)
        result = self.audit.inspect(record, receive_ns, self.count_publishers(self.observation_topic))
        self.write({'schema_version': 1, 'kind': 'observation', 'receive_time_ns': receive_ns,
                    'observation': record, 'audit': result})

    def report(self):
        self.get_logger().info(json.dumps(self.audit.summary(), ensure_ascii=False))

    def destroy_node(self):
        self.write({'schema_version': 1, 'kind': 'summary', **self.audit.summary()})
        self.stream.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = ObservationRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
