"""Exercise the read-only recorder with real ROS messages and strict JSONL."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from weeder_msgs.msg import CropRowObservation

scripts = Path(__file__).parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
spec = importlib.util.spec_from_file_location('record_observations', scripts / 'record_observations.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RecorderTest(unittest.TestCase):
    def test_valid_then_stale_are_recorded_without_control_publishers(self):
        with tempfile.TemporaryDirectory(prefix='crop-row-record-') as folder:
            output = Path(folder) / 'observations.jsonl'
            rclpy.init(args=['--ros-args', '-p', f'output_path:={output}',
                            '-p', 'observation_topic:=/crop_row_test/record'])
            recorder = None
            client = None
            executor = SingleThreadedExecutor()
            try:
                recorder = module.ObservationRecorder()
                client = Node('crop_row_recorder_test')
                pub = client.create_publisher(CropRowObservation, '/crop_row_test/record', 10)
                executor.add_node(recorder)
                executor.add_node(client)

                def wait(predicate):
                    deadline = time.monotonic() + 4
                    while time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=0.01)
                        if predicate():
                            return True
                    return False

                self.assertTrue(wait(lambda: pub.get_subscription_count() == 1 and
                                     recorder.count_publishers('/crop_row_test/record') == 1))
                msg = CropRowObservation()
                msg.header.stamp = client.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.session_id = 'recorder-test'
                msg.sequence = 1
                msg.has_capture_time = True
                msg.status = msg.VALID
                msg.lateral_error_m = 0.12
                msg.heading_error_rad = -0.03
                msg.lookahead_m = 2.0
                msg.confidence = 0.9
                msg.processing_ms = 10.0
                msg.frame_age_ms = 12.0
                msg.sdk_version = 'test-only'
                msg.calibration_id = 'test-site'
                pub.publish(msg)
                self.assertTrue(wait(lambda: recorder.audit.messages == 1))
                msg.status = msg.STALE
                msg.confidence = 0.0
                msg.lateral_error_m = msg.heading_error_rad = msg.lookahead_m = float('nan')
                pub.publish(msg)
                self.assertTrue(wait(lambda: recorder.audit.messages == 2))
                executor.remove_node(recorder)
                recorder.destroy_node()
                recorder = None
                text = output.read_text()
                self.assertNotIn('NaN', text)
                records = [json.loads(line) for line in text.splitlines()]
                self.assertTrue(records[0]['audit']['new_live_valid_observation'])
                self.assertFalse(records[1]['audit']['new_live_valid_observation'])
                self.assertIsNone(records[1]['observation']['lateral_error_m'])
                self.assertEqual(records[-1]['processed_frames'], 1)
                self.assertEqual(records[-1]['contract_errors'], {})
            finally:
                executor.shutdown()
                if recorder is not None:
                    recorder.destroy_node()
                if client is not None:
                    client.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
