"""Run after sourcing ROS 2 and the built weeder_msgs workspace; no hardware required."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    import rclpy
    from weeder_msgs.msg import Pose
    package = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(package))
    from tools.field_assistant.config import AssistantConfig
    from tools.field_assistant.attitude import AttitudeReview

    topic = '/row_nav_calibration_test/pose'
    with tempfile.TemporaryDirectory(prefix='calibration-ros-bridge-') as directory:
        root = Path(directory)
        output = root / 'live.json'
        process = subprocess.Popen([sys.executable, str(package / 'scripts/row-nav-attitude-bridge'),
            '--output', str(output), '--installation-id', 'synthetic-ROS-test-only', '--topic', topic])
        rclpy.init()
        node = rclpy.create_node('calibration_bridge_test_publisher')
        publisher = node.create_publisher(Pose, topic, 10)
        try:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError('Attitude bridge stopped before receiving test messages')
                message = Pose()
                message.header.stamp = node.get_clock().now().to_msg()
                message.header.frame_id = 'odom'
                message.valid = True
                message.roll, message.pitch, message.speed_m_s = .03, -.02, 0.
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=.05)
                if output.exists():
                    sample = json.loads(output.read_text())
                    if sample['valid'] and sample['publishers'] == 1 and len(sample['history']) >= 8:
                        assert abs(sample['roll_rad'] - .03) < 1e-9
                        assert abs(sample['pitch_rad'] + .02) < 1e-9
                        assert sample['sample_monotonic_ns'] <= time.monotonic_ns()
                        break
            else:
                raise RuntimeError('No valid ROS attitude history within 15 seconds')
            review = AttitudeReview(AssistantConfig(
                workspace=root, session_root=root / 'sessions', attitude_file=output))
            assert review.live()['available'], 'Fresh ROS pose was rejected'
            time.sleep(1.2)
            assert not review.live()['available'], 'Stopped pose stream remained available'
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
            process.terminate()
            try:
                code = process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise RuntimeError('Bridge failed to stop within 5 seconds')
            assert code == 0, f'Bridge shutdown returned {code}'
    print('PASS: ROS Pose transport, bridge history, calibration reader, stale rejection and clean shutdown')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
