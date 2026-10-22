"""Run the real production executable and dynamically linked SDK error paths."""
import subprocess
import sys

cases = [
    ([], 'invalid observer config'),
    (['--ros-args', '-p', 'model_path:=/nonexistent-crop-row-test/model.onnx',
      '-p', 'intrinsic_path:=/nonexistent-crop-row-test/intrinsics.yaml',
      '-p', 'bev_path:=/nonexistent-crop-row-test/bev.yaml',
      '-p', 'calibration_id:=intentional-startup-error-test'], 'RowNav create failed'),
]
for binary in sys.argv[1:]:
    for arguments, expected in cases:
        result = subprocess.run([binary, *arguments], capture_output=True, text=True, timeout=5)
        output = result.stdout + result.stderr
        if result.returncode != 1 or expected not in output:
            raise AssertionError(f'Unexpected startup result {result.returncode}: {output}')
print('Production executable, real SDK loading and startup rejection passed')
