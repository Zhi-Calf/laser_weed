"""Integration test: real C ABI/remap/geometry with test-only segmentation backend."""
import importlib.util
import json
import os
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import cv2
import numpy as np

script, library = map(Path, sys.argv[1:])
with tempfile.TemporaryDirectory() as temp:
    root=Path(temp); images=root/'images';images.mkdir()
    model=root/'test-only';model.write_bytes(b'test backend only')
    intrinsic=root/'intrinsic.yaml'
    intrinsic.write_text('''%YAML:1.0
schema_version: 1
calibration_valid: 1
image_width: 320
image_height: 240
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [300.,0.,160.,0.,300.,120.,0.,0.,1.]
distortion_coefficients: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [0.,0.,0.,0.,0.]
''')
    bev=root/'bev.yaml';bev.write_text('''%YAML:1.0
schema_version: 1
calibration_valid: 1
output_width: 320
output_height: 240
meters_per_pixel: 0.02
vehicle_origin_u: 160.
vehicle_origin_v: 239.
image_to_bev: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [1.,0.,0.,0.,1.,0.,0.,0.,1.]
''')
    for i in range(4):
        image=np.zeros((240,320,3),np.uint8)
        if i!=2:
            image[:,128:133,0]=255;image[:,178:183,0]=255
        cv2.imwrite(str(images/f'{i}.png'),image)
    base=[sys.executable,str(script),'--library',str(library),'--model',str(model),'--intrinsics',str(intrinsic),'--bev',str(bev),'--fps','120','--overlay-every','1']
    out=root/'out'
    subprocess.run(base+['--input',str(images),'--output',str(out),'--warmup','1'],check=True)
    summary=json.loads((out/'summary.json').read_text());assert summary['measured_frames']==3
    assert summary['valid_frames']==2 and summary['jumps']==0
    assert json.loads((out/'manifest.json').read_text())['completed']
    records=[json.loads(s) for s in (out/'frames.jsonl').read_text().splitlines()]
    assert len(records)==4 and records[0]['warmup']
    assert abs(records[1]['lateral_error_m']-.1)<.03
    assert records[2]['lateral_error_m'] is None
    assert len(list((out/'overlays').glob('*.jpg')))==3
    assert 'npu_run_ms' not in summary['timing']['stages']
    # Refuse to mix old and new artifacts.
    assert subprocess.run(base+['--input',str(images),'--output',str(out)],capture_output=True).returncode
    video=root/'video.avi';writer=cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'MJPG'),15,(320,240))
    assert writer.isOpened()
    for image_path in sorted(images.iterdir()):writer.write(cv2.imread(str(image_path)))
    writer.release()
    subprocess.run(base+['--input',str(video),'--output',str(root/'video-out'),'--limit','2'],check=True)
    assert json.loads((root/'video-out/summary.json').read_text())['measured_frames']==2
    bad=root/'bad.png';bad.write_bytes(b'not an image')
    failed=root/'failed'
    assert subprocess.run(base+['--input',str(bad),'--output',str(failed)],capture_output=True).returncode
    assert not json.loads((failed/'manifest.json').read_text())['completed']
    if os.environ.get('REPLAY_TEST_ARTIFACTS'):
        shutil.copytree(out,os.environ['REPLAY_TEST_ARTIFACTS'])
    print('Replay ABI, warmup, validity, timing, image/video and artifact checks passed')
