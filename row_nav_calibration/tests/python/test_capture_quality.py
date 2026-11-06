import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

from tools.field_assistant.capture_quality import assess
from tools.field_assistant.config import AssistantConfig
from tools.field_assistant.sessions import SessionStore


@pytest.fixture
def board_image(tmp_path):
    board = cv2.aruco.CharucoBoard((7, 10), .0565, .0422,
                                 cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000))
    image = np.full((1200, 1400), 160, np.uint8)
    image[100:1100, 350:1050] = board.generateImage((700, 1000), marginSize=0)
    path = tmp_path / 'board.png'
    cv2.imwrite(str(path), image)
    return path


def test_quality_detects_corners_and_preserves_original(board_image, tmp_path):
    original = board_image.read_bytes()
    report = assess(board_image, tmp_path/'overlay.jpg', .0565, .0422)
    assert report['corners'] == 54
    assert report['raw_sha256'] == hashlib.sha256(original).hexdigest()
    assert report['overlay_available'] and report['advisory_only']
    assert (tmp_path/'overlay.jpg').is_file()
    assert board_image.read_bytes() == original


@pytest.mark.parametrize('level,warning', [(0, '纯黑'), (255, '纯白')])
def test_extreme_exposure_is_advisory(tmp_path, level, warning):
    path = tmp_path/'frame.jpg'
    cv2.imwrite(str(path), np.full((300,400),level,np.uint8))
    report = assess(path,tmp_path/'overlay.jpg',.0565,.0422)
    assert report['state'] == 'review' and report['corners'] == 0
    assert any(warning in message for message in report['warnings'])


def test_blur_hint_and_unreadable_file(board_image,tmp_path):
    blurred = cv2.GaussianBlur(cv2.imread(str(board_image)), (0,0), 10)
    path=tmp_path/'blurred.jpg'; cv2.imwrite(str(path),blurred)
    report=assess(path,tmp_path/'overlay.jpg',.0565,.0422)
    assert any('清晰度' in message for message in report['warnings'])
    path.write_bytes(b'invalid image')
    report=assess(path,tmp_path/'missing.jpg',.0565,.0422)
    assert '解码' in report['warnings'][0]
    assert not report['overlay_available']


def store_for(root, preview, python=Path(sys.executable)):
    store=SessionStore(AssistantConfig(workspace=root,session_root=root/'sessions',
        preview_path=preview,calibration_python=python))
    store.start(dict(operator='quality test',square_width_mm=56.5,square_height_mm=56.5,
        marker_mm=42.2,check_line_mm=100,board_flat=True),{})
    return store


def test_overlay_never_enters_training_images_or_counts(board_image,tmp_path):
    store=store_for(tmp_path,board_image)
    result=store.capture(dict(group='intrinsic',slot=0))
    directory=Path(result['session']['path'])/'captures/intrinsic'
    assert result['session']['capture_counts']['intrinsic'] == 1
    assert len(store._images_in(directory)) == 1
    assert result['session']['capture_quality']['intrinsic']['0']['corners'] == 54
    assert store.capture_image(result['session']['id'],'intrinsic',0,'corners').is_file()
    assert (directory/'position-00.jpg').read_bytes() == board_image.read_bytes()
    timestamp = directory/'position-00.json'
    original_timestamp = b'{"capture_time_ns":12345,"clock":"monotonic"}\n'
    timestamp.write_bytes(original_timestamp)
    store._assess_capture(directory/'position-00.jpg', result['session']['metadata']['board'])
    assert timestamp.read_bytes() == original_timestamp
    assert (directory/'position-00.jpg').read_bytes() == board_image.read_bytes()



def test_worker_failure_keeps_frame_and_bev_skips_diagnostic_worker(board_image,tmp_path,monkeypatch):
    store=store_for(tmp_path,board_image,Path('/missing/calibration-python'))
    result=store.capture(dict(group='validation',slot=0))
    report=result['session']['capture_quality']['validation']['0']
    assert report['state'] == 'unavailable'
    assert store.capture_image(result['session']['id'],'validation',0).read_bytes() == board_image.read_bytes()
    def forbidden(*args):
        raise AssertionError('BEV exposure sync must not wait for quality analysis')
    monkeypatch.setattr(store,'_assess_capture',forbidden)
    store.capture(dict(group='bev',slot=0))
