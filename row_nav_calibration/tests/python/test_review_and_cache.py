import json
from pathlib import Path
import sys
import threading
import urllib.request
import urllib.error

import cv2
import numpy as np
import pytest

from tools.field_assistant.config import AssistantConfig
from tools.field_assistant.server import FieldAssistantServer
from tools.field_assistant.sessions import SessionStore, SessionError


def store_at(root):
    config=AssistantConfig(workspace=root,session_root=root/'sessions',calibration_python=Path(sys.executable),preview_path=root/'preview.jpg')
    store=SessionStore(config)
    session=store.start(dict(operator='review',square_width_mm=56.5,square_height_mm=56.5,
        marker_mm=42.2,check_line_mm=100,board_flat=True),{})
    return store,Path(session['path'])


def test_review_preserves_accepted_image_identity_and_rejects_path_escape(tmp_path):
    store,root=store_at(tmp_path)
    (root/'raw/intrinsic/train-002.jpg').write_bytes(b'training')
    (root/'raw/validation/validation-003.jpg').write_bytes(b'validation')
    (root/'validation/intrinsic').mkdir()
    (root/'validation/intrinsic/validation-001-compare.jpg').write_bytes(b'paired')
    (root/'intrinsic-sources.json').write_text(json.dumps({'intrinsic/train-002.jpg':{'image':'position-08.jpg','slot':8}}))
    summary={'training':{'per_view':[{'image':'/old/path/train-002.jpg','reprojection_rms_px':1.2},
                                  {'image':'../../secret','reprojection_rms_px':0}]},
             'validation':{'per_view':[{'image':'/old/path/validation-003.jpg','reprojection_rms_px':.2}]}}
    (root/'validation/intrinsic-summary.json').write_text(json.dumps(summary))
    rows=store.describe()['intrinsic_review']
    assert len(rows)==2 and rows[0]['slot']==8
    assert rows[1]['comparison_available']
    assert store.intrinsic_image(root.name,'validation',0,'comparison').read_bytes()==b'paired'
    assert store.intrinsic_image(root.name,'validation',0,'original').read_bytes()==b'validation'
    for group,index,kind in [('validation',3,'original'),('intrinsic',0,'comparison'),('../',0,'original'),('validation',0,'../')]:
        with pytest.raises(SessionError):store.intrinsic_image(root.name,group,index,kind)


def test_legacy_training_report_does_not_guess_rejected_image_alignment(tmp_path):
    store,root=store_at(tmp_path)
    for i in range(3):(root/f'raw/intrinsic/train-{i+1:03d}.jpg').write_bytes(b'image')
    assert store.intrinsic_review(root.name,{'training':{'usable':2,'matched':3,'per_view_rms_px':[.1,.2]}})==[]


def test_thumbnail_size_cache_and_revision_change_on_retake(tmp_path):
    store,root=store_at(tmp_path)
    cv2.imwrite(str(tmp_path/'preview.jpg'),np.full((1080,1920,3),160,np.uint8))
    result=store.capture({'group':'intrinsic','slot':0})
    revision=result['session']['capture_versions']['intrinsic']['0']
    thumb=store.capture_image(root.name,'intrinsic',0,'thumbnail')
    assert max(cv2.imread(str(thumb)).shape[:2])<=192
    old_bytes=thumb.read_bytes()
    assert len(store._images_in(thumb.parent))==1
    server=FieldAssistantServer(('127.0.0.1',0),store.config)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    url=f'http://127.0.0.1:{server.server_port}/api/capture-image?session={root.name}&group=intrinsic&slot=0&kind=thumbnail&v={revision}'
    try:
        with urllib.request.urlopen(url) as response:
            etag=response.headers['ETag']
            assert 'immutable' in response.headers['Cache-Control']
            assert response.read()==old_bytes
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(urllib.request.Request(url,headers={'If-None-Match':etag}))
        assert error.value.code==304
        cv2.imwrite(str(tmp_path/'preview.jpg'),np.full((1080,1920,3),90,np.uint8))
        fresh=store.capture({'group':'intrinsic','slot':0})['session']['capture_versions']['intrinsic']['0']
        assert fresh!=revision and thumb.read_bytes()==old_bytes
        assert store.capture_image(root.name,'intrinsic',0,'thumbnail').read_bytes()!=old_bytes
        with urllib.request.urlopen(url) as response:
            assert response.headers['Cache-Control']=='no-store'
        assert store.describe()['capture_counts']['intrinsic']==1
    finally:
        server.shutdown();server.server_close();thread.join(timeout=3)
