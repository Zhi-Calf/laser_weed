"""Known pixel confusion, polygon parsing and artifact identity tests."""
import sys
from pathlib import Path
import tempfile
import json
import cv2
import numpy as np
sys.path.insert(0,str(Path(sys.argv[1]).resolve().parent))
from evaluate_masks import counts,metrics,rasterize,evaluate
from inspect_segmentation import sha
truth=np.array([[1,1],[0,0]],bool);pred=np.array([[1,0],[1,0]],bool)
assert counts(pred,truth)==dict(tp=1,fp=1,fn=1)
assert metrics(counts(pred,truth))['iou']==1/3
assert metrics(counts(pred,truth))['recall']==.5
assert metrics(counts(truth,truth))['iou']==1
assert metrics(counts(np.zeros_like(truth),np.zeros_like(truth)))['iou'] is None
with tempfile.TemporaryDirectory() as t:
    root=Path(t);run=root/'run';(run/'masks').mkdir(parents=True);labels=root/'labels';labels.mkdir()
    image=root/'sample.png';cv2.imwrite(str(image),np.zeros((10,10,3),np.uint8))
    label=labels/'sample.txt';label.write_text('0 0.1 0.1 0.5 0.1 0.5 0.9 0.1 0.9\n')
    target,regions=rasterize(label,10,10);assert len(regions)==1
    cv2.imwrite(str(run/'masks/0.png'),target.astype(np.uint8)*255)
    manifest=dict(completed=True,inputs=[dict(path=str(image),sha256=sha(image))]);(run/'manifest.json').write_text(json.dumps(manifest))
    result=evaluate(run,labels,root/'out');assert result['micro']['iou']==1 and result['near_micro']['recall']==1
    assert (root/'out/worst/0.png').is_file()
    top=target.copy();top[6:]=False
    cv2.imwrite(str(run/'masks/0.png'),top.astype(np.uint8)*255)
    partial=evaluate(run,labels,root/'partial');assert partial['near_micro']['recall']==0 and partial['micro']['recall']>0
    cv2.imwrite(str(run/'masks/0.png'),np.zeros((10,10),np.uint8))
    result=evaluate(run,labels,root/'empty');assert result['whole_image_misses']==1 and result['regions_below_recall']==1
    label.unlink()
    try:evaluate(run,labels,root/'missing');raise AssertionError('missing label was accepted')
    except FileNotFoundError:raise
    except ValueError:pass
    assert not (root/'missing').exists()
    label.write_text('0 nan 0 1 0 1 1\n')
    try:rasterize(label,10,10);raise AssertionError('NaN accepted')
    except ValueError:pass
    label.write_text('0 0.1 0.1 0.5 0.1 0.5 0.9 0.1 0.9\n')
    image.write_bytes(b'changed')
    try:evaluate(run,labels,root/'changed');raise AssertionError('changed image accepted')
    except ValueError:pass
print('Known pixel metrics, near ROI, empty predictions, labels and identity checks passed')
