#!/usr/bin/env python3
"""Compare a C++ OpenCV tensor dump with this Python OpenCV on the exact saved input."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from inspect_segmentation import sha

def main():
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('model','dump','output'):p.add_argument('--'+n,type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('Output must be new')
    blob=np.fromfile(a.dump/'input.bin',np.float32).reshape(1,3,640,640)
    if not np.isfinite(blob).all():raise ValueError('Invalid input')
    net=cv2.dnn.readNetFromONNX(str(a.model));net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV);net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU);net.setInput(blob)
    rows=[]
    for line in (a.dump/'tensors.tsv').read_text().splitlines():
        index,name,*shape=line.split('\t')
        if not index.isdecimal():raise ValueError('Invalid dump index')
        shape=tuple(map(int,shape));old=np.fromfile(a.dump/(index+'.bin'),np.float32).reshape(shape)
        new=net.forward(name)
        if new.shape!=old.shape or not np.isfinite(old).all() or not np.isfinite(new).all():raise ValueError('Invalid tensor shape/values')
        rows.append(dict(name=name,shape=shape,max_abs=float(np.max(np.abs(old-new))),mean_abs=float(np.mean(np.abs(old-new)))))
    if not rows:raise ValueError('No tensors')
    result=dict(model_sha256=sha(a.model),input_sha256=sha(a.dump/'input.bin'),old_opencv=(a.dump/'opencv.txt').read_text().strip(),
        new_opencv=cv2.__version__,outputs=rows,notes='Exact saved float input. Runtime builds differ; no root-cause operator identified.')
    a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
