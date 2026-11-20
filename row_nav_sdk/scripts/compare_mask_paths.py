#!/usr/bin/env python3
"""Paired masks: pixel agreement and image scanline fragmentation/centre diagnostics.
Not ground-truth accuracy or vehicle-coordinate error.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from inspect_segmentation import sha

def centers(row):
    edges=np.diff(np.pad(row.astype(np.int8),(1,1)))
    starts=np.flatnonzero(edges==1);ends=np.flatnonzero(edges==-1)-1
    return (starts+ends)/2

def compare(a,b):
    ma=json.loads((a/'manifest.json').read_text());mb=json.loads((b/'manifest.json').read_text())
    if ma.get('completed') is not True or mb.get('completed') is not True:raise ValueError('Incomplete run')
    n=min(len(ma['inputs']),len(mb['inputs']))
    if not n:raise ValueError('No paired images')
    if ma['inputs'][:n]!=mb['inputs'][:n]:raise ValueError('Input identities or order differ')
    rows=[];shifts=[];same_count=0;different_count=0;eligible=0
    for i in range(n):
        arrays=[]
        for r in (a,b):
            m=cv2.imread(str(r/'masks'/f'{i}.png'),0)
            if m is None or not np.isin(m,[0,255]).all():raise ValueError('Invalid binary mask')
            arrays.append(m>0)
        x,y=arrays
        if x.shape!=y.shape:raise ValueError('Shape mismatch')
        union=np.count_nonzero(x|y);delta=[];mismatch=0;matched=0
        for v in range(x.shape[0]//5,x.shape[0],8):
            cx,cy=centers(x[v]),centers(y[v])
            if not len(cx) and not len(cy):continue
            eligible+=1
            if len(cx)!=len(cy):different_count+=1;mismatch+=1;continue
            same_count+=1;matched+=1
            delta.extend(np.abs(cx-cy).tolist())
        shifts.extend(delta)
        rows.append(dict(index=i,iou=float(np.count_nonzero(x&y)/union) if union else 1.,
            equal_nonempty_scanlines=matched,different_run_count_scanlines=mismatch,
            conditional_max_center_shift_px=max(delta) if delta else None))
    result=dict(paired_images=n,mean_mask_iou=float(np.mean([r['iou'] for r in rows])),
        minimum_mask_iou=min(r['iou'] for r in rows),eligible_scanlines=eligible,
        equal_nonempty_run_count_scanlines=same_count,different_run_count_scanlines=different_count,
        conditional_center_samples=len(shifts),conditional_center_p50_px=float(np.median(shifts)) if shifts else None,
        conditional_center_p95_px=float(np.percentile(shifts,95)) if shifts else None,
        manifests_sha256=[sha(a/'manifest.json'),sha(b/'manifest.json')],per_image=rows,
        notes='Run centres compared by horizontal order only when counts agree; this is conditional pixel evidence, not fitted path/vehicle error. Run-count changes can reflect cropping or fragmentation; no physical distance implied.')
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('a','b','output'):p.add_argument('--'+n,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('Output exists')
    result=compare(args.a,args.b);args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='per_image'},indent=2))
