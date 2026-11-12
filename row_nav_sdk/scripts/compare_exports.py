#!/usr/bin/env python3
"""Paired square-image export audit using local Ultralytics reference and OpenCV DNN.
Requires the exact local exporter package on PYTHONPATH. Does not install packages.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
import torch
import ultralytics
import inspect
import onnx
from ultralytics import YOLO
from ultralytics.utils import ops
from inspect_segmentation import sha

def mask(pred,proto,shape):
    selected=ops.non_max_suppression(pred.clone(),conf_thres=.25,iou_thres=.7,nc=1,max_det=16)[0]
    if not len(selected):return np.zeros(shape,np.uint8)
    boxes=ops.scale_boxes((640,640),selected[:,:4].clone(),shape)
    pixels=ops.process_mask_native(proto[0],selected[:,6:],boxes,shape)
    return pixels.any(dim=0).cpu().numpy().astype(np.uint8)*255

def delta(a,b):
    a=np.asarray(a);b=np.asarray(b)
    if a.shape!=b.shape:raise ValueError(f'Shape mismatch {a.shape} != {b.shape}')
    if not np.isfinite(a).all() or not np.isfinite(b).all():raise ValueError('Non-finite output')
    return float(np.max(np.abs(a-b)))

def overlap(a,b):
    a=a>0;b=b>0;union=np.count_nonzero(a|b)
    return float(np.count_nonzero(a&b)/union) if union else 1.

def main():
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('weights','standard','optimized','run','output'):p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--limit',type=int,default=50)
    a=p.parse_args()
    if a.output.exists() or a.limit<1:p.error('New output and positive limit required')
    prior=json.loads((a.run/'manifest.json').read_text())
    if prior.get('completed') is not True or sha(a.optimized)!=prior['model']['sha256']:raise ValueError('Mismatched optimized model or incomplete run')
    sources=prior['inputs'][:a.limit]
    if not sources:raise ValueError('No images')
    torch.set_num_threads(4);cv2.setNumThreads(4)
    model=YOLO(str(a.weights)).model.cpu().float().eval();head=model.model[-1];head.export=True
    standard=cv2.dnn.readNetFromONNX(str(a.standard));optimized=cv2.dnn.readNetFromONNX(str(a.optimized))
    for net in (standard,optimized):
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV);net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    standard_names=[o.name for o in onnx.load(str(a.standard)).graph.output]
    optimized_names=[o.name for o in onnx.load(str(a.optimized)).graph.output]
    a.output.mkdir(parents=True)
    manifest=dict(schema_version=1,inputs=sources,completed=False,preprocessing='square RGB /255 resize640; CPU float32',
        files={n:dict(path=str(getattr(a,n).resolve()),sha256=sha(getattr(a,n))) for n in ('weights','standard','optimized')},
        ultralytics=ultralytics.__version__,export_head_sha256=sha(Path(inspect.getfile(type(head)))),torch=torch.__version__,opencv=cv2.__version__,conf=.25,iou=.7,max_det=16,reference='local Ultralytics native mask reconstruction',
        sdk_inference_manifest_sha256=sha(a.run/'manifest.json'))
    for name in ('pytorch','standard'):(a.output/name/'masks').mkdir(parents=True)
    (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    records=[]
    with torch.inference_mode():
        for i,source in enumerate(sources):
            path=Path(source['path'])
            if sha(path)!=source['sha256']:raise ValueError('Input hash changed')
            image=cv2.imread(str(path));h,w=image.shape[:2]
            if h!=w:raise ValueError('Audit currently requires square inputs (no padding ambiguity)')
            blob=cv2.dnn.blobFromImage(image,1/255.,(640,640),swapRB=True)
            head.format='onnx'; pt_pred,pt_proto=model(torch.from_numpy(blob))
            standard.setInput(blob);outputs=standard.forward(standard_names)
            on_pred=next(t for t in outputs if t.ndim==3);on_proto=next(t for t in outputs if t.ndim==4)
            pt_mask=mask(pt_pred,pt_proto,(h,w));on_mask=mask(torch.from_numpy(on_pred),torch.from_numpy(on_proto),(h,w))
            head.format='rknn';raw=model(torch.from_numpy(blob))
            optimized.setInput(blob);raw_onnx=optimized.forward(optimized_names)
            # ONNX graph output order is exporter contract, never guessed by dimensions.
            errors=[delta(x.cpu().numpy(),y) for x,y in zip(raw,raw_onnx)]
            if len(raw)!=13 or len(raw_onnx)!=13:raise ValueError('Expected 13 optimized outputs')
            sdk=cv2.imread(str(a.run/'masks'/f'{i}.png'),0)
            if sdk is None or sdk.shape!=(h,w):raise ValueError('Invalid SDK mask')
            if not cv2.imwrite(str(a.output/'pytorch/masks'/f'{i}.png'),pt_mask) or not cv2.imwrite(str(a.output/'standard/masks'/f'{i}.png'),on_mask):raise ValueError('Cannot save mask')
            records.append(dict(index=i,source=str(path),decoded_max_abs=delta(pt_pred.cpu().numpy(),on_pred),
                proto_max_abs=delta(pt_proto.cpu().numpy(),on_proto),optimized_output_max_abs=errors,
                pytorch_standard_mask_iou=overlap(pt_mask,on_mask),pytorch_sdk_mask_iou=overlap(pt_mask,sdk)))
    manifest['completed']=True
    for name in ('pytorch','standard'):(a.output/name/'manifest.json').write_text(json.dumps(dict(manifest,reference_backend=name),indent=2))
    (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    (a.output/'per-image.json').write_text(json.dumps(records,indent=2))
    summary=dict(images=len(records),pytorch_standard_mean_mask_iou=float(np.mean([r['pytorch_standard_mask_iou'] for r in records])),
        pytorch_sdk_mean_mask_iou=float(np.mean([r['pytorch_sdk_mask_iou'] for r in records])),
        max_decoded_abs=max(r['decoded_max_abs'] for r in records),max_proto_abs=max(r['proto_max_abs'] for r in records),
        max_optimized_abs=max(max(r['optimized_output_max_abs']) for r in records),
        notes='Mask agreement is pairwise, not label accuracy. Raw box errors are in input pixel units. Empty/empty pair agreement=1.')
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
