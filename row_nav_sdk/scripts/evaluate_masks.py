#!/usr/bin/env python3
"""Pixel consistency against derived single-class YOLO polygons, not navigation accuracy."""
import argparse
import json
import math
from pathlib import Path
import cv2
import numpy as np
from inspect_segmentation import sha

def rasterize(path, width, height):
    mask=np.zeros((height,width),np.uint8);regions=[]
    for number,line in enumerate(path.read_text().splitlines(),1):
        if not line.strip():continue
        values=line.split()
        if values[0]!='0' or len(values)<7 or (len(values)-1)%2:raise ValueError(f'Invalid polygon at {path}:{number}')
        coordinates=np.array([float(v) for v in values[1:]],dtype=np.float64).reshape(-1,2)
        if not np.isfinite(coordinates).all() or (coordinates<0).any() or (coordinates>1).any():raise ValueError(f'Invalid coordinates: {path}:{number}')
        points=np.rint(coordinates*np.array([width,height])).astype(np.int32)
        points[:,0]=np.clip(points[:,0],0,width-1);points[:,1]=np.clip(points[:,1],0,height-1)
        region=np.zeros_like(mask);cv2.fillPoly(region,[points],1)
        if cv2.countNonZero(region)==0:raise ValueError('Empty polygon: '+str(path))
        mask|=region;regions.append(region.astype(bool))
    return mask.astype(bool),regions

def counts(pred,truth):
    return dict(tp=int(np.count_nonzero(pred&truth)),fp=int(np.count_nonzero(pred&~truth)),fn=int(np.count_nonzero(~pred&truth)))
def metrics(c):
    tp,fp,fn=c['tp'],c['fp'],c['fn']
    return dict(iou=tp/(tp+fp+fn) if tp+fp+fn else None,
        precision=tp/(tp+fp) if tp+fp else None,recall=tp/(tp+fn) if tp+fn else None,
        f1=2*tp/(2*tp+fp+fn) if 2*tp+fp+fn else None)
def read_prediction(path):
    mask=cv2.imread(str(path),cv2.IMREAD_UNCHANGED)
    if mask is None or mask.ndim!=2 or mask.dtype!=np.uint8 or not np.isin(mask,[0,255]).all():raise ValueError('Expected uint8 binary mask: '+str(path))
    return mask>0

def evaluate(run,labels,output,near_fraction=.333333333,minimum_region_recall=.5,worst_count=12):
    if output.exists():raise ValueError('Output must be new')
    if not 0<near_fraction<=1 or not 0<minimum_region_recall<=1 or worst_count<1:raise ValueError('Invalid thresholds')
    manifest=json.loads((run/'manifest.json').read_text())
    if manifest.get('completed') is not True:raise ValueError('Incomplete inference run')
    rows=[]
    for index,source in enumerate(manifest['inputs']):
        image=Path(source['path']);prediction=run/'masks'/f'{index}.png';label=labels/(image.stem+'.txt')
        if sha(image)!=source['sha256']:raise ValueError('Input changed: '+str(image))
        if not label.is_file():raise ValueError('Missing annotation (not treated as background): '+str(label))
        pred=read_prediction(prediction);height,width=pred.shape
        original=cv2.imread(str(image))
        if original is None or original.shape[:2]!=pred.shape:raise ValueError('Image/mask shape mismatch')
        truth,regions=rasterize(label,width,height)
        cut=height-math.ceil(height*near_fraction)
        c=counts(pred,truth);near=counts(pred[cut:],truth[cut:])
        coverage=[float(np.count_nonzero(pred&r))/int(np.count_nonzero(r)) for r in regions]
        rows.append(dict(index=index,source=str(image),source_sha256=source['sha256'],
            label=str(label),label_sha256=sha(label),mask_sha256=sha(prediction),counts=c,**metrics(c),
            near_counts=near,near=metrics(near),regions=len(regions),
            regions_below_recall=sum(v<minimum_region_recall for v in coverage),region_recalls=coverage))
    if not rows:raise ValueError('No inputs')
    def aggregate(key):return metrics({n:sum(r[key][n] for r in rows) for n in ('tp','fp','fn')})
    valid=[r['iou'] for r in rows if r['iou'] is not None]
    summary=dict(inference_manifest_sha256=sha(run/'manifest.json'), images=len(rows),micro=aggregate('counts'),macro_iou=sum(valid)/len(valid) if valid else None,
        images_with_defined_iou=len(valid),near_micro=aggregate('near_counts'),near_fraction=near_fraction,
        minimum_region_recall=minimum_region_recall,regions=sum(r['regions'] for r in rows),
        regions_below_recall=sum(r['regions_below_recall'] for r in rows),
        whole_image_misses=sum(r['counts']['tp']==0 and r['counts']['fn']>0 for r in rows),
        notes='Derived polygon consistency only. Near=bottom image fraction, not metric distance. Region recall threshold is a diagnostic, not detection mAP.')
    worst=sorted(rows,key=lambda r: (r['iou'] if r['iou'] is not None else 2,r['index']))[:worst_count]
    near_worst=sorted((r for r in rows if r['near']['recall'] is not None),key=lambda r:(r['near']['recall'],r['index']))[:worst_count]
    output.mkdir(parents=True);(output/'worst').mkdir()
    for row in {r['index']:r for r in worst+near_worst}.values():
        image=cv2.imread(row['source']);pred=read_prediction(run/'masks'/f'{row["index"]}.png')
        truth,_=rasterize(Path(row['label']),image.shape[1],image.shape[0]);tint=image.copy()
        tint[pred&truth]=(0,255,0);tint[~pred&truth]=(0,0,255);tint[pred&~truth]=(255,0,0)
        overlay=cv2.addWeighted(image,.5,tint,.5,0)
        canvas=cv2.copyMakeBorder(overlay,0,65,0,max(0,800-image.shape[1]),cv2.BORDER_CONSTANT)
        fmt=lambda value: f'{value:.4f}' if value is not None else 'n/a'
        cv2.putText(canvas,f'{Path(row["source"]).name} IoU={fmt(row["iou"])} near_recall={fmt(row["near"]["recall"])}',(8,image.shape[0]+20),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
        cv2.putText(canvas,'Green=TP Red=FN Blue=FP (derived label, not vegetation ground truth)',(8,image.shape[0]+45),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
        if not cv2.imwrite(str(output/'worst'/f'{row["index"]}.png'),canvas):raise ValueError('Cannot save gallery')
    summary['worst_iou_indices']=[r['index'] for r in worst];summary['worst_near_recall_indices']=[r['index'] for r in near_worst]
    (output/'per-image.json').write_text(json.dumps(rows,indent=2,allow_nan=False)+'\n')
    (output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('run','labels','output'):p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--near-fraction',type=float,default=1/3);p.add_argument('--minimum-region-recall',type=float,default=.5)
    a=p.parse_args();print(json.dumps(evaluate(a.run,a.labels,a.output,a.near_fraction,a.minimum_region_recall),indent=2))
