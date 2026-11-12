#!/usr/bin/env python3
"""Record model/input identity and segmentation-only SDK replay results."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
from summarize_diagnostics import summarize

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('binary','model','input','output'):p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--backend',choices=['onnx','rknn'],default='onnx')
    p.add_argument('--mask-mode',choices=['fast','full'],default='fast')
    p.add_argument('--limit',type=int,default=50)
    a=p.parse_args()
    if a.limit<1 or a.output.exists():p.error('Positive limit and new output directory required')
    sources=sorted(f for f in a.input.iterdir() if f.is_file() and f.suffix.lower() in {'.jpg','.jpeg','.png'})[:a.limit]
    if not sources:p.error('No input images')
    manifest=dict(schema_version=1,backend=a.backend,limit=a.limit,mask_mode=a.mask_mode,
        model=dict(path=str(a.model.resolve()),sha256=sha(a.model)),
        binary=dict(path=str(a.binary.resolve()),sha256=sha(a.binary)),
        inputs=[dict(path=str(f.resolve()),sha256=sha(f)) for f in sources],
        metric_boundary='Segmentation-only independent images; no calibration or vehicle accuracy',completed=False)
    result=subprocess.run([str(a.binary.resolve()),a.backend,str(a.model.resolve()),str(a.input.resolve()),str(a.output.resolve()),str(a.limit),a.mask_mode],capture_output=True,text=True)
    a.output.mkdir(parents=True,exist_ok=True)
    (a.output/'run.log').write_text(result.stdout+'\n'+result.stderr)
    manifest['return_code']=result.returncode;manifest['completed']=result.returncode==0
    (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    if result.returncode:raise SystemExit('Segmentation replay failed; see run.log')
    with (a.output/'frames.csv').open() as f:rows=list(csv.DictReader(f))
    summary=dict(frames=len(rows),nonempty_frames=sum(float(r['mask_fraction'])>0 for r in rows),
        timing=summarize([dict(available='1',inference_ms=r['inference_ms']) for r in rows]),
        notes='SDK infer wall time includes decoding, excludes image IO. Cold first frame included. Nonempty is not accuracy. Files are sorted, not random sampled.')
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
