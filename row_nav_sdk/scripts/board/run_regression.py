#!/usr/bin/env python3
"""Offline RK3588 kit inspection/run. Never installs libraries or controls hardware motion."""
import argparse
import csv
import ctypes
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tarfile
import time

ROOT=Path(__file__).resolve().parent

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()

def verify(root):
    checks=json.loads((root/'checksums.json').read_text())
    expected={str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and p.name!='checksums.json'}
    if expected!=set(checks):raise ValueError('Package file list changed')
    for name,digest in checks.items():
        path=root/name
        if root.resolve() not in path.resolve().parents or sha(path)!=digest:raise ValueError('Checksum mismatch: '+name)
    return len(checks)

def command(args,env,timeout=30):
    try:
        r=subprocess.run(args,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=timeout)
        return dict(command=args,returncode=r.returncode,output=r.stdout)
    except (OSError,subprocess.TimeoutExpired) as e:return dict(command=args,returncode=-1,output=str(e))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true',help='Run fixed-image RKNN inference after checks')
    p.add_argument('--rounds',type=int,default=1)
    p.add_argument('--output',type=Path,help='New output directory outside this extracted kit')
    a=p.parse_args()
    if not 1<=a.rounds<=100:p.error('rounds must be 1..100')
    output=(a.output or ROOT.parent/('board-results-'+time.strftime('%Y%m%d-%H%M%S'))).resolve()
    if output.exists() or output==ROOT or ROOT in output.parents:p.error('Use new output outside the kit')
    output.mkdir(parents=True)
    report=dict(mode='board_run' if a.run else 'inspection_only',board_acceptance=False,architecture=platform.machine(),
        kernel=platform.release(),rounds=a.rounds,status='failed',time=time.strftime('%Y-%m-%d %H:%M:%S %z'))
    env=os.environ.copy();env['LD_LIBRARY_PATH']=str(ROOT/'lib')+(':'+env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
    code=1
    try:
        report['verified_files']=verify(ROOT)
        report['package']=json.loads((ROOT/'package-info.json').read_text())
        for name,path in [('os_release','/etc/os-release'),('device_model','/proc/device-tree/model'),('compatible','/proc/device-tree/compatible')]:
            try:report[name]=Path(path).read_bytes().replace(b'\0',b' ').decode(errors='replace')
            except OSError:report[name]='unavailable'
        if not a.run:
            report['status']='package_integrity_ok_not_board_test';code=0
        else:
            if platform.machine().lower() not in ('aarch64','arm64'):raise ValueError('Requires ARM64 RK3588; do not run ARM binaries on this host')
            if 'rk3588' not in (report['device_model']+' '+report['compatible']).lower():raise ValueError('RK3588 model is not approved for this chip; do not use RK3576')
            report['dependencies']=[command(['ldd',str(ROOT/n)],env) for n in ('lib/librk_row_nav.so','bin/segmentation_replay')]
            if any(x['returncode'] or 'not found' in x['output'] for x in report['dependencies']):raise ValueError('Missing runtime dependency; send result archive, do not replace system libraries')
            report['npu_devices']=[dict(path=str(q),readable=os.access(q,os.R_OK),writable=os.access(q,os.W_OK)) for pattern in ('/dev/rknpu','/dev/dri/renderD*') for q in Path('/').glob(pattern.lstrip('/'))]
            # Load only the library shipped with this package, after architecture and deps checks.
            lib=ctypes.CDLL(str(ROOT/'lib/librk_row_nav.so'));lib.rk_row_nav_version.restype=ctypes.c_char_p
            report['sdk_version']=lib.rk_row_nav_version().decode()
            if report['sdk_version']!=report['package']['sdk_version']:raise ValueError('SDK version mismatch')
            model=ROOT/'models/model.rknn';binary=ROOT/'bin/segmentation_replay'
            run=command([str(binary),'rknn',str(model),str(ROOT/'images'),str(output/'inference'),str(report['package']['sample_count']),'fast',str(a.rounds)],env,timeout=1800)
            (output/'inference.log').write_text(run['output']);report['inference']=run
            if run['returncode']:raise ValueError('RKNN inference failed; inspect inference.log')
            with (output/'inference/frames.csv').open() as f:rows=list(csv.DictReader(f))
            expected=report['package']['sample_count']*a.rounds
            if len(rows)!=expected:raise ValueError('Incomplete frame report')
            measured=rows[report['package']['sample_count']:] if a.rounds>1 else rows
            values=sorted(float(r['inference_ms']) for r in measured)
            import math
            if not values or any(not math.isfinite(v) or v<0 for v in values):raise ValueError('Invalid timing samples')
            report['summary']=dict(frames=len(rows),measured_frames=len(values),excluded_first_round=a.rounds>1,
                mean_ms=sum(values)/len(values),p50_ms=values[math.ceil(.5*len(values))-1],p95_ms=values[math.ceil(.95*len(values))-1],max_ms=max(values),
                nonempty_masks=sum(float(r['mask_fraction'])>0 for r in rows))
            report['status']='fixed_image_inference_completed_accuracy_and_vehicle_acceptance_pending';code=0
    except Exception as error:report['error']=str(error)
    finally:
        (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        archive=output.with_suffix('.tar.gz')
        with tarfile.open(archive,'w:gz') as tar:tar.add(output,arcname=output.name)
        print('Result:',output);print('Send this file:',archive);print('Status:',report['status'])
    raise SystemExit(code)
if __name__=='__main__':main()
