#!/usr/bin/env python3
"""Offline SDK replay. Timestamps are playback monotonic time, not sensor time."""
import argparse
import csv
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import time

from summarize_diagnostics import summarize, FIELDS

U, Q, D = C.c_uint32, C.c_uint64, C.c_double
class Config(C.Structure):
    _fields_ = [('struct_size', U), ('abi_version', U)] + [(n, C.c_char_p) for n in ('model_path', 'intrinsic_path', 'bev_path')] + [(n, U) for n in ('backend', 'model_width', 'model_height')] + [('detection_confidence', C.c_float), ('minimum_navigation_confidence', C.c_float), ('max_frame_age_ms', D)]
class Frame(C.Structure):
    _fields_ = [('struct_size', U), ('abi_version', U), ('data', C.POINTER(C.c_uint8)), ('data_size_bytes', Q)] + [(n, U) for n in ('width', 'height', 'stride_bytes', 'pixel_format')] + [('capture_time_ns', Q), ('frame_id', Q)]
class Result(C.Structure):
    _fields_ = [('struct_size', U), ('abi_version', U), ('capture_time_ns', Q), ('frame_id', Q), ('observation_status', U), ('reserved', U)] + [(n, D) for n in ('lateral_error_m', 'heading_error_rad', 'lookahead_m', 'confidence', 'processing_ms', 'frame_age_ms')]
class Diagnostics(C.Structure):
    _fields_ = [(n, U) for n in ('struct_size', 'abi_version', 'available', 'detailed_inference')] + [('frame_id', Q)] + [(n, D) for n in FIELDS]

class DebugRow(C.Structure):
    _fields_=[('id',Q),('slope',D),('intercept_m',D),('rms_m',D),('inlier_count',Q)]
class Debug(C.Structure):
    _fields_=[(n,U) for n in ('struct_size','abi_version','available','observation_status')] + [('frame_id',Q),('capture_time_ns',Q)] + [(n,U) for n in ('mask_width','mask_height','instance_count','total_rows','returned_rows')] + [('selected_left',C.c_int32),('selected_right',C.c_int32)] + [(n,U) for n in ('pair_valid','centerline_valid','reserved')] + [(n,D) for n in ('joint_left_slope','joint_left_intercept_m','joint_right_slope','joint_right_intercept_m','centerline_rms_m')] + [('centerline_inliers',Q),('centerline_points',Q),('rows',DebugRow*32)]

def load_sdk(path):
    lib = C.CDLL(str(Path(path).resolve()))
    for name, typ in [('config', Config), ('frame', Frame), ('result', Result), ('diagnostics', Diagnostics)]:
        fn = getattr(lib, 'rk_row_nav_'+name+'_init'); fn.argtypes = [C.POINTER(typ)]; fn.restype = None
    lib.rk_row_nav_create.argtypes = [C.POINTER(Config), C.POINTER(C.c_void_p)]
    lib.rk_row_nav_process.argtypes = [C.c_void_p, C.POINTER(Frame), C.POINTER(Result)]
    lib.rk_row_nav_get_diagnostics.argtypes = [C.c_void_p, C.POINTER(Diagnostics)]
    lib.rk_row_nav_dynamic_bev_enabled.argtypes = [C.c_void_p]
    lib.rk_row_nav_destroy.argtypes = [C.c_void_p]; lib.rk_row_nav_destroy.restype = None
    lib.rk_row_nav_now_ns.restype = Q
    lib.rk_row_nav_version.restype = C.c_char_p
    lib.rk_row_nav_get_last_error.restype = C.c_char_p
    return lib

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1024*1024), b''): h.update(block)
    return h.hexdigest()

def clean(value):
    return value if not isinstance(value, float) or math.isfinite(value) else None

def frames(path):
    import cv2
    extensions = {'.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'}
    if path.is_dir() or path.suffix.lower() in extensions:
        paths = sorted(p for p in path.iterdir() if p.suffix.lower() in extensions) if path.is_dir() else [path]
        if not paths: raise ValueError('No images found')
        for p in paths:
            image = cv2.imread(str(p))
            if image is None: raise ValueError('Image decode failed: '+str(p))
            yield str(p), sha(p), image
    else:
        capture = cv2.VideoCapture(str(path))
        try:
            if not capture.isOpened(): raise ValueError('Cannot open video')
            index = 0
            while True:
                ok, image = capture.read()
                if not ok: break
                yield str(path)+'#'+str(index), None, image
                index += 1
            if index == 0: raise ValueError('Video contains no decodable frames')
        finally: capture.release()

def run(args):
    import cv2
    import numpy as np
    if not math.isfinite(args.fps) or not .1 <= args.fps <= 120: raise ValueError('fps must be in [0.1,120]')
    if args.limit < 1 or args.warmup < 0 or args.overlay_every < 1: raise ValueError('Invalid frame limits')
    for name in ('input', 'library', 'model', 'intrinsics', 'bev'):
        if not getattr(args, name).exists(): raise ValueError('Missing '+name)
    if args.input.is_dir() and (args.output.resolve()==args.input.resolve() or args.input.resolve() in args.output.resolve().parents):
        raise ValueError('Output must be outside the image input directory')
    args.output.mkdir(parents=True, exist_ok=False)
    lib = load_sdk(args.library)
    config = Config(); lib.rk_row_nav_config_init(C.byref(config))
    # Keep encoded path storage alive for create.
    paths = [str(p.resolve()).encode() for p in (args.model,args.intrinsics,args.bev)]
    config.model_path, config.intrinsic_path, config.bev_path = paths
    config.backend = 1 if args.backend=='onnx' else 2
    config.max_frame_age_ms = args.max_age_ms
    handle = C.c_void_p()
    manifest = dict(debug_masks=args.debug_masks, schema_version=1, sdk=lib.rk_row_nav_version().decode(), backend=args.backend,
        timestamp_mode='paced_playback_monotonic_not_sensor_time', fps=args.fps, warmup=args.warmup,
        limit=args.limit, max_frame_age_ms=args.max_age_ms, overlay='navigation_text_only_no_pixel_mask',
        independent_images=args.independent_images,
        files={name:dict(path=str(getattr(args,name).resolve()), sha256=sha(getattr(args,name))) for name in ('library','model','intrinsics','bev')},
        input=str(args.input.resolve()), video_sha256=sha(args.input) if args.input.is_file() else None,
        completed=False)
    def save(name, data): (args.output/name).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    rows=[]; previous=None; counts={}; jumps=0
    try:
        save('manifest.json',manifest)
        if lib.rk_row_nav_create(C.byref(config),C.byref(handle)):
            raise RuntimeError(lib.rk_row_nav_get_last_error().decode())
        if lib.rk_row_nav_dynamic_bev_enabled(handle): raise ValueError('Dynamic BEV requires recorded synchronized attitude; this replay supports static calibration only')
        save('manifest.json',manifest)
        if args.debug_masks:
            lib.rk_row_nav_debug_init.argtypes=[C.POINTER(Debug)]; lib.rk_row_nav_debug_init.restype=None
            lib.rk_row_nav_set_debug_enabled.argtypes=[C.c_void_p,U]
            lib.rk_row_nav_get_debug.argtypes=[C.c_void_p,C.POINTER(Debug)]
            lib.rk_row_nav_copy_debug_mask.argtypes=[C.c_void_p,Q,C.POINTER(C.c_uint8),Q,C.POINTER(Q)]
            if lib.rk_row_nav_set_debug_enabled(handle,1): raise RuntimeError('Cannot enable debug')
            (args.output/'debug-masks').mkdir()
        (args.output/'overlays').mkdir()
        with (args.output/'frames.jsonl').open('w') as log:
            last_submit=None
            for index,(source,digest,image) in enumerate(frames(args.input)):
                if index >= args.limit+args.warmup: break
                if args.independent_images and index:
                    lib.rk_row_nav_destroy(handle); handle=C.c_void_p()
                    if lib.rk_row_nav_create(C.byref(config),C.byref(handle)): raise RuntimeError(lib.rk_row_nav_get_last_error().decode())
                    if args.debug_masks and lib.rk_row_nav_set_debug_enabled(handle,1): raise RuntimeError('Cannot enable debug')
                if last_submit is not None:
                    time.sleep(max(0,1/args.fps-(time.monotonic()-last_submit)))
                frame=Frame(); lib.rk_row_nav_frame_init(C.byref(frame))
                image=np.ascontiguousarray(image)
                frame.data=image.ctypes.data_as(C.POINTER(C.c_uint8)); frame.data_size_bytes=image.nbytes
                frame.height,frame.width=image.shape[:2]; frame.stride_bytes=image.strides[0]
                frame.frame_id=index+1; frame.capture_time_ns=lib.rk_row_nav_now_ns(); last_submit=time.monotonic()
                result=Result(); lib.rk_row_nav_result_init(C.byref(result))
                code=lib.rk_row_nav_process(handle,C.byref(frame),C.byref(result))
                error=lib.rk_row_nav_get_last_error().decode() if code else ''
                diag=Diagnostics(); lib.rk_row_nav_diagnostics_init(C.byref(diag))
                diag_code=lib.rk_row_nav_get_diagnostics(handle,C.byref(diag))
                record=dict(index=index,source=source,source_sha256=digest,warmup=index<args.warmup,
                    return_code=code,error=error,status=int(result.observation_status),frame_id=int(frame.frame_id),
                    capture_time_ns=int(frame.capture_time_ns),
                    **{n:clean(getattr(result,n)) for n in ('lateral_error_m','heading_error_rad','confidence','processing_ms','frame_age_ms')})
                record.update({n:getattr(diag,n) for n in FIELDS})
                record['available']=int(diag_code==0 and diag.available==1)
                record['detailed_inference']=int(diag.detailed_inference)
                valid=code==0 and result.observation_status==1 and all(math.isfinite(v) for v in (result.lateral_error_m,result.heading_error_rad))
                if args.debug_masks:
                    debug=Debug();lib.rk_row_nav_debug_init(C.byref(debug))
                    if lib.rk_row_nav_get_debug(handle,C.byref(debug)): raise RuntimeError('Cannot get debug data')
                    record['debug_available']=bool(debug.available)
                    if debug.available:
                        if debug.frame_id!=frame.frame_id: raise RuntimeError('Debug frame mismatch')
                        record['debug']={n:clean(getattr(debug,n)) for n in ('observation_status','instance_count','total_rows','returned_rows','selected_left','selected_right','pair_valid','centerline_valid','joint_left_slope','joint_left_intercept_m','joint_right_slope','joint_right_intercept_m','centerline_rms_m','centerline_inliers','centerline_points')}
                        record['debug']['rows']=[{n:clean(getattr(row,n)) for n,_ in DebugRow._fields_} for row in debug.rows[:debug.returned_rows]]
                        required=Q()
                        if lib.rk_row_nav_copy_debug_mask(handle,frame.frame_id,None,0,C.byref(required)): raise RuntimeError('Cannot query debug mask')
                        if required.value!=debug.mask_width*debug.mask_height: raise RuntimeError('Invalid mask size')
                        pixels=(C.c_uint8*required.value)()
                        if lib.rk_row_nav_copy_debug_mask(handle,frame.frame_id,pixels,required.value,C.byref(required)): raise RuntimeError('Cannot copy debug mask')
                        mask=np.ctypeslib.as_array(pixels).reshape(debug.mask_height,debug.mask_width)
                        if not cv2.imwrite(str(args.output/'debug-masks'/f'{index:06d}.png'),mask): raise RuntimeError('Cannot save debug mask')
                record['jump']=False
                if not record['warmup']:
                    counts[str(code if code else result.observation_status)]=counts.get(str(code if code else result.observation_status),0)+1
                    if valid and previous is not None and not args.independent_images:
                        record['jump']=abs(result.lateral_error_m-previous[0])>args.jump_m or abs(result.heading_error_rad-previous[1])>math.radians(args.jump_deg)
                        jumps+=int(record['jump'])
                    previous=(result.lateral_error_m,result.heading_error_rad) if valid else None
                    record['valid']=valid; rows.append(record)
                    if (index-args.warmup)%args.overlay_every==0:
                        canvas=cv2.copyMakeBorder(image,0,110,0,max(0,800-image.shape[1]),cv2.BORDER_CONSTANT)
                        lines=[f'OFFLINE REPLAY frame={frame.frame_id} status={record["status"]} code={code}',
                            f'lateral={result.lateral_error_m:.3f} m heading={result.heading_error_rad:.4f} rad',
                            'Navigation annotation only - no pixel mask exposed by C SDK']
                        for line_no,text in enumerate(lines): cv2.putText(canvas,text,(8,image.shape[0]+24+line_no*24),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
                        if not cv2.imwrite(str(args.output/'overlays'/f'{index:06d}.jpg'),canvas): raise RuntimeError('Overlay write failed')
                log.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n');log.flush()
        if not rows: raise ValueError('No measured frames remain after warmup')
        with (args.output/'timing.csv').open('w',newline='') as out:
            names=['available','detailed_inference',*FIELDS];writer=csv.DictWriter(out,fieldnames=names,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
        timing=summarize([{k:str(v) for k,v in row.items()} for row in rows])
        save('summary.json',dict(measured_frames=len(rows),valid_frames=sum(r['valid'] for r in rows),
            valid_rate=sum(r['valid'] for r in rows)/len(rows),status_counts=counts,jumps=jumps,
            jump_threshold_m=args.jump_m,jump_threshold_deg=args.jump_deg,timing=timing,
            metric_boundary='No ground truth: validity and jumps are not navigation accuracy'))
        manifest['completed']=True;save('manifest.json',manifest)
    except Exception as error:
        manifest['error']=str(error); save('manifest.json',manifest)
        raise
    finally: lib.rk_row_nav_destroy(handle)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('input','library','model','intrinsics','bev','output'): p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--backend',choices=['onnx','rknn'],default='onnx')
    p.add_argument('--fps',type=float,default=15);p.add_argument('--limit',type=int,default=300)
    p.add_argument('--warmup',type=int,default=0);p.add_argument('--overlay-every',type=int,default=10)
    p.add_argument('--max-age-ms',type=float,default=250)
    p.add_argument('--jump-m',type=float,default=.15);p.add_argument('--jump-deg',type=float,default=5)
    p.add_argument('--debug-masks',action='store_true',help='Opt-in SDK debug masks in undistorted input coordinates; requires 0.2.2')
    p.add_argument('--independent-images',action='store_true',help='Reset handle between unrelated images; disable temporal jump metrics')
    args=p.parse_args()
    if any(not math.isfinite(v) or v<=0 for v in (args.max_age_ms,args.jump_m,args.jump_deg)): p.error('Thresholds must be finite and positive')
    try:
        run(args)
    except (ValueError, RuntimeError, OSError, AttributeError) as error:
        p.exit(1, 'Replay failed: '+str(error)+'\n')
if __name__=='__main__': main()
