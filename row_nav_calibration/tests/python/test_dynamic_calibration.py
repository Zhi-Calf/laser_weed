import hashlib
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pytest

# Worker modules intentionally run with their own directory on sys.path.
sys.path.insert(0, str(Path(__file__).parents[2]/'tools/field_assistant'))
from dynamic_solver import orientation, body_points, homography, image_to_body, solve, validate_file
from tools.field_assistant.config import AssistantConfig
from tools.field_assistant.dynamic_sync import capture_sync
from tools.field_assistant.sessions import SessionError, SessionStore
from tools.field_assistant.bev import BevService


def fixture():
    k=np.array([[800.,0,960],[0,800,540],[0,0,1.]])
    rotation=orientation(0,.55)@np.array([[0,0,1],[-1,0,0],[0,-1,0.]])
    position=np.array([.4,.1,1.])
    sync=dict(reference_roll_rad=.02,reference_pitch_rad=-.03,origin_height_m=.45)
    xy=np.array([[x,y] for x in (2.,3.,4.,5.) for y in (-.6,.6)]+[[x,y] for x in (2.5,4.5) for y in (-.3,.3)])
    fit=np.array([True]*8+[False]*4)
    body=body_points(xy,sync)
    camera=(body-position)@rotation
    pixels=camera@k.T;uv=pixels[:,:2]/pixels[:,2:]
    return k,rotation,position,sync,xy,fit,uv


def test_pnp_recovers_known_extrinsics_and_matches_independent_3d_tilts():
    k,r,t,s,xy,fit,uv=fixture()
    h=homography(k,r,t,s['origin_height_m'],s['reference_roll_rad'],s['reference_pitch_rad'])
    result=solve(k,uv,xy,fit,np.ones((8,1)),s,h)
    assert np.allclose(result['camera_to_vehicle_rotation'],r,atol=1e-6)
    assert np.allclose(result['camera_position_vehicle_m'],t,atol=1e-6)
    for roll,pitch in ((-.08,.07),(.07,-.09),(0,0)):
        rr=orientation(roll,pitch)
        body=(np.c_[xy,np.zeros(len(xy))]-[0,0,s['origin_height_m']])@rr
        camera=(body-t)@r
        pixels=camera@k.T
        recovered=image_to_body(homography(k,r,t,s['origin_height_m'],roll,pitch),pixels[:,:2]/pixels[:,2:])
        assert np.max(np.linalg.norm(recovered-body[:,:2],axis=1))<1e-8


def test_pnp_independent_check_cannot_be_used_to_fit_away_errors():
    k,r,t,s,xy,fit,uv=fixture()
    h=homography(k,r,t,s['origin_height_m'],s['reference_roll_rad'],s['reference_pitch_rad'])
    uv[~fit]+=100
    with pytest.raises(ValueError,match='独立'): solve(k,uv,xy,fit,np.ones((8,1)),s,h)


def sync_fixture(tmp_path):
    config=AssistantConfig(workspace=tmp_path,session_root=tmp_path/'sessions',mode='hardware',
        attitude_file=tmp_path/'live.json')
    config.workspace.joinpath('attitude').mkdir()
    identity=dict(installation_id='rig1',topic='/gnss_driver/pose',frame_id='odom')
    review=config.workspace/'attitude/direction-review.json'
    review.write_text(json.dumps(dict(accepted=True,identity=identity)))
    now=time.monotonic_ns(); image_time=now-30_000_000
    history=[dict(**identity,bridge_id='bridge',valid=True,publishers=1,roll_rad=.02,pitch_rad=-.03,
        speed_m_s=0,source_age_ms=5,stamp_ns=image_time-i*50_000_000,
        sample_monotonic_ns=image_time-i*50_000_000,received_monotonic_ns=now) for i in range(8,-1,-1)]
    live=dict(history[-1],history=history);config.attitude_file.write_text(json.dumps(live))
    raw=tmp_path/'image.jpg';raw.write_bytes(b'raw image fixture')
    metadata=dict(timestamp_valid=True,clock='CLOCK_MONOTONIC',source='v4l2_exposure_start',
        capture_monotonic_ns=image_time,raw_sha256=hashlib.sha256(raw.read_bytes()).hexdigest())
    raw.with_suffix('.json').write_text(json.dumps(metadata))
    return config,raw,live,metadata


def test_capture_sync_uses_image_timestamp_not_latest_pose(tmp_path):
    c,raw,live,m=sync_fixture(tmp_path)
    future=dict(live['history'][-1],sample_monotonic_ns=m['capture_monotonic_ns']+10_000_000,pitch_rad=.08)
    live['history'].append(future);c.attitude_file.write_text(json.dumps(live))
    s=capture_sync(c,raw,.45)
    assert s['reference_pitch_rad']==-.03
    assert s['sync_age_ms']==0


@pytest.mark.parametrize('bad', ['timestamp','stale','invalid','moving','unstable','identity','image','future_only'])
def test_capture_sync_rejects_bad_evidence(tmp_path,bad):
    c,raw,live,m=sync_fixture(tmp_path)
    if bad=='timestamp': m['timestamp_valid']=False
    if bad=='stale':
        for p in live['history']: p['sample_monotonic_ns']-=100_000_000
    if bad=='invalid': live['history'][-1]['valid']=False
    if bad=='moving': live['history'][-1]['speed_m_s']=.2
    if bad=='unstable': live['history'][0]['roll_rad']+=.1
    if bad=='identity': live['history'][-1]['installation_id']='changed'
    if bad=='image': raw.write_bytes(b'changed')
    if bad=='future_only':
        for p in live['history']: p['sample_monotonic_ns']+=2_000_000_000
    raw.with_suffix('.json').write_text(json.dumps(m));c.attitude_file.write_text(json.dumps(live))
    with pytest.raises(SessionError): capture_sync(c,raw,.45)


def test_full_dynamic_prepare_calculate_publish_and_tamper(tmp_path,monkeypatch):
    c,raw,live,m=sync_fixture(tmp_path)
    from dataclasses import replace
    k,r,t,s,xy,fit,uv=fixture()
    intrinsic=tmp_path/'camera.yaml'
    fs=cv2.FileStorage(str(intrinsic),cv2.FILE_STORAGE_WRITE)
    for name,value in dict(camera_matrix=k,distortion_coefficients=np.zeros((1,5)),image_width=1920,image_height=1080).items(): fs.write(name,value)
    fs.release()
    c=replace(c,calibration_path=intrinsic,calibration_python=Path(sys.executable))
    store=SessionStore(c);store.start(dict(operator='test',square_width_mm=56.5,square_height_mm=56.5,marker_mm=42.2,check_line_mm=100,board_flat=True),{})
    service=BevService(store)
    def fake_capture(payload):
        image=service.root()/'captures/bev/position-00.jpg';image.parent.mkdir(parents=True,exist_ok=True)
        cv2.imwrite(str(image),np.full((1080,1920,3),180,dtype=np.uint8))
        m['raw_sha256']=hashlib.sha256(image.read_bytes()).hexdigest()
        image.with_suffix('.json').write_text(json.dumps(m))
    monkeypatch.setattr(store,'capture',fake_capture)
    session=service.prepare(dict(reference_confirmed=True,intrinsic_confirmed=True,intrinsic_source='configured',
        dynamic_enabled=True,dynamic_reference_confirmed=True,origin_height_m=.45))
    image_id=session['bev']['snapshot']['image_id']
    points=[dict(u=float(p[0]),v=float(p[1]),x=float(q[0]),y=float(q[1]),role='fit' if f else 'check') for p,q,f in zip(uv,xy,fit)]
    result=service.calculate(dict(image_id=image_id,points=points))
    assert result['bev']['summary']['dynamic']['validated']
    # Refresh liveness; capture evidence remains unchanged.
    live['received_monotonic_ns']=time.monotonic_ns();c.attitude_file.write_text(json.dumps(live))
    service.publish(dict(image_id=image_id,review_confirmed=True))
    current=c.effective_runtime_calibration_root/'current'
    assert validate_file(current/'bev.yaml',current/'intrinsics.yaml')
    fs=cv2.FileStorage(str(current/'bev.yaml'),cv2.FILE_STORAGE_READ)
    assert fs.getNode('dynamic_bev').getNode('calibration_valid').real()==1;fs.release()
    overlay=(current/'observer.calibration.yaml').read_text()
    assert 'dynamic_bev.enabled: true' in overlay
    assert 'attitude_gate.enabled: true' in overlay
    assert 'attitude_gate.reference_pitch_rad: -0.03' in overlay
    previous=current.resolve()
    (service.root()/'bev/dynamic-sync.json').write_text('{}')
    with pytest.raises(SessionError): service.publish(dict(image_id=image_id,review_confirmed=True))
    assert current.resolve()==previous
