import json
import math
import time
from pathlib import Path

import pytest
from tools.field_assistant.attitude import AttitudeReview
from tools.field_assistant.config import AssistantConfig
from tools.field_assistant.sessions import SessionError


@pytest.fixture
def review(tmp_path):
    config = AssistantConfig(workspace=tmp_path, session_root=tmp_path/'sessions',
        mode='hardware', attitude_file=tmp_path/'live.json')
    service = AttitudeReview(config)
    def write(roll=0, pitch=0, **kw):
        s = dict(roll_rad=math.radians(roll), pitch_rad=math.radians(pitch), speed_m_s=0.,
            received_monotonic_ns=time.monotonic_ns(), source_age_ms=10, publishers=1, valid=True,
            installation_id='rig-v1', topic='/gnss_driver/pose', frame_id='odom', bridge_id='bridge1',
            stamp_ns=time.monotonic_ns())
        s.update(kw)
        config.attitude_file.write_text(json.dumps(s))
    write()
    return service, write


def complete(service, write):
    service.action({'action':'baseline'})
    write(pitch=-4)
    service.action({'action':'nose_up', 'performed':True})
    write()
    service.action({'action':'baseline'})
    write(roll=4)
    service.action({'action':'left_up', 'performed':True})


def test_full_direction_review_persists_without_enabling_bev(review):
    s, write = review
    complete(s, write)
    result = s.save({'operator':'tester', 'review_confirmed':True})
    assert result['saved_matches_installation']
    assert result['saved']['scope'] == 'direction_only_not_extrinsic_calibration'
    assert AttitudeReview(s.config).snapshot()['saved_matches_installation']
    assert not (s.config.workspace/'configs/calibration/current').exists()
    write(installation_id='rig-v2')
    assert not s.snapshot()['saved_matches_installation']


@pytest.mark.parametrize('changes', [dict(valid=False), dict(publishers=2), dict(source_age_ms=500),
    dict(received_monotonic_ns=0), dict(roll_rad=float('nan')), dict(speed_m_s=.3), dict(frame_id='')])
def test_invalid_pose_blocks_capture(review, changes):
    s, write = review
    write(**changes)
    assert not s.snapshot()['live']['available']
    with pytest.raises(SessionError): s.action({'action':'baseline'})


@pytest.mark.parametrize('roll,pitch', [(0,4), (4,0), (0,-.5), (4,-4)])
def test_wrong_direction_small_motion_and_cross_axis_rejected(review, roll, pitch):
    s, write = review
    s.action({'action':'baseline'})
    write(roll,pitch)
    result = s.action({'action':'nose_up','performed':True})
    assert not result['evidence']['nose_up']['passed']
    with pytest.raises(SessionError): s.save({'operator':'x','review_confirmed':True})


def test_each_action_requires_baseline_and_human_motion_confirmation(review):
    s, write = review
    with pytest.raises(SessionError): s.action({'action':'nose_up','performed':True})
    s.action({'action':'baseline'})
    write(pitch=-4)
    with pytest.raises(SessionError): s.action({'action':'nose_up'})
    s.action({'action':'nose_up','performed':True})
    with pytest.raises(SessionError): s.action({'action':'left_up','performed':True})


def test_restart_and_recheck_invalidate_acceptance(review):
    s, write = review
    complete(s, write)
    s.save({'operator':'x','review_confirmed':True})
    write()
    s.action({'action':'baseline'})
    assert not s.snapshot()['saved_matches_installation']
    write(bridge_id='bridge2')
    with pytest.raises(SessionError): s.save({'operator':'x','review_confirmed':True})


def test_mock_cannot_save_and_reset_removes_history(review):
    from dataclasses import replace
    s, write = review
    complete(s, write)
    s.config = replace(s.config, mode='mock')
    with pytest.raises(SessionError): s.save({'operator':'x','review_confirmed':True})
    s.config = replace(s.config, mode='hardware')
    s.save({'operator':'x','review_confirmed':True})
    s.action({'action':'reset'})
    assert not s.path.exists()
