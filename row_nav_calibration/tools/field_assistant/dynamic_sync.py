"""Capture-time evidence for dynamic calibration; no latest-pose fallback."""
import hashlib
import json
import math
import time

from .attitude import AttitudeReview
from .sessions import SessionError


def checked_installation(config):
    review = AttitudeReview(config)
    state = review.snapshot()
    if config.mode != 'hardware' or not state['saved_matches_installation']:
        raise SessionError('动态标定需要硬件模式及当前安装已通过的姿态方向核查')
    return state['saved'], hashlib.sha256(review.path.read_bytes()).hexdigest()


def capture_sync(config, raw, height):
    review, review_hash = checked_installation(config)
    try:
        stamp = json.loads(raw.with_suffix('.json').read_text())
        image_time = stamp['capture_monotonic_ns']
        if (stamp.get('timestamp_valid') is not True or stamp.get('clock') != 'CLOCK_MONOTONIC' or
            stamp.get('source') != 'v4l2_exposure_start' or not isinstance(image_time, int) or
            not 0 <= (time.monotonic_ns()-image_time)/1e9 <= 10 or
            stamp['raw_sha256'] != hashlib.sha256(raw.read_bytes()).hexdigest()):
            raise ValueError('照片缺少有效的曝光起始时间证据')
        live = json.loads(config.attitude_file.read_text())
        history = live['history']
        if not history or any(history[i]['sample_monotonic_ns'] >= history[i+1]['sample_monotonic_ns']
                              for i in range(len(history)-1)):
            raise ValueError('姿态历史为空或时钟顺序异常')
        causal = [s for s in history if s['sample_monotonic_ns'] <= image_time]
        if not causal:
            raise ValueError('照片之前没有姿态样本')
        sample = causal[-1]  # Never skip a newer invalid sample.
        age = (image_time-sample['sample_monotonic_ns'])/1e6
        if not 0 <= age <= 50:
            raise ValueError('照片与姿态时间差超过 50 ms，请重拍')
        window = [s for s in causal if image_time-s['sample_monotonic_ns'] <= 500_000_000]
        if len(window) < 5 or window[-1]['sample_monotonic_ns']-window[0]['sample_monotonic_ns'] < 300_000_000:
            raise ValueError('拍照前稳定姿态记录不足，请保持静止后重拍')
        for s in window:
            if (s.get('valid') is not True or s.get('publishers') != 1 or
                AttitudeReview.key(s) != review['identity'] or s['bridge_id'] != live['bridge_id'] or
                not all(math.isfinite(s[k]) for k in ('roll_rad','pitch_rad','speed_m_s')) or
                abs(s['speed_m_s']) > .05 or not 0 <= s['source_age_ms'] <= 200):
                raise ValueError('拍照前姿态无效、安装变化或车辆未停稳')
        for key in ('roll_rad','pitch_rad'):
            if max(s[key] for s in window)-min(s[key] for s in window) > math.radians(.5):
                raise ValueError('拍照前姿态不稳定，请停稳后重拍')
        if max(abs(sample['roll_rad']),abs(sample['pitch_rad'])) > math.radians(5):
            raise ValueError('请在平整地面将车辆调整至近水平（横滚俯仰各不超过 5°）')
        return dict(enabled=True, origin_height_m=height, reference_roll_rad=sample['roll_rad'],
            reference_pitch_rad=sample['pitch_rad'], capture=stamp, pose=sample, sync_age_ms=age,
            installation=review['identity'], direction_review_sha256=review_hash,
            measurement_frame='level_ground_rear_axle_vertical_projection',
            output_reference='rear_axle_center', axes='x_forward_y_left_z_up')
    except (OSError, KeyError, TypeError, ValueError) as exc:
        raise SessionError(f'动态照片同步失败：{exc}') from exc
