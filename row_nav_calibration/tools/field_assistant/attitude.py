"""Installation direction review, separate from BEV calibration or runtime enablement."""
import json
import math
import os
from pathlib import Path
import threading
import time

from .sessions import SessionError


class AttitudeReview:
    def __init__(self, config):
        self.config = config
        self.lock = threading.RLock()
        self.baseline = None
        self.evidence = {}
        self.identity = None
        self.path = config.workspace / 'attitude' / 'direction-review.json'

    def live(self):
        try:
            if not self.config.attitude_file:
                raise ValueError('未配置姿态数据源')
            sample = json.loads(self.config.attitude_file.read_text())
            age = (time.monotonic_ns() - sample['received_monotonic_ns']) / 1e6
            source_age = sample['source_age_ms']
            if not (0 <= age <= 1000 and 0 <= source_age <= 200):
                raise ValueError('姿态过期或时间异常')
            if sample['valid'] is not True or sample['publishers'] != 1:
                raise ValueError('姿态无效或存在多个发布者')
            for key in ('roll_rad', 'pitch_rad', 'speed_m_s'):
                if not math.isfinite(sample[key]):
                    raise ValueError('姿态包含无效数值')
            if abs(sample['roll_rad']) > math.pi or abs(sample['pitch_rad']) >= math.pi / 2:
                raise ValueError('姿态超出弧度范围，请检查单位')
            if abs(sample['speed_m_s']) > .05:
                raise ValueError('车辆仍在移动，请先停稳')
            if not all(isinstance(sample[k], str) and sample[k].strip()
                       for k in ('installation_id', 'topic', 'frame_id', 'bridge_id')):
                raise ValueError('缺少安装或数据源标识')
            sample.pop("history", None)
            return dict(sample, available=True, age_ms=round(age, 1),
                roll_deg=math.degrees(sample['roll_rad']), pitch_deg=math.degrees(sample['pitch_rad']))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            return {'available': False, 'reason': str(exc)}

    @staticmethod
    def key(sample):
        return {k: sample[k] for k in ('installation_id', 'topic', 'frame_id')}

    def snapshot(self):
        with self.lock:
            sample = self.live()
            saved = None
            try:
                saved = json.loads(self.path.read_text())
            except (OSError, ValueError):
                pass
            matches = bool(sample['available'] and saved and saved.get('accepted') and
                           saved.get('identity') == self.key(sample))
            return {'live': sample, 'baseline': self.baseline, 'evidence': self.evidence,
                    'saved': saved, 'saved_matches_installation': matches}

    def action(self, payload):
        with self.lock:
            action = payload.get('action')
            if action == 'reset':
                self.baseline = None
                self.evidence = {}
                self.identity = None
                self.path.unlink(missing_ok=True)
                return self.snapshot()
            sample = self.live()
            if not sample['available']:
                raise SessionError(sample['reason'])
            identity = self.key(sample)
            if action == 'baseline':
                if self.identity != identity or self.path.exists() or any(
                    v["sample"]["bridge_id"] != sample["bridge_id"] for v in self.evidence.values()
                ):
                    self.evidence = {}
                self.identity = identity
                self.baseline = sample
                # Rechecking invalidates the old acceptance until both actions are saved.
                self.path.unlink(missing_ok=True)
                return self.snapshot()
            if not self.baseline or identity != self.identity or sample['bridge_id'] != self.baseline['bridge_id']:
                raise SessionError('请先记录当前安装的基准姿态；数据源重启后需重新记录')
            if action in ('nose_up', 'left_up'):
                if payload.get('performed') is not True:
                    raise SessionError('请确认实际完成了指定动作')
                if sample['stamp_ns'] <= self.baseline['stamp_ns']:
                    raise SessionError('还没有收到基准之后的新姿态')
                dr = sample['roll_deg'] - self.baseline['roll_deg']
                dp = sample['pitch_deg'] - self.baseline['pitch_deg']
                primary, cross = (-dp, dr) if action == 'nose_up' else (dr, dp)
                passed = 2 <= primary <= 10 and abs(cross) <= max(1, .35 * primary)
                self.evidence[action] = {'passed': passed, 'roll_delta_deg': dr, 'pitch_delta_deg': dp,
                    'baseline': self.baseline, 'sample': sample, 'performed': True}
                self.path.unlink(missing_ok=True)
                self.baseline = None  # Each action starts from its own neutral baseline.
                return self.snapshot()
            if action == 'save':
                raise SessionError('请完成两个动作后使用确认保存')
            raise SessionError('未知的姿态核查操作')

    def save(self, payload):
        with self.lock:
            sample = self.live()
            if not sample['available']:
                raise SessionError(sample['reason'])
            if self.config.mode != 'hardware':
                raise SessionError('模拟模式不能保存正式方向确认')
            if payload.get('review_confirmed') is not True or not str(payload.get('operator', '')).strip():
                raise SessionError('请填写核查人并确认安装方向及驱动配置未改变')
            if self.key(sample) != self.identity or not all(self.evidence.get(k, {}).get('passed') for k in ('nose_up', 'left_up')):
                raise SessionError('当前安装的两项方向核查尚未通过')
            if any(v['sample']['bridge_id'] != sample['bridge_id'] for v in self.evidence.values()):
                raise SessionError('数据源已重启，请重新完成核查')
            result = {'accepted': True, 'identity': self.identity, 'operator': payload['operator'].strip(),
                      'confirmed_at_unix': time.time(), 'axes': 'x_forward_y_left_z_up',
                      'evidence': self.evidence, 'scope': 'direction_only_not_extrinsic_calibration'}
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2))
            os.replace(temporary, self.path)
            return self.snapshot()
