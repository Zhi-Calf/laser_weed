import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest

spec = importlib.util.spec_from_file_location(
    'observation_contract', Path(__file__).parents[1] / 'scripts' / 'observation_contract.py')
contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(contract)


def sample(**changes):
    record = dict(session_id='a', sequence=1, capture_time_ns=1_000_000_000,
                  has_capture_time=True, replay=False, status=1, sdk_return_code=0,
                  vehicle_frame='base_link', sdk_version='0.1.0', calibration_id='site-v1',
                  lateral_error_m=0.12, heading_error_rad=-0.03, lookahead_m=2.0,
                  confidence=0.9, processing_ms=10.0, frame_age_ms=12.0)
    record.update(changes)
    return record


class ContractTest(unittest.TestCase):
    def test_live_and_replay_are_distinguished(self):
        for replay in (False, True):
            result = contract.ObservationAudit().inspect(sample(replay=replay), 1_020_000_000)
            self.assertTrue(result['contract_ok'])
            self.assertEqual(result['new_live_valid_observation'], not replay)

    def test_duplicate_valid_does_not_refresh_or_count_frame(self):
        audit = contract.ObservationAudit()
        audit.inspect(sample(), 1_020_000_000)
        result = audit.inspect(sample(), 1_030_000_000)
        self.assertFalse(result['new_live_valid_observation'])
        self.assertEqual(audit.summary()['processed_frames'], 1)

    def test_stale_heartbeat_invalidates_same_sequence(self):
        audit = contract.ObservationAudit()
        audit.inspect(sample(), 1_020_000_000)
        value = sample(status=4, confidence=0.0, lateral_error_m=math.nan,
                       heading_error_rad=math.nan, lookahead_m=math.nan)
        result = audit.inspect(value, 1_400_000_000)
        self.assertTrue(result['contract_ok'])
        self.assertFalse(result['new_live_valid_observation'])
        self.assertEqual(audit.summary()['processed_frames'], 1)

    def test_invalid_cannot_leak_navigation_values(self):
        result = contract.ObservationAudit().inspect(sample(status=3), 1_020_000_000)
        self.assertIn('invalid_with_navigation_values', result['contract_errors'])
        self.assertIn('invalid_with_confidence', result['contract_errors'])

    def test_nonfinite_valid_and_sdk_error_are_rejected(self):
        for changes in ({'lateral_error_m': math.nan}, {'processing_ms': math.inf},
                        {'sdk_return_code': -5}, {'has_capture_time': False},
                        {'confidence': 2.0}, {'lookahead_m': -2.0}):
            self.assertFalse(contract.ObservationAudit().inspect(
                sample(**changes), 1_020_000_000)['contract_ok'])

    def test_receive_age_includes_transport_and_future_is_not_fresh(self):
        for now in (1_500_000_000, 999_000_000):
            result = contract.ObservationAudit().inspect(sample(), now)
            self.assertTrue(result['contract_ok'])
            self.assertFalse(result['fresh_at_receive'])
            self.assertFalse(result['new_live_valid_observation'])

    def test_regressed_sequence_and_capture_are_rejected(self):
        audit = contract.ObservationAudit()
        audit.inspect(sample(sequence=2), 1_020_000_000)
        self.assertIn('sequence_regressed', audit.inspect(sample(), 1_030_000_000)['contract_errors'])
        self.assertIn('capture_stamp_not_increasing', audit.inspect(
            sample(sequence=3), 1_030_000_000)['contract_errors'])

    def test_same_sequence_cannot_have_new_capture_time(self):
        audit = contract.ObservationAudit()
        audit.inspect(sample(), 1_020_000_000)
        self.assertIn('sequence_capture_changed', audit.inspect(
            sample(capture_time_ns=1_010_000_000), 1_030_000_000)['contract_errors'])

    def test_session_restart_and_retired_session(self):
        audit = contract.ObservationAudit()
        audit.inspect(sample(sequence=20), 1_020_000_000)
        self.assertTrue(audit.inspect(sample(session_id='b'), 1_020_000_000)['contract_ok'])
        self.assertIn('retired_session', audit.inspect(sample(), 1_030_000_000)['contract_errors'])

    def test_bad_record_does_not_poison_session_state(self):
        audit = contract.ObservationAudit()
        audit.inspect(sample(), 1_020_000_000)
        self.assertFalse(audit.inspect(sample(session_id=''), 1_030_000_000)['contract_ok'])
        self.assertTrue(audit.inspect(sample(sequence=2, capture_time_ns=1_030_000_000),
                                      1_040_000_000)['contract_ok'])

    def test_reference_changes_require_restart(self):
        for key, value in (('calibration_id', 'v2'), ('vehicle_frame', 'other'), ('replay', True)):
            audit = contract.ObservationAudit()
            audit.inspect(sample(), 1_020_000_000)
            changes = {key: value, 'sequence': 2, 'capture_time_ns': 1_030_000_000}
            self.assertIn('reference_changed_within_session', audit.inspect(
                sample(**changes), 1_040_000_000)['contract_errors'])

    def test_multiple_publishers_are_reported(self):
        self.assertIn('publisher_count_not_one', contract.ObservationAudit().inspect(
            sample(), 1_020_000_000, publisher_count=2)['contract_errors'])

    def test_no_input_does_not_count_frame(self):
        value = sample(sequence=0, status=5, has_capture_time=False, capture_time_ns=0,
                       confidence=0, lateral_error_m=None, heading_error_rad=None, lookahead_m=None)
        audit = contract.ObservationAudit()
        self.assertTrue(audit.inspect(value, 1_020_000_000)['contract_ok'])
        self.assertEqual(audit.processed_frames, 0)

    def test_json_is_standard_and_original_is_preserved(self):
        value = sample(lateral_error_m=math.nan, processing_ms=math.inf)
        before = copy.copy(value)
        result = json.loads(json.dumps(contract.json_safe(value), allow_nan=False))
        self.assertIsNone(result['lateral_error_m'])
        self.assertIsNone(result['processing_ms'])
        self.assertTrue(math.isnan(value['lateral_error_m']))
        self.assertEqual(value['sequence'], before['sequence'])


if __name__ == '__main__':
    unittest.main()
