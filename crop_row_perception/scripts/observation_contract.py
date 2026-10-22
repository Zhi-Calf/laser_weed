"""Read-only observation audit; not a driving authorization or controller."""
import math
from collections import Counter

STATUS_NAMES = {0: 'INVALID', 1: 'VALID', 2: 'LOW_CONFIDENCE', 3: 'NO_ROW',
                4: 'STALE', 5: 'NO_INPUT', 6: 'INVALID_INPUT', 7: 'SDK_ERROR'}
NAVIGATION_FIELDS = ('lateral_error_m', 'heading_error_rad', 'lookahead_m')


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(item) for item in value]
    return value


class ObservationAudit:
    def __init__(self, max_age_ms=250.0):
        if not finite(max_age_ms) or not 0 < max_age_ms <= 5000:
            raise ValueError('max_age_ms must be finite and in (0, 5000]')
        self.max_age_ms = max_age_ms
        self.session = None
        self.retired_sessions = set()
        self.sequence = 0
        self.capture_ns = 0
        self.reference = None
        self.messages = 0
        self.processed_frames = 0
        self.valid_frames = 0
        self.errors = Counter()
        self.statuses = Counter()

    def inspect(self, record, receive_time_ns, publisher_count=1):
        self.messages += 1
        errors = []
        status = record['status']
        session = record['session_id']
        sequence = record['sequence']
        capture = record['capture_time_ns']
        if status not in STATUS_NAMES:
            errors.append('unknown_status')
        if publisher_count != 1:
            errors.append('publisher_count_not_one')
        if not session:
            errors.append('missing_session')
        if not record['vehicle_frame'] or not record['sdk_version'] or not record['calibration_id']:
            errors.append('missing_reference_or_version')
        reference = (record['vehicle_frame'], record['sdk_version'], record['calibration_id'], record['replay'])
        new_session = session != self.session and session not in self.retired_sessions
        if session in self.retired_sessions:
            errors.append('retired_session')
        if not new_session and session == self.session and reference != self.reference:
            errors.append('reference_changed_within_session')
        previous_sequence = 0 if new_session else self.sequence
        previous_capture = 0 if new_session else self.capture_ns
        new_sequence = sequence > previous_sequence
        if sequence < 0 or (sequence == 0 and status != 5):
            errors.append('invalid_sequence')
        if not new_session and sequence < previous_sequence:
            errors.append('sequence_regressed')
        if record['has_capture_time'] and capture <= 0:
            errors.append('invalid_capture_stamp')
        processed = record['has_capture_time'] and status in (0, 1, 2, 3, 4, 7)
        if new_sequence and processed and capture <= previous_capture:
            errors.append('capture_stamp_not_increasing')
        if processed and sequence == previous_sequence and previous_capture and capture != previous_capture:
            errors.append('sequence_capture_changed')
        if status == 1:
            if record['sdk_return_code'] != 0:
                errors.append('valid_with_sdk_error')
            if not record['has_capture_time']:
                errors.append('valid_without_capture')
            fields = (*NAVIGATION_FIELDS, 'confidence', 'processing_ms', 'frame_age_ms')
            if not all(finite(record[key]) for key in fields):
                errors.append('valid_nonfinite_payload')
            else:
                if record['lookahead_m'] <= 0 or not 0 <= record['confidence'] <= 1:
                    errors.append('valid_out_of_range')
                if record['processing_ms'] < 0 or record['frame_age_ms'] < 0:
                    errors.append('negative_duration')
        else:
            if record['confidence'] != 0:
                errors.append('invalid_with_confidence')
            if any(record[key] is not None and not (
                    isinstance(record[key], float) and math.isnan(record[key]))
                   for key in NAVIGATION_FIELDS):
                errors.append('invalid_with_navigation_values')
        age_ms = (receive_time_ns - capture) / 1e6 if record['has_capture_time'] else None
        fresh = age_ms is not None and 0 <= age_ms <= self.max_age_ms
        new_frame = new_sequence and processed and not errors
        if not errors:
            if new_session:
                if self.session is not None:
                    self.retired_sessions.add(self.session)
                self.session = session
                self.sequence = self.capture_ns = 0
                self.reference = reference
            self.sequence = max(self.sequence, sequence)
            if new_frame:
                self.capture_ns = capture
                self.processed_frames += 1
                if status == 1:
                    self.valid_frames += 1
        self.statuses[STATUS_NAMES.get(status, 'UNKNOWN')] += 1
        self.errors.update(errors)
        return {
            'contract_errors': errors, 'contract_ok': not errors,
            'status_name': STATUS_NAMES.get(status, 'UNKNOWN'),
            'new_processed_frame': new_frame, 'receive_age_ms': age_ms,
            'fresh_at_receive': fresh,
            # Merely identifies new live data. It never means permission to move.
            'new_live_valid_observation': new_frame and status == 1 and fresh and not record['replay'],
        }

    def summary(self):
        return {'messages': self.messages, 'processed_frames': self.processed_frames,
                'valid_frames': self.valid_frames, 'status_messages': dict(self.statuses),
                'contract_errors': dict(self.errors)}
