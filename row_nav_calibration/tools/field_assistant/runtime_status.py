"""File publication is evidence of a default version, never of a running SDK load."""
import hashlib
import json
from pathlib import Path


def candidate_fingerprint(root: Path) -> str | None:
    files = ('snapshot.json', 'points.json', 'summary.json', 'intrinsics.yaml', 'candidate-bev.yaml')
    try:
        hashes = {name: hashlib.sha256((root/'bev'/name).read_bytes()).hexdigest() for name in files}
    except OSError:
        return None
    return hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()


def publication_state(root: Path, summary, active, published) -> dict:
    fingerprint = candidate_fingerprint(root)
    release = published.get('release_id') if published else None
    matches = bool(fingerprint and isinstance(release, str) and release and published.get('candidate_fingerprint') == fingerprint)
    default = active.get('release_id') if active else None
    default = default if isinstance(default, str) and default else None
    current = bool(matches and default == release)
    if (root/'bev/draft.json').exists():
        candidate = 'draft'
    elif not summary:
        candidate = 'not_started'
    elif summary.get('checks_passed') is not True:
        candidate = 'checks_failed'
    elif current:
        candidate = 'published_default'
    elif matches:
        candidate = 'published_replaced'
    else:
        candidate = 'pending_review'
    return {'candidate': candidate, 'default_release': default,
            'current_candidate_published': matches,
            'runtime_load': 'unknown', 'runtime_reason': '未接入算法进程加载回执，不能由默认参数文件推断已加载'}
