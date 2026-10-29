"""Exercise the real HTTP launcher from an unrelated cwd and persistent data root."""
import json
import os
from pathlib import Path
import subprocess
import sys
import urllib.request
import urllib.error


def test_launcher_paths_and_session_restart(tmp_path):
    source = Path(__file__).resolve().parents[2]
    launcher = Path(os.environ.get('CALIBRATION_TEST_LAUNCHER', source / 'scripts/row-nav-calibration'))
    data = tmp_path / 'data with spaces'
    env = dict(os.environ, ROW_NAV_CALIBRATION_DATA=str(data))
    env.pop('PYTHONPATH', None)
    session_id = None
    for attempt in range(2):
        proc = subprocess.Popen([sys.executable, str(launcher), '--port', '0'], cwd=tmp_path,
            env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            line = proc.stdout.readline().strip()
            assert line.startswith('Calibration page: http://127.0.0.1:'), line
            base = line.split(': ', 1)[1]
            def get(route):
                with urllib.request.urlopen(base + route, timeout=5) as response:
                    return response.read()
            assert json.loads(get('/healthz'))['ok']
            assert b'<html' in get('/')
            assert get('/static/bev.js')
            if attempt == 0:
                payload = dict(operator='migration test', square_width_mm=56.5,
                    square_height_mm=56.5, marker_mm=42.2, check_line_mm=100, board_flat=True)
                request = urllib.request.Request(base + '/api/sessions/start',
                    data=json.dumps(payload).encode(), headers={'Content-Type':'application/json'})
                with urllib.request.urlopen(request, timeout=5) as response:
                    session = json.load(response)['session']
                session_id = session['id']
                assert Path(session['path']).is_relative_to(data)
                request = urllib.request.Request(base + '/api/sessions/current/bev/publish',
                    data=b'{"review_confirmed": true}', headers={'Content-Type':'application/json'})
                try:
                    urllib.request.urlopen(request, timeout=5)
                    raise AssertionError('mock publication was allowed')
                except urllib.error.HTTPError as exc:
                    assert exc.code == 400
            else:
                assert json.loads(get('/api/sessions/current'))['session']['id'] == session_id
        finally:
            proc.terminate()
            proc.communicate(timeout=5)
    assert not (data / 'runtime/current').exists()


def test_default_solver_is_packaged_not_workspace(tmp_path):
    from tools.field_assistant.config import AssistantConfig
    config = AssistantConfig(workspace=tmp_path, session_root=tmp_path / 'sessions')
    assert config.effective_calibration_script.is_file()
    assert not config.effective_calibration_script.is_relative_to(tmp_path)
