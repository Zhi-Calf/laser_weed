from pathlib import Path
import json

import cv2
import numpy as np
import pytest

from tools.field_assistant.bev import BevService
from tools.field_assistant.config import AssistantConfig
from tools.field_assistant.sessions import SessionError, SessionStore


def fixture_files(root):
    root.mkdir(parents=True, exist_ok=True)
    image = np.full((600, 800, 3), 180, dtype=np.uint8)
    points = []
    for role, xs, ys in (("fit", [1, 2, 3, 4], [-1, 1]), ("check", [1.5, 3.5], [-.5, .5])):
        for x in xs:
            for y in ys:
                u, v = 400-y*100, 600-x*70
                points.append(dict(u=u, v=v, x=x, y=y, role=role))
                cv2.circle(image, (int(u), int(v)), 7, (0, 160, 20) if role == "fit" else (0, 130, 250), -1)
    cv2.imwrite(str(root / "ground.jpg"), image)
    fs = cv2.FileStorage(str(root / "intrinsics.yaml"), cv2.FILE_STORAGE_WRITE)
    for key, value in {"camera_matrix": np.array([[500., 0, 400], [0, 500, 300], [0, 0, 1]]),
                       "distortion_coefficients": np.zeros((1, 5)), "image_width": 800, "image_height": 600}.items():
        fs.write(key, value)
    fs.release()
    return points


@pytest.fixture
def bev(tmp_path):
    import sys
    points = fixture_files(tmp_path)
    config = AssistantConfig(workspace=tmp_path, session_root=tmp_path / "sessions",
        preview_path=tmp_path / "ground.jpg", calibration_path=tmp_path / "intrinsics.yaml",
        calibration_python=Path(sys.executable))
    sessions = SessionStore(config)
    sessions.start(dict(operator="test", square_width_mm=56.5, square_height_mm=56.5,
        marker_mm=42.2, check_line_mm=100, board_flat=True), {})
    service = BevService(sessions)
    session = service.prepare(dict(reference_confirmed=True, intrinsic_confirmed=True, intrinsic_source="configured"))
    return service, dict(image_id=session["bev"]["snapshot"]["image_id"], points=points)


def test_bev_projects_to_rear_axle_axes_and_exports_candidate(bev):
    service, payload = bev
    session = service.calculate(payload)
    summary = session["bev"]["summary"]
    assert summary["checks_passed"]
    assert summary["check"]["rmse_m"] < 1e-5
    assert summary["calibration_valid"] is False
    fs = cv2.FileStorage(str(service.root() / "bev/candidate-bev.yaml"), cv2.FILE_STORAGE_READ)
    h = fs.getNode("image_to_bev").mat()
    assert fs.getNode("calibration_valid").real() == 0
    assert fs.getNode("coordinate_reference").string() == "rear_axle_ground"
    fs.release()
    pixel = np.array([[[300., 460.]]])  # x=2, y=1 relative to rear axle
    uv = cv2.perspectiveTransform(pixel, h)[0, 0]
    assert (799-uv[1])*.01 == pytest.approx(2, abs=1e-5)
    assert (320-uv[0])*.01 == pytest.approx(1, abs=1e-5)
    exported = service.sessions.export_current({})
    assert exported["download_ready"]
    service.save_points(payload)
    assert not service.sessions.describe()["download_ready"]
    assert service.sessions.describe()["bev"]["summary"] is None


def test_independent_check_error_is_not_hidden_by_fitting(bev):
    service, payload = bev
    payload["points"][-1]["x"] += .3
    result = service.calculate(payload)["bev"]["summary"]
    assert not result["checks_passed"]
    assert result["check"]["max_m"] > .29


@pytest.mark.parametrize("problem", ["stale", "nan", "duplicate", "missing", "outside", "role"])
def test_bev_rejects_bad_points(bev, problem):
    service, payload = bev
    if problem == "stale": payload["image_id"] = "old"
    if problem == "nan": payload["points"][0]["x"] = float("nan")
    if problem == "duplicate": payload["points"][-1] = payload["points"][0].copy()
    if problem == "missing": payload["points"][0]["x"] = ""
    if problem == "outside": payload["points"][0]["u"] = 800
    if problem == "role": payload["points"][0]["role"] = "unknown"
    with pytest.raises(SessionError): service.save_points(payload)


def test_bev_requires_checks_and_noncollinear_points(bev):
    service, payload = bev
    payload["points"] = payload["points"][:8]
    with pytest.raises(SessionError, match="独立检查点"): service.calculate(payload)


def test_bev_rejects_collinear_fit_and_resolution_mismatch(bev):
    service, payload = bev
    for i, p in enumerate(payload["points"][:8]):
        p.update(u=100+i*20, v=100+i*20, x=1+i*.2, y=1+i*.2)
    with pytest.raises(SessionError, match="同一直线"): service.calculate(payload)
    cv2.imwrite(str(service.config.preview_path), np.zeros((300, 400, 3), dtype=np.uint8))
    with pytest.raises(SessionError, match="分辨率"):
        service.prepare(dict(reference_confirmed=True, intrinsic_confirmed=True, intrinsic_source="configured"))


def test_server_config_preserves_venv_python_symlink(tmp_path):
    from tools.field_assistant.server import build_parser, make_config
    target = tmp_path / "python-real"
    target.touch()
    link = tmp_path / "venv-python"
    link.symlink_to(target)
    config = make_config(build_parser().parse_args(["--calibration-python", str(link)]))
    assert config.calibration_python == link


def hardware_contract_fixture(service):
    """Synthetic hardware-labelled fixtures ONLY inside pytest's temporary directory."""
    from dataclasses import replace
    service.config = replace(service.config, mode="hardware")
    service.sessions.config = service.config
    for relative, key in (("session.json", "mode"), ("bev/snapshot.json", "mode"), ("bev/summary.json", "source_mode")):
        path = service.root() / relative
        data = json.loads(path.read_text())
        data[key] = "hardware"
        path.write_text(json.dumps(data))


def test_runtime_publication_retains_candidate_and_pins_versioned_paths(bev):
    service, payload = bev
    service.calculate(payload)
    with pytest.raises(SessionError, match="模拟结果"):
        service.publish(dict(image_id=payload["image_id"], review_confirmed=True))
    hardware_contract_fixture(service)
    with pytest.raises(SessionError, match="人工复核"):
        service.publish(dict(image_id=payload["image_id"]))
    original = (service.root() / "bev/candidate-bev.yaml").read_bytes()
    published = service.publish(dict(image_id=payload["image_id"], review_confirmed=True))
    current = service.config.effective_runtime_calibration_root / "current"
    first_version = current.resolve()
    binding = json.loads((current / "sdk-calibration.json").read_text())
    assert Path(binding["intrinsic_path"]).parent == first_version
    assert Path(binding["bev_path"]).parent == first_version
    for name in ("intrinsics.yaml", "bev.yaml"):
        fs = cv2.FileStorage(str(current / name), cv2.FILE_STORAGE_READ)
        assert fs.getNode("calibration_valid").real() == 1
        fs.release()
    assert (service.root() / "bev/candidate-bev.yaml").read_bytes() == original
    assert published["runtime_calibration"]["active"]["session_id"] == service.root().name
    assert str(first_version) in (current / "observer.calibration.yaml").read_text()
    service.publish(dict(image_id=payload["image_id"], review_confirmed=True))
    assert current.resolve() != first_version
    assert (first_version / "bev.yaml").is_file()


def test_runtime_publication_rejects_modified_files_and_failed_switch_keeps_old(bev, monkeypatch):
    service, payload = bev
    service.calculate(payload)
    hardware_contract_fixture(service)
    request = dict(image_id=payload["image_id"], review_confirmed=True)
    service.publish(request)
    current = service.config.effective_runtime_calibration_root / "current"
    original_version = current.resolve()
    path = service.root() / "bev/candidate-bev.yaml"
    original = path.read_bytes()
    path.write_bytes(original + b"\n# modified\n")
    with pytest.raises(SessionError, match="BEV 参数已改变"): service.publish(request)
    path.write_bytes(original)

    def fail_switch(source, target):
        raise OSError("test switch failure")
    monkeypatch.setattr("tools.field_assistant.bev.os.replace", fail_switch)
    with pytest.raises(SessionError, match="保存算法参数失败"): service.publish(request)
    assert current.resolve() == original_version
    assert not list(current.parent.glob(".current-*"))


def test_bev_prepare_requires_confirmation_and_resets_points(bev):
    service, payload = bev
    service.calculate(payload)
    with pytest.raises(SessionError): service.prepare({})
    result = service.prepare(dict(reference_confirmed=True, intrinsic_confirmed=True, intrinsic_source="configured"))
    assert result["bev"]["points"] is None
    assert result["bev"]["summary"] is None


if __name__ == "__main__":
    import sys
    points = fixture_files(Path(sys.argv[1]))
    (Path(sys.argv[1]) / "fixture-points.json").write_text(json.dumps(points))


@pytest.mark.parametrize("failure", ["timeout", "launch"])
def test_worker_failure_updates_log_and_allows_retry(bev, monkeypatch, failure):
    import subprocess
    service, payload = bev
    service.calculate(payload)
    hardware_contract_fixture(service)
    request = dict(image_id=payload["image_id"], review_confirmed=True)
    service.publish(request)
    current = service.config.effective_runtime_calibration_root / "current"
    previous = current.resolve()
    log = service.root() / "logs/bev.log"
    log.write_text("obsolete log")

    def fail(*args, **kwargs):
        if failure == "timeout":
            raise subprocess.TimeoutExpired("worker", 60, output=b"partial progress", stderr=b"timeout detail")
        raise OSError("launch unavailable")

    with monkeypatch.context() as patch:
        patch.setattr(subprocess, "run", fail)
        with pytest.raises(SessionError, match="BEV 程序无法完成"):
            service.calculate(payload)
    text = log.read_text()
    assert "obsolete log" not in text
    assert "operation=calculate" in text
    assert ("TimeoutExpired" if failure == "timeout" else "OSError") in text
    assert ("partial progress" if failure == "timeout" else "launch unavailable") in text
    assert service.sessions.describe()["bev"]["summary"] is None
    assert service.sessions.describe()["bev"]["points"]["points"] == payload["points"]
    with pytest.raises(SessionError):
        service.publish(request)
    assert current.resolve() == previous
    assert service.calculate(payload)["bev"]["summary"]["checks_passed"]
    assert current.resolve() == previous
