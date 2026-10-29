from __future__ import annotations

import json
import sys
import zipfile
from pathlib import Path

import pytest

from tools.field_assistant.config import AssistantConfig
from tools.field_assistant.sessions import SessionError, SessionStore
from tools.field_assistant.status import StatusService, read_calibration_summary, read_gnss_summary


def make_config(tmp_path: Path, **overrides: object) -> AssistantConfig:
    values: dict[str, object] = {
        "workspace": tmp_path,
        "session_root": tmp_path / "sessions",
        "mode": "mock",
        "preview_path": tmp_path / "preview.jpg",
        "calibration_path": tmp_path / "intrinsics.yaml",
        "import_root": tmp_path / "imports",
    }
    values.update(overrides)
    return AssistantConfig(**values)  # type: ignore[arg-type]


def write_calibration(path: Path, valid: int | None = None) -> None:
    suffix = "" if valid is None else f"calibration_valid: {valid}\n"
    path.write_text(
        "%YAML:1.0\n"
        "camera_name: \"CMD108\"\n"
        "image_width: 1920\n"
        "image_height: 1080\n"
        "reprojection_rms_px: 0.42\n"
        + suffix,
        encoding="utf-8",
    )


def valid_payload() -> dict[str, object]:
    return {
        "operator": "现场同事",
        "square_width_mm": 56.5,
        "square_height_mm": 56.5,
        "marker_mm": 42.2,
        "check_line_mm": 100.0,
        "board_flat": True,
    }


def test_calibration_summary_distinguishes_candidate_and_accepted(tmp_path: Path) -> None:
    candidate = tmp_path / "cmd108-candidate.yaml"
    write_calibration(candidate)
    assert read_calibration_summary(candidate)["state"] == "candidate"

    accepted = tmp_path / "cmd108.yaml"
    write_calibration(accepted, valid=1)
    summary = read_calibration_summary(accepted)
    assert summary["state"] == "ready"
    assert summary["camera_name"] == "CMD108"
    assert summary["reprojection_rms_px"] == pytest.approx(0.42)


def test_status_mock_mode_does_not_claim_hardware(tmp_path: Path) -> None:
    preview = tmp_path / "preview.jpg"
    preview.write_bytes(b"jpeg-placeholder")
    calibration = tmp_path / "intrinsics.yaml"
    write_calibration(calibration, valid=1)
    status = StatusService(make_config(tmp_path)).snapshot()

    checks = {item["id"]: item for item in status["checks"]}
    assert checks["camera"]["state"] == "ready"
    assert checks["gnss"]["state"] == "pending"
    assert checks["imu"]["state"] == "pending"
    assert status["ready_for_bev"] is False


def test_gnss_log_summary_keeps_zero_imu_evidence(tmp_path: Path) -> None:
    log = tmp_path / "gnss.log"
    log.write_text(
        "gps elapsed_ms=20 data=$GNGGA,,,,,,0,,,,,,,,*78\n"
        "summary polls=2457 gps_messages=960 gps_bytes=25440 "
        "imu_nonzero_samples=0 rtk_true_samples=0 last_imu_state=0 interrupted=0\n",
        encoding="utf-8",
    )
    summary = read_gnss_summary(log)
    assert summary is not None
    assert summary["fix_quality"] == 0
    assert summary["gps_messages"] == 960
    assert summary["imu_nonzero_samples"] == 0


def test_session_validation_rejects_bad_print_scale(tmp_path: Path) -> None:
    store = SessionStore(make_config(tmp_path))
    payload = valid_payload()
    payload["check_line_mm"] = 95
    with pytest.raises(SessionError, match="98–102"):
        store.start(payload, {"checks": []})


def test_session_export_is_traceable(tmp_path: Path) -> None:
    preview = tmp_path / "preview.jpg"
    preview.write_bytes(b"preview")
    calibration = tmp_path / "intrinsics.yaml"
    write_calibration(calibration, valid=1)
    store = SessionStore(make_config(tmp_path))

    session = store.start(valid_payload(), {"checks": [{"id": "camera"}]})
    result = store.export_current({"checks": [{"id": "camera", "state": "ready"}]})

    archive_path = Path(session["path"]) / "field-session-result.zip"
    assert result["download_ready"] is True
    assert archive_path.is_file()
    with zipfile.ZipFile(archive_path) as archive:
        names = set(archive.namelist())
        assert "session.json" in names
        assert "status-at-export.json" in names
        assert "checksums.sha256" in names
        metadata = json.loads(archive.read("session.json"))
        assert metadata["board"]["square_width_mm"] == pytest.approx(56.5)
        assert metadata["acceptance"]["calibration_valid"] is False


def test_intrinsic_import_cannot_escape_root(tmp_path: Path) -> None:
    config = make_config(tmp_path)
    store = SessionStore(config)
    store.start(valid_payload(), {"checks": []})
    with pytest.raises(SessionError, match="不能离开"):
        store.run_intrinsic({"train_dir": "../outside", "validation_dir": "validation"})


def test_intrinsic_runner_keeps_result_as_candidate(tmp_path: Path) -> None:
    imports = tmp_path / "imports"
    train = imports / "train"
    validation = imports / "validation"
    train.mkdir(parents=True)
    validation.mkdir(parents=True)
    for index in range(5):
        (train / f"{index}.jpg").write_bytes(b"synthetic-image")
    (validation / "0.jpg").write_bytes(b"synthetic-image")

    fake_script = tmp_path / "fake_calibrate.py"
    fake_script.write_text(
        "import argparse,json,pathlib\n"
        "p=argparse.ArgumentParser()\n"
        "p.add_argument('--output'); p.add_argument('--summary'); "
        "p.add_argument('--comparison-dir')\n"
        "p.add_argument('--train-glob'); p.add_argument('--validation-glob'); "
        "p.add_argument('--camera-name')\n"
        "p.add_argument('--squares-x'); p.add_argument('--squares-y'); "
        "p.add_argument('--square-m'); p.add_argument('--marker-m')\n"
        "a=p.parse_args()\n"
        "pathlib.Path(a.output).write_text('calibration_valid: 0\\n')\n"
        "pathlib.Path(a.summary).write_text(json.dumps({"
        "'training':{'matched':5,'usable':5,'calibration_rms_px':0.2},"
        "'validation':{'matched':1,'usable':1,'mean_reprojection_rms_px':0.3,"
        "'max_reprojection_rms_px':0.3},'human_review_required':True,"
        "'calibration_valid':False}))\n",
        encoding="utf-8",
    )
    config = make_config(
        tmp_path,
        calibration_python=Path(sys.executable),
        calibration_script=fake_script,
    )
    store = SessionStore(config)
    store.start(valid_payload(), {"checks": []})
    result = store.run_intrinsic(
        {"train_dir": "train", "validation_dir": "validation", "camera_name": "CMD108"}
    )

    assert result["intrinsic_summary"]["validation"]["usable"] == 1
    assert result["metadata"]["acceptance"]["intrinsic"] == "candidate"
    assert result["metadata"]["acceptance"]["calibration_valid"] is False
    assert (Path(result["path"]) / "raw" / "intrinsic" / "train-001.jpg").is_file()


def test_manual_capture_retake_and_group_isolation(tmp_path: Path) -> None:
    config = make_config(tmp_path)
    config.preview_path.write_bytes(b"first-frame")
    store = SessionStore(config)
    with pytest.raises(SessionError):
        store.capture({"group": "intrinsic", "slot": 0})
    store.start(valid_payload(), {"checks": []})
    first = store.capture({"group": "intrinsic", "slot": 0})
    assert first["simulated"] is True
    assert first["session"]["capture_counts"] == {"intrinsic": 1, "validation": 0}
    config.preview_path.write_bytes(b"retaken-frame")
    store.capture({"group": "intrinsic", "slot": 0})
    assert store.capture_image(first["session"]["id"], "intrinsic", 0).read_bytes() == b"retaken-frame"
    result = store.capture({"group": "validation", "slot": 0})
    assert result["session"]["capture_counts"] == {"intrinsic": 1, "validation": 1}
    with pytest.raises(SessionError):
        store.capture({"group": "validation", "slot": 9})
    with pytest.raises(SessionError, match="至少需要"):
        store.run_intrinsic({"source": "captured"})


def test_hardware_capture_never_falls_back_to_mock(tmp_path: Path) -> None:
    store = SessionStore(make_config(tmp_path, mode="hardware"))
    store.start(valid_payload(), {"checks": []})
    with pytest.raises(SessionError, match="尚未配置拍照程序"):
        store.capture({"group": "intrinsic", "slot": 0})


def test_hardware_capture_requests_exactly_one_frame(tmp_path: Path, monkeypatch) -> None:
    import subprocess
    binary = tmp_path / "charuco_capture"
    binary.touch()
    store = SessionStore(make_config(tmp_path, mode="hardware", capture_binary=binary))
    store.start(valid_payload(), {"checks": []})

    def fake_capture(command, **kwargs):
        assert command[2:6] == ["1", "5000", "0", "/dev/video12"]
        assert kwargs["timeout"] == 30
        (Path(command[1]) / "actual-frame.jpg").write_bytes(b"hardware-frame")
        return subprocess.CompletedProcess(command, 0, "saved=1/1", "")

    monkeypatch.setattr(subprocess, "run", fake_capture)
    result = store.capture({"group": "intrinsic", "slot": 26})
    assert result["simulated"] is False
    assert store.capture_image(result["session"]["id"], "intrinsic", 26).read_bytes() == b"hardware-frame"


@pytest.mark.parametrize("failure", ["exit", "timeout", "launch", "missing", "summary"])
def test_failed_intrinsics_can_retake_and_retry_without_stale_results(tmp_path, monkeypatch, failure):
    import subprocess

    script = tmp_path / "calibrate.py"
    script.touch()
    preview = tmp_path / "preview.jpg"
    preview.write_bytes(b"original")
    store = SessionStore(make_config(tmp_path, calibration_python=Path(sys.executable),
                                     calibration_script=script))
    session = store.start(valid_payload(), {"checks": []})
    root = Path(session["path"])
    for slot in range(5):
        store.capture({"group": "intrinsic", "slot": slot})
    store.capture({"group": "validation", "slot": 0})
    (root / "field-session-result.zip").write_bytes(b"old export")
    published = tmp_path / "configs/calibration/current/intrinsics.yaml"
    published.parent.mkdir(parents=True)
    published.write_bytes(b"previously accepted")

    def fail(command, **kwargs):
        output = Path(command[command.index("--output") + 1])
        summary = Path(command[command.index("--summary") + 1])
        output.write_text("partial")
        summary.write_text("invalid json" if failure == "summary" else "{}")
        if failure == "timeout":
            raise subprocess.TimeoutExpired(command, 300)
        if failure == "launch":
            raise OSError("cannot launch")
        if failure == "missing":
            output.unlink()
        return subprocess.CompletedProcess(command, 1 if failure == "exit" else 0, "", "failed")

    monkeypatch.setattr(subprocess, "run", fail)
    with pytest.raises(SessionError, match="本会话补拍或重拍"):
        store.run_intrinsic({"source": "captured"})
    assert store.describe()["intrinsic_summary"] is None
    assert not (root / "cmd108-intrinsics.yaml").exists()
    assert not (root / "field-session-result.zip").exists()
    assert len(list((root / "intrinsic-attempts").iterdir())) == 1
    assert published.read_bytes() == b"previously accepted"
    assert store.describe()["capture_counts"] == {"intrinsic": 5, "validation": 1}

    preview.write_bytes(b"retaken")
    store.capture({"group": "intrinsic", "slot": 0})
    store.capture({"group": "intrinsic", "slot": 5})

    def succeed(command, **kwargs):
        train_glob = Path(command[command.index("--train-glob") + 1])
        images = sorted(train_glob.parent.glob("*.jpg"))
        assert len(images) == 6
        assert images[0].read_bytes() == b"retaken"
        Path(command[command.index("--output") + 1]).write_text("calibration_valid: 0\n")
        Path(command[command.index("--summary") + 1]).write_text('{"training":{"usable":6}}')
        return subprocess.CompletedProcess(command, 0, "ok", "")

    monkeypatch.setattr(subprocess, "run", succeed)
    result = store.run_intrinsic({"source": "captured"})
    assert result["id"] == session["id"]
    assert result["intrinsic_summary"]["training"]["usable"] == 6
    assert result["metadata"]["acceptance"]["intrinsic"] == "candidate"
    with pytest.raises(SessionError, match="已完成"):
        store.run_intrinsic({"source": "captured"})
    with pytest.raises(SessionError, match="已运行"):
        store.capture({"group": "intrinsic", "slot": 0})
    assert published.read_bytes() == b"previously accepted"


def test_legacy_failed_import_does_not_lock_capture(tmp_path):
    preview = tmp_path / "preview.jpg"
    preview.write_bytes(b"new photo")
    store = SessionStore(make_config(tmp_path))
    session = store.start(valid_payload(), {"checks": []})
    root = Path(session["path"])
    (root / "raw/intrinsic/train-001.jpg").write_bytes(b"old import")
    (root / "cmd108-intrinsics.yaml").write_text("partial output")
    (root / "validation/intrinsic-summary.json").write_text("{}")
    result = store.capture({"group": "intrinsic", "slot": 0})
    assert result["session"]["intrinsic_summary"] is None
    assert not (root / "cmd108-intrinsics.yaml").exists()
    assert list((root / "intrinsic-attempts").rglob("train-001.jpg"))[0].read_bytes() == b"old import"
