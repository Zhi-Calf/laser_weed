import importlib.util
from pathlib import Path
import sys

import pytest


SCRIPT = Path(__file__).parents[2] / "scripts" / "calibration" / "calibrate_charuco.py"
SPEC = importlib.util.spec_from_file_location("calibrate_charuco", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("4.8.1", (4, 8, 1)),
        ("4.10.0-dev", (4, 10, 0)),
        ("5", (5, 0, 0)),
    ],
)
def test_opencv_version_tuple(text, expected):
    assert MODULE.opencv_version_tuple(text) == expected


def test_parser_requires_physical_board_dimensions():
    parser = MODULE.build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(
            [
                "--train-glob",
                "train/*.jpg",
                "--validation-glob",
                "validation/*.jpg",
                "--output",
                "camera.yaml",
                "--summary",
                "summary.json",
            ]
        )


def test_expand_images_is_sorted(tmp_path):
    (tmp_path / "b.jpg").touch()
    (tmp_path / "a.jpg").touch()
    assert MODULE.expand_images(str(tmp_path / "*.jpg")) == [
        tmp_path / "a.jpg",
        tmp_path / "b.jpg",
    ]


def test_failed_detection_writes_actionable_diagnostics(tmp_path, monkeypatch):
    import json
    import cv2
    import numpy as np
    train = tmp_path / "train"
    validation = tmp_path / "validation"
    train.mkdir()
    validation.mkdir()
    for i in range(5):
        cv2.imwrite(str(train / f"train-{i}.jpg"), np.full((300, 400, 3), 20+i, np.uint8))
    # Unreadable validation image must still yield a diagnostic, not lose all reasons.
    (validation / "bad.jpg").write_bytes(b"not an image")
    monkeypatch.setattr(sys, "argv", [str(SCRIPT), "--train-glob", str(train / "*.jpg"),
        "--validation-glob", str(validation / "*.jpg"), "--output", str(tmp_path / "camera.yaml"),
        "--summary", str(tmp_path / "summary.json"), "--square-m", "0.0565", "--marker-m", "0.0422"])
    with pytest.raises(RuntimeError, match="usable training"):
        MODULE.main()
    diagnostics = json.loads((tmp_path / "intrinsic-diagnostics.json").read_text())
    assert len(diagnostics["training_rejected"]) == 5
    assert "unreadable" in diagnostics["validation_rejected"][0]
    assert not (tmp_path / "camera.yaml").exists()
