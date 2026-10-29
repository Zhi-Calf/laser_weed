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
