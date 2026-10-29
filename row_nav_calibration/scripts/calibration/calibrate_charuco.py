#!/usr/bin/env python3
"""Calibrate a camera from ChArUco images and verify it on held-out views."""

from __future__ import annotations

import argparse
import glob
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


MIN_CHARUCO_CORNERS = 6


@dataclass
class DetectedView:
    path: Path
    object_points: np.ndarray
    image_points: np.ndarray


def opencv_version_tuple(version: str) -> tuple[int, int, int]:
    parts = version.split(".")
    values: list[int] = []
    for part in parts[:3]:
        digits = "".join(character for character in part if character.isdigit())
        values.append(int(digits) if digits else 0)
    return tuple((values + [0, 0, 0])[:3])  # type: ignore[return-value]


def require_supported_opencv(legacy_pattern: bool) -> None:
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "CharucoDetector"):
        raise RuntimeError(
            "OpenCV with the modern aruco/CharucoDetector API is required; "
            "use OpenCV >= 4.7 (the validated environment uses 4.8.1)"
        )
    if opencv_version_tuple(cv2.__version__) < (4, 7, 0):
        raise RuntimeError(
            f"OpenCV {cv2.__version__} is too old for layout-safe ChArUco calibration; "
            "use OpenCV >= 4.7"
        )
    if legacy_pattern and not hasattr(cv2.aruco.CharucoBoard, "setLegacyPattern"):
        raise RuntimeError("this OpenCV build cannot select the legacy ChArUco pattern")


def expand_images(pattern: str) -> list[Path]:
    return [Path(path) for path in sorted(glob.glob(pattern))]


def create_board(
    squares_x: int,
    squares_y: int,
    square_m: float,
    marker_m: float,
    legacy_pattern: bool,
):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y), square_m, marker_m, dictionary
    )
    board.setLegacyPattern(legacy_pattern)
    return board, dictionary


def detect_views(
    paths: Iterable[Path], board, dictionary
) -> tuple[list[DetectedView], list[str], tuple[int, int]]:
    detector = cv2.aruco.CharucoDetector(board)
    accepted: list[DetectedView] = []
    rejected: list[str] = []
    image_size: tuple[int, int] | None = None

    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            rejected.append(f"{path}: unreadable")
            continue
        current_size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = current_size
        if current_size != image_size:
            rejected.append(f"{path}: resolution {current_size} != {image_size}")
            continue

        charuco_corners, charuco_ids, _, _ = detector.detectBoard(image)
        count = 0 if charuco_ids is None else len(charuco_ids)
        if count < MIN_CHARUCO_CORNERS:
            rejected.append(f"{path}: only {count} ChArUco corners")
            continue
        object_points, image_points = board.matchImagePoints(charuco_corners, charuco_ids)
        accepted.append(
            DetectedView(
                path=path,
                object_points=np.asarray(object_points, dtype=np.float32),
                image_points=np.asarray(image_points, dtype=np.float32),
            )
        )

    if image_size is None:
        raise RuntimeError("no readable images matched the input pattern")
    return accepted, rejected, image_size


def view_reprojection_rms(
    view: DetectedView,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    rvec: np.ndarray | None = None,
    tvec: np.ndarray | None = None,
) -> tuple[float, np.ndarray, np.ndarray]:
    if rvec is None or tvec is None:
        ok, rvec, tvec = cv2.solvePnP(
            view.object_points, view.image_points, camera_matrix, distortion
        )
        if not ok:
            raise RuntimeError(f"solvePnP failed for {view.path}")
    projected, _ = cv2.projectPoints(
        view.object_points, rvec, tvec, camera_matrix, distortion
    )
    residual = projected.reshape(-1, 2) - view.image_points.reshape(-1, 2)
    rms = math.sqrt(float(np.mean(np.sum(residual * residual, axis=1))))
    return rms, rvec, tvec


def write_calibration(
    path: Path,
    camera_name: str,
    image_size: tuple[int, int],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    rms: float,
    args: argparse.Namespace,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise RuntimeError(f"cannot write calibration file: {path}")
    storage.write("schema_version", 1)
    storage.write("camera_name", camera_name)
    storage.write("image_width", image_size[0])
    storage.write("image_height", image_size[1])
    storage.write("camera_matrix", camera_matrix.astype(np.float64))
    storage.write("distortion_coefficients", distortion.reshape(1, -1).astype(np.float64))
    storage.write("reprojection_rms_px", float(rms))
    storage.write("charuco_squares_x", args.squares_x)
    storage.write("charuco_squares_y", args.squares_y)
    storage.write("charuco_square_m", args.square_m)
    storage.write("charuco_marker_m", args.marker_m)
    storage.write("charuco_legacy_pattern", int(args.legacy_pattern))
    storage.write("opencv_version", cv2.__version__)
    storage.write("human_review_required", 1)
    storage.release()


def write_comparisons(
    views: Iterable[DetectedView],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    output_dir: Path,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for index, view in enumerate(views, start=1):
        image = cv2.imread(str(view.path), cv2.IMREAD_COLOR)
        undistorted = cv2.undistort(image, camera_matrix, distortion)
        comparison = np.hstack((image, undistorted))
        cv2.putText(
            comparison,
            "original",
            (24, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            comparison,
            "undistorted",
            (image.shape[1] + 24, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imwrite(str(output_dir / f"validation-{index:03d}-compare.jpg"), comparison)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Fit and independently validate a ChArUco camera calibration"
    )
    parser.add_argument("--train-glob", required=True)
    parser.add_argument("--validation-glob", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--summary", required=True, type=Path)
    parser.add_argument("--comparison-dir", type=Path)
    parser.add_argument("--camera-name", default="camera")
    parser.add_argument("--squares-x", type=int, default=7)
    parser.add_argument("--squares-y", type=int, default=10)
    parser.add_argument("--square-m", type=float, required=True)
    parser.add_argument("--marker-m", type=float, required=True)
    parser.add_argument(
        "--legacy-pattern",
        action="store_true",
        help="use only for a board confirmed to use the pre-OpenCV-4.6 layout",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.squares_x < 3 or args.squares_y < 3:
        raise ValueError("ChArUco board must contain at least 3x3 squares")
    if not 0 < args.marker_m < args.square_m:
        raise ValueError("marker length must be positive and smaller than square length")
    require_supported_opencv(args.legacy_pattern)

    board, dictionary = create_board(
        args.squares_x,
        args.squares_y,
        args.square_m,
        args.marker_m,
        args.legacy_pattern,
    )
    train_paths = expand_images(args.train_glob)
    validation_paths = expand_images(args.validation_glob)
    if len(train_paths) < 5:
        raise RuntimeError("at least five training images are required")
    if not validation_paths:
        raise RuntimeError("an independent validation set is required")

    train_views, train_rejected, image_size = detect_views(train_paths, board, dictionary)
    validation_views, validation_rejected, validation_size = detect_views(
        validation_paths, board, dictionary
    )
    if len(train_views) < 5:
        raise RuntimeError("fewer than five usable training views")
    if not validation_views:
        raise RuntimeError("no usable independent validation views")
    if validation_size != image_size:
        raise RuntimeError(
            f"validation resolution {validation_size} does not match training {image_size}"
        )

    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        [view.object_points for view in train_views],
        [view.image_points for view in train_views],
        image_size,
        None,
        None,
        flags=cv2.CALIB_RATIONAL_MODEL,
    )
    train_errors = [
        view_reprojection_rms(view, camera_matrix, distortion, rvec, tvec)[0]
        for view, rvec, tvec in zip(train_views, rvecs, tvecs)
    ]
    validation_errors = [
        view_reprojection_rms(view, camera_matrix, distortion)[0]
        for view in validation_views
    ]

    write_calibration(
        args.output,
        args.camera_name,
        image_size,
        camera_matrix,
        distortion,
        rms,
        args,
    )
    if args.comparison_dir is not None:
        write_comparisons(validation_views, camera_matrix, distortion, args.comparison_dir)

    summary = {
        "schema_version": 1,
        "camera_name": args.camera_name,
        "opencv_version": cv2.__version__,
        "image_size": list(image_size),
        "board": {
            "dictionary": "DICT_5X5_1000",
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "square_m": args.square_m,
            "marker_m": args.marker_m,
            "legacy_pattern": args.legacy_pattern,
        },
        "training": {
            "matched": len(train_paths),
            "usable": len(train_views),
            "rejected": train_rejected,
            "calibration_rms_px": float(rms),
            "per_view_rms_px": train_errors,
        },
        "validation": {
            "matched": len(validation_paths),
            "usable": len(validation_views),
            "rejected": validation_rejected,
            "mean_reprojection_rms_px": float(np.mean(validation_errors)),
            "max_reprojection_rms_px": float(np.max(validation_errors)),
            "per_view": [
                {"image": str(view.path), "reprojection_rms_px": error}
                for view, error in zip(validation_views, validation_errors)
            ],
        },
        "human_review_required": True,
        "calibration_valid": False,
        "validation_note": (
            "Set calibration_valid only after reviewing edge straightness, black borders, "
            "and retained field of view in the comparison images."
        ),
    }
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")

    print(
        f"saved={args.output} train={len(train_views)}/{len(train_paths)} "
        f"train_rms_px={rms:.6f} validation={len(validation_views)}/{len(validation_paths)} "
        f"validation_mean_rms_px={np.mean(validation_errors):.6f} "
        f"validation_max_rms_px={np.max(validation_errors):.6f} "
        "human_review_required=1"
    )


if __name__ == "__main__":
    main()
