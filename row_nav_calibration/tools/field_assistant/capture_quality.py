"""Best-effort per-photo guidance. Run with the calibration Python, not HTTP Python.

Metrics and thresholds are advisory; neither this report nor corner detection accepts
an intrinsic calibration. Original capture bytes and timestamp sidecars stay unchanged.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def assess(image_path: Path, overlay_path: Path, square_m: float, marker_m: float, thumbnail_path: Path | None = None) -> dict:
    raw_hash = hashlib.sha256(image_path.read_bytes()).hexdigest()
    image = cv2.imread(str(image_path))
    if image is None:
        return {"state": "review", "raw_sha256": raw_hash, "corners": 0,
                "warnings": ["照片无法解码，请重新拍摄"], "overlay_available": False}
    height, width = image.shape[:2]
    if thumbnail_path is not None:
        scale = min(1., 192/max(width, height))
        small = cv2.resize(image, (max(1, round(width*scale)), max(1, round(height*scale))), interpolation=cv2.INTER_AREA)
        if not cv2.imwrite(str(thumbnail_path), small, [cv2.IMWRITE_JPEG_QUALITY, 75]):
            raise OSError("cannot write capture thumbnail")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    board = cv2.aruco.CharucoBoard((7, 10), square_m, marker_m,
                                 cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000))
    corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(image)
    count = 0 if ids is None else len(ids)
    coverage = 0.
    focus_image = gray
    if count >= 3:
        xy = corners.reshape(-1, 2)
        coverage = float(cv2.contourArea(cv2.convexHull(xy))) / (width * height)
        x, y, w, h = cv2.boundingRect(xy)
        focus_image = gray[max(0, y):min(height, y+h), max(0, x):min(width, x+w)]
    sharpness = float(cv2.Laplacian(focus_image, cv2.CV_64F).var())
    bright_fraction = float(np.mean(gray >= 250))
    dark_fraction = float(np.mean(gray <= 8))
    warnings = []
    if count < 12:
        warnings.append(f"只检测到 {count}/54 个角点；请靠近、避免遮挡和反光，确保标定板清晰完整入镜")
    if sharpness < 40:
        warnings.append("清晰度指标偏低；请保持不动并检查对焦")
    if bright_fraction > .65:
        warnings.append("大面积接近纯白；请检查过曝或反光")
    if dark_fraction > .65:
        warnings.append("大面积接近纯黑；请检查光照和曝光")
    if count >= 12 and coverage < .025:
        warnings.append("检测区域偏小；可适当靠近，远距离轮次仍需确保角点清晰")
    annotated = image.copy()
    if count:
        for point, corner_id in zip(corners.reshape(-1, 2), ids.reshape(-1)):
            center = tuple(int(round(float(value))) for value in point)
            cv2.circle(annotated, center, 4, (20, 190, 60), -1)
            cv2.putText(annotated, str(int(corner_id)), (center[0]+5, center[1]-5),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, (20, 130, 40), 1, cv2.LINE_AA)
    scale = min(1., 1280 / width)
    if scale < 1:
        annotated = cv2.resize(annotated, (round(width*scale), round(height*scale)))
    if not cv2.imwrite(str(overlay_path), annotated):
        raise OSError("cannot write corner overlay")
    return {"schema_version": 1, "state": "review" if warnings else "detected",
            "raw_sha256": raw_hash, "width": width, "height": height,
            "corners": count, "expected_corners": 54, "corner_coverage": coverage,
            "sharpness": sharpness, "bright_fraction": bright_fraction, "dark_fraction": dark_fraction,
            "warnings": warnings, "overlay_available": True, "advisory_only": True,
            "thresholds": {"corner_warning": 12, "sharpness_warning": 40,
                           "extreme_exposure_fraction": .65, "small_coverage": .025}}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("report", type=Path)
    parser.add_argument("overlay", type=Path)
    parser.add_argument("square_m", type=float)
    parser.add_argument("marker_m", type=float)
    parser.add_argument("--thumbnail", type=Path)
    args = parser.parse_args()
    report = assess(args.image, args.overlay, args.square_m, args.marker_m, args.thumbnail)
    args.report.write_text(json.dumps(report, ensure_ascii=False, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    main()
