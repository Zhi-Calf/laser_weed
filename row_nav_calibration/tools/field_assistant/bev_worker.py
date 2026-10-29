"""OpenCV subprocess for the web assistant; runtime SDK remains independent."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import re

import cv2
import numpy as np


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def prepare(root: Path, source: Path, intrinsic: Path, mode: str):
    image = cv2.imread(str(source))
    if image is None:
        raise ValueError("无法读取地面照片")
    fs = cv2.FileStorage(str(intrinsic), cv2.FILE_STORAGE_READ)
    k = fs.getNode("camera_matrix").mat()
    d = fs.getNode("distortion_coefficients").mat()
    width, height = (int(fs.getNode(key).real()) for key in ("image_width", "image_height"))
    fs.release()
    if k is None or d is None or k.shape != (3, 3) or not np.isfinite(k).all() or not np.isfinite(d).all():
        raise ValueError("内参矩阵缺失或无效")
    if image.shape[:2] != (height, width):
        raise ValueError("照片分辨率与内参不一致，请保持原始分辨率和相机设置")
    # Same convention as runtime: preserve K and resolution, no crop/new K.
    undistorted = cv2.undistort(image, k, d, None, k)
    if not cv2.imwrite(str(root / "snapshot.jpg"), undistorted):
        raise ValueError("无法保存去畸变照片")
    identity = hashlib.sha256((root / "snapshot.jpg").read_bytes() + intrinsic.read_bytes()).hexdigest()
    save_json(root / "snapshot.json", {"image_id": identity, "width": width, "height": height,
        "mode": mode, "intrinsic_sha256": hashlib.sha256(intrinsic.read_bytes()).hexdigest(),
        "reference": "rear_axle_ground", "axes": "x_forward_y_left", "undistorted": True})


def calculate(root: Path):
    points = json.loads((root / "points.json").read_text(encoding="utf-8"))["points"]
    uv = np.array([[p["u"], p["v"]] for p in points], dtype=np.float64)
    xy = np.array([[p["x"], p["y"]] for p in points], dtype=np.float64)
    fit = np.array([p["role"] == "fit" for p in points])
    if np.linalg.matrix_rank(np.c_[uv[fit], np.ones(fit.sum())]) < 3 or np.linalg.matrix_rank(np.c_[xy[fit], np.ones(fit.sum())]) < 3:
        raise ValueError("标定点不能排在同一直线上，请覆盖近、远、左、右")
    target = np.c_[320 - xy[:, 1] / .01, 799 - xy[:, 0] / .01]
    h, mask = cv2.findHomography(uv[fit], target[fit], cv2.RANSAC, 3.0)
    if h is None or not np.isfinite(h).all() or abs(np.linalg.det(h)) < 1e-12:
        raise ValueError("无法计算有效 BEV，请检查点的对应关系")
    homogeneous = np.c_[uv, np.ones(len(uv))] @ h.T
    if np.any(abs(homogeneous[:, 2]) < 1e-9):
        raise ValueError("有点位于投影奇异位置，请检查地面点")
    mapped = homogeneous[:, :2] / homogeneous[:, 2:]
    recovered = np.c_[(799 - mapped[:, 1]) * .01, (320 - mapped[:, 0]) * .01]
    errors = np.linalg.norm(recovered - xy, axis=1)
    metrics = lambda e: {"count": int(len(e)), "rmse_m": float(np.sqrt(np.mean(e ** 2))), "max_m": float(e.max())}
    inliers = int(mask.sum())
    checks = metrics(errors[~fit])
    passed = inliers / int(fit.sum()) >= .75 and checks["rmse_m"] <= .05 and checks["max_m"] <= .10
    snapshot = json.loads((root / "snapshot.json").read_text())
    summary = {"fit": metrics(errors[fit]), "check": checks, "fit_inliers": inliers,
        "checks_passed": bool(passed), "calibration_valid": False, "human_review_required": True,
        "reference": "rear_axle_ground", "axes": "x_forward_y_left",
        "image_id": snapshot["image_id"], "source_mode": snapshot["mode"],
        "point_errors_m": errors.tolist(), "output_width": 640, "output_height": 800,
        "meters_per_pixel": .01, "vehicle_origin_u": 320, "vehicle_origin_v": 799}
    fs = cv2.FileStorage(str(root / "candidate-bev.yaml"), cv2.FILE_STORAGE_WRITE)
    for key, value in {"schema_version": 1, "calibration_valid": 0, "human_review_required": 1,
        "coordinate_reference": "rear_axle_ground", "axis_convention": "x_forward_y_left",
        "source_mode": snapshot["mode"], "intrinsic_sha256": snapshot["intrinsic_sha256"],
        "output_width": 640, "output_height": 800, "meters_per_pixel": .01,
        "vehicle_origin_u": 320., "vehicle_origin_v": 799., "max_check_rmse_m": .05,
        "fit_point_count": int(fit.sum()), "fit_inlier_count": inliers, "check_point_count": int((~fit).sum()),
        "fit_rmse_m": summary["fit"]["rmse_m"], "check_rmse_m": checks["rmse_m"],
        "check_max_error_m": checks["max_m"], "image_to_bev": h}.items():
        fs.write(key, value)
    fs.release()
    image = cv2.imread(str(root / "snapshot.jpg"))
    bev = cv2.warpPerspective(image, h, (640, 800))
    overlay = image.copy()
    for i, (pixel, ground, p) in enumerate(zip(uv, target, points)):
        color = (50, 160, 30) if p["role"] == "fit" else (0, 140, 245)
        for img, pt in ((overlay, pixel), (bev, mapped[i])):
            q = tuple(np.rint(pt).astype(int))
            cv2.circle(img, q, 6, color, 2)
            cv2.putText(img, str(i + 1), (q[0] + 7, q[1] - 7), cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
        expected = tuple(np.rint(ground).astype(int))
        projected = tuple(np.rint(mapped[i]).astype(int))
        cv2.line(bev, projected, expected, color, 1)
        cv2.drawMarker(bev, expected, (255, 255, 255), cv2.MARKER_CROSS, 8, 1)
    cv2.arrowedLine(bev, (320, 790), (320, 700), (255, 230, 60), 2)
    cv2.arrowedLine(bev, (320, 790), (230, 790), (255, 230, 60), 2)
    cv2.putText(bev, "rear axle O / x forward / y left", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    for name, img in (("bev.jpg", bev), ("overlay.jpg", overlay)):
        if not cv2.imwrite(str(root / name), img):
            raise ValueError("无法保存 BEV 检查图")
    summary["bev_sha256"] = hashlib.sha256((root / "candidate-bev.yaml").read_bytes()).hexdigest()
    save_json(root / "summary.json", summary)


def accept_files(root: Path):
    """Validate the staged pair and mark copies accepted; never edit evidence."""
    for name, matrix_key in (("intrinsics.yaml", "camera_matrix"), ("bev.yaml", "image_to_bev")):
        path = root / name
        fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        matrix = fs.getNode(matrix_key).mat()
        fs.release()
        if matrix is None or matrix.shape != (3, 3) or not np.isfinite(matrix).all() or abs(np.linalg.det(matrix)) < 1e-12:
            raise ValueError("待发布的参数矩阵无效")
        text = path.read_text(encoding="utf-8")
        for key, value in (("calibration_valid", "1"), ("human_review_required", "0")):
            pattern = rf"(?m)^{key}:.*$"
            matches = len(re.findall(pattern, text))
            if matches > 1:
                raise ValueError("参数文件包含重复的验收标记")
            text = re.sub(pattern, f"{key}: {value}", text) if matches else text.rstrip() + f"\n{key}: {value}\n"
        path.write_text(text, encoding="utf-8")
        fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        if fs.getNode("calibration_valid").real() != 1:
            raise ValueError("无法写入已复核标记")
        fs.release()


if __name__ == "__main__":
    try:
        if sys.argv[1] == "prepare":
            prepare(Path(sys.argv[2]), Path(sys.argv[3]), Path(sys.argv[4]), sys.argv[5])
        elif sys.argv[1] == "accept":
            accept_files(Path(sys.argv[2]))
        else:
            calculate(Path(sys.argv[2]))
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
