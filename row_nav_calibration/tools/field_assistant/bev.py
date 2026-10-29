from __future__ import annotations

import csv
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import hashlib
import os
import uuid
from datetime import datetime, timezone

from .sessions import SessionError, SessionStore
from .bev_worker_io import read_json, write_json


class BevService:
    def __init__(self, sessions: SessionStore):
        self.sessions = sessions
        self.config = sessions.config

    def root(self):
        sid = self.sessions.current_id()
        if not sid:
            raise SessionError("请先建立现场会话")
        return self.sessions._session_dir(sid)

    def worker(self, *args):
        script = Path(__file__).with_name("bev_worker.py")
        log_path = self.root() / "logs" / "bev.log"
        header = f"operation={args[0]} time={datetime.now(timezone.utc).isoformat()}\n"
        try:
            result = subprocess.run([str(self.config.calibration_python), str(script), *map(str, args)],
                capture_output=True, text=True, timeout=60, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            def text(value):
                return value.decode("utf-8", errors="replace") if isinstance(value, bytes) else (value or "")
            log_path.write_text(
                header + f"{type(exc).__name__}: {exc}\n"
                + "stdout:\n" + text(getattr(exc, "stdout", None))
                + "\nstderr:\n" + text(getattr(exc, "stderr", None)), encoding="utf-8")
            raise SessionError("BEV 程序无法完成，请检查部署环境") from exc
        log_path.write_text(header + f"exit_code={result.returncode}\nstdout:\n"
                            + result.stdout + "\nstderr:\n" + result.stderr, encoding="utf-8")
        if result.returncode:
            message = result.stderr.strip().splitlines()[-1] if result.stderr.strip() else "请查看 BEV 日志"
            raise SessionError(f"BEV 处理失败：{message}")

    def prepare(self, payload):
        root = self.root()
        if payload.get("reference_confirmed") is not True or payload.get("intrinsic_confirmed") is not True:
            raise SessionError("请确认后轴原点、地面平整、相机固定以及内参已经人工复核")
        choice = payload.get("intrinsic_source")
        intrinsic = root / ("cmd108-intrinsics.yaml" if choice == "session" else "candidate-intrinsics.yaml")
        if choice not in {"session", "configured"} or not intrinsic.is_file():
            raise SessionError("所选内参不存在，请先完成内参标定或联系部署人员配置")
        # Capture into the separate BEV slot; it never touches intrinsic photos.
        self.sessions.capture({"group": "bev", "slot": 0})
        raw = root / "captures" / "bev" / "position-00.jpg"
        with tempfile.TemporaryDirectory(dir=root) as tmp:
            staging = Path(tmp) / "bev"
            staging.mkdir()
            shutil.copy2(raw, staging / "raw.jpg")
            shutil.copy2(intrinsic, staging / "intrinsics.yaml")
            self.worker("prepare", staging, staging / "raw.jpg", staging / "intrinsics.yaml", self.config.mode)
            write_json(staging / "reference.json", {"reference": "rear_axle_ground", "axes": "x_forward_y_left",
                "reference_confirmed": True, "intrinsic_confirmed": True, "intrinsic_source": choice})
            destination = root / "bev"
            if destination.exists():
                shutil.rmtree(destination)
            shutil.move(str(staging), destination)
        self.invalidate_export()
        self.mark_candidate("not_started")
        return self.sessions.describe()

    def save_points(self, payload):
        root = self.root() / "bev"
        snapshot = read_json(root / "snapshot.json")
        if not snapshot or snapshot["image_id"] != payload.get("image_id"):
            raise SessionError("照片已变化，请刷新页面并重新点选")
        values = payload.get("points")
        if not isinstance(values, list) or not 1 <= len(values) <= 64:
            raise SessionError("请填写 1–64 个地面点")
        points = []
        for i, value in enumerate(values):
            if not isinstance(value, dict) or value.get("role") not in ("fit", "check"):
                raise SessionError(f"第 {i+1} 点用途无效")
            p = {"role": value["role"]}
            for key in ("u", "v", "x", "y"):
                try:
                    if isinstance(value.get(key), bool) or value.get(key) in (None, ""):
                        raise ValueError()
                    p[key] = float(value[key])
                except (ValueError, TypeError):
                    raise SessionError(f"第 {i+1} 点未完成点选或距离填写")
                if not math.isfinite(p[key]):
                    raise SessionError("坐标必须是有限数字")
            if not (0 <= p["u"] < snapshot["width"] and 0 <= p["v"] < snapshot["height"]):
                raise SessionError("点选位置超出照片")
            if not (0 <= p["x"] <= 30 and abs(p["y"]) <= 15):
                raise SessionError("前向距离须为 0–30 m，横向距离须在左右 15 m 内")
            for previous in points:
                if math.hypot(p["u"]-previous["u"], p["v"]-previous["v"]) < 2 or math.hypot(p["x"]-previous["x"], p["y"]-previous["y"]) < .001:
                    raise SessionError("点不能重复，独立检查点也不能复用标定点")
            points.append(p)
        write_json(root / "points.json", {"image_id": snapshot["image_id"], "points": points})
        with (root / "points.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["image_u", "image_v", "forward_m", "left_m", "role"])
            writer.writerows([p["u"], p["v"], p["x"], p["y"], p["role"]] for p in points)
        for name in ("summary.json", "candidate-bev.yaml", "bev.jpg", "overlay.jpg"):
            (root / name).unlink(missing_ok=True)
        self.invalidate_export()
        self.mark_candidate("points_saved")
        return self.sessions.describe()

    def calculate(self, payload):
        self.save_points(payload)
        points = payload["points"]
        if sum(p["role"] == "fit" for p in points) < 6 or sum(p["role"] == "check" for p in points) < 4:
            raise SessionError("至少需要 6 个标定点和 4 个独立检查点；建议 8–12 个标定点")
        self.worker("calculate", self.root() / "bev")
        self.mark_candidate("candidate")
        return self.sessions.describe()

    def mark_candidate(self, status):
        path = self.root() / "session.json"
        metadata = read_json(path)
        metadata["acceptance"]["bev"] = status
        metadata["acceptance"]["calibration_valid"] = False
        write_json(path, metadata)

    def publish(self, payload):
        root = self.root()
        snapshot = read_json(root / "bev/snapshot.json") or {}
        summary = read_json(root / "bev/summary.json") or {}
        metadata = read_json(root / "session.json") or {}
        if self.config.mode != "hardware" or snapshot.get("mode") != "hardware" or metadata.get("mode") != "hardware" or summary.get("source_mode") != "hardware":
            raise SessionError("模拟结果不能设为正式算法参数，请在真机完成标定")
        if payload.get("review_confirmed") is not True:
            raise SessionError("请确认内参、BEV 检查图和车体参考点已经人工复核")
        if not summary.get("checks_passed") or payload.get("image_id") != snapshot.get("image_id") or summary.get("image_id") != snapshot.get("image_id"):
            raise SessionError("请先完成并通过本次照片的 BEV 独立检查")
        intrinsic = root / "bev/intrinsics.yaml"
        bev = root / "bev/candidate-bev.yaml"
        if not intrinsic.is_file() or not bev.is_file() or hashlib.sha256(intrinsic.read_bytes()).hexdigest() != snapshot.get("intrinsic_sha256"):
            raise SessionError("参数文件缺失或已改变，请重新标定")
        if hashlib.sha256(bev.read_bytes()).hexdigest() != summary.get("bev_sha256"):
            raise SessionError("BEV 参数已改变，请重新计算并检查")
        output = self.config.effective_runtime_calibration_root
        release = f"{root.name}-{uuid.uuid4().hex[:12]}"
        versions = output / "versions"
        target = versions / release
        pointer = output / f".current-{uuid.uuid4().hex}"
        try:
            versions.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(dir=versions) as tmp:
                stage = Path(tmp) / "release"
                stage.mkdir()
                shutil.copy2(intrinsic, stage / "intrinsics.yaml")
                shutil.copy2(bev, stage / "bev.yaml")
                self.worker("accept", stage)
                manifest = {"release_id": release, "session_id": root.name,
                    "created_at": datetime.now(timezone.utc).isoformat(), "reviewer": metadata.get("operator"),
                    "reference": "rear_axle_ground", "axes": "x_forward_y_left", "image_id": snapshot["image_id"],
                    "intrinsic_path": str((target / "intrinsics.yaml").absolute()),
                    "bev_path": str((target / "bev.yaml").absolute()),
                    "sha256": {name: hashlib.sha256((stage / name).read_bytes()).hexdigest()
                               for name in ("intrinsics.yaml", "bev.yaml")}}
                write_json(stage / "manifest.json", manifest)
                # Pin both paths to this immutable version, even if current changes at startup.
                write_json(stage / "sdk-calibration.json", {key: manifest[key]
                    for key in ("intrinsic_path", "bev_path")})
                import json
                (stage / "observer.calibration.yaml").write_text(
                    "crop_row_perception:\n  ros__parameters:\n"
                    + "".join(f"    {key}: {json.dumps(value)}\n" for key, value in {
                        "intrinsic_path": manifest["intrinsic_path"], "bev_path": manifest["bev_path"],
                        "calibration_id": release}.items()), encoding="utf-8")
                stage.rename(target)
            pointer.symlink_to(Path("versions") / release, target_is_directory=True)
            os.replace(pointer, output / "current")
        except OSError as exc:
            raise SessionError("保存算法参数失败，请检查目标目录权限；原参数不会被逐文件覆盖") from exc
        finally:
            pointer.unlink(missing_ok=True)
        write_json(root / "runtime-publication.json", manifest)
        self.invalidate_export()
        return self.sessions.describe()

    def invalidate_export(self):
        (self.root() / "field-session-result.zip").unlink(missing_ok=True)
