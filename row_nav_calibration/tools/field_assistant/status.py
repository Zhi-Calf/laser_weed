from __future__ import annotations

import json
import platform
import re
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .config import AssistantConfig


_SCALAR_PATTERN = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*?)\s*$")


def _clean_yaml_scalar(value: str) -> str:
    value = value.split("#", 1)[0].strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
        return value[1:-1]
    return value


def read_calibration_summary(path: Path | None) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "state": "missing",
        "label": "未加载",
        "message": "尚未选择相机内参文件",
    }
    if path is None:
        return summary
    if not path.is_file():
        return {
            **summary,
            "state": "error",
            "label": "文件不存在",
            "message": str(path),
        }

    fields: dict[str, str] = {}
    try:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            match = _SCALAR_PATTERN.match(line)
            if match:
                fields[match.group(1)] = _clean_yaml_scalar(match.group(2))
    except OSError as exc:
        return {
            **summary,
            "state": "error",
            "label": "读取失败",
            "message": str(exc),
        }

    valid_raw = fields.get("calibration_valid")
    is_candidate = "candidate" in path.name.lower()
    if valid_raw is None:
        state = "candidate" if is_candidate else "review"
        label = "候选参数" if is_candidate else "等待验收"
    elif valid_raw.lower() in {"1", "true", "yes"}:
        state = "ready"
        label = "已通过"
    else:
        state = "candidate"
        label = "未通过"

    def number(name: str) -> float | None:
        try:
            return float(fields[name])
        except (KeyError, ValueError):
            return None

    def integer(name: str) -> int | None:
        try:
            return int(fields[name])
        except (KeyError, ValueError):
            return None

    return {
        "state": state,
        "label": label,
        "message": str(path),
        "path": str(path),
        "camera_name": fields.get("camera_name", "未知相机"),
        "image_width": integer("image_width"),
        "image_height": integer("image_height"),
        "reprojection_rms_px": number("reprojection_rms_px"),
        "calibration_valid": valid_raw,
    }


def read_last_navigation(path: Path | None) -> dict[str, Any] | None:
    if path is None or not path.is_file():
        return None
    last: dict[str, Any] | None = None
    try:
        with path.open("r", encoding="utf-8") as stream:
            for line in stream:
                if line.strip():
                    candidate = json.loads(line)
                    if isinstance(candidate, dict):
                        last = candidate
    except (OSError, json.JSONDecodeError):
        return None
    return last


def read_gnss_summary(path: Path | None) -> dict[str, Any] | None:
    if path is None or not path.is_file():
        return None
    summary_line = ""
    fix_quality: int | None = None
    try:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            if "data=$GNGGA" in line:
                payload = line.split("data=", 1)[1]
                fields = payload.split(",")
                if len(fields) > 6:
                    try:
                        fix_quality = int(fields[6] or "0")
                    except ValueError:
                        pass
            if line.startswith("summary "):
                summary_line = line
    except OSError:
        return None
    values = {
        key: int(value)
        for key, value in re.findall(r"([a-z_]+)=([0-9]+)", summary_line)
    }
    return {
        "fix_quality": fix_quality,
        "gps_messages": values.get("gps_messages", 0),
        "imu_nonzero_samples": values.get("imu_nonzero_samples", 0),
        "rtk_true_samples": values.get("rtk_true_samples", 0),
        "source": str(path),
    }


class StatusService:
    def __init__(self, config: AssistantConfig):
        config.validate()
        self.config = config

    def snapshot(self) -> dict[str, Any]:
        cfg = self.config
        cfg.session_root.mkdir(parents=True, exist_ok=True)
        calibration = read_calibration_summary(cfg.calibration_path)
        navigation = read_last_navigation(cfg.navigation_log)
        gnss = read_gnss_summary(cfg.gnss_log)
        disk = shutil.disk_usage(cfg.session_root)

        preview_available = bool(cfg.preview_path and cfg.preview_path.is_file())
        if cfg.mode == "hardware":
            camera_available = cfg.camera_device.exists()
            camera_message = (
                f"检测到 {cfg.camera_device}" if camera_available
                else f"未检测到 {cfg.camera_device}"
            )
        else:
            camera_available = preview_available
            camera_message = (
                "使用已有图片模拟相机" if preview_available else "未配置模拟预览图片"
            )

        if gnss is None:
            gnss_card = {
                "state": "pending",
                "label": "等待真机",
                "message": "MyUbuntu 不模拟卫星定位结果",
            }
            imu_card = {
                "state": "pending",
                "label": "等待厂家",
                "message": "真机 IMU 数据流尚未确认",
            }
        else:
            fixed = bool(gnss.get("fix_quality", 0))
            gnss_card = {
                "state": "ready" if fixed else "warning",
                "label": "已定位" if fixed else "未定位",
                "message": f"历史日志记录 {gnss['gps_messages']} 条 GNSS 消息",
                "details": gnss,
            }
            imu_samples = int(gnss.get("imu_nonzero_samples", 0))
            imu_card = {
                "state": "ready" if imu_samples else "warning",
                "label": "有数据" if imu_samples else "数据全零",
                "message": f"非零 IMU 样本 {imu_samples}",
                "details": gnss,
            }

        calibration_ready = calibration["state"] == "ready"
        items = [
            {
                "id": "camera",
                "name": "相机输入",
                "state": "ready" if camera_available else "error",
                "label": "可用" if camera_available else "不可用",
                "message": camera_message,
            },
            {
                "id": "intrinsics",
                "name": "相机内参",
                "state": "ready" if calibration_ready else calibration["state"],
                "label": calibration["label"],
                "message": calibration["message"],
            },
            {"id": "gnss", "name": "GNSS", **gnss_card},
            {"id": "imu", "name": "IMU", **imu_card},
        ]

        return {
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "mode": cfg.mode,
            "platform": {
                "hostname": platform.node() or "unknown",
                "system": f"{platform.system()} {platform.release()}",
                "machine": platform.machine(),
            },
            "camera": {
                "device": str(cfg.camera_device),
                "preview_available": preview_available,
            },
            "calibration": calibration,
            "navigation": navigation,
            "disk": {
                "free_gb": round(disk.free / (1024**3), 1),
                "total_gb": round(disk.total / (1024**3), 1),
            },
            "checks": items,
            "ready_for_intrinsic": camera_available,
            "ready_for_bev": calibration_ready and cfg.mode == "hardware",
            "limitations": [
                "MyUbuntu 可以验证网页流程、文件回放、标定结果读取和导出。",
                "真实相机占用、GNSS/IMU、RTK 和最终 BEV 必须在 AWU202 上验收。",
            ],
        }
