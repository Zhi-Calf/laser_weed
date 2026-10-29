from __future__ import annotations

import hashlib
import json
import re
import shutil
import subprocess
import zipfile
import tempfile
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .config import AssistantConfig


class SessionError(ValueError):
    pass


def _safe_token(value: str, fallback: str = "operator") -> str:
    token = re.sub(r"[^A-Za-z0-9_-]+", "-", value.strip()).strip("-")
    return token[:32] or fallback


def _git_revision(workspace: Path) -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short=12", "HEAD"],
            cwd=workspace,
            check=True,
            capture_output=True,
            text=True,
            timeout=3,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return "unknown"


class SessionStore:
    def __init__(self, config: AssistantConfig):
        self.config = config
        self.current_file = config.session_root / ".current-session"

    def start(self, payload: dict[str, Any], status: dict[str, Any]) -> dict[str, Any]:
        operator = str(payload.get("operator", "")).strip()
        if not operator:
            raise SessionError("请填写操作人员姓名")
        board_flat = payload.get("board_flat") is True
        if not board_flat:
            raise SessionError("开始前必须确认标定板已经平整固定")

        dimensions: dict[str, float] = {}
        for key, label in (
            ("square_width_mm", "横向方格边长"),
            ("square_height_mm", "纵向方格边长"),
            ("marker_mm", "标记边长"),
            ("check_line_mm", "100 mm 校验线"),
        ):
            try:
                value = float(payload.get(key, 0))
            except (TypeError, ValueError) as exc:
                raise SessionError(f"{label}必须是数字") from exc
            if value <= 0:
                raise SessionError(f"{label}必须大于 0")
            dimensions[key] = value

        mean_square = (dimensions["square_width_mm"] + dimensions["square_height_mm"]) / 2
        if abs(dimensions["square_width_mm"] - dimensions["square_height_mm"]) > max(0.8, mean_square * 0.02):
            raise SessionError("横纵方格尺寸差异过大，请检查打印缩放或纸张变形")
        if not 98.0 <= dimensions["check_line_mm"] <= 102.0:
            raise SessionError("底部校验线应在 98–102 mm 之间，请重新测量打印尺寸")
        if dimensions["marker_mm"] >= min(dimensions["square_width_mm"], dimensions["square_height_mm"]):
            raise SessionError("标记边长必须小于方格边长")

        now = datetime.now(timezone.utc)
        session_id = f"{now.strftime('%Y%m%d-%H%M%S')}-{_safe_token(operator)}"
        session_dir = self.config.session_root / session_id
        if session_dir.exists():
            raise SessionError("同名会话已存在，请稍后再试")
        for relative in (
            "raw/intrinsic",
            "raw/validation",
            "raw/bev",
            "validation",
            "logs",
        ):
            (session_dir / relative).mkdir(parents=True, exist_ok=True)

        metadata = {
            "schema_version": 1,
            "session_id": session_id,
            "created_at": now.isoformat(),
            "operator": operator,
            "mode": self.config.mode,
            "git_revision": _git_revision(self.config.workspace),
            "camera_device": str(self.config.camera_device),
            "board": {
                "type": "A2 7x10 DICT_5X5_1000",
                **dimensions,
                "flat_confirmed": True,
            },
            "acceptance": {
                "intrinsic": "not_started",
                "bev": "not_started",
                "calibration_valid": False,
            },
            "status_at_start": status,
        }
        (session_dir / "session.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        for source, destination in (
            (self.config.calibration_path, session_dir / "candidate-intrinsics.yaml"),
            (self.config.preview_path, session_dir / "preview.jpg"),
            (self.config.navigation_log, session_dir / "logs/navigation.jsonl"),
            (self.config.gnss_log, session_dir / "logs/gnss-probe.log"),
        ):
            if source is not None and source.is_file():
                shutil.copy2(source, destination)

        self.current_file.parent.mkdir(parents=True, exist_ok=True)
        self.current_file.write_text(session_id + "\n", encoding="utf-8")
        return self.describe(session_id)

    def current_id(self) -> str | None:
        try:
            session_id = self.current_file.read_text(encoding="utf-8").strip()
        except OSError:
            return None
        return session_id if self._session_dir(session_id).is_dir() else None

    def describe(self, session_id: str | None = None) -> dict[str, Any] | None:
        session_id = session_id or self.current_id()
        if not session_id:
            return None
        path = self._session_dir(session_id)
        try:
            metadata = json.loads((path / "session.json").read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
        intrinsic_summary = self._read_json(path / "validation" / "intrinsic-summary.json")
        snapshot = self._read_json(path / "bev" / "snapshot.json")
        return {
            "id": session_id,
            "path": str(path),
            "metadata": metadata,
            "capture_counts": {group: len(self._images_in(path / "captures" / group))
                               if (path / "captures" / group).is_dir() else 0
                               for group in ("intrinsic", "validation")},
            "intrinsic_summary": intrinsic_summary,
            "bev": {"snapshot": snapshot, "points": self._read_json(path / "bev" / "points.json"),
                    "reference": self._read_json(path / "bev" / "reference.json"),
                    "summary": self._read_json(path / "bev" / "summary.json")} if snapshot else None,
            "download_ready": (path / "field-session-result.zip").is_file(),
            "runtime_calibration": {
                "current_path": str(self.config.effective_runtime_calibration_root / "current"),
                "active": self._read_json(self.config.effective_runtime_calibration_root / "current" / "manifest.json"),
                "published": self._read_json(path / "runtime-publication.json"),
            },
        }

    def run_intrinsic(self, payload: dict[str, Any]) -> dict[str, Any]:
        session_id = self.current_id()
        if not session_id:
            raise SessionError("请先开始一个现场会话")
        root = self._session_dir(session_id)
        metadata = self._read_json(root / "session.json")
        if metadata is None:
            raise SessionError("当前会话元数据损坏")
        if metadata.get("intrinsic_run"):
            raise SessionError("本会话已完成内参计算，请新建会话后重新标定")
        # Recover unfinished attempts from earlier versions or interrupted runs.
        self._archive_failed_intrinsic(root)
        (root / "field-session-result.zip").unlink(missing_ok=True)
        try:
            return self._run_intrinsic(payload)
        except (SessionError, OSError) as exc:
            self._archive_failed_intrinsic(root)
            raise SessionError(
                f"{exc}。可在本会话补拍或重拍后再次计算；失败记录保留在测试包的 intrinsic-attempts 目录。"
            ) from exc

    def _archive_failed_intrinsic(self, root: Path) -> None:
        """Keep diagnostics, but never expose partial output as a candidate."""
        relatives = (
            "raw/intrinsic", "raw/validation", "cmd108-intrinsics.yaml",
            "validation/intrinsic-summary.json", "validation/intrinsic",
            "logs/intrinsic-calibration.log",
        )
        paths = [name for name in relatives if (root / name).exists()
                 and (not (root / name).is_dir() or any((root / name).iterdir()))]
        if paths:
            archive = root / "intrinsic-attempts" / uuid.uuid4().hex
            for name in paths:
                destination = archive / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                (root / name).rename(destination)
            (root / "field-session-result.zip").unlink(missing_ok=True)
        for name in ("raw/intrinsic", "raw/validation"):
            (root / name).mkdir(parents=True, exist_ok=True)

    def _run_intrinsic(self, payload: dict[str, Any]) -> dict[str, Any]:
        session_id = self.current_id()
        if not session_id:
            raise SessionError("请先开始一个现场会话")
        session_dir = self._session_dir(session_id)
        metadata_path = session_dir / "session.json"
        metadata = self._read_json(metadata_path)
        if metadata is None:
            raise SessionError("当前会话元数据损坏")

        if payload.get("source") == "captured":
            train_dir = session_dir / "captures" / "intrinsic"
            validation_dir = session_dir / "captures" / "validation"
            if not train_dir.is_dir() or not validation_dir.is_dir():
                raise SessionError("请先完成拟合与独立验证两组拍照")
        else:
            train_dir = self._resolve_import_dir(str(payload.get("train_dir", "")))
            validation_dir = self._resolve_import_dir(str(payload.get("validation_dir", "")))
        train_images = self._images_in(train_dir)
        validation_images = self._images_in(validation_dir)
        minimum_train = 5 if self.config.mode == "mock" else 25
        minimum_validation = 1 if self.config.mode == "mock" else 8
        if len(train_images) < minimum_train:
            raise SessionError(f"训练图片至少需要 {minimum_train} 张，当前只有 {len(train_images)} 张")
        if len(validation_images) < minimum_validation:
            raise SessionError(
                f"独立验证图片至少需要 {minimum_validation} 张，当前只有 {len(validation_images)} 张"
            )

        raw_train = session_dir / "raw" / "intrinsic"
        raw_validation = session_dir / "raw" / "validation"
        if any(raw_train.iterdir()) or any(raw_validation.iterdir()):
            raise SessionError("当前会话已经导入过标定图片，请建立新会话后重试")
        for index, source in enumerate(train_images, start=1):
            shutil.copy2(source, raw_train / f"train-{index:03d}{source.suffix.lower()}")
        for index, source in enumerate(validation_images, start=1):
            shutil.copy2(source, raw_validation / f"validation-{index:03d}{source.suffix.lower()}")

        board = metadata["board"]
        output = session_dir / "cmd108-intrinsics.yaml"
        summary = session_dir / "validation" / "intrinsic-summary.json"
        comparisons = session_dir / "validation" / "intrinsic"
        log_path = session_dir / "logs" / "intrinsic-calibration.log"
        camera_name = str(payload.get("camera_name", "CMD108-P200A-H120")).strip()
        if not camera_name or len(camera_name) > 80:
            raise SessionError("相机名称不能为空且不能超过 80 个字符")

        python = self.config.calibration_python
        script = self.config.effective_calibration_script
        if not python.is_file():
            raise SessionError(f"标定 Python 不存在：{python}")
        if not script.is_file():
            raise SessionError(f"标定脚本不存在：{script}")
        command = [
            str(python),
            str(script),
            "--train-glob",
            str(raw_train / "*"),
            "--validation-glob",
            str(raw_validation / "*"),
            "--output",
            str(output),
            "--summary",
            str(summary),
            "--comparison-dir",
            str(comparisons),
            "--camera-name",
            camera_name,
            "--squares-x",
            "7",
            "--squares-y",
            "10",
            "--square-m",
            f"{(float(board['square_width_mm']) + float(board['square_height_mm'])) / 2000.0:.9f}",
            "--marker-m",
            f"{float(board['marker_mm']) / 1000.0:.9f}",
        ]
        try:
            result = subprocess.run(
                command,
                cwd=self.config.workspace,
                capture_output=True,
                text=True,
                timeout=300,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            log_path.write_text(f"{type(exc).__name__}: {exc}\n", encoding="utf-8")
            raise SessionError(f"标定程序无法完成：{exc}") from exc
        log_path.write_text(
            f"exit_code={result.returncode}\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            encoding="utf-8",
        )
        if result.returncode != 0 or not output.is_file() or not summary.is_file():
            raise SessionError("标定失败，请查看测试包中的 intrinsic-calibration.log")

        calibration_summary = self._read_json(summary)
        if calibration_summary is None:
            raise SessionError("标定程序没有生成有效摘要")
        metadata["acceptance"]["intrinsic"] = "candidate"
        metadata["acceptance"]["calibration_valid"] = False
        metadata["intrinsic_run"] = {
            "completed_at": datetime.now(timezone.utc).isoformat(),
            "train_imported": len(train_images),
            "validation_imported": len(validation_images),
            "camera_name": camera_name,
            "human_review_required": True,
        }
        metadata_path.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        return self.describe(session_id) or {}

    def capture(self, payload: dict[str, Any]) -> dict[str, Any]:
        session_id = self.current_id()
        if not session_id:
            raise SessionError("请先建立现场会话")
        group = payload.get("group")
        slot = payload.get("slot")
        maximum = {"intrinsic": 27, "validation": 9, "bev": 1}.get(group, 0) if isinstance(group, str) else 0
        if type(slot) is not int or not 0 <= slot < maximum:
            raise SessionError("无效的拍照位置")
        root = self._session_dir(session_id)
        metadata = self._read_json(root / "session.json")
        if metadata is None:
            raise SessionError("当前会话元数据损坏")
        if group != "bev" and metadata.get("intrinsic_run"):
            raise SessionError("本会话已运行标定，请新建会话后重新采集")
        if group != "bev":
            self._archive_failed_intrinsic(root)
        destination = root / "captures" / group
        destination.mkdir(parents=True, exist_ok=True)
        target = destination / f"position-{slot:02d}.jpg"
        with tempfile.TemporaryDirectory(dir=root) as temporary:
            scratch = Path(temporary)
            if self.config.mode == "mock":
                source = self.config.preview_path
                if source is None or not source.is_file():
                    raise SessionError("模拟拍照需要先配置回放图片")
                shutil.copy2(source, scratch / "frame.jpg")
            else:
                binary = self.config.capture_binary
                if binary is None or not binary.is_file():
                    raise SessionError("尚未配置拍照程序，请联系部署人员")
                try:
                    result = subprocess.run(
                        [str(binary), str(scratch), "1", "5000", "0", str(self.config.camera_device)],
                        capture_output=True, text=True, timeout=30, check=False,
                    )
                except (OSError, subprocess.TimeoutExpired) as exc:
                    raise SessionError("拍照失败或超时，请检查相机连接") from exc
                (root / "logs" / "last-capture.log").write_text(
                    result.stdout + result.stderr, encoding="utf-8")
                if result.returncode != 0:
                    raise SessionError("拍照失败，请检查相机是否被其他程序占用")
            images = self._images_in(scratch)
            if len(images) != 1:
                raise SessionError("拍照程序未生成一张有效照片")
            images[0].replace(target)
        # Re-taking a position replaces only that position; old exports are stale.
        (root / "field-session-result.zip").unlink(missing_ok=True)
        return {"session": self.describe(session_id), "image_url":
                f"/api/capture-image?session={session_id}&group={group}&slot={slot}",
                "simulated": self.config.mode == "mock"}

    def capture_image(self, session_id: str, group: str, slot: int) -> Path:
        maximum = 27 if group == "intrinsic" else 9
        if group not in {"intrinsic", "validation"} or not 0 <= slot < maximum:
            raise SessionError("无效的拍照位置")
        return self._session_dir(session_id) / "captures" / group / f"position-{slot:02d}.jpg"

    def export_current(self, status: dict[str, Any]) -> dict[str, Any]:
        session_id = self.current_id()
        if not session_id:
            raise SessionError("请先开始一个现场会话")
        session_dir = self._session_dir(session_id)
        (session_dir / "status-at-export.json").write_text(
            json.dumps(status, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        checksum_path = session_dir / "checksums.sha256"
        zip_path = session_dir / "field-session-result.zip"
        checksum_lines: list[str] = []
        for path in sorted(session_dir.rglob("*")):
            if not path.is_file() or path in {checksum_path, zip_path}:
                continue
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            checksum_lines.append(f"{digest}  {path.relative_to(session_dir).as_posix()}")
        checksum_path.write_text("\n".join(checksum_lines) + "\n", encoding="utf-8")

        with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(session_dir.rglob("*")):
                if path.is_file() and path != zip_path:
                    archive.write(path, path.relative_to(session_dir))
        result = self.describe(session_id)
        assert result is not None
        result["download_url"] = "/api/sessions/current/download"
        result["size_bytes"] = zip_path.stat().st_size
        return result

    def download_path(self) -> Path:
        session_id = self.current_id()
        if not session_id:
            raise SessionError("没有可下载的当前会话")
        path = self._session_dir(session_id) / "field-session-result.zip"
        if not path.is_file():
            raise SessionError("请先导出测试包")
        return path

    def _session_dir(self, session_id: str) -> Path:
        if not re.fullmatch(r"[A-Za-z0-9_-]+", session_id):
            raise SessionError("无效的会话编号")
        return self.config.session_root / session_id

    def _resolve_import_dir(self, relative: str) -> Path:
        if not relative.strip():
            raise SessionError("请输入图片目录")
        relative_path = Path(relative)
        if relative_path.is_absolute():
            raise SessionError("图片目录必须是导入根目录下的相对路径")
        root = self.config.effective_import_root.resolve()
        path = (root / relative_path).resolve()
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise SessionError("图片目录不能离开导入根目录") from exc
        if not path.is_dir():
            raise SessionError(f"图片目录不存在：{relative}")
        return path

    @staticmethod
    def _images_in(path: Path) -> list[Path]:
        suffixes = {".jpg", ".jpeg", ".png"}
        return sorted(
            item for item in path.iterdir()
            if item.is_file() and item.suffix.lower() in suffixes
        )

    @staticmethod
    def _read_json(path: Path) -> dict[str, Any] | None:
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
        return value if isinstance(value, dict) else None
