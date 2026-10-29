from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys


@dataclass(frozen=True)
class AssistantConfig:
    workspace: Path
    session_root: Path
    mode: str = "mock"
    camera_device: Path = Path("/dev/video12")
    preview_path: Path | None = None
    calibration_path: Path | None = None
    navigation_log: Path | None = None
    gnss_log: Path | None = None
    import_root: Path | None = None
    calibration_python: Path = Path(sys.executable)
    calibration_script: Path | None = None
    capture_binary: Path | None = None
    runtime_calibration_root: Path | None = None

    @property
    def effective_runtime_calibration_root(self) -> Path:
        return self.runtime_calibration_root or (self.workspace / "configs" / "calibration")

    def validate(self) -> None:
        if self.mode not in {"mock", "hardware"}:
            raise ValueError("mode must be 'mock' or 'hardware'")

    @property
    def static_root(self) -> Path:
        return Path(__file__).resolve().parent / "static"

    @property
    def effective_import_root(self) -> Path:
        return self.import_root or (self.workspace / "data" / "field-import")

    @property
    def effective_calibration_script(self) -> Path:
        return self.calibration_script or (
            Path(__file__).resolve().parents[2] / "scripts" / "calibration" / "calibrate_charuco.py"
        )
