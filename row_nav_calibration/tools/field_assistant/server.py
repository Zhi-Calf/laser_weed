from __future__ import annotations

import argparse
import json
import mimetypes
import sys
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse, parse_qs

from .config import AssistantConfig
from .sessions import SessionError, SessionStore
from .status import StatusService
from .bev import BevService


class FieldAssistantServer(ThreadingHTTPServer):
    def __init__(self, address: tuple[str, int], config: AssistantConfig):
        super().__init__(address, FieldAssistantHandler)
        self.config = config
        self.status_service = StatusService(config)
        self.sessions = SessionStore(config)
        self.bev = BevService(self.sessions)
        self.operation_lock = threading.Lock()


class FieldAssistantHandler(BaseHTTPRequestHandler):
    server: FieldAssistantServer
    protocol_version = "HTTP/1.1"

    def do_GET(self) -> None:  # noqa: N802
        route = urlparse(self.path).path
        if route == "/healthz":
            self._json({"ok": True})
        elif route == "/api/status":
            self._json(self.server.status_service.snapshot())
        elif route == "/api/sessions/current":
            self._json({"session": self.server.sessions.describe()})
        elif route == "/api/capture-image":
            try:
                query = parse_qs(urlparse(self.path).query)
                self._file(self.server.sessions.capture_image(
                    query["session"][0], query["group"][0], int(query["slot"][0])))
            except (KeyError, ValueError):
                self._json({"error": "无效的照片路径"}, HTTPStatus.BAD_REQUEST)
        elif route == "/api/bev-image":
            try:
                query = parse_qs(urlparse(self.path).query)
                name = query["name"][0]
                if name not in {"snapshot.jpg", "overlay.jpg", "bev.jpg"}:
                    raise ValueError()
                root = self.server.sessions._session_dir(query["session"][0])
                self._file(root / "bev" / name)
            except (KeyError, ValueError):
                self._json({"error": "无效的 BEV 图片路径"}, HTTPStatus.BAD_REQUEST)
        elif route == "/api/sessions/current/download":
            try:
                self._file(self.server.sessions.download_path(), download=True)
            except SessionError as exc:
                self._json({"error": str(exc)}, HTTPStatus.NOT_FOUND)
        elif route == "/api/preview":
            preview = self.server.config.preview_path
            if preview is None or not preview.is_file():
                self._json({"error": "当前没有预览图片"}, HTTPStatus.NOT_FOUND)
            else:
                self._file(preview)
        elif route == "/":
            self._file(self.server.config.static_root / "index.html")
        elif route.startswith("/static/"):
            relative = route.removeprefix("/static/")
            if "/" in relative or "\\" in relative or relative.startswith("."):
                self._json({"error": "无效资源路径"}, HTTPStatus.BAD_REQUEST)
            else:
                self._file(self.server.config.static_root / relative)
        else:
            self._json({"error": "页面不存在"}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        if not self.server.operation_lock.acquire(blocking=False):
            self.close_connection = True
            self._json({"error": "正在拍照或计算，请稍后再试"}, HTTPStatus.CONFLICT)
            return
        try:
            self._post_locked()
        finally:
            self.server.operation_lock.release()

    def _post_locked(self) -> None:
        route = urlparse(self.path).path
        try:
            payload = self._read_json()
            if route == "/api/sessions/start":
                result = self.server.sessions.start(
                    payload, self.server.status_service.snapshot()
                )
                self._json({"session": result}, HTTPStatus.CREATED)
            elif route == "/api/sessions/current/export":
                result = self.server.sessions.export_current(
                    self.server.status_service.snapshot()
                )
                self._json({"session": result})
            elif route == "/api/sessions/current/intrinsics/run":
                result = self.server.sessions.run_intrinsic(payload)
                self._json({"session": result})
            elif route == "/api/sessions/current/capture":
                self._json(self.server.sessions.capture(payload))
            elif route == "/api/sessions/current/bev/prepare":
                self._json({"session": self.server.bev.prepare(payload)})
            elif route == "/api/sessions/current/bev/points":
                self._json({"session": self.server.bev.save_points(payload)})
            elif route == "/api/sessions/current/bev/run":
                self._json({"session": self.server.bev.calculate(payload)})
            elif route == "/api/sessions/current/bev/publish":
                self._json({"session": self.server.bev.publish(payload)})
            else:
                self._json({"error": "接口不存在"}, HTTPStatus.NOT_FOUND)
        except SessionError as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except (ValueError, json.JSONDecodeError):
            self._json({"error": "请求内容必须是有效 JSON"}, HTTPStatus.BAD_REQUEST)

    def _read_json(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("invalid content length") from exc
        if length <= 0 or length > 64 * 1024:
            raise ValueError("invalid body size")
        data = json.loads(self.rfile.read(length))
        if not isinstance(data, dict):
            raise ValueError("body must be an object")
        return data

    def _json(self, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status.value)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _file(self, path: Path, download: bool = False) -> None:
        if not path.is_file():
            self._json({"error": "资源不存在"}, HTTPStatus.NOT_FOUND)
            return
        content = path.read_bytes()
        mime = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK.value)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        if download:
            self.send_header(
                "Content-Disposition", f'attachment; filename="{path.name}"'
            )
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, format: str, *args: object) -> None:
        print(f"field-assistant: {self.address_string()} {format % args}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="RK field calibration assistant")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--mode", choices=("mock", "hardware"), default="mock")
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--sessions", type=Path, default=Path("output/field-sessions"))
    parser.add_argument("--camera-device", type=Path, default=Path("/dev/video12"))
    parser.add_argument("--preview", type=Path)
    parser.add_argument("--capture-binary", type=Path, help="已编译的 charuco_capture 程序")
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--runtime-calibration-dir", type=Path, default=Path("configs/calibration"))
    parser.add_argument("--navigation-log", type=Path)
    parser.add_argument("--gnss-log", type=Path)
    parser.add_argument("--import-root", type=Path, default=Path("data/field-import"))
    parser.add_argument("--calibration-python", type=Path, default=Path(sys.executable))
    return parser


def make_config(args: argparse.Namespace) -> AssistantConfig:
    workspace = args.workspace.resolve()

    def absolute(path: Path | None) -> Path | None:
        if path is None:
            return None
        return path.resolve() if path.is_absolute() else (workspace / path).resolve()

    sessions = absolute(args.sessions)
    assert sessions is not None
    return AssistantConfig(
        workspace=workspace,
        session_root=sessions,
        mode=args.mode,
        camera_device=args.camera_device,
        preview_path=absolute(args.preview),
        capture_binary=absolute(args.capture_binary),
        calibration_path=absolute(args.calibration),
        runtime_calibration_root=absolute(args.runtime_calibration_dir),
        navigation_log=absolute(args.navigation_log),
        gnss_log=absolute(args.gnss_log),
        import_root=absolute(args.import_root),
        # Preserve a venv executable symlink: resolving it drops that venv's packages.
        calibration_python=(args.calibration_python if args.calibration_python.is_absolute()
                            else workspace / args.calibration_python).absolute(),
    )


def main() -> int:
    args = build_parser().parse_args()
    config = make_config(args)
    server = FieldAssistantServer((args.host, args.port), config)
    print(f"Field assistant: http://{args.host}:{args.port}")
    print(f"Mode: {config.mode}; sessions: {config.session_root}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
