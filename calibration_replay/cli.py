from __future__ import annotations

import argparse
from pathlib import Path

import uvicorn

from .app import AppConfig, create_app


def build_parser() -> argparse.ArgumentParser:
    project_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description="Human-supervised H2 calibration trajectory replay (left/right arm per plan)"
    )
    parser.add_argument("--network-interface", help="DDS network interface for H2")
    parser.add_argument(
        "--hand-eye-3d-project",
        default=str(project_root / "calib_workstation"),
        help="兼容参数：包含统一H2标定/运动运行时的 calib_workstation 项目",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18004)
    parser.add_argument(
        "--data-root",
        default=str(Path.cwd() / "replay_data"),
        help="Internal authoritative plans and run records",
    )
    parser.add_argument("--base-url-2d", default="http://127.0.0.1:18005")
    parser.add_argument("--base-url-3d", default="http://127.0.0.1:18005/three-d")
    parser.add_argument(
        "--capability-url",
        default="http://127.0.0.1:18000",
        help="18000 capability registry (shows which arm/hand is active)",
    )
    parser.add_argument(
        "--robot-mesh-dir",
        default=None,
        help="Directory containing meshes/*.stl for the in-page 3D preview "
        "(default: calib_workstation or IK_replay H2 assets)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="No DDS, no physical motion, and no capture-service network calls",
    )
    parser.add_argument(
        "--mock-capture-http",
        action="store_true",
        help="With --mock: still call the 2D/3D capture service over HTTP "
        "(start it in its own mock mode) for a full-stack rehearsal",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    config = AppConfig(
        data_root=args.data_root,
        h2_project=args.hand_eye_3d_project,
        network_interface=args.network_interface,
        mock=args.mock,
        mock_capture_http=args.mock_capture_http,
        base_url_2d=args.base_url_2d,
        base_url_3d=args.base_url_3d,
        robot_mesh_dir=args.robot_mesh_dir,
        capability_url=args.capability_url,
    )
    uvicorn.run(create_app(config), host=args.host, port=args.port)
