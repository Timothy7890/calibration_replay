from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from typing import Any


class NonRetryableCaptureError(RuntimeError):
    """Retrying would only repeat the damage (e.g. more misplaced episodes)."""


class CaptureAdapter:
    def preflight(self, run_id: str) -> dict[str, Any]:
        raise NotImplementedError

    def capture(
        self,
        *,
        capture_id: str,
        run_id: str,
        waypoint_id: str,
        target_q_rad: list[float],
        stability: dict[str, Any],
        record_dir: str | None = None,
    ) -> dict[str, Any]:
        raise NotImplementedError


class HttpCaptureAdapter(CaptureAdapter):
    def __init__(
        self,
        base_url: str,
        target: str,
        *,
        arm: str = "right",
        require_corners: bool = True,
        camera_serial: str | None = None,
        frame_count: int = 5,
        timeout_s: float = 10.0,
        retries: int = 2,
    ):
        self.base_url = base_url.rstrip("/")
        self.target = target
        self.arm = arm
        self.arm_selectable = False
        self.require_corners = require_corners
        self.camera_serial = camera_serial.strip() if camera_serial else None
        self.frame_count = int(frame_count)
        self.timeout_s = timeout_s
        self.retries = retries

    def _request(
        self, method: str, path: str, body: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        data = None if body is None else json.dumps(body).encode("utf-8")
        request = urllib.request.Request(
            self.base_url + path,
            data=data,
            method=method,
            headers={"Accept": "application/json", "Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"{method} {path} returned HTTP {exc.code}: {detail}") from exc
        except (urllib.error.URLError, TimeoutError) as exc:
            raise RuntimeError(f"{method} {path} failed: {exc}") from exc

    @staticmethod
    def _response_ok(payload: dict[str, Any]) -> bool:
        return bool(payload.get("ok", payload.get("success", True)))

    def preflight(self, run_id: str) -> dict[str, Any]:
        health = self._request("GET", "/api/status")
        arm = self._request("GET", "/api/arm/status")
        dangerous = any(
            bool(arm.get(key))
            for key in (
                "armed",
                "engaged",
                "jog_enabled",
                "motion_enabled",
                "control_enabled",
                "available",
                "enabled",
            )
        )
        if dangerous:
            raise RuntimeError(
                "capture target exposes active/enabled arm control; restart it without --arm-control"
            )
        active_arm = health.get("arm")
        recording = health.get("recording") or {}
        # 新版 8132 可按请求记录任一条臂（同一帧 lowstate 的另一组电机）；
        # 旧版只记 --arm 那条臂，此时必须与计划一致，否则关节数据是另一条臂的。
        self.arm_selectable = bool(recording.get("arm_selectable"))
        if active_arm is not None and active_arm != self.arm and not self.arm_selectable:
            raise RuntimeError(
                f"capture service is recording the {active_arm} arm but this plan is for the "
                f"{self.arm} arm; restart it with --arm {self.arm}"
            )
        result = {"ok": True, "health": health, "arm": arm}
        if self.target.startswith("hand_eye_2D"):
            try:
                session = self._request(
                    "POST", "/api/session/start", {"run_id": run_id}
                )
                if not self._response_ok(session):
                    raise RuntimeError(
                        str(session.get("error", "2D session start rejected"))
                    )
                if session.get("run_id") != run_id or int(session.get("count", -1)) != 0:
                    raise RuntimeError(
                        "2D session start did not return the requested empty session"
                    )
            except RuntimeError:
                # A response may be lost after the server created the directory.
                # Verify once instead of blindly retrying a non-idempotent start.
                current = self._request("GET", "/api/status")
                if current.get("run_id") != run_id or int(current.get("count", -1)) != 0:
                    raise
                session = {
                    "success": True,
                    "run_id": run_id,
                    "count": 0,
                    "verified_after_uncertain_response": True,
                }
            result["session"] = session

            if self.camera_serial:
                selected = self._request(
                    "POST", "/api/camera/select", {"serial": self.camera_serial}
                )
                if not self._response_ok(selected):
                    raise RuntimeError(
                        str(selected.get("error", "camera selection rejected"))
                    )
                camera_status = self._request("GET", "/api/status")
                current_serial = (camera_status.get("camera") or {}).get("serial")
                if (
                    camera_status.get("run_id") != run_id
                    or int(camera_status.get("count", -1)) != 0
                ):
                    raise RuntimeError("2D session changed during camera selection")
                if current_serial != self.camera_serial:
                    raise RuntimeError(
                        f"camera verification failed: requested {self.camera_serial!r}, "
                        f"target reports {current_serial!r}"
                    )
                result["camera"] = camera_status.get("camera")

            try:
                detection = self._request("POST", "/api/checkerboard/detect", {})
            except RuntimeError as exc:
                detection = {
                    "success": False,
                    "found": False,
                    "informational": True,
                    "error": str(exc),
                }
            result["detection"] = detection
        elif (health.get("recording") or {}).get("enabled") is False:
            raise RuntimeError("3D target recording is not enabled")
        return result

    def capture(
        self,
        *,
        capture_id: str,
        run_id: str,
        waypoint_id: str,
        target_q_rad: list[float],
        stability: dict[str, Any],
        record_dir: str | None = None,
    ) -> dict[str, Any]:
        body = {
            "run_id": run_id,
            "waypoint_id": waypoint_id,
            "capture_id": capture_id,
            "target_q_rad": target_q_rad,
            "stability": stability,
        }
        if self.target == "hand_eye_3D":
            path = "/api/record/episode"
            body["frame_count"] = self.frame_count
            body["arm"] = self.arm
            if record_dir:
                # 8132 与本服务同机：episode 直接落到本次运行的目录
                body["record_dir"] = record_dir
        else:
            path = "/api/capture"
            body["require_corners"] = self.require_corners
        last_error: Exception | None = None
        for attempt in range(self.retries + 1):
            try:
                result = self._request("POST", path, body)
                ok = result.get("ok", result.get("success", True))
                if not ok:
                    raise RuntimeError(str(result.get("error", "capture rejected")))
                if (
                    self.target.startswith("hand_eye_2D")
                    and self.require_corners
                    and result.get("corners_detected") is False
                ):
                    raise RuntimeError("capture rejected: chessboard corners not detected")
                if self.target == "hand_eye_3D" and result.get("arm") not in (None, self.arm):
                    raise NonRetryableCaptureError(
                        f"capture service recorded the {result.get('arm')} arm instead of "
                        f"{self.arm}; restart the hand_eye_3D backend (8132) with --arm {self.arm}"
                    )
                if record_dir and self.target == "hand_eye_3D":
                    path = str(result.get("path") or "")
                    if not path.startswith(record_dir.rstrip("/") + "/"):
                        # 旧版 8132 会忽略 record_dir 并写进默认目录：宁可失败也不能让数据混进去
                        raise NonRetryableCaptureError(
                            "capture service ignored record_dir and wrote to "
                            f"{path or '?'}; restart the hand_eye_3D backend (8132) "
                            "so episodes land in the run directory"
                        )
                return result
            except NonRetryableCaptureError:
                raise
            except Exception as exc:
                last_error = exc
                if attempt < self.retries:
                    time.sleep(0.25 * (attempt + 1))
        raise RuntimeError(f"capture failed after retries: {last_error}")


class MockCaptureAdapter(CaptureAdapter):
    def __init__(self):
        self.calls: list[dict[str, Any]] = []
        self.preflight_calls: list[str] = []

    def preflight(self, run_id: str) -> dict[str, Any]:
        self.preflight_calls.append(run_id)
        return {
            "ok": True,
            "mock": True,
            "run_id": run_id,
            "arm": {"enabled": False},
        }

    def capture(self, **kwargs) -> dict[str, Any]:
        self.calls.append(kwargs)
        return {
            "ok": True,
            "mock": True,
            "capture_id": kwargs["capture_id"],
            "index": len(self.calls) - 1,
        }
