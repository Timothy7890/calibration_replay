# H2 calibration trajectory replay (MVP)

Standalone, human-supervised replay for either H2 arm. Every plan carries an
`arm` (`left` or `right`); both arms are first-class peers and one plan drives
exactly one arm. It keeps separate plans for `hand_eye_2D_head`,
`hand_eye_2D_waist`, and `hand_eye_3D`. Internal
versioned plan JSON under the configured data root is authoritative; IK_replay
files are exports.

> **PHYSICAL ROBOT WARNING:** This process must be the sole `rt/arm_sdk` owner.
> Never start either 2D or 3D capture service with `--arm-control` while this
> service is running. Keep an operator at the robot and ready to press
> **IMMEDIATE STOP-HOLD**. Support the arm during guide/disarm and controller
> handoff. The engaged arm must match the plan's arm; the run is refused otherwise.

## Left / right arms

* `Plan.hold_hand_zero` (default on, hand_eye_3D only): before the arm moves the
  engine calls the capture service's `POST /api/mount/hand-hold/start` with the
  hand active in 18000 and the plan's arm as side; hand_eye_3D then streams
  all-zero finger positions to 18089 every 0.3 s for the whole run and the engine
  stops it in `finally` (completed, stopped, fault). Markers are glued to the
  hand, so finger posture must not change between captures. A refusal (18089
  busy, no active hand) is a preflight fault, never a takeover.
* New nodes can be auto-placed: `POST /api/plans/{id}/nodes[/record]` with
  `place: "auto"` inserts into the gap with the least detour
  (`Δ(A,q) + Δ(q,B) − Δ(A,B)`, Δ = max single-joint delta, return leg included);
  `POST /api/plans/{id}/nodes/{node_id}/autoplace` re-places an existing node.
  Home always stays first. The UI checkbox “自动放到最合适位置” (default on) and
  the per-row ⇅ button use these.
* `Plan.arm` selects the arm. Joint names, URDF limits, the 3D preview chain and
  the capture request all follow it: every `POST /api/record/episode` carries
  `arm`, so a hand_eye_3D backend started for either arm records the plan's arm
  (both arms live in the same `rt/lowstate` frame). Only an old backend without
  `recording.arm_selectable` must itself be started with `--arm <plan arm>`;
  a result whose `arm` differs from the plan aborts the run without retry.
* `POST /api/plans/{id}/mirror` copies a plan for the other arm. H2's arms share
  joint axes and are mounted mirrored about Y, so pitch/elbow keep their sign and
  roll/yaw flip (`MIRROR_SIGNS = [1,-1,-1,1,-1,1,-1]`); limits are symmetric.
  The copy is a draft: place the arm at the mirrored home, re-validate, preview.
* `GET /api/capability` proxies the 18000 registry (`--capability-url`) so the
  UI shows which arm/hand the robot currently has active and flags a mismatch.
* Empty plans can switch arm in the UI; plans with nodes must be mirrored instead.

## Install

The robot workstation already has the required runtime in its `fastapi` conda
environment. For a portable clean environment (requires the OS
`python3-venv` package):

```bash
cd /home/robot/yx/project/calib/calibration_replay
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -r requirements.txt
```

## Launch without hardware

No DDS, arm motion, capture HTTP, or runtime network dependency is used:

```bash
cd /home/robot/yx/project/calib/calibration_replay
/home/robot/miniconda3/envs/fastapi/bin/python -m calibration_replay \
  --mock \
  --host 127.0.0.1 \
  --port 18004 \
  --data-root /tmp/calibration_replay_data
```

Open <http://127.0.0.1:18004>. The frontend is a Vue 3 single page under
`calibration_replay/static/` (`index.html`, `app.js`, `app.css`); Vue is vendored
at `static/vendor/vue.global.prod.js`, so there is no CDN dependency and no
build step. The page is organised as five numbered steps: nodes → engage &
home → validate → export/preview → run.

## Launch on H2

First start the desired 2D or 3D camera/capture service **without
`--arm-control`** (for hand_eye_3D that is `./start.sh --no-arm`). Then either
use the launcher:

```bash
cd /home/robot/yx/project/calib/calibration_replay
./replay.sh            # start (H2 mode, background, logs/service/replay.log)
./replay.sh status
./replay.sh stop
./replay.sh start --mock   # no hardware
```

or run the equivalent command by hand:

```bash
cd /home/robot/yx/project/calib/calibration_replay
/home/robot/miniconda3/envs/fastapi/bin/python -m calibration_replay \
  --network-interface enp86s0 \
  --hand-eye-3d-project /home/robot/yx/project/calib/hand_eye_3D \
  --host 127.0.0.1 \
  --port 18004 \
  --data-root /home/robot/yx/project/calib/calibration_replay_data \
  --base-url-2d http://127.0.0.1:8131 \
  --base-url-3d http://127.0.0.1:8132
```

`enp86s0` is the H2 interface on this machine. Binding to
`127.0.0.1` is the conservative default; use a LAN address only on a trusted
operator network.

The isolated bridge dynamically loads `H2ArmController`, H2 joint reading, and
URDF limits from `--hand-eye-3d-project`. Capture services are HTTP-only targets
and preflight rejects target arm status that advertises arm control as enabled,
armed, engaged, or available.

## Operator flow and safety behavior

1. Select/create a plan. Record exactly one `home`; add enabled transit/sample
   nodes. Imported plans intentionally remain drafts until home is recorded.
2. Save and validate. Validation checks finite seven-joint vectors, unique IDs,
   positive motion/stability settings, H2 URDF limits when available, and maximum
   adjacent delta.
3. Engage, use cooperative guide to place the arm at `home`, then catch/hold.
   Run is refused when the current pose differs from `home` by more than the
   configured start delta (default `0.15 rad`), preventing an unplanned move
   from an arbitrary pose into the saved route.
4. Optionally name the run (`[A-Za-z0-9._-]`, default `<plan-slug>_<local time>`)
   and, for 2D, set the robot's camera serial. Run. The route is
   `home → W1…WN → home`: the return is a single segment straight from `WN` to
   home, and validation rejects the plan up front (`return leg … exceeds …`)
   when that jump is larger than `max_adjacent_delta_rad`, so append transit
   nodes after the last sample (the UI's auto-transit button does this too).
   Enabled sample nodes capture only on the forward leg.
5. Pause requests take effect after the current node. Immediate stop aborts the
   route and rigidly holds.
6. Disarm only after the run is complete/stopped and while physically supporting
   the arm.

Segments use quintic interpolation at 50 Hz. Duration accounts for quintic peak
velocity (`1.875`) and acceleration (`10/√3`), configured limits, and minimum
duration. Arrival is judged on the arm's own state, like IK_replay's reach
service: first the rate-limited controller must have delivered the final command
(`desired ≈ cmd`, bounded by `command_settle_s`), then the *measured* state must
stay still for `window_s` — encoder velocity (`max_velocity_rad_s`), in-window
encoder drift (`max_range_rad`) and torso IMU angular rate (`max_gyro_rad_s`,
only when lowstate carries an IMU) all under threshold, with fresh data. The
planned joint vector is only a reference: the certificate reports
`measured_q_rad`, `residual_max_rad` and `residual_within_reference`
(`max_error_rad`), and a large residual is logged but never blocks the capture.
The capture services record live joint readings themselves. The resulting
JSON stability certificate is sent with each capture. Capture IDs are
deterministic from run ID and waypoint ID so retries carry the same idempotency
key.

For 2D, preflight creates/selects the empty run session first, optionally selects
and verifies the configured camera serial, then calls
`POST /api/checkerboard/detect`. A missing board is informative; the final
`require_corners` check remains atomic inside `POST /api/capture`.

## Import supplied 2D sessions

The UI button or this command imports solver inliers as capture samples. Rejected
samples remain enabled `transit` nodes because they are known, physically reached
poses that preserve the original safe route, but they never trigger capture.
Source sessions are read only.

```bash
cd /home/robot/yx/project/calib/calibration_replay
/home/robot/miniconda3/envs/fastapi/bin/python scripts/import_2d_sessions.py \
  --data-root /home/robot/yx/project/calib/calibration_replay_data \
  --base-url-2d http://127.0.0.1:8131
```

Sources:

- head: `/home/robot/yx/project/calib/hand_eye_2D/handeye_data/20260902_170106`
- waist: `/home/robot/yx/project/calib/hand_eye_2D/handeye_data/20260902_173157`

This mapping is explicit; it is not inferred from camera model names. The
original session camera serial is copied into each imported draft as an editable
default.

## Import a hand_eye_3D episode directory

The UI button "导入 3D episode 目录" (or `POST /api/import/session` with
`target: "hand_eye_3D"`) reads `episode_*/data.json` produced by the 7012
capture page and turns every `info.measured_q_rad` into an enabled `sample`
node. Passing `result_path` to a solved `handeye3d_result.json` turns episodes
absent from its `pose_ids` into `transit` nodes. Episodes recorded with another
arm are rejected. The plan stays a draft until `home` is recorded; if two
consecutive episodes differ by more than `max_adjacent_delta_rad`, insert
transit nodes between them before recording `home`.

Step 4 of the page previews the full route in the browser (Three.js, URDF from
`--hand-eye-3d-project`, STL meshes from `--robot-mesh-dir`, defaulting to the
IK_replay H2 assets). `GET /api/plans/{id}/preview` returns the interpolated
frames plus the frame index of every route stop. Exporting to IK_replay
(`data/sequences/*.json`) is still available as an option for the 18002 bench.

## Main API

- Plans: `GET/POST /api/plans`, `GET/PUT/DELETE /api/plans/{id}`
- Nodes: add/record/patch/delete and `POST .../nodes/reorder`
- Validation/export: `POST /api/plans/{id}/validate|export`
- Import: `POST /api/import/default-sessions`, `POST /api/import/session`
- Control: `engage`, `guide`, `catch`, `run/{plan_id}`, `pause`, `resume`,
  `stop`, and `disarm` under `/api/control/`
- Live state: `GET /api/status`, `GET /api/joints`

State machine values are `idle`, `preflight`, `armed`, `moving`, `settling`,
`capturing`, `paused`, `returning`, `completed`, `fault`, and `stopped`.
`POST /api/control/run/{plan_id}` accepts `{"run_id": "optional-safe-label"}`;
omitting it generates `<plan-slug>_<YYYYmmdd-HHMMSS>`. Every run owns
`<data-root>/runs/<left|right>/<run_id>/` (the arm is the first layer; a name
that already exists for that arm is refused): for
`hand_eye_3D` the engine passes that directory as `record_dir` to
`POST /api/record/episode`, so the 8132 capture service writes
`episode_0000…` there instead of its default `--record-task-dir`, and
`run.json` (plan snapshot, certificates, capture results) is written beside
them when the run ends. `GET /api/runs` lists the runs. For `hand_eye_2D_*`
the same directory is passed as `record_dir` to `POST /api/session/start`
together with the plan's `arm`, so 8131 writes `left/`, `right/`, `joints/`,
`session_meta.json` and `camera_intrinsics.json` there; preflight refuses to
run when 8131 echoes a different `arm` or `save_path` (legacy backend). Solve
that session with 8131 `POST /api/solve {"session": "<absolute run dir>"}`.
Point the hand_eye_3D solver at a run directory with
`--teleop-task-dir <data-root>/runs/<arm>/<run_id>` (or import it from the 7012 page).
The local operator UI is Chinese and exposes both the run name and the per-plan
2D camera serial.

## Tests

```bash
cd /home/robot/yx/project/calib/calibration_replay
. .venv/bin/activate
pytest -q
```
