#!/usr/bin/env bash
# 18004 标定轨迹回放服务启动器（左右臂由计划选择，人工监护）。
#
# 启动：./replay.sh            （等价 ./replay.sh start）
# 状态：./replay.sh status
# 停止：./replay.sh stop
# 日志：./replay.sh log
# 联调：./replay.sh start --mock   （无硬件）
#
# 启动后只读 rt/lowstate；页面点「接管」之后才发布 rt/arm_sdk。
# 运行期间 hand_eye_3D 采集端必须用 ./start.sh --no-arm 启动，否则两边抢 rt/arm_sdk。
# stop 只关闭由本脚本启动并记录 PID 的进程。

set -u
set -o pipefail
cd "$(dirname "$0")"

PYTHON=${PYTHON:-/home/robot/miniconda3/envs/fastapi/bin/python}
PORT=${PORT:-18004}
HOST=${HOST:-0.0.0.0}   # 0.0.0.0 允许局域网访问；只想本机访问改 127.0.0.1
NETWORK_INTERFACE=${NETWORK_INTERFACE:-enp86s0}
DATA_ROOT=${DATA_ROOT:-/home/robot/yx/project/calib/calibration_replay_data}
HAND_EYE_3D_PROJECT=${HAND_EYE_3D_PROJECT:-/home/robot/yx/project/calib/hand_eye_3D}
BASE_URL_3D=${BASE_URL_3D:-http://127.0.0.1:8132}
BASE_URL_2D=${BASE_URL_2D:-http://127.0.0.1:8131}

LOG_DIR=logs/service
# 按端口区分 PID/日志，避免同时跑 mock 联调实例时互相覆盖
LOG_FILE="$LOG_DIR/replay-$PORT.log"
PID_FILE="$LOG_DIR/replay-$PORT.pid"
TOKEN="calibration_replay.*--port $PORT"
mkdir -p "$LOG_DIR"

port_in_use() {
    ss -ltn 2>/dev/null | awk -v port="$1" \
        '$4 ~ (":" port "$") {found=1} END {exit !found}'
}

owned_pid() {
    local pid
    [[ -f "$PID_FILE" ]] || return 1
    pid=$(<"$PID_FILE")
    [[ "$pid" =~ ^[0-9]+$ ]] || return 1
    kill -0 "$pid" 2>/dev/null || return 1
    [[ -r "/proc/$pid/cmdline" ]] || return 1
    tr '\0' ' ' <"/proc/$pid/cmdline" | grep -q -- "$TOKEN" || return 1
    echo "$pid"
}

capture_arm_control_enabled() {
    # 8132（3D）与 8131（2D）任一开启了手臂控制都会和本服务抢 rt/arm_sdk
    local url
    for url in "$BASE_URL_3D" "$BASE_URL_2D"; do
        if curl -sf --max-time 2 "$url/api/arm/status" 2>/dev/null |
            grep -Eq '"(enabled|armed|publishing|available)": *true'; then
            CONFLICT_URL="$url"
            return 0
        fi
    done
    return 1
}

do_status() {
    local pid
    if pid=$(owned_pid); then
        echo "[回放] 运行中 pid=$pid  页面 http://127.0.0.1:$PORT/（监听 $HOST）"
    elif port_in_use "$PORT"; then
        echo "[回放] 端口 $PORT 有服务在监听，但不是本脚本启动的"
    else
        echo "[回放] 未运行"
    fi
}

do_stop() {
    local pid
    if pid=$(owned_pid); then
        kill "$pid"
        for _ in {1..30}; do kill -0 "$pid" 2>/dev/null || break; sleep 0.2; done
        kill -0 "$pid" 2>/dev/null && kill -9 "$pid"
        echo "[回放] 已停止 pid=$pid"
    else
        echo "[回放] 没有由本脚本启动的服务"
    fi
    rm -f "$PID_FILE"
}

do_start() {
    local mode_args=() mode="H2 真机（网卡 $NETWORK_INTERFACE）" pid
    if pid=$(owned_pid); then
        echo "[回放] 已在运行 pid=$pid  页面 http://127.0.0.1:$PORT/"
        return 0
    fi
    if port_in_use "$PORT"; then
        echo "[回放] 端口 $PORT 已被占用，未启动" >&2
        return 1
    fi
    if [[ ! -x "$PYTHON" ]]; then
        echo "[回放] 找不到 Python: $PYTHON" >&2
        return 1
    fi
    if [[ "${1:-}" == "--mock" ]]; then
        mode_args=(--mock)
        mode="mock 联调（无硬件）"
        # --mock --capture-http：采集仍走 HTTP（对方也 mock），全链路联调
        if [[ "${2:-}" == "--capture-http" ]]; then
            mode_args+=(--mock-capture-http)
            mode="mock 联调（无硬件，采集走 HTTP）"
        fi
    else
        mode_args=(--network-interface "$NETWORK_INTERFACE"
                   --hand-eye-3d-project "$HAND_EYE_3D_PROJECT")
        CONFLICT_URL=""
        if capture_arm_control_enabled; then
            echo "[回放] 采集端 $CONFLICT_URL 启用了手臂控制，会和本服务抢 rt/arm_sdk。" >&2
            echo "       请不带 --arm-control 重启采集端（3D: hand_eye_3D/start.sh --no-arm）。" >&2
            return 1
        fi
    fi

    nohup "$PYTHON" -m calibration_replay "${mode_args[@]}" \
        --host "$HOST" --port "$PORT" \
        --data-root "$DATA_ROOT" \
        --base-url-3d "$BASE_URL_3D" \
        --base-url-2d "$BASE_URL_2D" \
        >>"$LOG_FILE" 2>&1 &
    pid=$!
    echo "$pid" >"$PID_FILE"
    disown

    sleep 2
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "[回放] 启动失败，最近日志：" >&2
        tail -n 20 "$LOG_FILE" >&2
        rm -f "$PID_FILE"
        return 1
    fi
    echo "[回放] 已启动 pid=$pid  模式：$mode"
    local ip
    ip=$(ip route get 8.8.8.8 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i=="src") {print $(i+1); exit}}')
    echo "[回放] 页面: http://${ip:-127.0.0.1}:$PORT/   （本机 http://127.0.0.1:$PORT/）"
    echo "[回放] 计划目录: $DATA_ROOT"
    echo "[回放] 日志: $LOG_FILE"
}

case "${1:-start}" in
    start)  shift; do_start "$@" ;;
    stop)   do_stop ;;
    status) do_status ;;
    log)    exec tail -f "$LOG_FILE" ;;
    *)
        echo "用法: $0 [start [--mock [--capture-http]]|status|stop|log]"
        exit 1 ;;
esac
