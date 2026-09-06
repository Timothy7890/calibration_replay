/* H2 标定轨迹复现 —— Vue 3 前端（本地 vendor，无 CDN、无构建步骤） */
import { createViewer } from "/static/viewer.js";

const { createApp, ref, computed, onMounted, nextTick } = Vue;

const stateName = {
  idle: "空闲", preflight: "预检中", armed: "已接管", moving: "前向运动", settling: "稳定确认",
  capturing: "采集中", paused: "已暂停", returning: "安全返回", completed: "已完成", fault: "故障", stopped: "已停止",
};
const targetName = { hand_eye_3D: "3D 手眼", hand_eye_2D_head: "2D 头相机", hand_eye_2D_waist: "2D 腰相机" };
const armName = { left: "左臂", right: "右臂" };
const otherArm = (a) => (a === "left" ? "right" : "left");
const RUNNING = new Set(["preflight", "moving", "settling", "capturing", "paused", "returning"]);

async function api(path, options = {}) {
  const r = await fetch(path, { headers: { "Content-Type": "application/json" }, ...options });
  const d = await r.json().catch(() => ({}));
  if (!r.ok) {
    const detail = d.detail ?? d;
    throw new Error(zh(Array.isArray(detail) ? detail.join("\n") : typeof detail === "string" ? detail : JSON.stringify(detail)));
  }
  return d;
}
/** 把后端常见英文校验信息翻成中文（未匹配的原样显示） */
function zh(msg) {
  return String(msg)
    .replace(/exactly one enabled home is required/g, "还没有原点：先在第 2 步把手臂摆到起点并「录为原点」")
    .replace(/at least one enabled transit or sample node is required/g, "至少需要一个启用的过渡点或采样点")
    .replace(/adjacent nodes (\S+)->(\S+) delta ([\d.]+) exceeds ([\d.]+)/g, "相邻节点 $1 → $2 差值 $3 rad 超过 $4，中间要补过渡点")
    .replace(/return leg (\S+)->home (\S+) delta ([\d.]+) exceeds ([\d.]+); append transit nodes after the last node/g, "回程过渡不足：末点 $1 → 原点 差值 $3 rad 超过 $4，点「自动补过渡点」在末尾生成回程过渡帧")
    .replace(/capture service ignored record_dir and wrote to (\S+); restart the hand_eye_3D backend \(8132\).*/g, "8132 采集端是旧版本，忽略了运行目录并把数据写到了 $1；请重启 hand_eye_3D 后端（./start.sh）后重跑")
    .replace(/run name already used: .*?already exists at (\S+)/g, "运行名已被用过（$1），换一个名字")
    .replace(/run_id must be 1-128 safe characters.*/g, "运行名只能用英文字母、数字、. _ -（1～128 个字符）")
    .replace(/node (\S+) joint (\d+)=([-\d.]+) outside \[([-\d.]+), ([-\d.]+)\]/g, "节点 $1 第 $2 关节 $3 超出限位 [$4, $5]")
    .replace(/cannot engage during a run/g, "运行中不能重复接管")
    .replace(/plan is for the (left|right) arm but the (left|right) arm is engaged; disarm and engage the plan's arm/g, (m, a, b) => `计划是${armName[a]}，但当前接管的是${armName[b]}；先解除接管，再按计划接管${armName[a]}`)
    .replace(/(left|right) arm is engaged; disarm before switching to (left|right)/g, (m, a, b) => `${armName[a]}正在接管中，切换到${armName[b]}前先解除接管`)
    .replace(/capture service is recording the (left|right) arm but this plan is for the (left|right) arm; restart it with --arm (left|right)/g, (m, a, b, c) => `8132 采集端当前记录的是${armName[a]}，计划是${armName[b]}；请用 --arm ${c} 重启采集端`)
    .replace(/was recorded with the (left|right) arm but earlier episodes use the (left|right) arm; one plan drives one arm/g, (m, a, b) => `混有${armName[a]}和${armName[b]}的 episode，一个计划只能驱动一条臂`)
    .replace(/arm is not engaged/g, "手臂尚未接管");
}
const post = (path, value) => api(path, { method: "POST", body: JSON.stringify(value || {}) });

createApp({
  setup() {
    const plans = ref([]);
    const plan = ref(null);
    const capability = ref({ available: false });
    const newArm = ref("right");
    const status = ref({ state: "idle", message: "", progress: {}, captures: [], logs: [], arm: {} });
    const online = ref(false);
    const joints = ref({ names: [], q: [] });
    const validation = ref(null);
    const exportResult = ref(null);
    const toast = ref(null);

    // ---------- 页面内 3D 预览 ----------
    const viewerEl = ref(null);
    let viewer = null;
    const preview = ref(null);
    const previewLoading = ref(false);
    const previewError = ref("");
    const previewFrame = ref(0);
    const previewPlaying = ref(false);
    const previewSpeed = ref(2);
    const currentStopIndex = computed(() => {
      if (!preview.value) return -1;
      let idx = 0;
      preview.value.stops.forEach((s, i) => { if (s.frame_index <= previewFrame.value) idx = i; });
      return idx;
    });
    const currentStop = computed(() => preview.value?.stops[currentStopIndex.value] || null);

    const newName = ref("");
    const newTarget = ref("hand_eye_3D");
    const import3dDir = ref("/home/robot/yx/project/calib/hand_eye_3D/teleop_data/biaoding");
    const import3dResult = ref("");
    const nodeName = ref("");
    const manualQ = ref("");
    const manualRole = ref("transit");
    const exportDir = ref("/home/robot/yx/project/IK_replay");
    const runId = ref("");

    const arm = computed(() => status.value.arm || {});
    const running = computed(() => RUNNING.has(status.value.state));
    const home = computed(() => plan.value?.nodes.find((n) => n.enabled && n.role === "home") || null);
    const homeDelta = computed(() => {
      if (!home.value || !joints.value.q.length) return NaN;
      return Math.max(...home.value.q_rad.map((v, i) => Math.abs(v - joints.value.q[i])));
    });

    /** 每一行与上一启用节点的最大单关节差（按运行顺序：原点在前） */
    const deltas = computed(() => {
      if (!plan.value) return [];
      const nodes = plan.value.nodes;
      const out = new Array(nodes.length).fill(null);
      let prev = home.value ? home.value.q_rad : null;
      nodes.forEach((n, i) => {
        if (!n.enabled || n.role === "home") return;
        if (prev) out[i] = Math.max(...n.q_rad.map((v, k) => Math.abs(v - prev[k])));
        prev = n.q_rad;
      });
      return out;
    });
    /** 回程：最后一个启用节点 → 原点 的跳变（null = 没有原点或没有节点） */
    const returnDelta = computed(() => {
      if (!plan.value || !home.value) return null;
      const last = [...plan.value.nodes].reverse().find((n) => n.enabled && n.role !== "home");
      return last ? Math.max(...last.q_rad.map((v, k) => Math.abs(v - home.value.q_rad[k]))) : null;
    });
    const gaps = computed(() => {
      const limit = plan.value.motion.max_adjacent_delta_rad;
      const out = deltas.value.map((d, i) => ({ d, i })).filter((x) => x.d != null && x.d > limit);
      if (returnDelta.value != null && returnDelta.value > limit) out.push({ d: returnDelta.value, i: -1, ret: true });
      return out;
    });

    const steps = computed(() => {
      if (!plan.value) return [];
      const hasNodes = plan.value.nodes.some((n) => n.enabled && n.role !== "home");
      const st = status.value.state;
      return [
        { id: "s1", n: 1, title: "节点", status: !hasNodes ? "todo" : gaps.value.length ? "blocked" : "done" },
        { id: "s2", n: 2, title: "接管 & 原点", status: home.value ? "done" : "todo" },
        { id: "s3", n: 3, title: "校验", status: validation.value ? (validation.value.ok ? "done" : "blocked") : "todo" },
        { id: "s4", n: 4, title: "预览", status: preview.value ? "done" : "todo" },
        { id: "s5", n: 5, title: "运行", status: st === "completed" ? "done" : RUNNING.has(st) ? "blocked" : "todo" },
      ];
    });

    const logText = computed(() =>
      (status.value.logs || []).slice().reverse().map((x) => `${x.at}  [${stateName[x.state] || x.state}]  ${x.message}`).join("\n")
    );

    function say(text, kind = "ok", ms = 3500) {
      toast.value = { text, kind };
      clearTimeout(say._t);
      say._t = setTimeout(() => (toast.value = null), ms);
    }
    async function guard(fn) {
      try { return await fn(); } catch (e) { say(e.message, "err", 7000); }
    }

    const fmt = (v) => (Number.isFinite(v) ? Number(v).toFixed(4) : "—");
    const fmtQ = (q) => q.map((x) => Number(x).toFixed(4)).join(", ");
    const parseQ = (s) => s.split(/[,\s]+/).filter(Boolean).map(Number);
    const shortJoint = (n) => n.replace(/^right_/, "").replace(/_joint$/, "");
    const summarize = (r) => {
      if (!r) return "";
      const keys = ["ok", "episode", "capture_id", "error", "message"];
      return keys.filter((k) => r[k] !== undefined).map((k) => `${k}=${JSON.stringify(r[k])}`).join("  ") || JSON.stringify(r).slice(0, 120);
    };

    // ---------- 计划 ----------
    async function loadPlans(selectId) {
      const d = await api("/api/plans");
      plans.value = d.plans;
      const wanted = [selectId, plan.value?.id, plans.value[0]?.id].filter(Boolean);
      const id = wanted.find((x) => plans.value.some((p) => p.id === x));
      if (id) await loadPlan(id);
      else plan.value = null;
    }
    async function loadPlan(id) {
      plan.value = await api("/api/plans/" + id);
      validation.value = null;
      exportResult.value = null;
      preview.value = null; previewError.value = "";
      if (viewer) viewer.pause();
    }
    const savePlan = () => guard(async () => {
      const p = { ...plan.value, camera_serial: plan.value.camera_serial?.trim() || null };
      plan.value = await api("/api/plans/" + p.id, { method: "PUT", body: JSON.stringify(p) });
      await loadPlans(p.id);
      say("计划已保存");
    });
    const createPlan = () => guard(async () => {
      const p = await post("/api/plans", { name: newName.value, target: newTarget.value, arm: newArm.value });
      await loadPlans(p.id);
    });
    /** 复制成另一侧手臂的计划（左↔右对等，节点按矢状面镜像） */
    const mirrorPlan = () => guard(async () => {
      const target = armName[otherArm(plan.value.arm)];
      if (!confirm(`把「${plan.value.name}」镜像成${target}计划？会新建一份草稿，原计划不变。`)) return;
      const p = await post(`/api/plans/${plan.value.id}/mirror`);
      await loadPlans(p.id);
      say(`已生成${target}计划「${p.name}」。请接管${target}、按镜像原点摆位后重新校验。`, "ok", 7000);
    });
    /** 空计划才允许改臂：已有节点的关节值属于原来那条臂 */
    const setArm = (arm) => guard(async () => {
      if (plan.value.arm === arm) return;
      if (plan.value.nodes.length) {
        say(`计划里已有 ${plan.value.nodes.length} 个${armName[plan.value.arm]}节点，不能直接改臂；用「镜像为${armName[arm]}计划」生成一份。`, "warn", 6000);
        return;
      }
      plan.value.arm = arm;
      await savePlan();
    });
    const deletePlan = () => guard(async () => {
      if (!confirm(`删除计划「${plan.value.name}」？`)) return;
      await api("/api/plans/" + plan.value.id, { method: "DELETE" });
      plan.value = null;
      await loadPlans();
    });
    const import3D = () => guard(async () => {
      const p = await post("/api/import/session", {
        target: "hand_eye_3D", session_dir: import3dDir.value.trim(),
        result_path: import3dResult.value.trim() || null, name: newName.value || "Imported 3D",
      });
      await loadPlans(p.id);
      say(`已导入 ${p.nodes.length} 个 episode。下一步：补过渡点、接管手臂、录原点。`, "ok", 6000);
    });
    const importDefaults = () => guard(async () => {
      const d = await post("/api/import/default-sessions");
      say(`已创建 ${d.created.length} 个 2D 草稿计划`);
      await loadPlans(d.created[0]?.id);
    });

    // ---------- 节点 ----------
    const reload = () => loadPlan(plan.value.id);
    const nodeEdit = (id, key, value) => guard(async () => {
      plan.value = await api(`/api/plans/${plan.value.id}/nodes/${id}`, { method: "PATCH", body: JSON.stringify({ [key]: value }) });
    });
    const moveNode = (i, d) => guard(async () => {
      const ids = plan.value.nodes.map((n) => n.id);
      const j = i + d;
      if (j < 0 || j >= ids.length) return;
      [ids[i], ids[j]] = [ids[j], ids[i]];
      plan.value = await post(`/api/plans/${plan.value.id}/nodes/reorder`, { ids });
    });
    const removeNode = (id) => guard(async () => {
      plan.value = await api(`/api/plans/${plan.value.id}/nodes/${id}`, { method: "DELETE" });
    });
    const recordNode = (role) => guard(async () => {
      const label = { home: "原点", transit: "过渡点", sample: "采样点" }[role];
      await post(`/api/plans/${plan.value.id}/nodes/record`, { role, name: nodeName.value || label });
      nodeName.value = "";
      await reload();
      say(`已把当前姿态录为${label}`);
    });
    const manualNode = () => guard(async () => {
      const q = parseQ(manualQ.value);
      if (q.length !== 7 || q.some((x) => !Number.isFinite(x))) throw new Error("需要 7 个有限数值");
      await post(`/api/plans/${plan.value.id}/nodes`, { role: manualRole.value, name: nodeName.value || "手工姿态", q_rad: q });
      manualQ.value = ""; nodeName.value = "";
      await reload();
    });

    /** 在每个超限的相邻对之间按线性插值插入足够的过渡点（保持顺序） */
    const autoInsertTransits = () => guard(async () => {
      const limit = plan.value.motion.max_adjacent_delta_rad;
      const nodes = plan.value.nodes;
      const order = nodes.map((n) => n.id);
      let inserted = 0;
      let prev = home.value;
      for (const n of nodes) {
        if (!n.enabled || n.role === "home") continue;
        if (prev) {
          const d = Math.max(...n.q_rad.map((v, k) => Math.abs(v - prev.q_rad[k])));
          if (d > limit) {
            const count = Math.ceil(d / (limit * 0.9)) - 1;
            const at = order.indexOf(n.id);
            for (let k = 1; k <= count; k++) {
              const f = k / (count + 1);
              const q = n.q_rad.map((v, i) => prev.q_rad[i] + (v - prev.q_rad[i]) * f);
              const made = await post(`/api/plans/${plan.value.id}/nodes`, { role: "transit", name: `过渡 ${prev.name}→${n.name} ${k}/${count}`, q_rad: q });
              order.splice(at + k - 1, 0, made.id);
              inserted++;
            }
          }
        }
        prev = n;
      }
      // 回程：最后一个节点直接回原点，这一跳也要够小，不够就在末尾追加过渡点
      if (prev && home.value && prev.id !== home.value.id) {
        const d = Math.max(...home.value.q_rad.map((v, k) => Math.abs(v - prev.q_rad[k])));
        if (d > limit) {
          const count = Math.ceil(d / (limit * 0.9)) - 1;
          for (let k = 1; k <= count; k++) {
            const f = k / (count + 1);
            const q = home.value.q_rad.map((v, i) => prev.q_rad[i] + (v - prev.q_rad[i]) * f);
            const made = await post(`/api/plans/${plan.value.id}/nodes`, { role: "transit", name: `回程 ${prev.name}→原点 ${k}/${count}`, q_rad: q });
            order.push(made.id);
            inserted++;
          }
        }
      }
      if (inserted) plan.value = await post(`/api/plans/${plan.value.id}/nodes/reorder`, { ids: order });
      say(inserted ? `已插入 ${inserted} 个过渡点，请在导出预览里确认路径` : "没有需要补的间隙");
    });

    let viewerArm = null;
    async function ensureViewer() {
      if (viewer && viewerArm !== plan.value.arm) { viewer.dispose(); viewer = null; }
      if (viewer) return viewer;
      viewerArm = plan.value.arm;
      const cfg = await api(`/api/robot/preview-config?arm=${plan.value.arm}`);
      if (!cfg.available) throw new Error(`预览不可用：找不到 URDF 或 STL（${cfg.urdf_path} / ${cfg.mesh_dir}）`);
      viewer = createViewer(viewerEl.value);
      viewer.onFrame = (i) => { previewFrame.value = i; };
      viewer.onEnd = () => { previewPlaying.value = false; };
      await viewer.loadRobot(cfg);
      viewer.setSpeed(previewSpeed.value);
      return viewer;
    }
    const loadPreview = async () => {
      previewError.value = ""; previewLoading.value = true;
      try {
        const traj = await api(`/api/plans/${plan.value.id}/preview`);
        preview.value = traj;
        await nextTick();
        const v = await ensureViewer();
        v.setTrajectory(traj);
        previewPlaying.value = false;
        previewFrame.value = 0;
      } catch (e) {
        preview.value = null;
        previewError.value = e.message;
      } finally { previewLoading.value = false; }
    };
    const togglePlay = () => {
      if (!viewer) return;
      if (viewer.playing) { viewer.pause(); previewPlaying.value = false; }
      else { viewer.play(); previewPlaying.value = true; }
    };
    const scrub = (i) => { if (!viewer) return; viewer.pause(); previewPlaying.value = false; viewer.seek(Number(i)); };

    // ---------- 校验 / 导出 / 控制 ----------
    const validatePlan = () => guard(async () => {
      const v = await post(`/api/plans/${plan.value.id}/validate`);
      validation.value = { ok: v.ok, errors: (v.errors || []).map(zh) };
    });
    const exportPlan = () => guard(async () => {
      exportResult.value = await post(`/api/plans/${plan.value.id}/export`, { output_dir: exportDir.value });
      say("已导出，去 18002 离线轨迹回放里查看", "ok", 5000);
    });
    const act = (name) => guard(async () => {
      await post("/api/control/" + name, name === "engage" && plan.value ? { plan_id: plan.value.id } : {});
      await refreshStatus();
    });
    const run = () => guard(async () => {
      if (!confirm("确认开始运行？手臂会沿计划路线运动，请确保有人在机器人旁监护。")) return;
      const d = await post("/api/control/run/" + plan.value.id, { run_id: runId.value.trim() || null });
      runId.value = d.run_id;
      await refreshStatus();
    });

    async function refreshStatus() {
      try { status.value = await api("/api/status"); online.value = true; }
      catch { online.value = false; }
    }
    async function refreshJoints() {
      try {
        const q = plan.value ? `?arm=${plan.value.arm}` : "";
        const d = await api("/api/joints" + q);
        joints.value = { names: d.joint_names, q: d.q, arm: d.arm };
      } catch { /* 顶栏已显示离线 */ }
    }
    async function refreshCapability() {
      try { capability.value = await api("/api/capability"); } catch { capability.value = { available: false }; }
    }

    onMounted(() => {
      // ?plan=<id>&preview=1 直接打开某计划并生成预览（便于分享链接/自动化检查）
      const params = new URLSearchParams(location.search);
      guard(async () => {
        await loadPlans(params.get("plan") || undefined);
        if (params.get("preview") && plan.value) await loadPreview();
      });
      refreshStatus(); refreshJoints(); refreshCapability();
      setInterval(refreshStatus, 500);
      setInterval(refreshCapability, 5000);
      setInterval(refreshJoints, 250);
    });

    return {
      stateName, targetName, armName, otherArm, plans, plan, status, online, joints, validation, exportResult, toast,
      capability, newArm, mirrorPlan, setArm,
      newName, newTarget, import3dDir, import3dResult, nodeName, manualQ, manualRole, exportDir, runId,
      arm, running, home, homeDelta, deltas, gaps, steps, logText,
      fmt, fmtQ, parseQ, shortJoint, summarize,
      loadPlans, loadPlan, savePlan, createPlan, deletePlan, import3D, importDefaults,
      nodeEdit, moveNode, removeNode, recordNode, manualNode, autoInsertTransits, returnDelta,
      validatePlan, exportPlan, act, run,
      viewerEl, preview, previewLoading, previewError, previewFrame, previewPlaying, previewSpeed,
      currentStop, currentStopIndex, loadPreview, togglePlay, scrub,
      get viewer() { return viewer; },
    };
  },
}).mount("#app");
