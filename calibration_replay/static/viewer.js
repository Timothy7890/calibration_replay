/* 页面内 3D 轨迹预览：URDF + STL（本地 vendor three.js），浏览器端 FK 回放。 */
import * as THREE from "three";
import { OrbitControls } from "/static/vendor/OrbitControls.js";
import { STLLoader } from "/static/vendor/STLLoader.js";

const parseVec = (s, fb) => {
  if (!s) return fb;
  const p = s.trim().split(/\s+/).map(Number);
  return p.length === 3 && p.every(Number.isFinite) ? p : fb;
};
const parseOrigin = (el) => ({
  xyz: parseVec(el?.getAttribute("xyz"), [0, 0, 0]),
  rpy: parseVec(el?.getAttribute("rpy"), [0, 0, 0]),
});
const applyOrigin = (obj, o) => { obj.position.set(...o.xyz); obj.rotation.set(...o.rpy, "XYZ"); };

export function createViewer(container) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1119);
  scene.add(new THREE.HemisphereLight(0xeaf5ff, 0x263544, 2.4));
  const key = new THREE.DirectionalLight(0xffffff, 2.0); key.position.set(1.8, -1.4, 2.8); scene.add(key);
  const fill = new THREE.DirectionalLight(0x91c9ff, 0.7); fill.position.set(-1.5, 1.2, 1.5); scene.add(fill);
  const grid = new THREE.GridHelper(2.4, 24, 0x3b566d, 0x1c3040); grid.rotation.x = Math.PI / 2; scene.add(grid);
  scene.add(new THREE.AxesHelper(0.15));

  const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 30);
  camera.up.set(0, 0, 1);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  container.appendChild(renderer.domElement);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true; controls.dampingFactor = 0.08; controls.screenSpacePanning = true;

  const pathGroup = new THREE.Group(); scene.add(pathGroup);
  let fitTarget = new THREE.Vector3(0.15, -0.2, 0.25);
  let fitRadius = 0.6;

  const state = {
    config: null, robot: null, jointNodes: new Map(), links: new Map(),
    frames: [], jointNames: [], stops: [], duration: 0,
    frameIndex: 0, playing: false, speed: 1, startedAt: 0, startedFrame: 0,
    onFrame: null, onEnd: null,
  };

  function resize() {
    const w = Math.max(1, container.clientWidth), h = Math.max(1, container.clientHeight);
    renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(container);
  resize();
  resetView();

  function resetView() {
    // 从机器人右前上方看向轨迹中心
    const dir = new THREE.Vector3(1.0, -1.35, 0.75).normalize();
    camera.position.copy(fitTarget).addScaledVector(dir, fitRadius * 3.2);
    controls.target.copy(fitTarget);
    controls.update();
  }

  function animate(now) {
    if (state.playing && state.frames.length > 1) {
      const perFrame = (state.duration * 1000) / (state.frames.length - 1) / Math.max(0.1, state.speed);
      const next = state.startedFrame + Math.floor((now - state.startedAt) / Math.max(perFrame, 1));
      if (next >= state.frames.length) { seek(state.frames.length - 1); state.playing = false; state.onEnd?.(); }
      else if (next !== state.frameIndex) seek(next);
    }
    controls.update();
    renderer.render(scene, camera);
    if (!state.disposed) requestAnimationFrame(animate);
  }
  requestAnimationFrame(animate);

  /** 切换手臂时整体重建：释放 WebGL 资源并移除画布 */
  function dispose() {
    state.disposed = true;
    state.playing = false;
    controls.dispose();
    renderer.dispose();
    renderer.domElement.remove();
  }

  async function loadRobot(config) {
    if (state.robot) return;
    state.config = config;
    const res = await fetch(config.urdf_url, { cache: "no-store" });
    if (!res.ok) throw new Error(`URDF 读取失败 HTTP ${res.status}`);
    const xml = new DOMParser().parseFromString(await res.text(), "application/xml");
    if (xml.querySelector("parsererror")) throw new Error("URDF 解析失败");

    const armLinks = new Set(config.arm_links || []);
    const loader = new STLLoader();
    const tasks = [];
    const links = new Map(), byParent = new Map(), children = new Set();
    for (const linkEl of xml.querySelectorAll("link")) {
      const name = linkEl.getAttribute("name");
      const group = new THREE.Group(); group.name = name; links.set(name, group);
      for (const vis of linkEl.querySelectorAll("visual")) {
        const meshEl = vis.querySelector("geometry > mesh");
        if (!meshEl) continue;
        const vg = new THREE.Group(); applyOrigin(vg, parseOrigin(vis.querySelector("origin")));
        const scale = parseVec(meshEl.getAttribute("scale"), [1, 1, 1]);
        const file = (meshEl.getAttribute("filename") || "").replace(/^package:\/\/[^/]+\//, "");
        const isArm = armLinks.has(name);
        const material = new THREE.MeshStandardMaterial({
          color: isArm ? 0x4fc3f7 : 0x8a97a6, roughness: 0.6, metalness: 0.08,
          transparent: !isArm, opacity: isArm ? 1 : 0.55,
        });
        tasks.push(loader.loadAsync(config.mesh_base_url + file).then((geo) => {
          geo.computeVertexNormals();
          const mesh = new THREE.Mesh(geo, material); mesh.scale.set(...scale); vg.add(mesh);
        }).catch((e) => console.warn("mesh 加载失败", file, e)));
        group.add(vg);
      }
    }
    for (const jEl of xml.querySelectorAll("joint")) {
      const parent = jEl.querySelector("parent")?.getAttribute("link");
      const child = jEl.querySelector("child")?.getAttribute("link");
      if (!parent || !child) continue;
      const joint = {
        name: jEl.getAttribute("name"), type: jEl.getAttribute("type") || "fixed", parent, child,
        axis: new THREE.Vector3(...parseVec(jEl.querySelector("axis")?.getAttribute("xyz"), [0, 0, 1])).normalize(),
        origin: parseOrigin(jEl.querySelector("origin")),
      };
      children.add(child);
      if (!byParent.has(parent)) byParent.set(parent, []);
      byParent.get(parent).push(joint);
    }
    const rootName = [...links.keys()].find((n) => !children.has(n));
    if (!rootName) throw new Error("URDF 没有根 link");
    const root = new THREE.Group(); root.add(links.get(rootName));
    const attach = (parentName) => {
      for (const j of byParent.get(parentName) || []) {
        const origin = new THREE.Group(); applyOrigin(origin, j.origin);
        const motion = new THREE.Group(); origin.add(motion); motion.add(links.get(j.child));
        links.get(parentName).add(origin);
        state.jointNodes.set(j.name, { ...j, motion });
        attach(j.child);
      }
    };
    attach(rootName);
    scene.add(root);
    state.robot = root; state.links = links;
    await Promise.all(tasks);
    setJoints({});
  }

  function setJoints(values) {
    const all = { ...(state.config?.initial_joints || {}), ...values };
    for (const [name, j] of state.jointNodes) {
      const v = Number(all[name] || 0);
      j.motion.position.set(0, 0, 0); j.motion.quaternion.identity();
      if (j.type === "revolute" || j.type === "continuous") j.motion.quaternion.setFromAxisAngle(j.axis, v);
      else if (j.type === "prismatic") j.motion.position.copy(j.axis).multiplyScalar(v);
    }
  }

  function frameJoints(i) {
    const out = {};
    state.jointNames.forEach((n, k) => { out[n] = state.frames[i][k]; });
    return out;
  }

  function seek(i) {
    if (!state.frames.length) return;
    state.frameIndex = Math.max(0, Math.min(state.frames.length - 1, i));
    setJoints(frameJoints(state.frameIndex));
    state.onFrame?.(state.frameIndex);
  }

  /** 画出腕部轨迹线和各停点标记 */
  function buildPath() {
    pathGroup.clear();
    const wrist = state.links.get(state.config?.wrist_link);
    if (!wrist || state.frames.length < 2) return;
    const pts = [];
    const stopPts = new Map();
    const p = new THREE.Vector3();
    const step = Math.max(1, Math.floor(state.frames.length / 1500));
    const stopIdx = new Set(state.stops.map((s) => s.frame_index));
    for (let i = 0; i < state.frames.length; i += step) {
      setJoints(frameJoints(i)); state.robot.updateMatrixWorld(true);
      wrist.getWorldPosition(p); pts.push(p.clone());
    }
    for (const s of state.stops) {
      setJoints(frameJoints(s.frame_index)); state.robot.updateMatrixWorld(true);
      wrist.getWorldPosition(p); stopPts.set(s.frame_index, p.clone());
    }
    pathGroup.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: 0xf3b23c })));
    // 以轨迹（加上躯干中心）为中心自动取景
    const box = new THREE.Box3().setFromPoints(pts);
    box.expandByPoint(new THREE.Vector3(0, 0, 0.35));
    box.getCenter(fitTarget);
    fitRadius = Math.max(0.35, box.getSize(new THREE.Vector3()).length() / 2);
    resetView();
    const sphere = new THREE.SphereGeometry(0.012, 12, 12);
    for (const s of state.stops) {
      if (s.leg === "reverse") continue;
      const color = s.role === "home" ? 0x4c8dff : s.role === "sample" ? 0x43cf8a : 0x9aa7b4;
      const m = new THREE.Mesh(sphere, new THREE.MeshBasicMaterial({ color }));
      m.position.copy(stopPts.get(s.frame_index)); pathGroup.add(m);
    }
    void stopIdx;
  }

  function setTrajectory(traj) {
    state.frames = traj.frames; state.jointNames = traj.joint_names;
    state.stops = traj.stops || []; state.duration = traj.duration_s || 1;
    state.playing = false;
    buildPath();
    seek(0);
  }

  function play() {
    if (state.frames.length < 2) return;
    if (state.frameIndex >= state.frames.length - 1) state.frameIndex = 0;
    state.startedAt = performance.now(); state.startedFrame = state.frameIndex; state.playing = true;
  }
  function pause() { state.playing = false; }
  function setSpeed(v) { state.speed = v; if (state.playing) { state.startedAt = performance.now(); state.startedFrame = state.frameIndex; } }

  return {
    loadRobot, setTrajectory, seek, play, pause, setSpeed, resetView, setJoints, dispose,
    get frameIndex() { return state.frameIndex; }, get playing() { return state.playing; },
    set onFrame(fn) { state.onFrame = fn; }, set onEnd(fn) { state.onEnd = fn; },
  };
}
