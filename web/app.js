import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const els = {
  taskText: document.getElementById("taskText"),
  taskType: document.getElementById("taskType"),
  retries: document.getElementById("retries"),
  localFirst: document.getElementById("localFirst"),
  model: document.getElementById("model"),
  status: document.getElementById("status"),
  randomTaskBtn: document.getElementById("randomTaskBtn"),
  runBtn: document.getElementById("runBtn"),
  playBtn: document.getElementById("playBtn"),
  pauseBtn: document.getElementById("pauseBtn"),
  resetBtn: document.getElementById("resetBtn"),
  frameSlider: document.getElementById("frameSlider"),
  stepInfo: document.getElementById("stepInfo"),
  positionInfo: document.getElementById("positionInfo"),
  gripperInfo: document.getElementById("gripperInfo"),
  steps: document.getElementById("steps"),
  jsonBox: document.getElementById("jsonBox"),
  viewport: document.getElementById("armViewport"),
};

const ARM = { baseHeight: 260, upper: 280, forearm: 240 };
const FRAME_INTERVAL_MS = 60;
const HOME = { joints: { j1: 0, j2: 90, j3: 0 }, gripper: { state: "open", width: 100, force: 0 }, step: 0, action: "init", comment: "3-DOF HOME" };
let run = null;
let motionFrames = [];
let frameIndex = 0;
let timer = null;
let currentFrame = frameFromApi(HOME);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);
const camera = new THREE.PerspectiveCamera(48, 1, 1, 5000);
camera.up.set(0, 0, 1);
camera.position.set(720, -980, 560);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
els.viewport.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(260, 0, 190);
controls.enableDamping = true;
controls.screenSpacePanning = false;
controls.minDistance = 260;
controls.maxDistance = 1800;
controls.minPolarAngle = 0.1;
controls.maxPolarAngle = Math.PI * 0.48;

const mat = {
  arm: new THREE.MeshStandardMaterial({ color: 0xf4f1ea, roughness: 0.42, metalness: 0.16 }),
  joint: new THREE.MeshStandardMaterial({ color: 0x9b9b9b, roughness: 0.38, metalness: 0.34 }),
  base: new THREE.MeshStandardMaterial({ color: 0xf4f1ea, roughness: 0.5, metalness: 0.2 }),
  tool: new THREE.MeshStandardMaterial({ color: 0xd71921, roughness: 0.34, metalness: 0.24 }),
};
const armGroup = new THREE.Group();
scene.add(armGroup);
let traceLine = null;

initScene();
motionFrames = [frameFromApi(HOME)];
renderFrame();

function initScene() {
  scene.add(new THREE.AmbientLight(0xffffff, 0.54));
  const key = new THREE.DirectionalLight(0xffffff, 1.16);
  key.position.set(500, -700, 900);
  scene.add(key);
  const rim = new THREE.DirectionalLight(0xd71921, 0.62);
  rim.position.set(-800, 500, 540);
  scene.add(rim);
  const grid = new THREE.GridHelper(1500, 15, 0x565656, 0x242424);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);
  scene.add(new THREE.AxesHelper(600));
}

function setStatus(text, kind = "") {
  els.status.textContent = text;
  els.status.className = `inlineStatus ${kind}`;
}

async function post(url, body) {
  const response = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || "request failed");
  return data;
}

function payload() {
  return { task_text: els.taskText.value.trim(), type: els.taskType.value, retries: Number(els.retries.value || 3), model: els.model.value.trim() || null, local_first: els.localFirst.checked };
}

function plannerStatus(metadata = {}) {
  const source = metadata.planner_source;
  if (source === "llm") return { text: "[ 完成 ] LLM 语义解析 + 本地轨迹规划", kind: "ok" };
  if (source === "fallback_local") return { text: `[ 回退 ] LLM 调用失败，已使用本地规则：${metadata.llm_error || "未知错误"}`, kind: "err" };
  if (source === "local_first") return { text: "[ 完成 ] 本地优先规划，未调用 LLM", kind: "ok" };
  if (source === "llm_normalized_local") return { text: "[ 完成 ] LLM 归一化后由本地规则规划", kind: "ok" };
  return { text: "[ 完成 ] 3-DOF 轨迹已生成", kind: "ok" };
}

function frameFromApi(apiFrame) {
  const joints = apiFrame.joints || HOME.joints;
  const normalized = {
    j1: finiteNumber(joints.j1, HOME.joints.j1),
    j2: finiteNumber(joints.j2, HOME.joints.j2),
    j3: finiteNumber(joints.j3, HOME.joints.j3),
  };
  const points = hasCompletePoints(apiFrame.points) ? pointsFromApi(apiFrame.points) : forward(normalized.j1, normalized.j2, normalized.j3);
  return { ...apiFrame, joints: normalized, points, gripper: apiFrame.gripper || HOME.gripper };
}

function finiteNumber(value, fallback) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function hasCompletePoints(points) {
  return Boolean(points?.base && points?.shoulder && points?.elbow && points?.tool);
}

function pointsFromApi(points) {
  return {
    base: vectorFromPoint(points.base),
    shoulder: vectorFromPoint(points.shoulder),
    elbow: vectorFromPoint(points.elbow),
    tool: vectorFromPoint(points.tool),
  };
}

function vectorFromPoint(point) {
  return new THREE.Vector3(Number(point.x), Number(point.y), Number(point.z));
}

function forward(j1Deg, j2Deg, j3Deg) {
  const yaw = THREE.MathUtils.degToRad(j1Deg);
  const shoulderPitch = THREE.MathUtils.degToRad(j2Deg);
  const forearmPitch = THREE.MathUtils.degToRad(j2Deg + j3Deg);
  const base = new THREE.Vector3(0, 0, 0);
  const shoulder = new THREE.Vector3(0, 0, ARM.baseHeight);
  const elbowRadial = ARM.upper * Math.cos(shoulderPitch);
  const elbowZ = ARM.baseHeight + ARM.upper * Math.sin(shoulderPitch);
  const toolRadial = elbowRadial + ARM.forearm * Math.cos(forearmPitch);
  const toolZ = elbowZ + ARM.forearm * Math.sin(forearmPitch);
  const elbow = new THREE.Vector3(elbowRadial * Math.cos(yaw), elbowRadial * Math.sin(yaw), elbowZ);
  const tool = new THREE.Vector3(toolRadial * Math.cos(yaw), toolRadial * Math.sin(yaw), toolZ);
  return { base, shoulder, elbow, tool };
}

function buildMotionFrames(apiFrames) {
  const keys = apiFrames.map(frameFromApi);
  const output = [keys[0] || frameFromApi(HOME)];
  for (let i = 1; i < keys.length; i += 1) {
    const from = output[output.length - 1];
    const to = keys[i];
    const elapsedDelta = Number(to.elapsed_ms || 0) - Number(from.elapsed_ms || 0);
    const segments = Math.max(1, Math.round(Math.max(elapsedDelta, FRAME_INTERVAL_MS) / FRAME_INTERVAL_MS));
    for (let step = 1; step <= segments; step += 1) {
      const t = ease(step / segments);
      const joints = {
        j1: normalizeYaw(from.joints.j1 + yawDelta(from.joints.j1, to.joints.j1) * t),
        j2: lerp(from.joints.j2, to.joints.j2, t),
        j3: lerp(from.joints.j3, to.joints.j3, t),
      };
      output.push({ ...to, joints, points: forward(joints.j1, joints.j2, joints.j3), gripper: interpolateGripper(from.gripper, to.gripper, t) });
    }
  }
  return output;
}

function renderFrame() {
  const frame = motionFrames[frameIndex] || currentFrame;
  currentFrame = frame;
  clearGroup(armGroup);
  addBase();
  addLink(frame.points.base, frame.points.shoulder);
  addLink(frame.points.shoulder, frame.points.elbow);
  addLink(frame.points.elbow, frame.points.tool);
  addJoint(frame.points.base, 32);
  addJoint(frame.points.shoulder, 25);
  addJoint(frame.points.elbow, 22);
  addGripper(frame.points.tool, frame.joints.j1, frame.gripper);
  updateTrace();
  updateReadout(frame);
}

function addBase() {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(92, 112, 42, 48), mat.base);
  mesh.rotation.x = Math.PI / 2;
  mesh.position.z = 21;
  armGroup.add(mesh);
}

function addJoint(position, radius) {
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(radius, 32, 18), mat.joint);
  mesh.position.copy(position);
  armGroup.add(mesh);
}

function addLink(start, end) {
  const direction = new THREE.Vector3().subVectors(end, start);
  const length = direction.length();
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(18, 18, length, 24), mat.arm);
  mesh.position.copy(start).add(end).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
  armGroup.add(mesh);
}

function addGripper(position, yawDeg, gripper) {
  const width = Math.max(20, Math.min(90, Number(gripper.width) || 0));
  const material = gripper.state === "close" ? mat.tool : mat.joint;
  const group = new THREE.Group();
  group.position.copy(position);
  group.rotation.z = THREE.MathUtils.degToRad(yawDeg) + Math.PI / 2;
  const palm = new THREE.Mesh(new THREE.BoxGeometry(70, 18, 18), material);
  palm.position.set(0, 0, -24);
  group.add(palm);
  [-1, 1].forEach((side) => {
    const finger = new THREE.Mesh(new THREE.BoxGeometry(14, 18, 66), material);
    finger.position.set(side * (width / 2), 0, -68);
    group.add(finger);
  });
  armGroup.add(group);
}

function updateTrace() {
  if (traceLine) {
    traceLine.geometry.dispose();
    traceLine.material.dispose();
    scene.remove(traceLine);
    traceLine = null;
  }
  const points = motionFrames.slice(0, frameIndex + 1).map((frame) => frame.points.tool);
  if (points.length < 2) return;
  traceLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), new THREE.LineBasicMaterial({ color: 0xd71921, transparent: true, opacity: 0.9 }));
  scene.add(traceLine);
}

function updateReadout(frame) {
  els.frameSlider.max = String(Math.max(motionFrames.length - 1, 0));
  els.frameSlider.value = String(frameIndex);
  els.stepInfo.textContent = `步骤 ${frame.step} / ${actionLabel(frame.action)}`;
  els.positionInfo.textContent = `J1=${frame.joints.j1.toFixed(1)}°, J2=${frame.joints.j2.toFixed(1)}°, J3=${frame.joints.j3.toFixed(1)}°`;
  els.gripperInfo.textContent = `${gripperLabel(frame.gripper.state)}, 开口=${Math.round(frame.gripper.width)}mm`;
  renderSteps(frame.step);
}

function actionLabel(action) {
  return { init: "初始化", move_joints: "关节运动", gripper: "夹爪", wait: "等待" }[action] || action;
}

function gripperLabel(state) {
  return state === "close" ? "闭合" : "打开";
}

function renderSteps(activeStep) {
  els.steps.innerHTML = "";
  if (!run) return;
  run.task.steps.forEach((step) => {
    const item = document.createElement("div");
    item.className = `step ${step.step === activeStep ? "active" : ""}`;
    item.innerHTML = `<div class="stepTop"><span>步骤 ${step.step}</span><span>${actionLabel(step.action)}</span></div><div>${step.comment || ""}</div><div class="small">${JSON.stringify(step.params)}</div>`;
    els.steps.appendChild(item);
  });
}

function resetResult(text = "[ 空 ] 尚未生成规划结果") {
  stop();
  run = null;
  motionFrames = [currentFrame];
  frameIndex = 0;
  els.jsonBox.textContent = text;
  els.steps.innerHTML = "";
  renderFrame();
}

async function runWorkflow() {
  const request = payload();
  if (!request.task_text) {
    resetResult();
    setStatus("[ 错误 ] 请先输入任务，或点击随机生成", "err");
    return;
  }
  resetResult("[ 运行中 ] 正在生成新的规划结果");
  setStatus(`[ 运行中 ] ${request.task_text}`, "running");
  try {
    run = await post("/api/workflow", request);
    els.taskText.value = run.task_text;
    els.jsonBox.textContent = JSON.stringify(run.task, null, 2);
    motionFrames = buildMotionFrames(run.frames);
    frameIndex = 0;
    renderFrame();
    const status = plannerStatus(run.task.metadata || {});
    setStatus(status.text, status.kind);
    play();
  } catch (error) {
    resetResult(`[ 错误 ] ${error.message || error}`);
    setStatus(`[ 错误 ] ${error.message || error}`, "err");
  }
}

function stop() {
  if (timer) clearInterval(timer);
  timer = null;
}

function play() {
  stop();
  timer = setInterval(() => {
    if (frameIndex >= motionFrames.length - 1) return stop();
    frameIndex += 1;
    renderFrame();
  }, FRAME_INTERVAL_MS);
}

function clearGroup(group) {
  while (group.children.length) {
    const child = group.children.pop();
    disposeObject(child);
    group.remove(child);
  }
}

function disposeObject(object) {
  object.traverse?.((child) => {
    child.geometry?.dispose?.();
    if (Array.isArray(child.material)) {
      child.material.forEach((material) => material.dispose?.());
    } else if (!Object.values(mat).includes(child.material)) {
      child.material?.dispose?.();
    }
  });
}

function resize() {
  const width = els.viewport.clientWidth;
  const height = els.viewport.clientHeight;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

function lerp(a, b, t) { return a + (b - a) * t; }
function ease(t) { return t * t * (3 - 2 * t); }
function interpolateGripper(from, to, t) {
  return {
    state: t < 1 ? from.state : to.state,
    width: lerp(Number(from.width) || 0, Number(to.width) || 0, t),
    force: lerp(Number(from.force) || 0, Number(to.force) || 0, t),
  };
}
function normalizeYaw(angle) {
  let value = angle;
  while (value <= -180) value += 360;
  while (value > 180) value -= 360;
  return value;
}
function yawDelta(from, to) { return normalizeYaw(to - from); }

els.randomTaskBtn.addEventListener("click", async () => {
  setStatus("[ 运行中 ] 正在生成随机任务", "running");
  try {
    const result = await post("/api/random", { type: els.taskType.value });
    els.taskText.value = result.task_text;
    setStatus("[ 完成 ] 随机任务已生成", "ok");
  } catch (error) {
    setStatus(`[ 错误 ] ${error.message || error}`, "err");
  }
});
els.runBtn.addEventListener("click", runWorkflow);
els.playBtn.addEventListener("click", play);
els.pauseBtn.addEventListener("click", stop);
els.resetBtn.addEventListener("click", () => { stop(); frameIndex = 0; renderFrame(); });
els.frameSlider.addEventListener("input", () => { stop(); frameIndex = Number(els.frameSlider.value); renderFrame(); });
window.addEventListener("resize", resize);

resize();
animate();
