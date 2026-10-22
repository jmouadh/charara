/* Charara field console - vanilla JS, offline friendly (no CDN).
 *
 * Coordinate systems:
 *   - field/robot frame: millimetres, origin at the bottom-left corner,
 *     +x to the right, +y up, theta CCW positive (same as the STM32 frame);
 *   - screen: pixels, y grows downwards - every conversion flips y.
 */
"use strict";

// -------------------------------------------------- state --
const DEFAULT_FIELD = { width_mm: 3000, height_mm: 2000 };

// the console plays the "camera" for the demo: 300 mm opponent robots - one
// by default at (2700, 1790); count and positions are set in Config. They
// are reported over /ws/vision (client "console") and stay draggable while
// our robot drives. When a REAL camera streams on the channel (any source
// that is not this console), the console yields and that input is used.
const OPPONENT_SIZE_MM = 300;
const MAX_OPPONENTS = 4;
const OPPONENT_SPOTS = [[2700, 1790], [2450, 1750], [2050, 1750], [1750, 1750]];
// Snapshot lifetime: long enough that a background tab - whose timers are
// throttled to ~1/min - never lets its own report expire between pushes (an
// expiry/reappear cycle used to trigger a pointless mid-run re-plan that
// replaced the running route). A tab that really goes away clears the
// snapshot immediately via WS close.
const VISION_TTL_MS = 180000;
const VISION_KEEPALIVE_MS = 1500;   // active tab: keep the snapshot fresh
const VISION_CAMERA_HZ = 15;       // mock camera report rate while a robot is dragged
const VISION_PUSH_MS = 1000 / VISION_CAMERA_HZ;
function defaultRobots(count = 1) {
  const robots = [];
  for (let i = 0; i < count; i++) {
    const [cx, cy] = OPPONENT_SPOTS[i] || OPPONENT_SPOTS[OPPONENT_SPOTS.length - 1];
    robots.push({ id: `R${i + 1}`, cx, cy, w: OPPONENT_SIZE_MM, h: OPPONENT_SIZE_MM });
  }
  return robots;
}

// drawing palette (persisted in localStorage) - every map layer is themeable
const COLOR_DEFAULTS = {
  stroke: "#e03434",    // drawn (pending) path
  path: "#e2528c",      // planned path
  ref: "#9b9188",       // fitted reference
  actions: "#a02a3f",   // turns, pauses, releases
  waypoint: "#ffb347",  // waypoint marker + arrival heading
  robot: "#2e7d52",     // our robot
  opponent: "#ff6b4a",  // other robots (camera reports) - the red squares
  obstacle: "#e74c3c",  // static box/zone obstacles
  goal: "#2ecc71",      // goal marker
  measure: "#3ec9d6",   // ruler / angle tool
  draft: "#f5a623",     // pose placement preview
};
const COLORS_KEY = "charara.colors";
const THEME_KEY = "charara.theme";
const THEME_CHOICES = ["light", "dark", "system"];

// robot / opponent photos (data URLs, persisted in localStorage) - they are
// drawn as the background of the robot square and of the opponent squares
const ROBOT_PHOTO_KEY = "charara.robotPhoto";
const OPPONENT_PHOTO_KEY = "charara.opponentPhoto";

function loadPhoto(key) {
  try {
    const url = localStorage.getItem(key) || "";
    if (!url) return { image: null, url: "" };
    const image = new Image();
    image.src = url; // the render loop draws it once decoded
    return { image, url };
  } catch (error) {
    return { image: null, url: "" };
  }
}

const app = {
  field: { ...DEFAULT_FIELD },
  view: { x: 1500, y: 1000, zoom: 0.2 },
  tool: "pan",
  telemetry: null,
  wsConnected: false,
  exec: null,
  items: [],         // ordered sequence: {kind:'stroke',points} | {kind:'rotate',angle_deg} | {kind:'pause',seconds}
  obstacles: [],     // {kind:'rect',x,y,w,h} | {kind:'circle',x,y,r}
  vision: [],        // camera-reported obstacles (WS 'vision' events, dashed overlay)
  robots: defaultRobots(), // opponent robots this console reports as the camera
  cameraLive: false, // true while a REAL camera owns /ws/vision (console yields)
  visionPeer: null,  // this console's peer id on /ws/vision (comes with the hello)
  visionSource: "",  // the live camera's source id while cameraLive
  goal: null,        // {x,y}
  poseDraft: null,   // {x,y,theta}
  waiting: null,     // set while the robot waits for a moving obstacle to clear
  plan: null,
  planElements: null, // deep copy of the items the current plan was built from
  runSnapshot: null,  // {elements, refs, ok} of the last executed run, awaiting validation
  strategy: { name: "", elements: [], savedAt: 0, dirty: false, showOnMap: false, team: "", editing: false, baseline: "" },
  measure: { kind: "", points: [] }, // ruler / angle tool clicks (world mm)
  drag: null,
  bg: { image: null, ok: false },
  i2c: { open: false, paused: false, count: 0 },
  history: [],       // snapshots of items[] for Undo
  redo: [],          // snapshots of items[] for Redo (cleared by new edits)
  transport: null,   // "i2c" (real board) or "mock" (simulator)
  team: null,        // current team info from /api/team (+ snapshot): sides, starts
  calib: null,       // last /config/robot response (register metadata + values)
  calibDriveOdom: null, // board-reported distance of the last drive test [mm]
  calibTurnOdom: null,  // board-reported angle of the last turn test [deg]
  theme: loadTheme(),   // "light" | "dark" | "system", persisted in the browser
  colors: loadColors(), // drawing palette, persisted in the browser
  robotPhoto: loadPhoto(ROBOT_PHOTO_KEY),    // background of the robot square
  opponentPhoto: loadPhoto(OPPONENT_PHOTO_KEY), // background of the R1.. squares
  ghostAnim: null,   // transparent preview robot (after-the-commands pose)
  ghostRawDelta: null, // one-shot: show a commanded rotation its true way
  ghostLockUntil: 0,  // while a scripted preview runs, telemetry must not steal it
};

// ------------------------------------------------------------- colours ----
function loadColors() {
  try {
    const saved = JSON.parse(localStorage.getItem(COLORS_KEY) || "{}");
    return { ...COLOR_DEFAULTS, ...saved };
  } catch (error) {
    return { ...COLOR_DEFAULTS };
  }
}

function saveColors() {
  try { localStorage.setItem(COLORS_KEY, JSON.stringify(app.colors)); } catch (error) { /* private mode */ }
}

function loadTheme() {
  try {
    const saved = localStorage.getItem(THEME_KEY);
    return THEME_CHOICES.includes(saved) ? saved : "light";
  } catch (error) {
    return "light";
  }
}

/** The dark veil is applied to the table artwork in the canvas, so every
 *  helper that needs to know the ACTIVE theme asks the DOM (system included). */
function isDarkTheme() {
  return document.documentElement.dataset.theme === "dark";
}

/** Native canvas outline that stays visible on the bright table and on the
 *  dark one (robot body, goal pole). */
function themeOutline() {
  return isDarkTheme() ? "rgba(226, 236, 250, .82)" : "#0b0f14";
}

/** Lighten a hex colour toward white (readable labels on a dark chip). */
function lightenHex(hex, amount) {
  const m = /^#?([0-9a-f]{6})$/i.exec(hex || "");
  if (!m) return hex;
  const n = parseInt(m[1], 16);
  const mix = (channel) => Math.round(channel + (255 - channel) * amount);
  return `rgb(${mix((n >> 16) & 255)}, ${mix((n >> 8) & 255)}, ${mix(n & 255)})`;
}

function hexToRgba(hex, alpha) {
  const m = /^#?([0-9a-f]{6})$/i.exec(hex || "");
  if (!m) return hex;
  const n = parseInt(m[1], 16);
  return `rgba(${(n >> 16) & 255}, ${(n >> 8) & 255}, ${n & 255}, ${alpha})`;
}

// a new path starting within this distance of the previous end snaps onto it
const SNAP_MM = 250;
// a waypoint clicked this close to a drawn path snaps onto its nearest point
const WP_SNAP_MM = 150;

const canvas = document.getElementById("field-canvas");
const ctx = canvas.getContext("2d");

const el = (id) => document.getElementById(id);
const els = {
  cameraPill: el("camera-pill"), imuPill: el("imu-pill"),
  swVersion: el("sw-version"), fwVersion: el("fw-version"),
  robotPill: el("robot-pill"), statePill: el("state-pill"), execPill: el("exec-pill"),
  modeSelect: el("mode-select"), toolHint: el("tool-hint"), stats: el("plan-stats"), log: el("log-lines"),
  logPanel: el("log"),
  toolParams: el("tool-params"), toolParamsTitle: el("tool-params-title"),
  cursorCoords: el("cursor-coords"), cursorX: el("cursor-x"), cursorY: el("cursor-y"),
  progress: el("progress"), progressBar: el("progress").querySelector("i"), progressText: el("progress-text"),
  tx: el("t-x"), ty: el("t-y"), tth: el("t-th"), tvl: el("t-vl"), tvr: el("t-vr"), tfaults: el("t-faults"),
  telemetry: el("telemetry"), telemetryToggle: el("telemetry-toggle"),
  execBtn: el("btn-exec"), seqList: el("seq-list"),
  teamSelect: el("team-select"), replayBtn: el("btn-replay"),
  posePopup: el("pose-popup"), poseX: el("pose-x"), poseY: el("pose-y"), poseTh: el("pose-th"),
  i2c: el("i2c"), i2cBody: el("i2c-body"), i2cCount: el("i2c-count"),
  calibForm: el("calib-form"), calibNote: el("calib-note"), calibStatus: el("calib-status"),
  boardErrors: el("board-errors"), boardStatus: el("board-status"),
  imuStatus: el("imu-status"), imuMode: el("imu-mode"), imuState: el("imu-state"),
  imuAxis: el("imu-axis"), imuBias: el("imu-bias"), imuIntegral: el("imu-integral"),
  imuStream: el("imu-stream"), imuBoard: el("imu-board"), imuScale: el("imu-scale"),
  imuToggle: el("btn-imu-toggle"), imuNote: el("imu-note"),
};

// ----------------------------------------------------------- full screen --
/** Header button: full-screen the whole console (F11-like); the icon and
 *  tooltip follow the browser state, Escape leaves too. */
const fullscreenBtn = el("btn-fullscreen");
const FS_ICONS = {
  enter: '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M9 4H4v5"/><path d="M15 4h5v5"/><path d="M15 20h5v-5"/><path d="M9 20H4v-5"/></svg>',
  exit: '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M4 9h5V4"/><path d="M20 9h-5V4"/><path d="M20 15h-5v5"/><path d="M4 15h5v5"/></svg>',
};

function fullscreenActive() {
  return Boolean(document.fullscreenElement);
}

function syncFullscreenButton() {
  if (!fullscreenBtn) return;
  const active = fullscreenActive();
  fullscreenBtn.innerHTML = active ? FS_ICONS.exit : FS_ICONS.enter;
  fullscreenBtn.title = active ? "Exit full screen" : "Enter full screen";
  fullscreenBtn.classList.toggle("active", active);
}

if (fullscreenBtn) {
  fullscreenBtn.addEventListener("click", async () => {
    try {
      if (fullscreenActive()) await document.exitFullscreen();
      else await document.documentElement.requestFullscreen();
    } catch (error) {
      logLine("warn", `full screen refused by the browser: ${error.message}`);
    }
  });
  document.addEventListener("fullscreenchange", syncFullscreenButton);
  syncFullscreenButton();
}

// ------------------------------------------------------------- utilities --
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const deg = (rad) => (rad * 180) / Math.PI;
const norm180 = (a) => (((a % 360) + 540) % 360) - 180;

// ------------------------------------------------ mission panel resizing --
/** The mission panel keeps its own width: drag the border strip (col-resize),
 *  use the arrow keys on it, or double-click to go back to the default. */
const PANEL_WIDTH_KEY = "charara.panelWidth";
const PANEL_WIDTH_DEFAULT = 320;
const PANEL_WIDTH_MIN = 250;
const PANEL_WIDTH_MAX = 620;
let panelWidthPx = PANEL_WIDTH_DEFAULT;

function panelMaxWidth() {
  return Math.max(PANEL_WIDTH_MIN, Math.min(PANEL_WIDTH_MAX, Math.round(window.innerWidth * 0.6)));
}

function applyPanelWidth(width, { persist = true } = {}) {
  panelWidthPx = Math.round(clamp(Number(width) || PANEL_WIDTH_DEFAULT, PANEL_WIDTH_MIN, panelMaxWidth()));
  document.documentElement.style.setProperty("--panel-w", `${panelWidthPx}px`);
  const resizer = el("panel-resizer");
  if (resizer) {
    resizer.setAttribute("aria-valuenow", String(panelWidthPx));
    resizer.setAttribute("aria-valuemax", String(panelMaxWidth()));
  }
  if (persist) {
    try { localStorage.setItem(PANEL_WIDTH_KEY, String(panelWidthPx)); } catch (error) { /* private mode */ }
  }
}

function loadPanelWidth() {
  try {
    const saved = Number(localStorage.getItem(PANEL_WIDTH_KEY));
    return Number.isFinite(saved) && saved > 0 ? saved : PANEL_WIDTH_DEFAULT;
  } catch (error) {
    return PANEL_WIDTH_DEFAULT;
  }
}

const panelResizer = el("panel-resizer");
if (panelResizer) {
  const panel = el("panel");
  let dragStartX = 0;
  let dragStartWidth = 0;
  let dragging = false;

  const endDrag = (event) => {
    if (!dragging) return;
    dragging = false;
    panelResizer.classList.remove("dragging");
    document.body.classList.remove("resizing-panel");
    if (event && panelResizer.hasPointerCapture(event.pointerId)) {
      panelResizer.releasePointerCapture(event.pointerId);
    }
    applyPanelWidth(panelWidthPx);   // persist the final width
  };

  panelResizer.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    dragging = true;
    dragStartX = event.clientX;
    dragStartWidth = panel ? panel.getBoundingClientRect().width : panelWidthPx;
    panelResizer.setPointerCapture(event.pointerId);
    panelResizer.classList.add("dragging");
    document.body.classList.add("resizing-panel");
    event.preventDefault();
  });
  panelResizer.addEventListener("pointermove", (event) => {
    if (!dragging) return;
    applyPanelWidth(dragStartWidth + (event.clientX - dragStartX), { persist: false });
  });
  panelResizer.addEventListener("pointerup", endDrag);
  panelResizer.addEventListener("pointercancel", endDrag);
  panelResizer.addEventListener("dblclick", () => applyPanelWidth(PANEL_WIDTH_DEFAULT));
  panelResizer.addEventListener("keydown", (event) => {
    if (event.key !== "ArrowLeft" && event.key !== "ArrowRight") return;
    event.preventDefault();
    const step = event.shiftKey ? 40 : 12;
    applyPanelWidth(panelWidthPx + (event.key === "ArrowRight" ? step : -step));
  });
  window.addEventListener("resize", () => applyPanelWidth(loadPanelWidth(), { persist: false }));
  applyPanelWidth(loadPanelWidth(), { persist: false });
}

function cssSize() {
  const rect = canvas.parentElement.getBoundingClientRect();
  return { w: Math.max(rect.width, 50), h: Math.max(rect.height, 50) };
}

function worldToScreen(x, y) {
  const { w, h } = cssSize();
  return [w / 2 + (x - app.view.x) * app.view.zoom, h / 2 - (y - app.view.y) * app.view.zoom];
}

function screenToWorld(sx, sy) {
  const { w, h } = cssSize();
  return [(sx - w / 2) / app.view.zoom + app.view.x, (h / 2 - sy) / app.view.zoom + app.view.y];
}

function fitView() {
  const { w, h } = cssSize();
  app.view.zoom = Math.min(w / app.field.width_mm, h / app.field.height_mm) * 0.92;
  app.view.x = app.field.width_mm / 2;
  app.view.y = app.field.height_mm / 2;
}

/** Clamp a world point to the playfield - nothing can be drawn off the map. */
function clampToField(wx, wy) {
  return [clamp(wx, 0, app.field.width_mm), clamp(wy, 0, app.field.height_mm)];
}

/** Keep a Box/Zone obstacle entirely on the playfield (the planner clips it
 *  server-side too, so the drawing and the grid always agree). */
function clampObstacle(obstacle) {
  if (!obstacle) return obstacle;
  const width = app.field.width_mm;
  const height = app.field.height_mm;
  if (obstacle.kind === "rect") {
    const x = Number(obstacle.x) || 0;
    const y = Number(obstacle.y) || 0;
    const w = Number(obstacle.w) || 0;
    const h = Number(obstacle.h) || 0;
    const x0 = clamp(x, 0, width);
    const y0 = clamp(y, 0, height);
    const x1 = clamp(x + w, 0, width);
    const y1 = clamp(y + h, 0, height);
    return { ...obstacle, x: x0, y: y0, w: x1 - x0, h: y1 - y0 };
  }
  if (obstacle.kind === "circle") {
    const r = clamp(Number(obstacle.r) || 0, 0, Math.min(width, height) / 2);
    const x = clamp(Number(obstacle.x) || 0, r, Math.max(r, width - r));
    const y = clamp(Number(obstacle.y) || 0, r, Math.max(r, height - r));
    return { ...obstacle, x, y, r };
  }
  return obstacle;
}

function clampObstacles(list) {
  return (Array.isArray(list) ? list : []).map(clampObstacle);
}

function logLine(level, message) {
  const line = document.createElement("div");
  line.className = `line ${level || "info"}`;
  const time = new Date().toLocaleTimeString([], { hour12: false });
  line.textContent = `[${time}] ${message}`;
  els.log.prepend(line);
  while (els.log.childElementCount > 120) els.log.lastChild.remove();
}

// ------------------------------------------------------------------- API --
async function api(path, method = "GET", body = null) {
  // live state must never be served from the browser/HTTP cache
  const options = { method, headers: {}, cache: "no-store" };
  if (body !== null) {
    options.headers["Content-Type"] = "application/json";
    options.body = JSON.stringify(body);
  }
  const response = await fetch(`/api${path}`, options);
  let data = null;
  try { data = await response.json(); } catch { /* empty body */ }
  if (!response.ok) {
    const detail = data && data.detail ? data.detail : response.statusText;
    throw new Error(`${method} ${path}: ${detail}`);
  }
  return data;
}

// -------------------------------------------------------------------- WS --
function connectWs() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => {
    app.wsConnected = true;
    updateLinkPills();
    // a reconnect usually means the API was redeployed: pick up the new
    // SW version/build without needing a full page reload
    refreshVersions();
  };
  ws.onclose = () => {
    app.wsConnected = false;
    updateLinkPills();
    setTimeout(connectWs, 1500);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (event) => {
    let message = null;
    try { message = JSON.parse(event.data); } catch { return; }
    if (message.type === "telemetry") onTelemetry(message.data);
    else if (message.type === "exec") onExec(message.data);
    else if (message.type === "plan") onPlan(message.data);
    else if (message.type === "log") logLine(message.data.level, message.data.message);
    else if (message.type === "trace") i2cAppend(message.data);
    else if (message.type === "vision") onVision(message.data);
  };
}

// ---------------------------------------------- camera role (robots) --
/** Update a status pill's instant hover description and screen-reader label.
 *  ``data-tip`` feeds the styled tooltip; no native ``title`` is used so only
 *  ONE tooltip shows on hover. */
function setPillHint(pill, text) {
  pill.dataset.tip = text;
  pill.setAttribute("aria-label", text);
}

/** Header camera icon (surveillance camera): the vision system state - green
 *  while a REAL camera stream owns /ws/vision (its obstacles drive the plan),
 *  neutral while the console's own opponent-robot reports act as the camera,
 *  red when the service link is down (no telemetry at all). */
function updateLinkPills() {
  if (!els.cameraPill) return;
  if (!app.wsConnected) {
    els.cameraPill.className = "pill icon off";
    setPillHint(els.cameraPill, "Camera / vision: service link down - no telemetry (reconnecting)");
  } else if (app.cameraLive) {
    els.cameraPill.className = "pill icon on";
    setPillHint(
      els.cameraPill,
      `Camera live: a real camera streams on /ws/vision (${app.visionSource || "camera"}) - its detections drive the plan`
    );
  } else {
    els.cameraPill.className = "pill icon";
    setPillHint(els.cameraPill, "Camera idle: no real camera - the console reports the opponent robots as the camera role");
  }
}

/** Header IMU pill: green = ready (gyro-aided heading), amber = calibrating,
 *  neutral = not available / stopped (localization runs wheel-only). */
function updateImuPill(data) {
  if (!els.imuPill) return;
  const imu = data && data.imu;
  if (!imu || !imu.present) {
    els.imuPill.className = "pill";
    setPillHint(els.imuPill, "IMU not available - localization runs wheel-only");
    return;
  }
  if (imu.calibrating) {
    els.imuPill.className = "pill warn";
    setPillHint(els.imuPill, "IMU calibrating - hold the robot still (~0.2 s)");
  } else if (imu.state === "ready") {
    els.imuPill.className = "pill on";
    setPillHint(
      els.imuPill,
      `IMU ready - ${imu.yaw_axis} yaw, bias ${imu.bias_deg_s.toFixed(3)} deg/s, ${imu.samples} samples`
    );
  } else {
    els.imuPill.className = "pill";
    setPillHint(els.imuPill, `IMU ${imu.state} - localization runs wheel-only`);
  }
}
/** The configured opponent robots are pushed to the service over /ws/vision
 *  exactly like a camera would report them: they are ordinary vision
 *  obstacles, so the planner avoids them and a running mission re-checks its
 *  remaining route on every move - detouring, or stopping and waiting for
 *  the robot in the way to clear. A REAL camera on the channel takes over:
 *  the console sees a foreign source and stops reporting (applyCameraRole). */
let visionWs = null;
let lastVisionPush = 0;
let visionDragTimer = 0;

function visionObstacles() {
  return app.robots.map((robot) => ({
    kind: "rect", id: robot.id,
    x: robot.cx - robot.w / 2, y: robot.cy - robot.h / 2, w: robot.w, h: robot.h,
  }));
}

function pushVision(throttled = false) {
  if (app.cameraLive) return; // a real camera owns the channel: stay quiet
  if (!visionWs || visionWs.readyState !== WebSocket.OPEN) return;
  const now = Date.now();
  if (throttled && now - lastVisionPush < VISION_PUSH_MS) return;
  lastVisionPush = now;
  visionWs.send(JSON.stringify({
    type: "obstacles", client: "console", obstacles: visionObstacles(), ttl_ms: VISION_TTL_MS, t: now,
  }));
}

function connectVision() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws/vision`);
  visionWs = ws;
  ws.onmessage = (event) => {
    let message = null;
    try { message = JSON.parse(event.data); } catch { return; }
    if (message.type === "hello") app.visionPeer = message.peer || null;
  };
  ws.onopen = () => {
    pushVision();
    logLine("info", app.cameraLive
      ? "camera role: paused - a REAL camera stream owns /ws/vision"
      : cameraRoleLine());
  };
  ws.onclose = () => { visionWs = null; setTimeout(connectVision, 2000); };
  ws.onerror = () => ws.close();
}

/** Mock camera: report the opponent position at the camera rate for the
 *  whole time it is held, even when the pointer is not moving. */
function startVisionDragStream() {
  stopVisionDragStream();
  pushVision();
  visionDragTimer = setInterval(() => pushVision(true), VISION_PUSH_MS);
}

function stopVisionDragStream() {
  if (visionDragTimer) {
    clearInterval(visionDragTimer);
    visionDragTimer = 0;
  }
}

/** The console's own line: how many opponent robots it reports right now. */
function cameraRoleLine() {
  const ids = app.robots.map((r) => r.id).join(", ");
  return `camera role: ${app.robots.length} opponent robot(s)${ids ? ` (${ids})` : ""} reported to the service - drag them anytime`;
}

// ------------------------------------------------------------ link mode --
/** Two modes: a real board over I2C, or the built-in simulator (mock).
 *  The header selector shows the active one and forces a switch. */
function applyTransport(transport) {
  if (!transport || transport === app.transport) return;
  app.transport = transport;
  const mock = transport === "mock";
  els.modeSelect.value = transport;
  els.modeSelect.className = mock ? "mode-select mock" : "mode-select real";
  els.modeSelect.disabled = false;
  els.modeSelect.title = mock
    ? "MOCK - every robot answer (pose, state, faults, execution) is simulated in the RPI service. Pick REAL to force the board over I2C."
    : `REAL - talking to the board over ${transport}: pose, state and faults come from the STM32. Pick MOCK to simulate.`;
  if (mock) {
    logLine("warn", "MOCK mode - the robot is simulated (pick REAL in the header for the board)");
  } else {
    logLine("info", `REAL mode - talking to the board over ${transport}`);
  }
}

els.modeSelect.addEventListener("change", async () => {
  const wanted = els.modeSelect.value;
  if (!wanted || wanted === app.transport) return;
  els.modeSelect.disabled = true;
  try {
    await api("/mode", "POST", { transport: wanted });
    logLine("info", `switching robot link to ${wanted === "mock" ? "MOCK (simulator)" : "REAL (i2c board)"}...`);
  } catch (error) {
    const detail = String((error && error.message) || error || "unknown error");
    logLine("error", `mode change refused: ${detail}`);
    if (wanted === "i2c" && /smbus2|i2c|bus|errno/i.test(detail)) {
      logLine("warn", "REAL needs the Raspberry Pi with the board on /dev/i2c-1 - this dev server can only simulate, stay on MOCK");
    }
    els.modeSelect.value = app.transport || "i2c";
  } finally {
    els.modeSelect.disabled = false;
  }
});

// ------------------------------------------------------------ team side --
// Eurobot is played by two sides (this edition: Yellow and Blue); the header
// selector says which one WE are. The team determines our robot's match-start
// pose AND the opponent's; the map draws both, and strategies authored for
// the other side are mirrored server-side when loaded (symmetric table).
const TEAM_COLORS = { yellow: "#efc13c", blue: "#4b7bec" };

function applyTeam(info) {
  if (!info || !info.current) return;
  app.team = info;
  const select = els.teamSelect;
  if (select) {
    if (info.teams) {
      for (const side of Object.keys(info.teams)) {
        const option = Array.from(select.options).find((entry) => entry.value === side);
        if (option) option.textContent = String(info.teams[side].label || side).toUpperCase();
      }
    }
    select.value = info.current;
    select.className = `team-select team-${info.current}`;
    select.disabled = false;
    const start = info.start || [];
    const opponent = info.opponent || { label: "?", start: [] };
    select.title =
      `We play ${info.label} - our match start (${start[0]}, ${start[1]}) mm, ${start[2]} deg; ` +
      `the opponent robot R1 (${opponent.label}) starts at (${opponent.start[0]}, ${opponent.start[1]}) mm. ` +
      "Strategies saved for the other side are mirrored when loaded (symmetric table).";
  }
}

/** Mock "Redo" availability, straight from the service snapshot. */
function applyRedo(redo) {
  app.redo = redo || null;
  if (!els.replayBtn) return;
  const available = Boolean(redo && redo.available);
  els.replayBtn.disabled = !available;
  if (!available) {
    els.replayBtn.title = "Mock rehearsal: replay the last run from its exact start pose (available in MOCK after a run)";
    return;
  }
  const start = redo.start || [];
  els.replayBtn.title =
    `Mock rehearsal: put the robot back exactly at the last run's start ` +
    `(${start[0]}, ${start[1]} mm, ${start[2]} deg) - then press Execute to replay it` +
    (redo.stashed ? ". A redo is armed: saving the strategy puts the robot back at the run's end instead" : "");
}

if (els.teamSelect) {
  els.teamSelect.addEventListener("change", async () => {
    const wanted = els.teamSelect.value;
    // ignore anything before the server's team config arrived: browsers may
    // restore/autofill the old select value on load and fire a change, which
    // must never switch the side by itself
    if (!app.team || !wanted || wanted === app.team.current) return;
    if (app.strategy.name && strategyDirty() &&
        !window.confirm("Switching side reloads the loaded strategy mirrored from its file - unsaved changes are lost. Continue?")) {
      els.teamSelect.value = app.team.current;
      return;
    }
    els.teamSelect.disabled = true;
    try {
      const previous = app.team ? app.team.current : null;
      const info = await api("/team", "POST", { team: wanted });
      applyTeam(info);
      applyTeamScenario(previous, info);
      if (app.strategy.name) {
        // the loaded strategy follows the side: re-load it MIRRORED for the
        // new team and put the robot at its recorded start ON THIS SIDE -
        // running the old-side copy made the robot dance across the field
        // to the other half and execute the wrong-side mission (2026-10-01)
        const doc = await api(`/strategies/${encodeURIComponent(app.strategy.name)}/load`, "POST");
        await applyStrategyDocument(doc, { editing: app.strategy.editing });
      }
    } catch (error) {
      logLine("error", `team change refused: ${error.message}`);
      if (app.team) els.teamSelect.value = app.team.current;
    } finally {
      els.teamSelect.disabled = false;
    }
  });
}

/** The rehearsal scenario follows the side. The opponent robot R1 is placed
 *  at ITS match start (when we switch to Blue, R1 is the YELLOW robot - it
 *  stands in the yellow corner); the extra reported robots swap halves with
 *  the table. Called on the first load and on every team change. */
function applyTeamScenario(previous, info) {
  if (!info || !info.opponent || !info.opponent.start) return;
  const width = app.field.width_mm;
  if (previous && previous !== info.current) {
    app.robots = app.robots.map((robot) => ({ ...robot, cx: width - robot.cx }));
  }
  if (app.robots.length) {
    app.robots[0].cx = info.opponent.start[0];
    app.robots[0].cy = info.opponent.start[1];
  }
  pushVision();
  if (typeof renderRobotRows === "function") renderRobotRows();
  logLine(
    "info",
    previous && previous !== info.current
      ? `rehearsal switched to ${info.label}: R1 (${info.opponent.label}) starts at ` +
          `(${info.opponent.start[0]}, ${info.opponent.start[1]}) mm`
      : `R1 (${info.opponent.label}) stands at its match start ` +
          `(${info.opponent.start[0]}, ${info.opponent.start[1]}) mm`,
  );
}

function onTelemetry(data) {
  applyTransport(data.transport);
  if (data.team) applyTeam(data.team);
  if (data.redo) applyRedo(data.redo);
  if (data.connected === false) {
    els.robotPill.className = "pill off";
    return;
  }
  app.telemetry = data;
  els.robotPill.className = "pill on";
  els.tx.textContent = data.x_mm.toFixed(1);
  els.ty.textContent = data.y_mm.toFixed(1);
  els.tth.textContent = data.theta_deg.toFixed(1);
  els.tvl.textContent = data.v_left_mm_s.toFixed(0);
  els.tvr.textContent = data.v_right_mm_s.toFixed(0);
  els.tfaults.textContent = data.fault_names.length ? data.fault_names.join(",") : "-";
  const stateClass = data.state === 5 || data.state === 6 ? "pill off" :
    (data.state !== 0 ? "pill warn" : "pill on");
  els.statePill.className = stateClass;
  els.statePill.textContent = data.state_name;
  app.waiting = data.waiting || null;
  if (app.waiting && !execActive()) {
    els.execPill.className = "pill warn";
    els.execPill.textContent = "EXEC waiting";
    els.execPill.classList.remove("hidden");
  }
  applyCameraRole(data.vision);
  applyBoardTelemetry(data);
  updateImuCard(data);
  updateImuPill(data);
  syncReleaseButtons();
  motionCheck();
  ghostRefresh();
}

function onExec(data) {
  app.exec = data;
  const running = data.status === "running" || data.status === "paused";
  const streaming = data.mode === "follow";
  els.execBtn.disabled = running ? true : !app.plan;
  setStaticToolsLocked(running);
  if (running || data.status === "done" || data.status === "aborted") {
    els.execPill.className = running ? "pill warn" : (data.status === "done" ? "pill on" : "pill off");
    els.execPill.textContent = streaming
      ? `EXEC ${data.status} stream`
      : `EXEC ${data.status} ${data.index + 1}/${data.total}`;
  }
  if (running) {
    els.progress.classList.remove("hidden");
    if (streaming && Number.isFinite(data.remaining_mm) && data.path_mm) {
      // streaming backend (Track B): one continuous run - show the path progress
      const pct = (1 - data.remaining_mm / data.path_mm) * 100;
      els.progressBar.style.width = `${clamp(pct, 0, 100)}%`;
      els.progressText.textContent = `streaming mode active - ${Math.round(data.remaining_mm)} mm left`;
    } else if (streaming) {
      els.progressText.textContent = data.message || "streaming mode active";
    } else {
      const pct = data.total ? ((data.index + (data.status === "running" ? 0.5 : 0)) / data.total) * 100 : 0;
      els.progressBar.style.width = `${clamp(pct, 0, 100)}%`;
      els.progressText.textContent = `node ${data.index + 1}/${data.total} ${data.message || ""}`;
    }
  } else {
    if (data.status === "done") els.progressBar.style.width = "100%";
    els.progressText.textContent = data.message || data.status;
    setTimeout(() => els.progress.classList.add("hidden"), 4000);
  }
  if (data.message && (data.status === "aborted")) {
    const waiting = /waiting/i.test(data.message);
    const replan = /re-planning/i.test(data.message);
    logLine(waiting || replan ? "warn" : "error", `execution: ${data.message}`);
  }
  // run validation: a finished run opens the strategy bar; a TRANSIENT abort
  // (mid-run re-plan / waiting for a moving obstacle) is not an end
  if (app.runSnapshot && app.runSnapshot.ok === null) {
    if (data.status === "done") {
      app.runSnapshot.ok = true;
      showVerify(true, data);
    } else if (
      data.status === "aborted" &&
      !/re-planning|waiting for the moving obstacle/i.test(data.message || "")
    ) {
      app.runSnapshot.ok = false;
      showVerify(false, data);
    }
  }
  // a whole-strategy Run has no step to validate: just report the outcome
  if (app.strategyRun && (data.status === "done" || (data.status === "aborted" &&
      !/re-planning|waiting for the moving obstacle/i.test(data.message || "")))) {
    const ok = data.status === "done";
    logLine(ok ? "info" : "error",
      `strategy run ${ok ? "finished" : `aborted: ${data.message || ""}`} - ` +
      `press Run again to replay, or Edit to modify it`);
    app.strategyRun = false;
  }
  syncStrategyButtons();
  syncReleaseButtons();
}

/** The release controls are only usable after a movement has completely
 *  finished. The board and the API enforce the same rule; disabling the
 *  buttons keeps the operator from even trying while the wheels turn. */
let releaseConfirmSettle = null;

function releaseBlocked() {
  const state = app.telemetry ? app.telemetry.state : null;
  return execActive() || (state !== null && state !== 0);
}

/** UI-styled inline confirmation instead of the native window.confirm. The
 *  small card is moved below whichever release button was pressed. */
function inlineReleaseConfirm(anchor) {
  const panel = el("release-confirm");
  if (!panel) return Promise.resolve(true);
  if (releaseConfirmSettle) settleReleaseConfirm(false);
  if (anchor) {
    const grid = anchor.closest(".grid2");
    const row = anchor.closest(".row");
    if (grid) anchor.insertAdjacentElement("afterend", panel);
    else if (row && row.parentElement) row.insertAdjacentElement("afterend", panel);
    else anchor.insertAdjacentElement("afterend", panel);
  }
  panel.classList.remove("hidden");
  const yes = el("release-confirm-yes");
  if (yes) setTimeout(() => yes.focus(), 0);
  return new Promise((resolve) => { releaseConfirmSettle = resolve; });
}

function settleReleaseConfirm(result) {
  const panel = el("release-confirm");
  if (panel) panel.classList.add("hidden");
  const settle = releaseConfirmSettle;
  releaseConfirmSettle = null;
  if (settle) settle(result);
}

function syncReleaseButtons() {
  const blocked = releaseBlocked();
  ["btn-release", "btn-release-now"].forEach((id) => {
    const button = el(id);
    if (!button) return;
    button.disabled = blocked;
    button.title = blocked
      ? "disabled while the robot is moving - release only after the movement has completely finished"
      : id === "btn-release-now"
        ? "Release the stepper motors now (drivers off; next move re-energizes)"
        : "Release the stepper motors (drivers off) when the robot stands still - the next move re-energizes them automatically";
  });
  const confirmYes = el("release-confirm-yes");
  if (confirmYes) confirmYes.disabled = blocked;
  if (blocked && releaseConfirmSettle) settleReleaseConfirm(false);
}

async function releaseMotorsNow(anchor) {
  if (releaseBlocked()) {
    logLine("warn", "release motors refused: the robot is still moving - wait until the movement has completely finished");
    return;
  }
  if (!(await inlineReleaseConfirm(anchor))) return;
  try {
    await api("/release_motors", "POST");
    logLine("info", "motors released - stepper drivers off; the next move re-energizes them");
  } catch (error) { logLine("error", error.message); }
}

/** Box / Zone are STATIC obstacles: set them before running. While the robot
 *  moves they are locked - the draggable opponent robots cover the dynamic case. */
function setStaticToolsLocked(locked) {
  document.querySelectorAll('.tool[data-tool="rect"], .tool[data-tool="circle"]').forEach((button) => {
    button.disabled = locked;
    button.title = locked ? "locked while the robot is executing - set static obstacles beforehand" : "";
  });
  if (locked && (app.tool === "rect" || app.tool === "circle")) {
    activateTool("pan");
    logLine("info", "static obstacle tools locked while the robot moves - the opponent robots stay draggable");
  }
}

/** True while a run is executing (also when paused). */
function execActive() {
  return Boolean(app.exec && (app.exec.status === "running" || app.exec.status === "paused"));
}

/** Camera obstacle reports: drawn as dashed outlines, respected automatically
 *  by the planner (they are merged into every plan service-side). A plan on
 *  screen re-plans when they move; mid-run the SERVICE re-checks the
 *  remaining route on its own (stop -> detour, or wait for the way to clear). */
let lastVisionLog = 0;
function onVision(data) {
  applyCameraRole(data);
  const obstacles = (data && data.obstacles) || [];
  const before = JSON.stringify(app.vision);
  app.vision = obstacles;
  if (JSON.stringify(obstacles) !== before) {
    // 15 Hz reports would flood the log; the map still follows every frame
    const now = Date.now();
    if (now - lastVisionLog >= 250) {
      lastVisionLog = now;
      if (obstacles.length) {
        const preview = obstacles.slice(0, 4)
          .map((o) => `(${Math.round(o.x)}, ${Math.round(o.y)})`).join(", ");
        const more = obstacles.length > 4 ? ` +${obstacles.length - 4} more` : "";
        logLine("warn", `camera: ${obstacles.length} obstacle(s) - ${preview}${more}`);
      } else {
        logLine("info", "camera: no obstacles reported");
      }
    }
    scheduleVisionReplan();
  }
}

/** A REAL camera - any vision source that is not this console and does not
 *  call itself "console" - owns /ws/vision: the console stands down (stops
 *  reporting its mock robots, dims them on the map). When that source goes
 *  away (the service releases it) the console takes its camera role back. */
function applyCameraRole(vision) {
  if (!vision || !app.visionPeer) return;
  const source = vision.source || "";
  const live = Boolean(source) && source !== app.visionPeer && vision.client !== "console";
  if (live === app.cameraLive) return;
  app.cameraLive = live;
  app.visionSource = live ? source : "";
  updateLinkPills();
  if (live) {
    logLine("warn", `REAL camera stream on /ws/vision (${source}) - the console robot reports are paused, its input is used`);
  } else {
    logLine("info", "camera stream ended - the console reports its opponent robots again");
    pushVision();
  }
  updateRobotsInfo();
}

/** Debounced re-plan after a camera change: like a hand-drawn obstacle, an
 *  idle console updates the plan on screen - mid-execution the SERVICE owns
 *  the remaining route (the console must never drive it twice). */
let visionReplanTimer = 0;
function scheduleVisionReplan() {
  if (!app.plan || execActive()) return;
  clearTimeout(visionReplanTimer);
  visionReplanTimer = setTimeout(() => planNow(), 300);
}

// ------------------------------------------------------------- rendering --
function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const { w, h } = cssSize();
  canvas.width = Math.round(w * dpr);
  canvas.height = Math.round(h * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function draw() {
  const { w, h } = cssSize();
  ctx.clearRect(0, 0, w, h);

  // field background
  const [fx, fy] = worldToScreen(0, app.field.height_mm);
  const [fw, fh] = worldToScreen(app.field.width_mm, 0);
  const fieldW = fw - fx;
  const fieldH = fh - fy;
  if (app.bg.ok && app.bg.image) {
    ctx.drawImage(app.bg.image, fx, fy, fieldW, fieldH);
    if (isDarkTheme()) {
      // night mode: black veil over the printed table, drawn layers keep contrast
      ctx.save();
      ctx.fillStyle = "rgba(0, 0, 0, .64)";
      ctx.fillRect(fx, fy, fieldW, fieldH);
      ctx.restore();
    }
  } else {
    ctx.fillStyle = isDarkTheme() ? "#0f0f0f" : "#f3efe6";
    ctx.fillRect(fx, fy, fieldW, fieldH);
    ctx.fillStyle = isDarkTheme() ? "#303030" : "#c9c2b4";
    ctx.fillRect(fx, fy, fieldW, 6);
    ctx.fillRect(fx, fy + fieldH - 6, fieldW, 6);
    ctx.fillRect(fx, fy, 6, fieldH);
    ctx.fillRect(fx + fieldW - 6, fy, 6, fieldH);
    ctx.fillStyle = isDarkTheme() ? "#a0a0a0" : "#8a8577";
    ctx.font = "12px sans-serif";
    ctx.fillText(`Eurobot table ${app.field.width_mm} x ${app.field.height_mm} mm`, fx + 14, fy + 20);
  }

  if (app.transport === "mock") {
    // unmistakable: everything on this field is a simulation
    ctx.save();
    ctx.font = `bold ${Math.max(14, Math.min(30, fieldW / 28))}px "Segoe UI", sans-serif`;
    ctx.textAlign = "right";
    ctx.textBaseline = "top";
    ctx.fillStyle = "rgba(245, 166, 35, .5)";
    ctx.fillText("MOCK - SIMULATED ROBOT", fx + fieldW - 12, fy + 10);
    ctx.restore();
  }

  drawGrid();
  drawObstacles();
  drawTeamStarts();
  drawVisionObstacles();
  drawRobots();
  drawItems();
  drawWaypoints();
  drawPlan();
  drawGoal();
  drawMeasure();
  drawGhost();
  drawRobot();
  drawPoseDraft();
  if (app.poseDraft) layoutPosePopup(); // keep the popup glued to the marker
}

function drawGrid() {
  const step = app.view.zoom > 0.25 ? 100 : 500;
  ctx.strokeStyle = isDarkTheme() ? "rgba(235, 235, 235, .14)" : "rgba(60, 70, 90, .18)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = 0; x <= app.field.width_mm; x += step) {
    const [sx, sy0] = worldToScreen(x, 0);
    const [, sy1] = worldToScreen(x, app.field.height_mm);
    ctx.moveTo(sx, sy0); ctx.lineTo(sx, sy1);
  }
  for (let y = 0; y <= app.field.height_mm; y += step) {
    const [sx0, sy] = worldToScreen(0, y);
    const [sx1] = worldToScreen(app.field.width_mm, y);
    ctx.moveTo(sx0, sy); ctx.lineTo(sx1, sy);
  }
  ctx.stroke();
}

function drawObstacles() {
  const obstacleColor = app.colors.obstacle || COLOR_DEFAULTS.obstacle;
  ctx.fillStyle = hexToRgba(obstacleColor, .22);
  ctx.strokeStyle = hexToRgba(obstacleColor, .75);
  ctx.lineWidth = 1.5;
  for (const obstacle of app.obstacles) {
    if (obstacle.kind === "rect") {
      const [x0, y0] = worldToScreen(obstacle.x, obstacle.y + obstacle.h);
      const [x1, y1] = worldToScreen(obstacle.x + obstacle.w, obstacle.y);
      ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
      ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
    } else {
      const [cx, cy] = worldToScreen(obstacle.x, obstacle.y);
      const r = obstacle.r * app.view.zoom;
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
  }
  if (app.drag && app.drag.mode === "rect-preview") {
    const d = app.drag;
    const [x0, y0] = worldToScreen(Math.min(d.a[0], d.b[0]), Math.max(d.a[1], d.b[1]));
    const [x1, y1] = worldToScreen(Math.max(d.a[0], d.b[0]), Math.min(d.a[1], d.b[1]));
    ctx.strokeStyle = obstacleColor;
    ctx.setLineDash([4, 3]);
    ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
    ctx.setLineDash([]);
  }
  if (app.drag && app.drag.mode === "circle-preview") {
    const d = app.drag;
    const [cx, cy] = worldToScreen(d.a[0], d.a[1]);
    const r = Math.hypot(d.b[0] - d.a[0], d.b[1] - d.a[1]) * app.view.zoom;
    ctx.strokeStyle = obstacleColor;
    ctx.setLineDash([4, 3]);
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.stroke();
    ctx.setLineDash([]);
  }
}

/** Key of a rect at the service's quantisation (10 mm) - the vision store
 *  rebuilds obstacles as kind/x/y/w/h, so ids cannot be used to match. */
function visionRectKey(rect) {
  const q = (value) => Math.round(value / 10) * 10;
  return `${q(rect.x)},${q(rect.y)},${q(rect.w)},${q(rect.h)}`;
}

/** Camera obstacles (WS 'vision' events): dashed orange-red outlines - they
 *  are NOT part of app.obstacles; the service merges them into every plan.
 *  Our own robot reports are drawn as draggable squares instead. */
function drawVisionObstacles() {
  if (!app.vision || !app.vision.length) return;
  // our own mock rects are not the camera's business - unless a REAL camera
  // is streaming, in which case they are just another robot on the table
  const ownRects = new Set(app.cameraLive ? [] : app.robots.map((robot) => visionRectKey({
    x: robot.cx - robot.w / 2, y: robot.cy - robot.h / 2, w: robot.w, h: robot.h,
  })));
  ctx.save();
  const opponentColor = app.colors.opponent || COLOR_DEFAULTS.opponent;
  ctx.setLineDash([7, 5]);
  ctx.strokeStyle = opponentColor;
  ctx.fillStyle = hexToRgba(opponentColor, .10);
  ctx.lineWidth = 2;
  for (const obstacle of app.vision) {
    if (obstacle.kind !== "circle" && ownRects.has(visionRectKey(obstacle))) continue;
    if (obstacle.kind === "circle") {
      const [cx, cy] = worldToScreen(obstacle.x, obstacle.y);
      const r = Math.max(2, obstacle.r * app.view.zoom);
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    } else {
      const [x0, y0] = worldToScreen(obstacle.x, obstacle.y + obstacle.h);
      const [x1, y1] = worldToScreen(obstacle.x + obstacle.w, obstacle.y);
      ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
      ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
    }
  }
  ctx.restore();
}

/** The opponent robots reported by this console as the camera: dashed squares
 *  with their id, draggable at any time - also while our robot is executing. */
function drawOpponentPhoto(x0, y0, pw, ph) {
  const image = app.opponentPhoto && app.opponentPhoto.image;
  if (!image || !image.complete || !image.naturalWidth) return;
  ctx.save();
  ctx.beginPath();
  ctx.rect(x0, y0, pw, ph);
  ctx.clip();
  const scale = Math.max(pw / image.naturalWidth, ph / image.naturalHeight);
  const width = image.naturalWidth * scale;
  const height = image.naturalHeight * scale;
  ctx.globalAlpha = 0.9;
  ctx.drawImage(image, x0 + (pw - width) / 2, y0 + (ph - height) / 2, width, height);
  ctx.restore();
}

/** The two match-start poses for the CURRENT team (ours + the opponent's):
 *  the marker reflects when the team changes, so the rehearsal always shows
 *  where both robots begin this match. Ours is solid, the opponent's dashed. */
function drawTeamStarts() {
  const info = app.team;
  if (!info || !info.start || !info.opponent || !info.opponent.start) return;
  drawStartGhost(info.start, TEAM_COLORS[info.current] || "#efc13c", true);
  drawStartGhost(info.opponent.start, TEAM_COLORS[info.opponent.team] || "#4b7bec", false);
}

function drawStartGhost(pose, color, ours) {
  const [x, y, th] = pose;
  const rw = parseFloat(el("robot-w") && el("robot-w").value) || 300;
  const rh = parseFloat(el("robot-h") && el("robot-h").value) || 300;
  const [sx, sy] = worldToScreen(x, y);
  const [ex, ey] = worldToScreen(x + 140 * Math.cos(th * Math.PI / 180), y + 140 * Math.sin(th * Math.PI / 180));
  const [px, py] = worldToScreen(x - rw / 2, y + rh / 2);
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.globalAlpha = ours ? 0.95 : 0.75;
  ctx.lineWidth = ours ? 2 : 1.5;
  ctx.setLineDash(ours ? [] : [6, 5]);
  ctx.strokeRect(px, py, rw * app.view.zoom, rh * app.view.zoom);
  ctx.beginPath();
  ctx.moveTo(sx, sy);
  ctx.lineTo(ex, ey);
  ctx.stroke();
  const ang = Math.atan2(ey - sy, ex - sx);
  ctx.setLineDash([]);
  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(ex - 9 * Math.cos(ang - 0.42), ey - 9 * Math.sin(ang - 0.42));
  ctx.lineTo(ex - 9 * Math.cos(ang + 0.42), ey - 9 * Math.sin(ang + 0.42));
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function drawRobots() {
  if (!app.robots.length) return;
  const grabbed = app.drag && app.drag.mode === "robot" ? app.drag.id : null;
  const live = app.cameraLive; // a real camera owns the channel: dim the mocks
  ctx.save();
  ctx.lineWidth = 2;
  for (const robot of app.robots) {
    const [x0, y0] = worldToScreen(robot.cx - robot.w / 2, robot.cy + robot.h / 2);
    const [x1, y1] = worldToScreen(robot.cx + robot.w / 2, robot.cy - robot.h / 2);
    const opponentColor = app.colors.opponent || COLOR_DEFAULTS.opponent;
    ctx.setLineDash([6, 4]);
    ctx.fillStyle = hexToRgba(opponentColor, grabbed === robot.id ? .30 : .15);
    ctx.strokeStyle = opponentColor;
    if (live) {
      ctx.fillStyle = "rgba(160, 160, 160, .08)";
      ctx.strokeStyle = "rgba(170, 170, 170, .75)";
    }
    ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
    if (!live) drawOpponentPhoto(x0, y0, x1 - x0, y1 - y0);
    ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
    ctx.setLineDash([]);
    ctx.font = "bold 12px 'Segoe UI', sans-serif";
    const text = live ? `${robot.id} - paused` : robot.id;
    const width = ctx.measureText(text).width;
    ctx.fillStyle = "rgba(12, 12, 12, .85)";
    ctx.fillRect(x0 + 4, y0 + 4, width + 10, 16);
    ctx.fillStyle = live ? "#aeb6c2" : lightenHex(app.colors.opponent || COLOR_DEFAULTS.opponent, .35);
    ctx.fillText(text, x0 + 9, y0 + 16);
  }
  ctx.restore();
}

function drawItems() {
  // validated (committed) strategy elements are NOT part of the working map:
  // they are only previewed when explicitly asked (Strategy card toggle, or
  // automatically after Load so a saved strategy can be reviewed/extended)
  if (app.strategy.showOnMap) {
    for (const item of app.strategy.elements) {
      if (item.kind !== "stroke" || !item.points || item.points.length < 2) continue;
      ctx.strokeStyle = "rgba(96, 191, 132, .55)";
      ctx.lineWidth = 2;
      ctx.setLineDash([]);
      ctx.beginPath();
      item.points.forEach(([x, y], i) => {
        const [sx, sy] = worldToScreen(x, y);
        if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
      });
      ctx.stroke();
    }
  }
  let previousEnd = null;
  for (const item of app.items) {
    if (item.kind !== "stroke" || item.points.length < 2) continue;
    const start = item.points[0];
    if (previousEnd && Math.hypot(start[0] - previousEnd[0], start[1] - previousEnd[1]) > 5) {
      drawPolyline([previousEnd, start], "rgba(154, 167, 181, .55)", 1.5, [5, 5]);
    }
    ctx.strokeStyle = app.colors.stroke;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    item.points.forEach(([x, y], i) => {
      const [sx, sy] = worldToScreen(x, y);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.stroke();
    previousEnd = item.points[item.points.length - 1];
  }
  if (previousEnd) {
    const [sx, sy] = worldToScreen(previousEnd[0], previousEnd[1]);
    ctx.fillStyle = app.colors.stroke;
    ctx.beginPath();
    ctx.arc(sx, sy, 4, 0, Math.PI * 2);
    ctx.fill();
  }
  if (app.drag && app.drag.mode === "draw") {
    const anchor = snapAnchor();
    if (anchor && !app.drag.snapped && app.drag.stroke.length) {
      drawPolyline([anchor.point, app.drag.stroke[0]], "rgba(245, 166, 35, .6)", 1.5, [5, 5]);
    }
    if (app.drag.stroke.length > 1) {
      ctx.strokeStyle = "rgba(255, 209, 102, .7)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      app.drag.stroke.forEach(([x, y], i) => {
        const [sx, sy] = worldToScreen(x, y);
        if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
      });
      ctx.stroke();
    }
  }
}

function drawPolyline(points, color, width, dash = []) {
  if (!points || points.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash);
  ctx.beginPath();
  points.forEach(([x, y], i) => {
    const [sx, sy] = worldToScreen(x, y);
    if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
  });
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawPlan() {
  if (!app.plan) return;
  drawPolyline(app.plan.reference, hexToRgba(app.colors.ref, 0.8), 1.5, [6, 5]);
  drawPolyline(app.plan.path, app.colors.path, 2.5);
  let pos = app.telemetry ? [app.telemetry.x_mm, app.telemetry.y_mm] : null;
  if (!pos) {
    const firstGoto = app.plan.segments.find((segment) => segment.kind === "goto");
    if (firstGoto) pos = [firstGoto.x, firstGoto.y];
  }
  ctx.font = "12px sans-serif";
  for (const segment of app.plan.segments) {
    if (segment.kind === "goto") {
      const [sx, sy] = worldToScreen(segment.x, segment.y);
      ctx.fillStyle = app.colors.path;
      ctx.beginPath();
      ctx.arc(sx, sy, 3.5, 0, Math.PI * 2);
      ctx.fill();
      pos = [segment.x, segment.y];
    } else if (segment.kind === "turn" && pos) {
      const [sx, sy] = worldToScreen(pos[0], pos[1]);
      ctx.strokeStyle = app.colors.actions;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(sx, sy, 10, -0.4, 1.9);
      ctx.stroke();
      ctx.fillStyle = app.colors.actions;
      ctx.fillText(`${segment.angle_deg > 0 ? "+" : ""}${segment.angle_deg.toFixed(0)} deg`, sx + 13, sy - 9);
    } else if (segment.kind === "pause" && pos) {
      const [sx, sy] = worldToScreen(pos[0], pos[1]);
      ctx.fillStyle = app.colors.actions;
      ctx.fillText(`pause ${segment.seconds}s`, sx + 13, sy + 15);
    } else if (segment.kind === "release") {
      const point = pos || (app.telemetry && [app.telemetry.x_mm, app.telemetry.y_mm]);
      if (point) {
        const [sx, sy] = worldToScreen(point[0], point[1]);
        ctx.fillStyle = app.colors.actions;
        ctx.fillText("release motors", sx + 13, sy + 30);
      }
    } else if (segment.kind === "arc" && segment.points && segment.points.length >= 2) {
      // native arc: highlight the curve and label its radius
      drawPolyline(segment.points, hexToRgba(app.colors.actions, 0.9), 2, [4, 3]);
      const mid = segment.points[Math.floor(segment.points.length / 2)];
      const [mx, my] = worldToScreen(mid[0], mid[1]);
      ctx.fillStyle = app.colors.actions;
      ctx.fillText(`arc R${Math.round(Math.abs(segment.radius_mm))}`, mx + 10, my - 8);
      const last = segment.points[segment.points.length - 1];
      pos = [last[0], last[1]];
    }
  }
  // fitted arc centres (small crosses)
  ctx.strokeStyle = hexToRgba(app.colors.path, 0.5);
  for (const primitive of app.plan.primitives) {
    if (primitive.type !== "arc") continue;
    const [sx, sy] = worldToScreen(primitive.center[0], primitive.center[1]);
    ctx.beginPath();
    ctx.moveTo(sx - 5, sy); ctx.lineTo(sx + 5, sy);
    ctx.moveTo(sx, sy - 5); ctx.lineTo(sx, sy + 5);
    ctx.stroke();
  }
}

function drawGoal() {
  if (!app.goal) return;
  const [sx, sy] = worldToScreen(app.goal.x, app.goal.y);
  const goalColor = app.colors.goal || COLOR_DEFAULTS.goal;
  ctx.fillStyle = goalColor;
  ctx.beginPath();
  ctx.arc(sx, sy, 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = themeOutline();
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(sx, sy);
  ctx.lineTo(sx, sy - 18);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(sx, sy - 18);
  ctx.lineTo(sx + 12, sy - 13);
  ctx.lineTo(sx, sy - 8);
  ctx.closePath();
  ctx.fillStyle = goalColor;
  ctx.fill();
  if (app.goal.theta !== null && app.goal.theta !== undefined) {
    // arrival heading: drag from the goal to aim it
    const rad = (app.goal.theta * Math.PI) / 180;
    ctx.strokeStyle = goalColor;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(sx + Math.cos(rad) * 46, sy - Math.sin(rad) * 46);
    ctx.stroke();
  }
}

// editor-only markers: the plan never renders waypoints (the path only
// crosses them), but you must see where they are while building the mission
function drawWaypoints() {
  const waypointColor = app.colors.waypoint || COLOR_DEFAULTS.waypoint;
  for (const item of app.items) {
    if (item.kind !== "waypoint") continue;
    const [sx, sy] = worldToScreen(item.x, item.y);
    const hasAngle = item.theta_deg !== null && item.theta_deg !== undefined;
    ctx.save();
    ctx.translate(sx, sy);
    if (hasAngle) {
      // the arrival angle: a small arrow showing which way the robot faces
      // when it reaches the point (amber = carries a heading constraint)
      const rad = (item.theta_deg * Math.PI) / 180;
      ctx.strokeStyle = waypointColor;
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(Math.cos(rad) * 30, -Math.sin(rad) * 30);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(Math.cos(rad) * 30, -Math.sin(rad) * 30, 2.2, 0, Math.PI * 2);
      ctx.fillStyle = waypointColor;
      ctx.fill();
    }
    ctx.rotate(Math.PI / 4);
    ctx.globalAlpha = 0.55;
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 3.5;
    ctx.strokeRect(-5.5, -5.5, 11, 11);
    ctx.globalAlpha = 0.95;
    ctx.strokeStyle = hasAngle ? waypointColor : app.colors.path;
    ctx.lineWidth = 1.5;
    ctx.strokeRect(-5, -5, 10, 10);
    ctx.restore();
  }
}

/** Robot size from the Config tab (Width / Height, mm). */
function robotSize() {
  const width = clamp(Number(el("robot-w") && el("robot-w").value) || 300, 50, 4000);
  const height = clamp(Number(el("robot-h") && el("robot-h").value) || 300, 50, 4000);
  return { w: width, h: height };
}

/** Screen-frame of the body square: centre, heading, pixel size. */
function robotBoxFrame(x, y, thDeg, widthMm, heightMm) {
  const [sx, sy] = worldToScreen(x, y);
  return {
    sx, sy,
    rad: (thDeg * Math.PI) / 180,
    pw: Math.max(widthMm * app.view.zoom, 14),
    ph: Math.max(heightMm * app.view.zoom, 14),
  };
}

/** Body square outline centred on the robot (world theta CCW -> screen
 *  rotate(-theta)); the square is the whole body: no part sticks out. */
function robotBoxPath(frame) {
  const halfW = frame.pw / 2;
  const halfH = frame.ph / 2;
  ctx.save();
  ctx.translate(frame.sx, frame.sy);
  ctx.rotate(-frame.rad);
  ctx.beginPath();
  ctx.rect(-halfW, -halfH, halfW * 2, halfH * 2);
  ctx.restore();
}

/** Front-of-travel chevron INSIDE the body (body frame, points along +x). */
function robotFrontPath(frame) {
  const halfW = frame.pw / 2;
  const halfH = frame.ph / 2;
  const size = Math.max(7, Math.min(frame.pw, frame.ph) * 0.34);
  const tipX = halfW * 0.78;
  const spread = Math.min(halfH * 0.55, size * 0.6);
  ctx.save();
  ctx.translate(frame.sx, frame.sy);
  ctx.rotate(-frame.rad);
  ctx.beginPath();
  ctx.moveTo(tipX, 0);
  ctx.lineTo(tipX - size, spread);
  ctx.lineTo(tipX - size * 0.55, 0);
  ctx.lineTo(tipX - size, -spread);
  ctx.closePath();
  ctx.restore();
}

/** Photo as the square's background (cover-cropped, clipped to the body).
 *  Returns false when there is no decoded photo - the caller then paints
 *  the solid colour instead. */
function drawPhotoInFrame(frame, photo) {
  const image = photo && photo.image;
  if (!image || !image.complete || !image.naturalWidth) return false;
  const halfW = frame.pw / 2;
  const halfH = frame.ph / 2;
  ctx.save();
  ctx.translate(frame.sx, frame.sy);
  ctx.rotate(-frame.rad);
  ctx.beginPath();
  ctx.rect(-halfW, -halfH, halfW * 2, halfH * 2);
  ctx.clip();
  const scale = Math.max((halfW * 2) / image.naturalWidth, (halfH * 2) / image.naturalHeight);
  const width = image.naturalWidth * scale;
  const height = image.naturalHeight * scale;
  ctx.drawImage(image, -width / 2, -height / 2, width, height);
  ctx.restore();
  return true;
}

function drawRobot() {
  const pose = app.telemetry;
  if (!pose) return;
  const size = robotSize();
  const frame = robotBoxFrame(pose.x_mm, pose.y_mm, pose.theta_deg, size.w, size.h);
  const faulted = pose.state === 5 || pose.state === 6;
  ctx.save();
  robotBoxPath(frame);
  const painted = drawPhotoInFrame(frame, app.robotPhoto);
  if (!painted) {
    ctx.fillStyle = faulted ? "rgba(231, 76, 60, .85)" : hexToRgba(app.colors.robot, 0.25);
    ctx.fill();
  } else if (faulted) {
    ctx.fillStyle = "rgba(231, 76, 60, .45)"; // the body fault stays visible
    ctx.fill();
  }
  ctx.strokeStyle = faulted ? "#ff6b4a" : themeOutline();
  ctx.lineWidth = 2;
  ctx.stroke();
  // front-of-travel chevron, INSIDE the body outline
  robotFrontPath(frame);
  ctx.fillStyle = "rgba(11, 15, 20, .62)";
  ctx.fill();
  ctx.strokeStyle = "rgba(255, 255, 255, .85)";
  ctx.lineWidth = 1.2;
  ctx.stroke();
  ctx.restore();
}

// --------------------------------------------------------- ghost preview --
// Semi-transparent robot showing where the commands leave it: it simulates
// the whole sequence from the live robot pose - strokes end at their last
// drawn point facing along the line, rotations turn (in the commanded
// direction), pauses hold, waypoints pass through, the goal finishes - and
// glides/rotates smoothly from one state to the next while you edit.
function missionTargets() {
  if (!app.telemetry) return null;
  const items = app.items.slice();
  if (app.drag && app.drag.mode === "draw" && app.drag.stroke.length >= 2) {
    items.push({ kind: "stroke", points: app.drag.stroke });
  }
  if (!items.length && !app.goal) return null;
  const poses = [];
  let pose = { x: app.telemetry.x_mm, y: app.telemetry.y_mm, th: app.telemetry.theta_deg };
  for (const item of items) {
    if (item.kind === "stroke" && item.points.length >= 2) {
      const pts = item.points;
      const end = pts[pts.length - 1];
      // the planner follows the SMOOTHED path: estimate the arrival heading
      // over a ~90 mm baseline (mirrors the planner's resample chord) - the
      // raw last segment is usually a hand wobble and would show an angle
      // the robot never performs
      let th = pose.th;
      for (let i = pts.length - 2; i >= 0; i--) {
        const dx = end[0] - pts[i][0];
        const dy = end[1] - pts[i][1];
        const dist = Math.hypot(dx, dy);
        if (dist >= 90.0 || i === 0) {
          if (dist > 1.0) th = deg(Math.atan2(dy, dx));
          break;
        }
      }
      const [lx, ly] = clampToLegal(end[0], end[1]);
      pose = { x: lx, y: ly, th };
    } else if (item.kind === "rotate") {
      pose = { x: pose.x, y: pose.y, th: pose.th + item.angle_deg };
    } else if (item.kind === "drive") {
      const radians = (pose.th * Math.PI) / 180;
      const [lx, ly] = clampToLegal(
        pose.x + Math.cos(radians) * item.distance_mm,
        pose.y + Math.sin(radians) * item.distance_mm,
      );
      pose = { x: lx, y: ly, th: pose.th };
    } else if (item.kind === "waypoint") {
      const dx = item.x - pose.x;
      const dy = item.y - pose.y;
      let th = Math.hypot(dx, dy) > 1.0 ? deg(Math.atan2(dy, dx)) : pose.th;
      if (item.theta_deg !== null && item.theta_deg !== undefined) th = item.theta_deg;
      const [lx, ly] = clampToWaypointSafe(item.x, item.y);
      pose = { x: lx, y: ly, th };
    }
    // pause: the robot does not move
    poses.push({ ...pose });
  }
  if (app.goal) {
    const dx = app.goal.x - pose.x;
    const dy = app.goal.y - pose.y;
    const travel = Math.hypot(dx, dy) > 1.0 ? deg(Math.atan2(dy, dx)) : pose.th;
    const th = app.goal.theta !== null && app.goal.theta !== undefined ? app.goal.theta : travel;
    const [lx, ly] = clampToSafe(app.goal.x, app.goal.y);
    pose = { x: lx, y: ly, th };
  }
  return { final: { ...pose }, poses };
}

function ghostPose() {
  const anim = app.ghostAnim;
  if (!anim) return null;
  const f = clamp((performance.now() - anim.t0) / Math.max(anim.dur, 1), 0, 1);
  const e = f < 0.5 ? 2 * f * f : 1 - ((-2 * f + 2) ** 2) / 2; // ease in/out
  const delta = anim.rawDelta !== null ? anim.rawDelta : norm180(anim.to.th - anim.from.th);
  return {
    x: anim.from.x + (anim.to.x - anim.from.x) * e,
    y: anim.from.y + (anim.to.y - anim.from.y) * e,
    th: anim.from.th + delta * e,
  };
}

function ghostRefresh(opts = {}) {
  if (!opts.force && performance.now() < app.ghostLockUntil) return; // scripted preview running
  const targets = missionTargets();
  const rawDelta = opts.rawDelta !== undefined ? opts.rawDelta : app.ghostRawDelta;
  app.ghostRawDelta = null;
  if (!targets) {
    app.ghostAnim = null;
    return;
  }
  const to = targets.final;
  const anim = app.ghostAnim;
  if (
    anim
    && Math.hypot(to.x - anim.to.x, to.y - anim.to.y) < 0.5
    && Math.abs(norm180(to.th - anim.to.th)) < 0.5
  ) {
    return; // target unchanged (e.g. telemetry tick) - let the animation finish
  }
  const current = ghostPose();
  app.ghostAnim = {
    from: current ? { ...current } : { ...to },
    to: { ...to },
    t0: performance.now(),
    dur: opts.dur !== undefined ? opts.dur : (current ? 650 : 1),
    rawDelta: rawDelta !== undefined ? rawDelta : null,
  };
}

const GHOST_GLIDE_MS = 380;
const GHOST_TURN_MS = 750;

/** Scripted rotation preview: the shadow glides to the point where the
 *  rotation happens, TURNS THERE by the commanded angle (its true direction,
 *  even 270 deg), then releases back to the mission end pose. Used when a
 *  rotation is added or its angle is stepped in the sequence. */
function ghostAnimateRotation(index) {
  const item = app.items[index];
  const targets = missionTargets();
  if (!item || item.kind !== "rotate" || !targets || index >= targets.poses.length) return;
  const after = targets.poses[index];
  const before = { x: after.x, y: after.y, th: after.th - item.angle_deg };
  const current = ghostPose() || { ...before };
  const now = performance.now();
  app.ghostAnim = { from: { ...current }, to: { ...before }, t0: now, dur: GHOST_GLIDE_MS, rawDelta: null };
  app.ghostLockUntil = now + GHOST_GLIDE_MS + GHOST_TURN_MS + 350;
  setTimeout(() => {
    app.ghostAnim = { from: { ...before }, to: { ...after }, t0: performance.now(), dur: GHOST_TURN_MS, rawDelta: item.angle_deg };
  }, GHOST_GLIDE_MS);
  setTimeout(() => ghostRefresh({ force: true }), GHOST_GLIDE_MS + GHOST_TURN_MS + 350);
}

function drawGhost() {
  const pose = ghostPose();
  if (!pose) return;
  const size = robotSize();
  const frame = robotBoxFrame(pose.x, pose.y, pose.th, size.w, size.h);
  ctx.save();
  robotBoxPath(frame);
  ctx.globalAlpha = 0.30;
  ctx.fillStyle = app.colors.robot;
  ctx.fill();
  ctx.globalAlpha = 0.55;      // white halo: readable on any artwork
  ctx.lineWidth = 3.5;
  ctx.strokeStyle = "#ffffff";
  ctx.stroke();
  ctx.globalAlpha = 0.85;
  ctx.setLineDash([6, 5]);
  ctx.lineWidth = 1.5;
  ctx.strokeStyle = app.colors.robot;
  ctx.stroke();
  ctx.setLineDash([]);
  robotFrontPath(frame);
  ctx.globalAlpha = 0.8;
  ctx.fillStyle = app.colors.robot;
  ctx.fill();
  ctx.restore();
}

function drawPoseDraft() {
  if (!app.poseDraft) return;
  const { x, y, theta } = app.poseDraft;
  const [sx, sy] = worldToScreen(x, y);
  const draftColor = app.colors.draft || COLOR_DEFAULTS.draft;
  ctx.save();
  ctx.strokeStyle = draftColor;
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.arc(sx, sy, 14, 0, Math.PI * 2);
  ctx.stroke();
  if (theta !== null && theta !== undefined) {
    const rad = (theta * Math.PI) / 180;
    const tipX = sx + Math.cos(rad) * 40;
    const tipY = sy - Math.sin(rad) * 40;
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(tipX, tipY);
    ctx.stroke();
    // live heading readout at the arrow tip - it follows the cursor while
    // you aim, so the angle value is visible before you touch the popup
    ctx.setLineDash([]);
    const label = `theta ${Math.round(theta)} deg`;
    ctx.font = "600 12px 'Segoe UI', system-ui, sans-serif";
    const tw = ctx.measureText(label).width;
    const bx = tipX + Math.cos(rad) * 18;
    const by = tipY - Math.sin(rad) * 18;
    const w = tw + 12;
    const h = 18;
    ctx.lineWidth = 1;
    ctx.beginPath();
    if (ctx.roundRect) {
      ctx.roundRect(bx - w / 2, by - h / 2, w, h, 9);
    } else {
      const r = 9;
      const l = bx - w / 2;
      const t = by - h / 2;
      ctx.moveTo(l + r, t);
      ctx.arcTo(l + w, t, l + w, t + h, r);
      ctx.arcTo(l + w, t + h, l, t + h, r);
      ctx.arcTo(l, t + h, l, t, r);
      ctx.arcTo(l, t, l + w, t, r);
      ctx.closePath();
    }
    ctx.fillStyle = "rgba(10, 10, 10, .90)";
    ctx.fill();
    ctx.strokeStyle = draftColor;
    ctx.stroke();
    ctx.fillStyle = draftColor;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(label, bx, by);
  }
  ctx.restore();
}

function renderLoop() {
  draw();
  requestAnimationFrame(renderLoop);
}

// ------------------------------------------------------------ interaction --
function eventPos(event) {
  const rect = canvas.getBoundingClientRect();
  return [event.clientX - rect.left, event.clientY - rect.top];
}

canvas.addEventListener("pointerdown", (event) => {
  canvas.setPointerCapture(event.pointerId);
  const [sx, sy] = eventPos(event);
  const [rawX, rawY] = screenToWorld(sx, sy);
  const [wx, wy] = clampToField(rawX, rawY);

  // the opponent robots are always grabbable - also mid-execution
  const robot = hitRobot(wx, wy);
  if (robot) {
    app.drag = { mode: "robot", id: robot.id, dx: robot.cx - wx, dy: robot.cy - wy };
    canvas.style.cursor = "grabbing";
    startVisionDragStream();
    logLine("info", `${robot.id} grabbed - moving a robot re-checks our remaining route live`);
    return;
  }

  // clicking a waypoint opens its editor: position + arrival angle
  const wpIndex = hitWaypoint(wx, wy);
  if (wpIndex >= 0 && app.tool !== "pan" && app.tool !== "erase") {
    closeGoalPopup();
    openWaypointPopup(wpIndex);
    return;
  }
  closeWaypointPopup();

  // clicking the goal opens its editor with position + arrival heading
  let goalClick = false;
  if (app.tool !== "erase" && app.tool !== "goal" && hitGoal(wx, wy)) {
    if (app.tool !== "pan") {
      openGoalPopup();
      return;
    }
    goalClick = true;
  } else {
    closeGoalPopup();
  }

  switch (app.tool) {
    case "pan":
      app.drag = { mode: "pan", start: [sx, sy], view: { ...app.view }, goalClick };
      break;
    case "draw": {
      const anchor = snapAnchor();
      const near = anchor && Math.hypot(wx - anchor.point[0], wy - anchor.point[1]) <= SNAP_MM;
      const [lx, ly] = clampToLegal(wx, wy);
      app.drag = { mode: "draw", stroke: [near ? anchor.point : [lx, ly]], snapped: Boolean(near) };
      if (near) {
        logLine("info", anchor.kind === "robot"
          ? "path start snapped to the robot position"
          : "path start snapped to the previous endpoint");
      }
      break;
    }
    case "pose": {
      const [lx, ly] = clampToLegal(wx, wy);
      if (!app.poseDraft) app.poseDraft = { x: lx, y: ly, theta: 0 };
      else app.poseDraft = { ...app.poseDraft, x: lx, y: ly };
      app.drag = { mode: "pose" };
      syncPoseUI();
      break;
    }
    case "goal": {
      const [lx, ly] = clampToSafe(wx, wy);
      app.goal = { x: lx, y: ly, theta: null };
      app.drag = { mode: "goal" };
      ghostRefresh();
      syncGoalUI();
      break;
    }
    case "waypoint": {
      const snap = nearestPointOnItems(wx, wy);
      const wpSpeed = optionalSpeed("seq-waypoint-speed");
      pushHistory();
      let wp;
      let snapped = false;
      let shifted = 0;
      if (snap && snap.d <= WP_SNAP_MM) {
        // split the drawn path at its nearest point and insert the waypoint
        // there: the robot drives the path and passes exactly through it, in
        // order - no return detour
        const [cx, cy] = clampToWaypointSafe(snap.x, snap.y);
        const pos = app.items.indexOf(snap.item);
        const pieceLength = (pts) => {
          let total = 0;
          for (let k = 1; k < pts.length; k++) total += Math.hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1]);
          return total;
        };
        const before = snap.item.points.slice(0, snap.index).concat([[cx, cy]]);
        const after = [[cx, cy]].concat(snap.item.points.slice(snap.index));
        const pieceSpeed = snap.item.speed_mm_s;
        const speedField = pieceSpeed ? { speed_mm_s: pieceSpeed } : {};
        const replacement = [];
        if (before.length >= 2 && pieceLength(before) > 1) {
          replacement.push({ kind: "stroke", points: before, ...speedField });
        }
        wp = { kind: "waypoint", x: cx, y: cy };
        if (wpSpeed) wp.speed_mm_s = wpSpeed;
        replacement.push(wp);
        if (after.length >= 2 && pieceLength(after) > 1) {
          replacement.push({ kind: "stroke", points: after, ...speedField });
        }
        app.items.splice(pos, 1, ...replacement);
        snapped = true;
        shifted = Math.hypot(cx - snap.x, cy - snap.y);
      } else {
        const [lx, ly] = clampToWaypointSafe(wx, wy);
        wp = { kind: "waypoint", x: lx, y: ly };
        if (wpSpeed) wp.speed_mm_s = wpSpeed;
        app.items.push(wp);
        shifted = Math.hypot(lx - wx, ly - wy);
      }
      // the press places the point; dragging away from it aims the arrival
      // angle exactly like Pose / Goal, so both happen in one gesture
      app.drag = { mode: "waypoint", item: wp, snapped, shifted };
      invalidatePlan();
      renderSequence();
      break;
    }
    case "ruler": {
      app.measure.points.push([wx, wy]);
      if (app.measure.points.length > 2) app.measure.points = [[wx, wy]];
      updateMeasureHint();
      break;
    }
    case "angle": {
      app.measure.points.push([wx, wy]);
      if (app.measure.points.length > 3) app.measure.points = [[wx, wy]];
      updateMeasureHint();
      break;
    }
    case "rect":
    case "circle":
      if (execActive()) {
        logLine("warn", "the robot is moving - static Box/Zone obstacles can only be added when it stands still (drag the opponent robots on the map instead)");
        break;
      }
      app.drag = { mode: app.tool === "rect" ? "rect-preview" : "circle-preview", a: [wx, wy], b: [wx, wy] };
      break;
    case "erase":
      eraseAt(wx, wy);
      break;
    default:
      break;
  }
});

function updateCursorCoords(sx, sy, wx, wy) {
  const chip = els.cursorCoords;
  if (!chip) return;
  els.cursorX.textContent = wx.toFixed(1);
  els.cursorY.textContent = wy.toFixed(1);
  chip.classList.toggle(
    "outside",
    wx < 0 || wy < 0 || wx > app.field.width_mm || wy > app.field.height_mm,
  );
  chip.classList.remove("hidden");
  // follow the pointer, flipping to the other side near an edge so the
  // readout never runs under the settings dock or out of the stage
  const stage = canvas.parentElement;
  const cw = chip.offsetWidth;
  const ch = chip.offsetHeight;
  let left = sx + 14;
  let top = sy + 16;
  if (left + cw > stage.clientWidth - 6) left = sx - cw - 14;
  if (top + ch > stage.clientHeight - 6) top = sy - ch - 14;
  chip.style.left = `${Math.max(6, left)}px`;
  chip.style.top = `${Math.max(6, top)}px`;
}

function hideCursorCoords() {
  if (els.cursorCoords) els.cursorCoords.classList.add("hidden");
}

canvas.addEventListener("pointerleave", hideCursorCoords);
canvas.addEventListener("pointercancel", () => {
  stopVisionDragStream();
  app.drag = null;
});
window.addEventListener("blur", stopVisionDragStream);
canvas.addEventListener("pointermove", (event) => {
  const [sx, sy] = eventPos(event);
  const [rawX, rawY] = screenToWorld(sx, sy);
  updateCursorCoords(sx, sy, rawX, rawY);
  if (!app.drag) {
    // hover feedback: the robots are grabbable under every tool
    canvas.style.cursor = hitRobot(rawX, rawY)
      ? "grab"
      : (ACTION_TOOLS.has(app.tool) ? "default" : "");
    return;
  }
  const [wx, wy] = clampToField(rawX, rawY);
  switch (app.drag.mode) {
    case "robot": {
      const robot = app.robots.find((r) => r.id === app.drag.id);
      if (robot) {
        robot.cx = clamp(wx + app.drag.dx, robot.w / 2, app.field.width_mm - robot.w / 2);
        robot.cy = clamp(wy + app.drag.dy, robot.h / 2, app.field.height_mm - robot.h / 2);
        // the drag interval sends at the camera rate; this keeps a fast
        // pointer move from waiting for the next tick (still rate-limited)
        pushVision(true);
      }
      break;
    }
    case "pan": {
      const dx = (sx - app.drag.start[0]) / app.view.zoom;
      const dy = (sy - app.drag.start[1]) / app.view.zoom;
      app.view.x = app.drag.view.x - dx;
      app.view.y = app.drag.view.y + dy;
      if (goalPopupOpen()) layoutGoalPopup();
      break;
    }
    case "draw": {
      const last = app.drag.stroke[app.drag.stroke.length - 1];
      if (Math.hypot(wx - last[0], wy - last[1]) * app.view.zoom > 3) {
        const [lx, ly] = clampToLegal(wx, wy);
        app.drag.stroke.push([lx, ly]);
        ghostRefresh({ dur: 150 });
      }
      break;
    }
    case "pose": {
      const dx = wx - app.poseDraft.x;
      const dy = wy - app.poseDraft.y;
      if (Math.hypot(dx, dy) > 40) {
        app.poseDraft.theta = deg(Math.atan2(dy, dx));
        syncPoseUI();
      }
      break;
    }
    case "rect-preview":
      app.drag.b = [wx, wy];
      break;
    case "circle-preview":
      app.drag.b = [wx, wy];
      break;
    case "goal": {
      if (app.goal) {
        const dx = wx - app.goal.x;
        const dy = wy - app.goal.y;
        if (Math.hypot(dx, dy) > 40) {
          app.goal.theta = deg(Math.atan2(dy, dx));
          ghostRefresh({ dur: 150 });
          syncGoalUI();
        }
      }
      break;
    }
    case "waypoint": {
      const item = app.drag.item;
      if (item) {
        const dx = wx - item.x;
        const dy = wy - item.y;
        if (Math.hypot(dx, dy) > 40) {
          item.theta_deg = norm180(deg(Math.atan2(dy, dx)));
          item.combine = true;
          ghostRefresh({ dur: 150 });
        }
      }
      break;
    }
    default:
      break;
  }
});

canvas.addEventListener("pointerup", (event) => {
  if (!app.drag) return;
  const drag = app.drag;
  app.drag = null;
  canvas.style.cursor = "";
  if (drag.mode === "robot") {
    stopVisionDragStream();
    const robot = app.robots.find((r) => r.id === drag.id);
    if (robot) {
      pushVision();
      logLine("info", app.cameraLive
        ? `${robot.id} moved to (${robot.cx.toFixed(0)}, ${robot.cy.toFixed(0)}) mm - NOT reported: a real camera stream owns the channel`
        : `${robot.id} moved to (${robot.cx.toFixed(0)}, ${robot.cy.toFixed(0)}) mm - the camera reports it; our route is re-checked`);
      syncRobotFields();
    }
    return;
  }
  if (drag.mode === "pan" && drag.goalClick) {
    const [sx, sy] = eventPos(event);
    if (Math.hypot(sx - drag.start[0], sy - drag.start[1]) < 4) openGoalPopup();
  }
  if (drag.mode === "draw") {
    const [sx, sy] = eventPos(event);
    const [rx, ry] = clampToLegal(...clampToField(...screenToWorld(sx, sy)));
    const last = drag.stroke[drag.stroke.length - 1];
    if (Math.hypot(rx - last[0], ry - last[1]) > 1) drag.stroke.push([rx, ry]);
  }
  if (drag.mode === "draw" && drag.stroke.length > 1) {
    pushHistory();
    const step = { kind: "stroke", points: drag.stroke };
    const speed = optionalSpeed("seq-draw-speed");
    if (speed) step.speed_mm_s = speed;
    app.items.push(step);
    invalidatePlan();
    renderSequence();
    logLine("info", `path added (${drag.stroke.length} points)`
      + (speed ? ` at ${speed} mm/s` : ""));
  }
  if (drag.mode === "waypoint" && drag.item) {
    invalidatePlan();
    renderSequence();
    const item = drag.item;
    const aim = item.theta_deg !== null && item.theta_deg !== undefined
      ? `, arrival ${item.theta_deg.toFixed(1)} deg combined with the approach`
      : "";
    const speed = item.speed_mm_s ? `, speed ${item.speed_mm_s} mm/s` : "";
    const moved = drag.shifted > 1
      ? ` (moved ${drag.shifted.toFixed(0)} mm out of the field / obstacle clearance)`
      : "";
    if (drag.snapped) {
      logLine("info", `waypoint snapped on the path at (${item.x.toFixed(0)}, ${item.y.toFixed(0)}) mm - the path is split there` + moved + aim + speed);
    } else {
      logLine("info", `waypoint (${item.x.toFixed(0)}, ${item.y.toFixed(0)}) mm added - the planner makes the path cross it` + moved + aim + speed);
    }
  }
  if (drag.mode === "rect-preview") {
    const x = Math.min(drag.a[0], drag.b[0]);
    const y = Math.min(drag.a[1], drag.b[1]);
    const w = Math.abs(drag.b[0] - drag.a[0]);
    const h = Math.abs(drag.b[1] - drag.a[1]);
    if (w > 30 && h > 30) {
      app.obstacles.push(clampObstacle({ kind: "rect", x, y, w, h }));
      obstaclesChanged();
    }
  }
  if (drag.mode === "circle-preview") {
    const r = Math.hypot(drag.b[0] - drag.a[0], drag.b[1] - drag.a[1]);
    if (r > 30) {
      app.obstacles.push(clampObstacle({ kind: "circle", x: drag.a[0], y: drag.a[1], r }));
      obstaclesChanged();
    }
  }
  if (drag.mode === "goal" && app.goal) {
    const aim = app.goal.theta === null || app.goal.theta === undefined ? "" : `, arrival theta ${app.goal.theta.toFixed(0)} deg`;
    logLine("info", `goal set to (${app.goal.x.toFixed(0)}, ${app.goal.y.toFixed(0)}) mm${aim}`);
    goalChanged();
    openGoalPopup();
  }
});

canvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  const [sx, sy] = eventPos(event);
  const [wx, wy] = screenToWorld(sx, sy);
  const factor = event.deltaY < 0 ? 1.12 : 1 / 1.12;
  app.view.zoom = clamp(app.view.zoom * factor, 0.02, 2);
  const [nx, ny] = screenToWorld(sx, sy);
  app.view.x += wx - nx;
  app.view.y += wy - ny;
  if (goalPopupOpen()) layoutGoalPopup();
}, { passive: false });

window.addEventListener("keydown", (event) => {
  if (event.key === "Escape") {
    stopVisionDragStream();
    app.drag = null;
    app.poseDraft = null;
    app.goal = null;
    ghostRefresh();
    syncGoalUI();
    syncPoseUI();
  }
});

/** The opponent robots are always grabbable - also while our robot drives. */
function hitRobot(wx, wy) {
  for (const robot of app.robots) {
    if (Math.abs(wx - robot.cx) <= robot.w / 2 && Math.abs(wy - robot.cy) <= robot.h / 2) return robot;
  }
  return null;
}

function eraseAt(wx, wy) {
  for (let i = app.obstacles.length - 1; i >= 0; i--) {
    const obstacle = app.obstacles[i];
    const hit = obstacle.kind === "rect"
      ? wx >= obstacle.x && wx <= obstacle.x + obstacle.w && wy >= obstacle.y && wy <= obstacle.y + obstacle.h
      : Math.hypot(wx - obstacle.x, wy - obstacle.y) <= obstacle.r;
    if (hit) {
      app.obstacles.splice(i, 1);
      logLine("info", "obstacle removed");
      obstaclesChanged();
      return;
    }
  }
  const tolerance = 20 / app.view.zoom;
  for (let i = app.items.length - 1; i >= 0; i--) {
    const item = app.items[i];
    if (item.kind === "waypoint") {
      if (Math.hypot(wx - item.x, wy - item.y) <= tolerance) {
        pushHistory();
        app.items.splice(i, 1);
        invalidatePlan();
        renderSequence();
        logLine("info", "waypoint removed");
        return;
      }
      continue;
    }
    if (item.kind !== "stroke") continue;
    if (item.points.some(([x, y]) => Math.hypot(wx - x, wy - y) <= tolerance)) {
      pushHistory();
      app.items.splice(i, 1);
      invalidatePlan();
      renderSequence();
      logLine("info", "path removed");
      return;
    }
  }
  if (app.goal && Math.hypot(wx - app.goal.x, wy - app.goal.y) <= tolerance) {
    app.goal = null;
    logLine("info", "goal removed");
    ghostRefresh();
    syncGoalUI();
    goalChanged();
  }
}

// ------------------------------------------------------------------ tools --
const TOOL_HINTS = {
  pan: "Drag to pan - wheel zooms. The dashed squares are the opponent robots (reported as camera obstacles - count & positions in Config) - grab them anytime, also while your robot drives.",
  pose: "Place the robot by hand: click where it stands, then drag away from the point to aim the heading - the angle follows your cursor on the map. A popup next to the point lets you type X / Y / theta, then Validate (the robot does not move - you only tell the system where it is).",
  draw: "Draw a path - the start snaps to the robot position or the previous path end within 250 mm.",
  goal: "Click to place the goal, then DRAG away from it to aim the arrival heading theta (the last node arrives on that angle). A popup opens so you can fine-tune X, Y and theta; click an existing goal to edit it.",
  waypoint: "Click near a drawn path: the waypoint snaps onto its nearest point and the path is split there. Click away from any path to add a free one. Click an EXISTING waypoint to edit its coordinates and the arrival angle (with the option to weave the turn into the approach). The path sweeps smoothly through pass-through waypoints (within ~5 cm); a waypoint carrying an arrival angle is passed exactly.",
  rect: "Drag a rectangle STATIC obstacle. Set boxes before running: while the robot executes this tool is locked (drag the opponent robots for moving obstacles).",
  circle: "Drag out a circular STATIC zone. Locked while the robot executes, like the box.",
  erase: "Click a path, obstacle or the goal to remove it (steps: use x in the list).",
  ruler: "Ruler: click two points to measure the distance between them. Click again to start a new measurement (the tool button also clears it).",
  angle: "Angle: click the CENTRE point first, then the two ray points - the angle between the rays is shown live on the map. Click again to start a new angle.",
  pause: "Delay: enter the hold time in seconds here, then press Add delay. The step is inserted at the end of the sequence.",
  rotate: "Turn: enter the angle (positive = CCW, negative = CW) and an optional speed, then press Add turn.",
  forward: "Forward move: enter the distance in mm and an optional speed, then press Add forward. The robot drives straight along the heading it holds there.",
  back: "Backward move: enter the distance in mm and an optional speed, then press Add backward. The robot reverses without turning around.",
  release: "Release: add a release step to the sequence, or release the stepper drivers now. Both are allowed only when the robot has completely stopped; the next move re-energizes the drivers automatically.",
};

// action tools open a small value box under the tool grid instead of a
// separate block; they do not use the map
const ACTION_TOOLS = new Set(["pause", "rotate", "forward", "back", "release"]);
const TOOL_PARAMS = {
  pause: { title: "Delay", group: "tp-pause", input: "seq-pause" },
  rotate: { title: "Turn", group: "tp-rotate", input: "seq-angle" },
  forward: { title: "Forward move", group: "tp-drive", input: "seq-advance", sign: 1 },
  back: { title: "Backward move", group: "tp-drive", input: "seq-advance", sign: -1 },
  release: { title: "Release motors", group: "tp-release", input: null },
  waypoint: { title: "Waypoint", group: "tp-waypoint", input: "seq-waypoint-speed" },
  draw: { title: "Draw path", group: "tp-draw", input: "seq-draw-speed" },
};
// tools that keep the value box open; the rest are pure map tools
const PARAM_TOOLS = new Set(["pause", "rotate", "forward", "back", "release", "waypoint", "draw"]);
let driveToolSign = 1;

/** Optional per-step speed from a tool/sequence box (empty = system speed). */
function optionalSpeed(inputId) {
  const input = el(inputId);
  if (!input) return undefined;
  const raw = String(input.value || "").trim();
  if (!raw) return undefined;
  const value = Number(raw);
  return Number.isFinite(value) && value > 0
    ? clamp(Math.round(value), 100, 900)
    : undefined;
}

function hideToolParams() {
  if (els.toolParams) els.toolParams.classList.add("hidden");
}

/** Open the value box for one action tool (delay / turn / forward / back). */
function showToolParams(name) {
  const cfg = TOOL_PARAMS[name];
  if (!cfg) return;
  driveToolSign = cfg.sign || 1;
  els.toolParams.classList.remove("hidden");
  els.toolParamsTitle.textContent = cfg.title;
  Object.values(TOOL_PARAMS).forEach((entry) => el(entry.group).classList.add("hidden"));
  el(cfg.group).classList.remove("hidden");
  if (name === "forward" || name === "back") {
    el("tp-drive-direction").textContent = name === "back" ? "Backward" : "Forward";
    el("btn-add-drive").textContent = name === "back" ? "Add backward" : "Add forward";
  }
  const input = cfg.input ? el(cfg.input) : null;
  if (input) {
    input.focus();
    input.select();
  }
}

/** Activate one drawing tool (buttons + hint). */
function activateTool(name) {
  document.querySelectorAll(".tool").forEach((b) => b.classList.toggle("active", b.dataset.tool === name));
  const wasMeasure = app.tool === "ruler" || app.tool === "angle";
  app.tool = name;
  if (!PARAM_TOOLS.has(name)) hideToolParams();
  if (name === "erase" || name === "draw") closeGoalPopup();
  if (wasMeasure && name !== "ruler" && name !== "angle") {
    // leaving the ruler/angle: their traces must not linger on the map
    app.measure = { kind: app.measure.kind, points: [] };
    updateMeasureHint();
  }
  els.toolHint.textContent = TOOL_HINTS[name] || "";
}

document.querySelectorAll(".tool").forEach((button) => {
  button.addEventListener("click", () => {
    const wanted = button.dataset.tool;
    if ((wanted === "rect" || wanted === "circle") && execActive()) {
      logLine("warn", "the robot is moving - static Box/Zone obstacles can only be added when it stands still (drag the opponent robots on the map instead)");
      return;
    }
    if (ACTION_TOOLS.has(wanted)) {
      activateTool(wanted);
      showToolParams(wanted);
      return;
    }
    if (wanted === "waypoint" || wanted === "draw") {
      activateTool(wanted);
      showToolParams(wanted);
      return;
    }
    if (wanted === "ruler" || wanted === "angle") {
      if (app.measure.kind === wanted && app.measure.points.length) {
        // re-clicking the active measure tool clears the measurement
        app.measure = { kind: wanted, points: [] };
      } else {
        app.measure = { kind: wanted, points: [] };
      }
      activateTool(wanted);
      updateMeasureHint();
      return;
    }
    activateTool(wanted);
  });
});

// Enter adds the value, Esc closes the box and returns to panning
[
  ["seq-pause", "btn-add-pause"],
  ["seq-angle", "btn-add-rotate"],
  ["seq-turn-speed", "btn-add-rotate"],
  ["seq-advance", "btn-add-drive"],
  ["seq-advance-speed", "btn-add-drive"],
].forEach(([inputId, buttonId]) => {
  const input = el(inputId);
  if (!input) return;
  input.addEventListener("keydown", (event) => {
    if (event.key === "Enter") {
      event.preventDefault();
      el(buttonId).click();
    } else if (event.key === "Escape") {
      event.preventDefault();
      hideToolParams();
      activateTool("pan");
    }
  });
});
el("tool-params-close").addEventListener("click", () => {
  hideToolParams();
  activateTool("pan");
});

// ------------------------------------------------------------- toolbar ----
function sliderBinding(id, outId, apply) {
  const input = el(id);
  const out = el(outId);
  const update = () => {
    out.textContent = input.value;
    if (apply) apply(Number(input.value));
  };
  input.addEventListener("input", update);
  update();
}

sliderBinding("opt-speed", "opt-speed-out");
sliderBinding("opt-step", "opt-step-out");
sliderBinding("opt-corner", "opt-corner-out");
sliderBinding("opt-chord", "opt-chord-out");
sliderBinding("opt-fit", "opt-fit-out");

function planOptions() {
  return {
    speed_mm_s: Number(el("opt-speed").value),
    goto_step_mm: Number(el("opt-step").value),
    corner_radius_mm: Number(el("opt-corner").value),
    node_chord_tol_mm: Number(el("opt-chord").value),
    fit_tol_mm: Number(el("opt-fit").value),
    robot_width_mm: Number(el("robot-w").value) || 300,
    robot_height_mm: Number(el("robot-h").value) || 300,
    obstacle_margin_mm: Math.max(0, Number(el("robot-margin").value) || 0),
    waypoint_margin_mm: Math.max(0, Number(el("robot-wp-margin").value) || 0),
    fit_enabled: el("opt-fit-enabled").checked,
    avoid_obstacles: el("opt-avoid").checked,
    smooth: el("opt-smooth").checked,
  };
}

// -------------------------------------------------------- pose popup ----
/** Position the popup next to the draft marker (kept inside the stage). */
function layoutPosePopup() {
  if (!app.poseDraft) return;
  const popup = els.posePopup;
  const { w, h } = cssSize();
  const [sx, sy] = worldToScreen(app.poseDraft.x, app.poseDraft.y);
  const pw = popup.offsetWidth || 190;
  const ph = popup.offsetHeight || 150;
  let left = sx + 26;
  let top = sy - ph / 2;
  if (left + pw > w - 8) left = sx - 26 - pw;
  left = clamp(left, 8, Math.max(8, w - pw - 8));
  top = clamp(top, 8, Math.max(8, h - ph - 8));
  const nextLeft = `${Math.round(left)}px`;
  const nextTop = `${Math.round(top)}px`;
  if (popup.style.left !== nextLeft) popup.style.left = nextLeft;
  if (popup.style.top !== nextTop) popup.style.top = nextTop;
}

/** Show/hide the popup with the draft and mirror the draft into the fields. */
function syncPoseUI(updateFields = true) {
  if (!app.poseDraft) {
    els.posePopup.classList.add("hidden");
    return;
  }
  els.posePopup.classList.remove("hidden");
  if (updateFields) {
    const active = document.activeElement; // never clobber a field being typed in
    if (active !== els.poseX) els.poseX.value = String(Math.round(app.poseDraft.x));
    if (active !== els.poseY) els.poseY.value = String(Math.round(app.poseDraft.y));
    if (active !== els.poseTh) els.poseTh.value = String(Math.round(app.poseDraft.theta ?? 0));
  }
  layoutPosePopup();
}

/** Typed X / Y / theta edits move the draft marker live on the map. */
function poseFieldChanged() {
  if (!app.poseDraft) return;
  const x = Number(els.poseX.value);
  const y = Number(els.poseY.value);
  const theta = Number(els.poseTh.value);
  if (Number.isFinite(x) && Number.isFinite(y)) {
    const [cx, cy] = clampToField(x, y);
    app.poseDraft.x = cx;
    app.poseDraft.y = cy;
  }
  if (Number.isFinite(theta)) app.poseDraft.theta = clamp(norm180(theta), -180, 180);
  layoutPosePopup();
}

[els.poseX, els.poseY, els.poseTh].forEach((input) => {
  input.addEventListener("input", poseFieldChanged);
  input.addEventListener("change", () => syncPoseUI()); // show the clamped value
  input.addEventListener("keydown", (event) => {
    if (event.key === "Enter") setPoseFromDraft();
  });
});
el("pose-validate").addEventListener("click", () => setPoseFromDraft());
el("pose-cancel").addEventListener("click", () => {
  app.poseDraft = null;
  syncPoseUI();
});

async function setPoseFromDraft() {
  const draft = app.poseDraft;
  if (!draft) {
    logLine("warn", "place the robot first with the Pose tool");
    return;
  }
  const theta = draft.theta || 0;
  try {
    await api("/pose", "POST", { x_mm: draft.x, y_mm: draft.y, theta_deg: theta });
    app.poseDraft = null;
    syncPoseUI();
    // move the on-screen marker at once - the UI now "knows" where the robot is
    if (app.telemetry) onTelemetry({ ...app.telemetry, x_mm: draft.x, y_mm: draft.y, theta_deg: theta });
    logLine("info", `robot is now known to stand at (${draft.x.toFixed(0)}, ${draft.y.toFixed(0)}) mm, ${theta.toFixed(0)} deg - nothing was moved`);
  } catch (error) { logLine("error", error.message); }
}

el("btn-setpose").addEventListener("click", () => setPoseFromDraft());

el("btn-plan").addEventListener("click", () => planNow());

/** Show one plan document on the map and in the summary line.
 *  Used for the POST response AND for the mid-run swap broadcasts: the map
 *  must always show the route the robot will actually drive. */
function applyPlanDoc(result) {
  app.plan = result;
  // the planner projects a waypoint standing inside a clearance zone to the
  // closest legal point: move the marker too, so the map, the sequence and
  // the mission cursor all use the point the robot will really visit.
  // (Not for a strategy run: its elements are not the sequence on screen.)
  if (!app.strategyRun && Array.isArray(result.resolved_waypoints) && result.resolved_waypoints.length) {
    let index = 0;
    let moved = false;
    for (const item of app.items) {
      if (item.kind !== "waypoint" || index >= result.resolved_waypoints.length) continue;
      const [rx, ry] = result.resolved_waypoints[index++];
      if (Math.hypot(item.x - rx, item.y - ry) > 0.5) {
        item.x = rx;
        item.y = ry;
        moved = true;
      }
    }
    if (moved) {
      renderSequence();
      logLine("info", "a waypoint was moved to the closest point outside the field / obstacle clearance");
    }
  }
  const stats = result.stats;
  els.stats.textContent =
    `${(stats.length_mm / 1000).toFixed(2)} m - ${stats.n_segments} segments ` +
    `(${stats.n_gotos} GOTO, ${stats.n_arcs || 0} ARC, ${stats.n_actions} action) - ~${stats.est_s} s` +
    (stats.n_primitives ? `\n${stats.n_primitives} line/arc primitives` : "") +
    (result.warnings.length ? `\n! ${result.warnings.join("\n! ")}` : "");
  els.execBtn.disabled = false;
}

/** The service REGENERATED the remaining route mid-run (a dynamic obstacle
 *  re-routed it, or a wait cleared): redraw the map with the new document so
 *  the shown path matches what the robot will do. */
function onPlan(doc) {
  if (!doc || !doc.segments || !doc.path) return;
  applyPlanDoc(doc);
}

async function planNow() {
  if (!app.items.length && !app.goal) {
    logLine("warn", "draw a path or place a goal first");
    return;
  }
  try {
    const result = await api("/plan", "POST", {
      elements: app.items,
      obstacles: clampObstacles(app.obstacles),
      goal: app.goal ? { x: app.goal.x, y: app.goal.y, theta_deg: app.goal.theta ?? null } : null,
      options: planOptions(),
    });
    applyPlanDoc(result);
    app.planElements = JSON.parse(JSON.stringify(app.items));
    // (the backend broadcasts a matching log line over the WebSocket)
  } catch (error) { logLine("error", error.message); }
}

el("btn-exec").addEventListener("click", async () => {
  if (!app.plan) return;
  try {
    await api("/execute", "POST", { segments: app.plan.segments, follow_path: app.plan.path || null });
    // remember what this run covers: the validation bar offers to ADD the
    // step to the strategy (or DISCARD it as a test) when the run finishes.
    // While EDITING, the sequence IS the strategy - there is no step to
    // validate, so no bar (Save keeps the edited document).
    app.runSnapshot = app.strategy.editing ? null : {
      elements: JSON.parse(JSON.stringify(app.planElements || app.items)),
      refs: app.items.slice(),
      ok: null,
    };
    app.strategyRun = false;
    hideVerify();
  } catch (error) { logLine("error", error.message); }
});

el("btn-replay").addEventListener("click", async () => {
  try {
    const result = await api("/mission/redo", "POST");
    // the simulator is already back at the run start: jump the marker at once
    if (app.telemetry && result.start) {
      onTelemetry({ ...app.telemetry, x_mm: result.start[0], y_mm: result.start[1], theta_deg: result.start[2] });
    }
  } catch (error) {
    logLine("warn", error.message);
  }
});

el("btn-pause").addEventListener("click", async () => {
  try { await api("/pause", "POST"); } catch (error) { logLine("warn", error.message); }
});
el("btn-resume").addEventListener("click", async () => {
  try { await api("/resume", "POST"); } catch (error) { logLine("warn", error.message); }
});
el("btn-stop").addEventListener("click", async () => {
  try { await api("/stop", "POST"); } catch (error) { logLine("warn", error.message); }
});
el("btn-estop").addEventListener("click", async () => {
  try { await api("/estop", "POST"); } catch (error) { logLine("error", error.message); }
});
el("btn-clear").addEventListener("click", async () => {
  try { await api("/clear_faults", "POST"); } catch (error) { logLine("warn", error.message); }
});
el("btn-release").addEventListener("click", (event) => releaseMotorsNow(event.currentTarget));
el("btn-release-now").addEventListener("click", (event) => releaseMotorsNow(event.currentTarget));
el("release-confirm-yes").addEventListener("click", () => settleReleaseConfirm(true));
el("release-confirm-no").addEventListener("click", () => settleReleaseConfirm(false));
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && releaseConfirmSettle) {
    event.preventDefault();
    settleReleaseConfirm(false);
  }
});

el("btn-add-release").addEventListener("click", () => {
  pushHistory();
  app.items.push({ kind: "release" });
  invalidatePlan();
  renderSequence();
  logLine("info", "motor release step added - it runs only after the previous movement has completely finished; the next move re-energizes the drivers");
});

// ------------------------------------------------------------- strategy --
// Build a mission step by step: draw ONE step, run it, VALIDATE it (Add) or
// throw the test away (Discard). Added steps are committed to the strategy
// and removed from the pending sequence, so the next plan covers only the
// new work and starts from wherever the robot stopped.
function showVerify(ok, data) {
  const name = (el("strat-name").value || "").trim() || "untitled";
  const bar = el("strat-verify");
  bar.classList.remove("hidden", "ok", "fail");
  bar.classList.add(ok ? "ok" : "fail");
  el("strat-add").classList.toggle("hidden", !ok);
  el("strat-keep").classList.toggle("hidden", ok);
  el("strat-verify-text").textContent = ok
    ? `Step finished (${data.total} nodes) - add it to "${name}"?`
    : `Step did not finish (${data.message || data.status}) - nothing was saved.`;
}

function hideVerify() {
  el("strat-verify").classList.add("hidden");
}

function strategyRemovePending(snapshot) {
  app.items = app.items.filter((item) => !snapshot.refs.includes(item));
  invalidatePlan();
  renderSequence();
}

function ensureStrategyName() {
  let name = (el("strat-name").value || "").trim();
  if (!name) {
    name = "untitled";
    el("strat-name").value = name;
  }
  if (!/^[A-Za-z0-9][A-Za-z0-9._ -]{0,63}$/.test(name)) {
    logLine("warn", "strategy name: start with a letter/digit, then letters, digits, '.', '_', '-' or spaces (max 64)");
    return null;
  }
  app.strategy.name = name;
  return name;
}

el("strat-add").addEventListener("click", () => {
  const snapshot = app.runSnapshot;
  if (!snapshot || !snapshot.ok) return;
  const name = ensureStrategyName();
  if (!name) return;
  if (!app.strategy.elements.length && !app.strategy.startPose) {
    // the FIRST validated step defines where the strategy starts: the pose
    // the robot held when that run began - loading the strategy puts it back
    // there instead of driving to the start from wherever it stands
    app.strategy.startPose = (app.redo && app.redo.start) || null;
  }
  app.strategy.elements.push(...JSON.parse(JSON.stringify(snapshot.elements)));
  app.strategy.dirty = true;
  strategyRemovePending(snapshot);
  app.runSnapshot = null;
  hideVerify();
  updateStrategyUI();
  logLine("info", `step validated - strategy "${name}" now has ${app.strategy.elements.length} element(s) (press Save to store it)`);
});

el("strat-keep").addEventListener("click", () => {
  app.runSnapshot = null; // elements stay pending: press Execute to try again
  hideVerify();
  logLine("info", "step kept on the map for another try");
});

el("strat-discard").addEventListener("click", () => {
  const snapshot = app.runSnapshot;
  app.runSnapshot = null;
  hideVerify();
  if (snapshot) strategyRemovePending(snapshot);
  logLine("info", "step discarded (test only)");
});

el("strat-name").addEventListener("input", () => {
  app.strategy.name = el("strat-name").value.trim();
  updateStrategyUI();
});

el("strat-new").addEventListener("click", () => {
  const editing = app.strategy.editing;
  if (editing) {
    if ((strategyDirty() || app.items.length) &&
        !window.confirm("Start a new strategy? The edited mission on the map will be cleared.")) return;
    app.items = []; // the sequence WAS the document: clearing it clears the edit
    invalidatePlan();
    renderSequence();
  } else if (app.strategy.dirty && !window.confirm("The strategy has unsaved changes - discard them?")) return;
  app.strategy = { name: "", elements: [], savedAt: 0, dirty: false, showOnMap: false, team: "", editing: false, baseline: "" };
  el("strat-name").value = "";
  el("strat-show").checked = false;
  updateStrategyUI();
  logLine("info", "new empty strategy - name it and add validated steps");
});

el("strat-show").addEventListener("change", () => {
  app.strategy.showOnMap = el("strat-show").checked;
});

el("strat-save").addEventListener("click", async () => {
  const name = ensureStrategyName();
  if (!name) return;
  const elements = strategyElements();
  if (!elements.length) {
    logLine("warn", app.strategy.editing
      ? "the sequence is empty - draw the mission first"
      : "the strategy is empty - validate at least one step first (Add step)");
    return;
  }
  try {
    const result = await api(`/strategies/${encodeURIComponent(name)}`, "PUT", {
      elements: elements,
      options: planOptions(),
      goal: app.goal ? { x: app.goal.x, y: app.goal.y, theta_deg: app.goal.theta ?? null } : null,
      obstacles: clampObstacles(app.obstacles),
      start_pose: app.strategy.startPose || null,
      notes: "",
    });
    app.strategy.name = (result.strategy && result.strategy.name) || name;
    app.strategy.savedAt = (result.strategy && result.strategy.updated_at) || Date.now() / 1000;
    app.strategy.team = (result.strategy && result.strategy.team) || "";
    app.strategy.dirty = false;
    if (app.strategy.editing) app.strategy.baseline = JSON.stringify(app.items);
    updateStrategyUI();
    refreshStrategyList();
  } catch (error) { logLine("error", `save failed: ${error.message}`); }
});

// Run: plan + execute the WHOLE strategy (or the edited sequence) from the
// robot's CURRENT pose - the plan is re-computed every press, obstacles and
// the current goal/options included, so it can be replayed any time.
el("strat-run").addEventListener("click", async () => {
  if (execActive()) return;
  const editing = app.strategy.editing;
  const elements = strategyElements();
  if (!elements.length) {
    logLine("warn", editing ? "the sequence is empty - draw the mission first" : "no strategy loaded - press Load first");
    return;
  }
  try {
    const result = await api("/plan", "POST", {
      elements: JSON.parse(JSON.stringify(elements)),
      obstacles: clampObstacles(app.obstacles),
      goal: app.goal ? { x: app.goal.x, y: app.goal.y, theta_deg: app.goal.theta ?? null } : null,
      options: planOptions(),
    });
    app.strategyRun = true; // its outcome is logged when it ends
    applyPlanDoc(result);
    app.planElements = JSON.parse(JSON.stringify(elements));
    app.runSnapshot = null; // a whole-strategy run is not a step to validate
    hideVerify();
    await api("/execute", "POST", { segments: result.segments, follow_path: result.path || null });
  } catch (error) { logLine("error", error.message); }
});

// Edit: move the committed strategy INTO the sequence so every tool edits it
// (steps, waypoints, erase, order); from then on Save writes the sequence
// back to the file and the normal Plan/Execute buttons drive it.
el("strat-edit").addEventListener("click", () => {
  if (app.strategy.editing) return;
  if (!app.strategy.elements.length) {
    logLine("warn", "nothing to edit - Load a strategy first");
    return;
  }
  if (app.items.length && !window.confirm("The sequence already has steps - replace them with the strategy?")) return;
  app.items = app.strategy.elements.map((element) => JSON.parse(JSON.stringify(element)));
  app.strategy.elements = []; // the mission now lives in the sequence
  app.strategy.editing = true;
  app.strategy.dirty = false;
  app.strategy.baseline = JSON.stringify(app.items);
  invalidatePlan();
  renderSequence();
  updateStrategyUI();
  logLine("info",
    `strategy "${app.strategy.name || "untitled"}" moved into the sequence - ` +
    "modify it with every tool, then Plan + Execute (or Run); Save writes the file back");
});

/** Apply a strategy document (from load) to the editor state. ``editing``
 *  keeps editing mode: the sequence is replaced by the document so the tools
 *  keep editing the strategy itself. */
async function applyStrategyDocument(doc, { editing = false } = {}) {
  app.strategy = {
    name: doc.name,
    elements: (doc.elements || []).map((element) => JSON.parse(JSON.stringify(element))),
    savedAt: doc.updated_at || 0,
    dirty: false,
    showOnMap: true, // loaded to review: draw it right away
    team: doc.team || "",
    authoredFor: doc.authored_for || "",
    startPose: doc.start_pose || null,
    editing: false,
    baseline: "",
  };
  el("strat-name").value = app.strategy.name;
  el("strat-show").checked = true;
  if (doc.options) applyPlanOptions(doc.options);
  app.obstacles = clampObstacles(doc.obstacles);
  app.goal = doc.goal ? { x: doc.goal.x, y: doc.goal.y, theta: doc.goal.theta_deg ?? null } : null;
  syncGoalUI();
  if (editing) {
    app.items = app.strategy.elements.map((element) => JSON.parse(JSON.stringify(element)));
    app.strategy.elements = [];
    app.strategy.editing = true;
    app.strategy.baseline = JSON.stringify(app.items);
    invalidatePlan();
    renderSequence();
  }
  updateStrategyUI();
  const mirrored = doc.mirrored
    ? ` - authored for ${String(doc.authored_for || "?").toUpperCase()}, MIRRORED for ${String(doc.team || "").toUpperCase()} (symmetric table)`
    : "";
  const placed = doc.teleported && doc.start_pose
    ? ` - robot placed at its recorded start (${Math.round(doc.start_pose[0])}, ${Math.round(doc.start_pose[1])}) mm`
    : "";
  logLine("info", `strategy "${doc.name}" loaded (${(doc.elements || []).length} element(s))${mirrored}${placed}`);
}

el("strat-load").addEventListener("click", async () => {
  const name = el("strat-list").value;
  if (!name) { logLine("warn", "pick a saved strategy first"); return; }
  if (strategyDirty() && !window.confirm("The strategy has unsaved changes - discard them?")) return;
  if (app.strategy.editing && app.items.length &&
      !window.confirm("Stop editing and load another strategy? The edited mission on the map stays as a pending sequence.")) return;
  try {
    const doc = await api(`/strategies/${encodeURIComponent(name)}/load`, "POST");
    await applyStrategyDocument(doc);
  } catch (error) { logLine("error", `load failed: ${error.message}`); }
});

/** Push the plan options of a loaded strategy back into the sliders/checkboxes. */
function applyPlanOptions(options) {
  const setRange = (id, value) => {
    if (value === undefined || value === null) return;
    const input = el(id);
    if (!input) return;
    input.value = value;
    input.dispatchEvent(new Event("input", { bubbles: true })); // refresh the <output>
  };
  setRange("opt-speed", options.speed_mm_s);
  setRange("opt-step", options.goto_step_mm);
  setRange("opt-corner", options.corner_radius_mm);
  setRange("opt-chord", options.node_chord_tol_mm);
  setRange("opt-fit", options.fit_tol_mm);
  const setCheck = (id, value) => {
    if (value !== undefined && value !== null) el(id).checked = Boolean(value);
  };
  setCheck("opt-fit-enabled", options.fit_enabled);
  setCheck("opt-avoid", options.avoid_obstacles);
  setCheck("opt-smooth", options.smooth);
}

async function refreshStrategyList() {
  try {
    const data = await api("/strategies");
    const select = el("strat-list");
    const current = select.value;
    select.innerHTML = "";
    const rows = data.strategies || [];
    if (!rows.length) {
      const option = document.createElement("option");
      option.value = "";
      option.textContent = "(no saved strategies)";
      select.appendChild(option);
      return;
    }
    for (const row of rows) {
      const option = document.createElement("option");
      option.value = row.name;
      option.textContent = row.name;
      select.appendChild(option);
    }
    if (current) select.value = current;
  } catch (error) { logLine("warn", `strategy list: ${error.message}`); }
}

function updateStrategyUI() {
  const editing = app.strategy.editing;
  const count = editing ? app.items.length : app.strategy.elements.length;
  const bits = [];
  if (app.strategy.name) bits.push(`"${app.strategy.name}"`);
  bits.push(editing
    ? `${count} step${count === 1 ? "" : "s"} in the sequence - editing`
    : `${count} committed step${count === 1 ? "" : "s"}`);
  if (app.strategy.team) bits.push(`team ${String(app.strategy.team).toUpperCase()}`);
  if (strategyDirty()) bits.push("unsaved changes");
  else if (app.strategy.savedAt) bits.push(`saved ${new Date(app.strategy.savedAt * 1000).toLocaleTimeString([], { hour12: false })}`);
  el("strat-status").textContent = (count || app.strategy.name)
    ? bits.join(" - ")
    : "No strategy yet - name it, plan & run a step, then Add it here.";
  syncStrategyButtons();
}

/** The elements the strategy card currently MEANS: the edited sequence while
 *  editing, the committed steps otherwise. */
function strategyElements() {
  return app.strategy.editing ? app.items : app.strategy.elements;
}

/** Unsaved changes? While editing, the SEQUENCE is the document: compare it
 *  with the baseline captured at Edit / Load / Save time. */
function strategyDirty() {
  if (app.strategy.editing) return JSON.stringify(app.items) !== app.strategy.baseline;
  return Boolean(app.strategy.dirty);
}

/** Run needs something to run; Edit only makes sense on committed steps. */
function syncStrategyButtons() {
  const run = el("strat-run");
  const edit = el("strat-edit");
  if (run) run.disabled = execActive() || strategyElements().length === 0;
  if (edit) {
    edit.disabled = app.strategy.editing || app.strategy.elements.length === 0;
    edit.title = app.strategy.editing
      ? "Already editing: the strategy lives in the sequence - modify it, then Save"
      : "Move the strategy into the sequence so every tool edits it (draw, erase, waypoints, order); Save writes it back to the file";
  }
}

refreshStrategyList();
updateStrategyUI();

// -------------------------------------------------- field bounds & tools --
/** Half-diagonal of the robot + a small pad: the centre must stay that far
 *  from the border so the body never touches the border line, for ANY heading. */
function legalInset() {
  const size = robotSize();
  return Math.hypot(size.w, size.h) / 2 + 12; // the planner uses the same pad
}

function clampToLegal(wx, wy) {
  const inset = legalInset();
  return [
    clamp(wx, inset, Math.max(inset, app.field.width_mm - inset)),
    clamp(wy, inset, Math.max(inset, app.field.height_mm - inset)),
  ];
}

/** Half-diagonal + the configured obstacle margin: the planner's FULL
 *  clearance for the GOAL (a stop point that may turn in place). Waypoints
 *  use the separate, configurable stop clearance below. */
function obstacleInset() {
  const size = robotSize();
  const margin = Math.max(0, Number(el("robot-margin") && el("robot-margin").value) || 0);
  return Math.hypot(size.w, size.h) / 2 + margin;
}

/** Waypoint stop clearance: half the larger body side plus the configured
 *  extra margin. 0 mm therefore allows a 300 mm robot to stop 150 mm from
 *  an obstacle or the field border. */
function waypointInset() {
  const size = robotSize();
  const margin = Math.max(0, Number(el("robot-wp-margin") && el("robot-wp-margin").value) || 0);
  return Math.max(size.w, size.h) / 2 + margin;
}

function clampToWaypointLegal(wx, wy) {
  const inset = waypointInset();
  return [
    clamp(wx, inset, Math.max(inset, app.field.width_mm - inset)),
    clamp(wy, inset, Math.max(inset, app.field.height_mm - inset)),
  ];
}

/** Clamp a waypoint inside the field AND out of every drawn obstacle's
 *  waypoint clearance. Mirrors the planner's projection; the planner also
 *  checks the rotated body for the arrival heading and may move the point a
 *  little farther (the map follows ``resolved_waypoints``). */
function clampToWaypointSafe(wx, wy) {
  const inset = waypointInset();
  const limit = inset;
  let [x, y] = clampToWaypointLegal(wx, wy);
  for (let pass = 0; pass < 4; pass++) {
    let moved = false;
    for (const obstacle of app.obstacles) {
      if (!obstacle) continue;
      if (obstacle.kind === "rect") {
        const ox = Number(obstacle.x) || 0;
        const oy = Number(obstacle.y) || 0;
        const ow = Math.max(0, Number(obstacle.w) || 0);
        const oh = Math.max(0, Number(obstacle.h) || 0);
        if (!ow || !oh) continue;
        const nx = clamp(x, ox, ox + ow);
        const ny = clamp(y, oy, oy + oh);
        let dx = x - nx;
        let dy = y - ny;
        const distance = Math.hypot(dx, dy);
        if (distance >= limit) continue;
        if (distance > 1e-6) {
          x = nx + (dx / distance) * limit;
          y = ny + (dy / distance) * limit;
        } else {
          const left = x - ox;
          const right = ox + ow - x;
          const bottom = y - oy;
          const top = oy + oh - y;
          const nearest = Math.min(left, right, bottom, top);
          if (nearest === left) x = ox - limit;
          else if (nearest === right) x = ox + ow + limit;
          else if (nearest === bottom) y = oy - limit;
          else y = oy + oh + limit;
        }
        moved = true;
      } else if (obstacle.kind === "circle") {
        const cx = Number(obstacle.x) || 0;
        const cy = Number(obstacle.y) || 0;
        const reach = Math.max(0, Number(obstacle.r) || 0) + limit;
        let dx = x - cx;
        let dy = y - cy;
        const distance = Math.hypot(dx, dy);
        if (distance >= reach) continue;
        if (distance <= 1e-6) { dx = 1; dy = 0; }
        const d = Math.max(distance, 1e-6);
        x = cx + (dx / d) * reach;
        y = cy + (dy / d) * reach;
        moved = true;
      }
    }
    if (!moved) break;
    [x, y] = clampToWaypointLegal(x, y);
  }
  return [x, y];
}

/** Clamp a waypoint/goal inside the field AND out of every drawn obstacle's
 *  full clearance zone. Mirrors the planner's rounded-rect / circle
 *  projection, so the point the UI shows is the point the planner uses. */
function clampToSafe(wx, wy) {
  const inset = legalInset();
  const limit = obstacleInset();
  let [x, y] = clampToLegal(wx, wy);
  for (let pass = 0; pass < 4; pass++) {
    let moved = false;
    for (const obstacle of app.obstacles) {
      if (!obstacle) continue;
      if (obstacle.kind === "rect") {
        const ox = Number(obstacle.x) || 0;
        const oy = Number(obstacle.y) || 0;
        const ow = Math.max(0, Number(obstacle.w) || 0);
        const oh = Math.max(0, Number(obstacle.h) || 0);
        if (!ow || !oh) continue;
        const nx = clamp(x, ox, ox + ow);
        const ny = clamp(y, oy, oy + oh);
        let dx = x - nx;
        let dy = y - ny;
        const distance = Math.hypot(dx, dy);
        if (distance >= limit) continue;
        if (distance > 1e-6) {
          x = nx + (dx / distance) * limit;
          y = ny + (dy / distance) * limit;
        } else {
          // inside the box: leave through the nearest side
          const left = x - ox;
          const right = ox + ow - x;
          const bottom = y - oy;
          const top = oy + oh - y;
          const nearest = Math.min(left, right, bottom, top);
          if (nearest === left) x = ox - limit;
          else if (nearest === right) x = ox + ow + limit;
          else if (nearest === bottom) y = oy - limit;
          else y = oy + oh + limit;
        }
        moved = true;
      } else if (obstacle.kind === "circle") {
        const cx = Number(obstacle.x) || 0;
        const cy = Number(obstacle.y) || 0;
        const reach = Math.max(0, Number(obstacle.r) || 0) + limit;
        let dx = x - cx;
        let dy = y - cy;
        const distance = Math.hypot(dx, dy);
        if (distance >= reach) continue;
        if (distance <= 1e-6) { dx = 1; dy = 0; }
        const d = Math.max(distance, 1e-6);
        x = cx + (dx / d) * reach;
        y = cy + (dy / d) * reach;
        moved = true;
      }
    }
    if (!moved) break;
    x = clamp(x, inset, Math.max(inset, app.field.width_mm - inset));
    y = clamp(y, inset, Math.max(inset, app.field.height_mm - inset));
  }
  return [x, y];
}

// ------------------------------------------------- waypoint editor popup --
let wpPopupIndex = -1;

/** Is the goal marker under a world point (screen tolerance ~14 px)? */
function hitGoal(wx, wy) {
  if (!app.goal) return false;
  return Math.hypot(wx - app.goal.x, wy - app.goal.y) <= 14 / Math.max(0.05, app.view.zoom);
}

/** Index of the waypoint under a world point (screen tolerance ~12 px). */
function hitWaypoint(wx, wy) {
  const tolerance = 12 / Math.max(0.05, app.view.zoom);
  for (let index = app.items.length - 1; index >= 0; index--) {
    const item = app.items[index];
    if (item.kind !== "waypoint") continue;
    if (Math.hypot(item.x - wx, item.y - wy) <= tolerance) return index;
  }
  return -1;
}

function openWaypointPopup(index) {
  const item = app.items[index];
  if (!item || item.kind !== "waypoint") return;
  wpPopupIndex = index;
  el("wp-x").value = String(Math.round(item.x));
  el("wp-y").value = String(Math.round(item.y));
  const hasAngle = item.theta_deg !== null && item.theta_deg !== undefined;
  el("wp-th").value = hasAngle ? String(item.theta_deg) : "";
  el("wp-speed").value = item.speed_mm_s ? String(item.speed_mm_s) : "";
  el("wp-combine").checked = Boolean(item.combine);
  const popup = el("wp-popup");
  popup.classList.remove("hidden");
  layoutWaypointPopup(item);
}

function layoutWaypointPopup(item) {
  const [sx, sy] = worldToScreen(item.x, item.y);
  const { w, h } = cssSize();
  const popup = el("wp-popup");
  popup.style.left = `${clamp(sx + 18, 8, Math.max(8, w - 210))}px`;
  popup.style.top = `${clamp(sy - 20, 8, Math.max(8, h - 250))}px`;
}

function closeWaypointPopup() {
  wpPopupIndex = -1;
  el("wp-popup").classList.add("hidden");
}

el("wp-apply").addEventListener("click", () => {
  const item = app.items[wpPopupIndex];
  if (!item || item.kind !== "waypoint") { closeWaypointPopup(); return; }
  const x = Number(el("wp-x").value);
  const y = Number(el("wp-y").value);
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    logLine("warn", "waypoint X / Y must be numbers");
    return;
  }
  pushHistory();
  const [cx, cy] = clampToWaypointSafe(x, y);
  item.x = cx;
  item.y = cy;
  const theta = el("wp-th").value.trim();
  if (theta === "") {
    delete item.theta_deg;
    delete item.combine;
  } else {
    item.theta_deg = norm180(Number(theta));
    item.combine = el("wp-combine").checked;
  }
  const speed = optionalSpeed("wp-speed");
  if (speed) item.speed_mm_s = speed;
  else delete item.speed_mm_s;
  invalidatePlan();
  renderSequence();
  closeWaypointPopup();
  logLine("info", `waypoint set to (${cx.toFixed(0)}, ${cy.toFixed(0)}) mm` +
    (Math.hypot(cx - x, cy - y) > 1
      ? ` (moved ${Math.hypot(cx - x, cy - y).toFixed(0)} mm out of the field / obstacle clearance)`
      : "") +
    (item.theta_deg !== undefined
      ? `, arrival ${item.theta_deg.toFixed(1)} deg ${item.combine ? "combined with the approach" : "(turn in place at the point)"}`
      : "")
    + (speed ? `, speed ${speed} mm/s` : ""));
});

el("wp-delete").addEventListener("click", () => {
  const item = app.items[wpPopupIndex];
  closeWaypointPopup();
  if (!item || item.kind !== "waypoint") return;
  pushHistory();
  app.items.splice(app.items.indexOf(item), 1);
  invalidatePlan();
  renderSequence();
  logLine("info", "waypoint removed");
});

el("wp-cancel").addEventListener("click", () => closeWaypointPopup());

document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && !el("wp-popup").classList.contains("hidden")) {
    event.stopPropagation();
    closeWaypointPopup();
  }
});

// ------------------------------------------------------ goal editor popup --
let goalPopupReturnFocus = null;

function syncGoalPopupFields() {
  if (!app.goal) return;
  el("goal-x").value = String(Math.round(app.goal.x));
  el("goal-y").value = String(Math.round(app.goal.y));
  const hasAngle = app.goal.theta !== null && app.goal.theta !== undefined;
  el("goal-th").value = hasAngle ? String(Math.round(app.goal.theta)) : "";
}

function layoutGoalPopup() {
  const popup = el("goal-popup");
  const [sx, sy] = worldToScreen(app.goal.x, app.goal.y);
  const { w, h } = cssSize();
  popup.classList.remove("hidden");
  popup.style.left = `${clamp(sx + 18, 8, Math.max(8, w - 210))}px`;
  popup.style.top = `${clamp(sy - 20, 8, Math.max(8, h - 200))}px`;
}

function goalPopupOpen() {
  return Boolean(app.goal) && !el("goal-popup").classList.contains("hidden");
}

function openGoalPopup() {
  if (!app.goal) return;
  goalPopupReturnFocus = document.activeElement;
  syncGoalPopupFields();
  layoutGoalPopup();
  el("goal-x").focus();
  el("goal-x").select();
}

function closeGoalPopup(restoreFocus = false) {
  el("goal-popup").classList.add("hidden");
  if (restoreFocus && goalPopupReturnFocus && goalPopupReturnFocus.focus) {
    goalPopupReturnFocus.focus();
  }
  goalPopupReturnFocus = null;
}

function applyGoalPopup() {
  if (!app.goal) return;
  const x = Number.parseFloat(el("goal-x").value);
  const y = Number.parseFloat(el("goal-y").value);
  const thetaDeg = Number.parseFloat(el("goal-th").value);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return;
  pushHistory();
  app.goal.x = x;
  app.goal.y = y;
  app.goal.theta = Number.isFinite(thetaDeg) ? clamp(norm180(thetaDeg), -180, 180) : null;
  ghostRefresh();
  goalChanged();
  renderSequence();
  closeGoalPopup();
  logLine("info", `goal set → (${x.toFixed(0)}, ${y.toFixed(0)})`);
}

el("goal-apply").addEventListener("click", applyGoalPopup);
el("goal-delete").addEventListener("click", () => {
  if (!app.goal) return;
  pushHistory();
  app.goal = null;
  ghostRefresh();
  goalChanged();
  closeGoalPopup();
  logLine("info", "goal removed");
});
el("goal-cancel").addEventListener("click", () => closeGoalPopup(true));
el("goal-popup").addEventListener("keydown", (event) => {
  if (event.key === "Enter") {
    event.preventDefault();
    applyGoalPopup();
  }
});

document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && !el("goal-popup").classList.contains("hidden")) {
    event.stopPropagation();
    closeGoalPopup(true);
  }
});

// ------------------------------------------------------- ruler & angle ----
function updateMeasureHint() {
  const measure = app.measure;
  if (!measure || !measure.kind) return;
  const points = measure.points;
  if (measure.kind === "ruler") {
    if (points.length < 2) {
      els.toolHint.textContent = "Ruler: click the second point to measure the distance between them.";
      return;
    }
    const distance = Math.hypot(points[0][0] - points[1][0], points[0][1] - points[1][1]);
    els.toolHint.textContent = `Ruler: ${distance.toFixed(0)} mm between the two points (click for a new measurement, or press the tool button again to clear).`;
    return;
  }
  if (points.length === 0) {
    els.toolHint.textContent = "Angle: click the CENTRE point first.";
  } else if (points.length === 1) {
    els.toolHint.textContent = "Angle: click the first ray point.";
  } else if (points.length === 2) {
    els.toolHint.textContent = "Angle: click the second ray point.";
  } else {
    els.toolHint.textContent = `Angle: ${measureAngle(points).toFixed(1)} deg between the rays (click for a new angle).`;
  }
}

function measureAngle(points) {
  const a1 = Math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0]);
  const a2 = Math.atan2(points[2][1] - points[0][1], points[2][0] - points[0][0]);
  return Math.abs(deg(Math.atan2(Math.sin(a2 - a1), Math.cos(a2 - a1))));
}

function measureLabel(x, y, text) {
  const measureColor = app.colors.measure || COLOR_DEFAULTS.measure;
  ctx.save();
  ctx.font = "bold 12px 'Segoe UI', sans-serif";
  const width = ctx.measureText(text).width;
  ctx.fillStyle = "rgba(8, 8, 8, .85)";
  ctx.strokeStyle = hexToRgba(measureColor, .8);
  ctx.lineWidth = 1;
  const left = x - width / 2 - 6;
  const top = y - 10;
  ctx.beginPath();
  ctx.roundRect(left, top, width + 12, 20, 5);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = measureColor;
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  ctx.fillText(text, left + 6, top + 10);
  ctx.restore();
}

function measureMarker(x, y) {
  ctx.beginPath();
  ctx.arc(x, y, 4, 0, Math.PI * 2);
  ctx.fillStyle = hexToRgba(app.colors.measure || COLOR_DEFAULTS.measure, .95);
  ctx.fill();
  ctx.beginPath();
  ctx.arc(x, y, 7, 0, Math.PI * 2);
  ctx.strokeStyle = "rgba(255, 255, 255, .55)";
  ctx.lineWidth = 1.2;
  ctx.stroke();
}

/** Ruler / angle overlays: click-to-measure, values shown on the map. */
function drawMeasure() {
  const measure = app.measure;
  if (!measure || !measure.kind || !measure.points.length) return;
  const points = measure.points;
  const screen = points.map(([x, y]) => worldToScreen(x, y));
  ctx.save();
  ctx.strokeStyle = hexToRgba(app.colors.measure || COLOR_DEFAULTS.measure, .9);
  ctx.lineWidth = 1.6;
  ctx.setLineDash([7, 5]);
  if (measure.kind === "ruler") {
    if (screen.length >= 2) {
      ctx.beginPath();
      ctx.moveTo(screen[0][0], screen[0][1]);
      ctx.lineTo(screen[1][0], screen[1][1]);
      ctx.stroke();
      const distance = Math.hypot(points[0][0] - points[1][0], points[0][1] - points[1][1]);
      measureLabel((screen[0][0] + screen[1][0]) / 2, (screen[0][1] + screen[1][1]) / 2, `${distance.toFixed(0)} mm`);
    }
  } else if (measure.kind === "angle") {
    if (screen.length >= 2) {
      ctx.beginPath();
      ctx.moveTo(screen[0][0], screen[0][1]);
      ctx.lineTo(screen[1][0], screen[1][1]);
      ctx.stroke();
    }
    if (screen.length >= 3) {
      ctx.beginPath();
      ctx.moveTo(screen[0][0], screen[0][1]);
      ctx.lineTo(screen[2][0], screen[2][1]);
      ctx.stroke();
      const centre = screen[0];
      let a1 = Math.atan2(screen[1][1] - centre[1], screen[1][0] - centre[0]);
      let a2 = Math.atan2(screen[2][1] - centre[1], screen[2][0] - centre[0]);
      const delta = ((a2 - a1 + Math.PI * 3) % (Math.PI * 2)) - Math.PI;
      ctx.setLineDash([]);
      ctx.beginPath();
      ctx.arc(centre[0], centre[1], 42, a1, a1 + delta, delta < 0);
      ctx.stroke();
      const bisector = a1 + delta / 2;
      measureLabel(
        centre[0] + Math.cos(bisector) * 66,
        centre[1] + Math.sin(bisector) * 66,
        `${measureAngle(points).toFixed(1)} deg`,
      );
    }
  }
  ctx.setLineDash([]);
  for (const point of screen) measureMarker(point[0], point[1]);
  ctx.restore();
}

async function padMove(kind, arg) {
  const speed = clamp(Number(el("opt-speed").value), 100, 900);
  try {
    if (kind === "drive") await api("/move/drive", "POST", { distance_mm: arg, speed_mm_s: speed });
    else if (kind === "turn") await api("/move/turn", "POST", { angle_deg: arg, speed_mm_s: Math.min(speed, 150) });
    // board rule: radius and sweep signs must agree (+ = left/CCW, - = right/CW);
    // the Speed slider governs the arc cruise (the board auto-caps on small radii)
    else if (kind === "arc") await api("/move/arc", "POST", { radius_mm: arg < 0 ? -300 : 300, sweep_deg: arg, speed_mm_s: speed });
    logLine("info", `${kind} ${arg > 0 ? "+" : ""}${arg} sent`);
  } catch (error) { logLine("warn", error.message); }
}

function padDistance() { return clamp(Math.round(Number(el("pad-dist").value) || 100), 1, 3000); }

function padTurn() { return clamp(Number(el("pad-turn").value) || 90, -180, 180); }

/** The caption under each pad icon carries the value typed above. */
function padLabels() {
  const distance = padDistance();
  const angle = padTurn();
  const cap = (id, text) => {
    const label = el(id).querySelector(".pad-cap");
    if (label) label.textContent = text;
  };
  cap("pad-forward", `+${distance} mm`);
  cap("pad-back", `-${distance} mm`);
  cap("pad-turn-ccw", `+${angle}°`);
  cap("pad-turn-cw", `-${angle}°`);
}

el("pad-forward").addEventListener("click", () => padMove("drive", padDistance()));
el("pad-back").addEventListener("click", () => padMove("drive", -padDistance()));
el("pad-turn-ccw").addEventListener("click", () => padMove("turn", padTurn()));
el("pad-turn-cw").addEventListener("click", () => padMove("turn", -padTurn()));
document.querySelectorAll("[data-move='arc']").forEach((button) => {
  button.addEventListener("click", () => padMove("arc", Number(button.dataset.arg)));
});
["pad-dist", "pad-turn"].forEach((id) => el(id).addEventListener("input", padLabels));
padLabels();

// -------------------------------------------------------------- sequence --
function pushHistory() {
  app.history.push(JSON.stringify(app.items));
  if (app.history.length > 50) app.history.shift();
  app.redo = [];
}

function lastEndPoint() {
  for (let i = app.items.length - 1; i >= 0; i--) {
    const item = app.items[i];
    if (item.kind === "stroke" && item.points.length) return item.points[item.points.length - 1];
    if (item.kind === "waypoint") return [item.x, item.y];
  }
  return null;
}

/** Where a new path should start: the previous path end, else the robot. */
function snapAnchor() {
  const end = lastEndPoint();
  if (end) return { point: end, kind: "path" };
  if (app.telemetry) return { point: [app.telemetry.x_mm, app.telemetry.y_mm], kind: "robot" };
  return null;
}

function invalidatePlan() {
  app.planElements = null;
  if (app.plan) {
    app.plan = null;
    els.execBtn.disabled = true;
    els.stats.textContent = "Plan out of date - press Plan.";
  }
}

/** Obstacles are dynamic: if a plan is on screen, recompute it at once so
 *  the displayed path always respects the current obstacle map. During an
 *  execution the robot stops, re-routes the REMAINING mission around the new
 *  obstacle and continues (the backend swaps the plan). */
function obstaclesChanged() {
  const had = Boolean(app.plan);
  app.obstacles = clampObstacles(app.obstacles);
  invalidatePlan();
  if (!had) return;
  if (execActive()) {
    logLine("warn", "obstacle change during execution - re-checking the remaining route (the robot only stops and re-routes if that route is affected)");
  } else {
    logLine("info", "obstacles changed - re-planning");
  }
  planNow();
}

/** Same for a goal change (position or arrival heading). */
function goalChanged() {
  const had = Boolean(app.plan);
  invalidatePlan();
  if (!had) return;
  if (execActive()) logLine("warn", "goal moved during execution - re-planning the remaining route to the new goal");
  planNow();
}

function nearestPointOnItems(wx, wy) {
  let best = null;
  for (const item of app.items) {
    if (item.kind !== "stroke") continue;
    for (let i = 1; i < item.points.length; i++) {
      const [ax, ay] = item.points[i - 1];
      const [bx, by] = item.points[i];
      const dx = bx - ax;
      const dy = by - ay;
      const len2 = dx * dx + dy * dy;
      const t = len2 > 0 ? clamp(((wx - ax) * dx + (wy - ay) * dy) / len2, 0, 1) : 0;
      const px = ax + dx * t;
      const py = ay + dy * t;
      const d = Math.hypot(wx - px, wy - py);
      if (!best || d < best.d) best = { d, x: px, y: py, item, index: i };
    }
  }
  return best;
}

function itemDescription(item) {
  if (item.kind === "stroke") {
    let length = 0;
    for (let i = 1; i < item.points.length; i++) {
      length += Math.hypot(item.points[i][0] - item.points[i - 1][0], item.points[i][1] - item.points[i - 1][1]);
    }
    return ` path - ${item.points.length} pts - ${(length / 1000).toFixed(2)} m`;
  }
  if (item.kind === "rotate") {
    return ` turn ${item.angle_deg > 0 ? "+" : ""}${item.angle_deg} deg (${item.angle_deg > 0 ? "CCW" : "CW"})`;
  }
  if (item.kind === "pause") return ` delay ${item.seconds} s`;
  if (item.kind === "release") return " release motors (drivers off - next move re-energizes)";
  if (item.kind === "drive") {
    const distance = Number(item.distance_mm);
    return distance < 0
      ? ` backward ${Math.abs(distance)} mm (straight, current heading)`
      : ` forward ${distance} mm (straight, current heading)`;
  }
  if (item.kind === "waypoint") {
    const angle = item.theta_deg !== null && item.theta_deg !== undefined
      ? ` - arrival ${item.theta_deg} deg${item.combine ? " (combined)" : " (turn in place)"}`
      : "";
    return ` waypoint (${item.x.toFixed(0)}, ${item.y.toFixed(0)})${angle}`;
  }
  return item.kind;
}

/** Every value written in the sequence is editable in place: rotation angle,
 *  pause seconds and waypoint coordinates (handy to fine-tune a snapped point). */
function seqNumberInput(value, stepValue, title, apply) {
  const input = document.createElement("input");
  input.type = "number";
  input.className = "seq-edit";
  input.step = String(stepValue);
  input.title = title;
  input.value = String(value);
  input.addEventListener("change", () => {
    const parsed = Number(input.value);
    if (!Number.isFinite(parsed)) {
      input.value = String(value);
      return;
    }
    apply(parsed);
  });
  return input;
}

/** Optional per-step speed override: empty = the system speed.
 *  Only the step carrying it changes pace (e.g. creep up to a point, then
 *  the rest of the mission runs at the system speed again). */
function seqSpeedInput(item) {
  const wrap = document.createElement("span");
  wrap.className = "seq-speed";
  const input = document.createElement("input");
  input.type = "number";
  input.className = "seq-edit";
  input.step = "10";
  input.min = "100";
  input.max = "900";
  input.placeholder = "sys";
  input.title = "speed for THIS step only (mm/s) - empty = system speed";
  if (item.speed_mm_s) input.value = String(item.speed_mm_s);
  input.addEventListener("change", () => {
    const value = Number(input.value);
    pushHistory();
    if (!Number.isFinite(value) || value <= 0) {
      delete item.speed_mm_s;
      input.value = "";
    } else {
      item.speed_mm_s = clamp(Math.round(value), 100, 900);
      input.value = String(item.speed_mm_s);
    }
    invalidatePlan();
  });
  wrap.append(" @", input, " mm/s");
  return wrap;
}

/** One labeled axis of a waypoint (X / Y). Keeping each axis in its own
 *  inline box lets narrow panels wrap cleanly between X and Y. */
function seqAxisField(labelText, value, stepValue, title, apply) {
  const field = document.createElement("span");
  field.className = "seq-axis";
  const label = document.createElement("span");
  label.className = "seq-axis-label";
  label.textContent = labelText;
  field.append(label, seqNumberInput(value, stepValue, title, apply));
  return field;
}

function buildSeqEditor(desc, item, index) {
  if (item.kind === "rotate") {
    desc.append("CCW ");
    desc.appendChild(seqNumberInput(item.angle_deg, 5, "rotation angle (deg)", (value) => {
      if (value === 0) return;
      pushHistory();
      item.angle_deg = value;
      invalidatePlan();
      ghostAnimateRotation(index);   // show the shadow turning the new angle
    }));
    desc.append("\u00b0");
    desc.appendChild(seqSpeedInput(item));
    return;
  }
  if (item.kind === "pause") {
    desc.append(" ");
    desc.appendChild(seqNumberInput(item.seconds, 0.5, "pause (s)", (value) => {
      if (value <= 0) return;
      pushHistory();
      item.seconds = value;
      invalidatePlan();
      ghostRefresh();
    }));
    desc.append(" s");
    return;
  }
  if (item.kind === "waypoint") {
    desc.appendChild(seqAxisField("X", Math.round(item.x), 10, "waypoint X (mm)", (value) => {
      pushHistory();
      const [nx, ny] = clampToWaypointSafe(value, item.y);
      item.x = nx;
      item.y = ny;
      invalidatePlan();
      ghostRefresh();
    }));
    desc.appendChild(seqAxisField("Y", Math.round(item.y), 10, "waypoint Y (mm)", (value) => {
      pushHistory();
      const [nx, ny] = clampToWaypointSafe(item.x, value);
      item.x = nx;
      item.y = ny;
      invalidatePlan();
      ghostRefresh();
    }));
    desc.appendChild(seqSpeedInput(item));
    return;
  }
  if (item.kind === "stroke" || item.kind === "drive") {
    // paths and advances accept a per-step speed the same way rotations do;
    // the description text shrinks first so the speed input stays visible
    const txt = document.createElement("span");
    txt.className = "txt";
    txt.textContent = itemDescription(item);
    txt.title = itemDescription(item);   // full text when the row truncates
    desc.appendChild(txt);
    desc.appendChild(seqSpeedInput(item));
    return;
  }
  desc.textContent = itemDescription(item);
}

// ------------------------------------------------------ goal editor state --
function syncGoalUI() {
  const popup = el("goal-popup");
  if (!app.goal) {
    popup.classList.add("hidden");
  } else if (!popup.classList.contains("hidden")) {
    syncGoalPopupFields();
    layoutGoalPopup();
  }
}

function renderSequence() {
  els.seqList.innerHTML = "";
  if (!app.items.length) {
    const empty = document.createElement("div");
    empty.className = "seq-empty";
    empty.textContent = "No steps yet - draw a path or use Delay / Turn / Forward / Back / Release.";
    els.seqList.appendChild(empty);
    return;
  }
  app.items.forEach((item, index) => {
    const row = document.createElement("div");
    row.className = "seq-item";
    const idx = document.createElement("span");
    idx.className = "idx";
    idx.textContent = `${index + 1}.`;
    const desc = document.createElement("span");
    desc.className = "desc";
    buildSeqEditor(desc, item, index);
    const tools = document.createElement("span");
    tools.className = "seq-tools";
    const up = document.createElement("button");
    up.className = "move";
    up.innerHTML = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M5.5 14.5 12 8l6.5 6.5"/></svg>';
    up.title = "Move up";
    up.addEventListener("click", () => {
      if (index === 0) return;
      pushHistory();
      app.items.splice(index - 1, 0, app.items.splice(index, 1)[0]);
      invalidatePlan();
      renderSequence();
    });
    const del = document.createElement("button");
    del.className = "del";
    del.innerHTML = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M6 6l12 12M18 6L6 18"/></svg>';
    del.title = "Delete step";
    del.addEventListener("click", () => {
      pushHistory();
      app.items.splice(index, 1);
      invalidatePlan();
      renderSequence();
      logLine("info", `step ${index + 1} deleted`);
    });
    tools.append(up, del);
    row.append(idx, desc, tools);
    els.seqList.appendChild(row);
  });
  ghostRefresh();
  if (app.strategy.editing) updateStrategyUI(); // the sequence IS the document now
}

el("btn-add-rotate").addEventListener("click", () => {
  const angle = Number(el("seq-angle").value);
  if (!Number.isFinite(angle) || angle === 0) {
    logLine("warn", "rotation angle must be a non-zero number");
    return;
  }
  const speed = Number(el("seq-turn-speed").value);
  const step = { kind: "rotate", angle_deg: angle };
  if (Number.isFinite(speed) && speed > 0) step.speed_mm_s = clamp(Math.round(speed), 100, 900);
  pushHistory();
  app.items.push(step);
  invalidatePlan();
  renderSequence();
  ghostAnimateRotation(app.items.length - 1);
  logLine("info", `rotation ${angle > 0 ? "+" : ""}${angle} deg added`
    + (step.speed_mm_s ? ` at ${step.speed_mm_s} mm/s` : ""));
  el("seq-angle").focus();
  el("seq-angle").select();
});

el("btn-add-pause").addEventListener("click", () => {
  const seconds = Number(el("seq-pause").value);
  if (!Number.isFinite(seconds) || seconds <= 0) {
    logLine("warn", "pause must be a positive number of seconds");
    return;
  }
  pushHistory();
  app.items.push({ kind: "pause", seconds });
  invalidatePlan();
  renderSequence();
  logLine("info", `pause ${seconds} s added`);
  el("seq-pause").focus();
  el("seq-pause").select();
});

el("btn-add-drive").addEventListener("click", () => {
  const magnitude = Math.abs(Number(el("seq-advance").value));
  if (!Number.isFinite(magnitude) || magnitude < 0.5) {
    logLine("warn", "move distance must be a non-zero number of mm");
    return;
  }
  const distance = magnitude * driveToolSign;
  const speed = Number(el("seq-advance-speed").value);
  const step = { kind: "drive", distance_mm: distance };
  if (Number.isFinite(speed) && speed > 0) step.speed_mm_s = clamp(Math.round(speed), 100, 900);
  pushHistory();
  app.items.push(step);
  invalidatePlan();
  renderSequence();
  logLine("info", `${distance < 0 ? "backward" : "forward"} ${magnitude} mm added`
    + (step.speed_mm_s ? ` at ${step.speed_mm_s} mm/s` : "")
    + " - straight along the heading held at that point");
  el("seq-advance").focus();
  el("seq-advance").select();
});

el("btn-undo").addEventListener("click", () => {
  const snapshot = app.history.pop();
  if (!snapshot) {
    logLine("warn", "nothing to undo");
    return;
  }
  app.redo.push(JSON.stringify(app.items));
  if (app.redo.length > 50) app.redo.shift();
  app.items = JSON.parse(snapshot);
  invalidatePlan();
  renderSequence();
  logLine("info", "undo");
});

el("btn-redo").addEventListener("click", () => {
  const snapshot = app.redo.pop();
  if (!snapshot) {
    logLine("warn", "nothing to redo");
    return;
  }
  app.history.push(JSON.stringify(app.items));
  if (app.history.length > 50) app.history.shift();
  app.items = JSON.parse(snapshot);
  invalidatePlan();
  renderSequence();
  logLine("info", "redo");
});

// ----------------------------------------------------------- I2C monitor --
function formatTraceTime(t) {
  if (typeof t === "number" && Number.isFinite(t)) {
    const date = new Date(t);
    const pad = (value, size = 2) => String(value).padStart(size, "0");
    return `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}.${pad(date.getMilliseconds(), 3)}`;
  }
  return String(t || "").slice(0, 23);
}

function i2cAppend(entry, countIt = true) {
  if (countIt) {
    app.i2c.count += 1;
    els.i2cCount.textContent = `${app.i2c.count} frames`;
  }
  if (app.i2c.paused) return;
  const line = document.createElement("div");
  line.className = `entry ${entry.dir}`;
  const head = document.createElement("span");
  head.className = "head";
  const status = entry.status_name || entry.note || "";
  head.textContent =
    `${formatTraceTime(entry.t)} ${(entry.dir || "").toUpperCase().padEnd(2)} ` +
    `#${entry.seq} ${entry.cmd_name || "?"} ${status}`.trimEnd() + (entry.payload ? " " : "");
  const payload = document.createElement("span");
  payload.className = "payload";
  payload.textContent = entry.payload || "";
  line.append(head, payload);
  els.i2cBody.appendChild(line);
  while (els.i2cBody.childElementCount > 400) els.i2cBody.firstChild.remove();
  els.i2cBody.scrollTop = els.i2cBody.scrollHeight;
}

function applyI2cChrome() {
  els.i2c.classList.toggle("collapsed", !app.i2c.open);
  el("i2c-toggle").textContent = app.i2c.open ? "v Minimize" : "^ Open";
  el("i2c-pause").textContent = app.i2c.paused ? "> Follow" : "|| Pause view";
  setTimeout(resizeCanvas, 220); // let the canvas reclaim the freed space
}

el("i2c-toggle").addEventListener("click", () => {
  app.i2c.open = !app.i2c.open;
  applyI2cChrome();
});
el("i2c-pause").addEventListener("click", () => {
  app.i2c.paused = !app.i2c.paused;
  applyI2cChrome();
});
el("i2c-clear").addEventListener("click", async () => {
  try {
    await api("/trace/clear", "POST");
    els.i2cBody.innerHTML = "";
    app.i2c.count = 0;
    els.i2cCount.textContent = "0 frames";
  } catch (error) { logLine("warn", error.message); }
});
el("i2c-save").addEventListener("click", () => {
  const link = document.createElement("a");
  link.href = "/api/trace/download";
  link.download = "";
  document.body.appendChild(link);
  link.click();
  link.remove();
  logLine("info", "I2C log download started");
});

// the log overlay can cover the field: the little corner button collapses it
// to a slim bar (the choice persists across reloads)
function setLogCollapsed(collapsed) {
  els.logPanel.classList.toggle("collapsed", collapsed);
  try {
    localStorage.setItem("charara.log.collapsed", collapsed ? "1" : "0");
  } catch (error) { /* storage unavailable: just do not persist */ }
}
el("log-toggle").addEventListener("click", () => {
  setLogCollapsed(!els.logPanel.classList.contains("collapsed"));
});
setLogCollapsed((() => {
  try {
    return localStorage.getItem("charara.log.collapsed") === "1";
  } catch (error) {
    return false;
  }
})());

// the pose readout can shrink to one line: the velocities are useful while
// tuning, but on a busy table the block hid the corner of the field
function setTelemetryCompact(compact) {
  els.telemetry.classList.toggle("compact", compact);
  els.telemetryToggle.title = compact
    ? "Expand the pose readout (show wheel velocities)"
    : "Collapse the pose readout to one line";
  els.telemetryToggle.setAttribute("aria-pressed", compact ? "true" : "false");
  try {
    localStorage.setItem("charara.telemetry.compact", compact ? "1" : "0");
  } catch (error) { /* storage unavailable: just do not persist */ }
}
els.telemetryToggle.addEventListener("click", () => {
  setTelemetryCompact(!els.telemetry.classList.contains("compact"));
});
setTelemetryCompact((() => {
  try {
    return localStorage.getItem("charara.telemetry.compact") !== "0";
  } catch (error) {
    return true; // default compact: keep the table visible
  }
})());

el("btn-field").addEventListener("click", async () => {
  try {
    const result = await api("/field", "POST", {
      width_mm: Number(el("field-w").value),
      height_mm: Number(el("field-h").value),
    });
    app.field = result.field;
    fitView();
    logLine("info", `field set to ${app.field.width_mm} x ${app.field.height_mm} mm`);
  } catch (error) { logLine("error", error.message); }
});

el("btn-clear-path").addEventListener("click", () => {
  pushHistory();
  app.items = [];
  app.plan = null;
  els.execBtn.disabled = true;
  els.stats.textContent = "No plan yet.";
  renderSequence();
});
el("btn-clear-obs").addEventListener("click", () => { app.obstacles = []; obstaclesChanged(); });
el("btn-view").addEventListener("click", fitView);
el("btn-fit").addEventListener("click", () => {
  fitView();
  logLine("info", "view fitted: the whole field is on screen");
});

// ------------------------------------------------- tabs & robot geometry --
const settingsDock = el("settings-dock");
const settingsDrawer = el("settings-drawer");
const settingsTitle = el("settings-title");
const settingsTabs = [...document.querySelectorAll(".settings-tab")];
const SETTINGS_TITLES = {
  robot: "Robot configuration",
  ui: "Map & display",
  calibration: "Calibration",
  tests: "Tests & diagnostics",
};

function closeSettings(restoreFocus = false) {
  const previous = settingsDock.dataset.active;
  settingsDock.classList.remove("open");
  settingsDock.dataset.active = "";
  settingsDrawer.setAttribute("aria-hidden", "true");
  settingsDrawer.inert = true;
  settingsTabs.forEach((tab) => {
    tab.classList.remove("active");
    tab.setAttribute("aria-expanded", "false");
    tab.setAttribute("aria-selected", "false");
    tab.tabIndex = tab.dataset.settings === previous ? 0 : -1;
  });
  document.querySelectorAll(".settings-page").forEach((page) => {
    page.hidden = true;
    page.classList.add("hidden");
  });
  if (restoreFocus) settingsTabs.find((tab) => tab.dataset.settings === previous)?.focus();
}

function openSettings(name, focus = false) {
  const wasOpen = settingsDock.classList.contains("open");
  if (wasOpen && settingsDock.dataset.active === name) {
    if (focus) settingsTabs.find((tab) => tab.dataset.settings === name)?.focus();
    return;
  }
  settingsDock.classList.add("open");
  settingsDock.dataset.active = name;
  settingsDrawer.setAttribute("aria-hidden", "false");
  settingsDrawer.inert = false;
  settingsTitle.textContent = SETTINGS_TITLES[name] || "Settings";
  settingsTabs.forEach((tab) => {
    const active = tab.dataset.settings === name;
    tab.classList.toggle("active", active);
    tab.setAttribute("aria-expanded", String(active));
    tab.setAttribute("aria-selected", String(active));
    tab.tabIndex = active ? 0 : -1;
  });
  document.querySelectorAll(".settings-page").forEach((page) => {
    const active = page.id === `settings-${name}`;
    page.hidden = !active;
    page.classList.toggle("hidden", !active);
  });
  if (focus) settingsTabs.find((tab) => tab.dataset.settings === name)?.focus();
  if (name === "robot" || name === "calibration") calibLoad().catch(() => {});
}

settingsTabs.forEach((tab, index) => {
  tab.addEventListener("click", () => openSettings(tab.dataset.settings));
  tab.addEventListener("keydown", (event) => {
    if (!["ArrowUp", "ArrowDown", "Home", "End"].includes(event.key)) return;
    event.preventDefault();
    const next = event.key === "Home" ? 0 : event.key === "End" ? settingsTabs.length - 1
      : (index + (event.key === "ArrowDown" ? 1 : -1) + settingsTabs.length) % settingsTabs.length;
    openSettings(settingsTabs[next].dataset.settings, true);
  });
});
el("settings-close").addEventListener("click", () => closeSettings(true));

// ------------------------------------------------------------ calibration --
/** Board register settings, calibration procedures, and bench diagnostics.
 *
 *  The register map comes from GET /config/robot. Mask registers carry a
 *  ``bits`` list (bit number, column label, hint, tooltip): the UI shows one
 *  0/1 control per bit, loaded from the board (which reads its flash record
 *  at boot), so every motor and every encoder can be inverted individually -
 *  the bit of "FL" is motor ID 2, "FR" is 1, "RL" is 3 and "RR" is 0 (the
 *  wheel-to-motor mapping comes from the firmware robot_configs.h).
 *
 *  Every group has its own action bar: Apply (green triangle - write the
 *  group to the board RAM), Save (disk - persist the RAM image to the STM
 *  flash) and Restore (undo arrow - re-read that group from the board and
 *  drop its local edits). The board-level "Reload from flash" replaces the
 *  whole RAM image with the saved flash record; "Defaults" restores the
 *  compile-time values in RAM.
 *
 *  Corrections are plain ratios computed from the operator's tape
 *  measurement and the board-reported odometry of the test move:
 *    wheel size   := old * measured / commanded      (open-loop distance)
 *    odom wheel   := old * measured / board-reported (reported distance)
 *    drive track  := old * commanded / measured      (open-loop rotation)
 *    odom track   := old * board-reported / measured (reported heading)
 */

const CALIB_TITLES = {
  info: "identity", stepper_wheels: "stepper wheels", geometry: "geometry",
  odom_encoders: "odometry encoders", limits: "motion limits", gains: "control gains",
  flags: "direction & strafe flags",
};

/** Flags group: motor + encoder inversions shown before the strafe pattern. */
const CALIB_FLAG_ORDER = ["steer_invert_mask", "encoder_invert_mask", "strafe_pattern"];

/** Inline icons of the per-group action bars (green apply / disk / restore). */
const CALIB_ICONS = {
  apply: '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M4.2 2.6v10.8L13 8z" fill="currentColor"/></svg>',
  save: '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M2.6 1.6h8.3L14.4 5v9.4h-11.8z" fill="none" stroke="currentColor" stroke-width="1.3"/><path d="M5.3 1.6h4.2v3.6H5.3z" fill="currentColor"/><rect x="4.7" y="8.9" width="6.6" height="5.1" rx=".6" fill="currentColor"/></svg>',
  restore: '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M13.2 8A5.2 5.2 0 1 1 8 2.8c1.7 0 3.2.8 4.1 2" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round"/><path d="M12.9 1.3v4.3H8.6z" fill="currentColor"/></svg>',
};

function calibGroupTitle(group) {
  return CALIB_TITLES[group] || group;
}

function calibSetNote(message) {
  if (els.calibNote) els.calibNote.textContent = message;
}

async function calibLoad() {
  const data = await api("/config/robot");
  app.calib = data;
  renderCalibForm();
  els.calibStatus.textContent = `- map v${data.map_version}${data.dirty ? " - unsaved changes in board RAM" : ""}`;
}

/** Value of one register as last read from the board (null when unknown). */
function calibBoardValue(key) {
  const register = app.calib && app.calib.registers.find((item) => item.key === key);
  return register ? Number(register.value) : null;
}

/** Writable registers grouped in map order (read-only ones are skipped: the
 *  map version already shows in the card title). */
function calibGroupList() {
  const groups = [];
  const byGroup = new Map();
  for (const register of (app.calib && app.calib.registers) || []) {
    if (register.read_only) continue;
    if (!byGroup.has(register.group)) {
      byGroup.set(register.group, []);
      groups.push([register.group, byGroup.get(register.group)]);
    }
    byGroup.get(register.group).push(register);
  }
  return groups;
}

/** One numeric register: label above, input + unit below (units aligned). */
function buildCalibNumberField(register) {
  const field = document.createElement("label");
  field.className = "calib-field";
  const name = document.createElement("span");
  name.textContent = register.label;
  const inputRow = document.createElement("div");
  inputRow.className = "calib-input-row";
  const input = document.createElement("input");
  input.type = "number";
  input.value = String(register.value);
  input.step = register.step;
  input.min = register.min;
  input.max = register.max;
  input.dataset.key = register.key;
  input.title = `${register.label} - allowed ${register.min}..${register.max}` +
    (register.unit ? ` ${register.unit}` : "");
  input.addEventListener("change", () => { // clamp after typing, before Apply
    const value = Number(input.value);
    input.value = String(Number.isFinite(value)
      ? clamp(value, Number(input.min), Number(input.max))
      : register.value);
  });
  inputRow.appendChild(input);
  const unit = document.createElement("span");
  unit.className = "unit";
  unit.textContent = register.unit || ""; // always reserve the column:
  inputRow.appendChild(unit);          // every input gets the same width
  field.append(name, inputRow);
  return field;
}

/** One mask register: a column per bit, each an independent 0/1 control.
 *  The board value is split with the metadata bit numbers, so per-wheel and
 *  per-encoder inversions always land on the right bit. */
function buildCalibBitBlock(register) {
  const block = document.createElement("div");
  block.className = "calib-bits-block";
  block.dataset.key = register.key;
  const head = document.createElement("div");
  head.className = "calib-bits-head";
  const title = document.createElement("span");
  title.className = "calib-bits-title";
  title.textContent = register.label;
  const hint = document.createElement("span");
  hint.className = "calib-bits-hint";
  hint.textContent = register.bits_hint || "0 = normal, 1 = inverted";
  head.append(title, hint);
  block.appendChild(head);
  const row = document.createElement("div");
  row.className = "calib-bits";
  const value = Number(register.value) || 0;
  for (const spec of register.bits) {
    const column = document.createElement("label");
    column.className = "bit-col";
    column.title = spec.tooltip || `${register.label}: ${spec.label}`;
    const name = document.createElement("span");
    name.className = "bit-name";
    name.textContent = spec.label;
    const select = document.createElement("select");
    select.className = "bit-select";
    select.dataset.key = register.key;
    select.dataset.bit = String(spec.bit);
    select.setAttribute("aria-label", column.title);
    for (const optionValue of ["0", "1"]) {
      const option = document.createElement("option");
      option.value = optionValue;
      option.textContent = optionValue;
      select.appendChild(option);
    }
    select.value = String((value >> spec.bit) & 1);
    const caption = document.createElement("span");
    caption.className = "bit-hint";
    caption.textContent = spec.hint || "";
    column.append(name, select, caption);
    row.appendChild(column);
  }
  block.appendChild(row);
  return block;
}

/** One settings group: header with its Apply / Save / Restore icon bar. */
function buildCalibGroup(group, registers) {
  const wrap = document.createElement("div");
  wrap.className = "calib-group";
  wrap.dataset.group = group;
  const head = document.createElement("div");
  head.className = "calib-group-head";
  const title = document.createElement("h4");
  title.textContent = calibGroupTitle(group);
  head.appendChild(title);
  const actions = document.createElement("div");
  actions.className = "calib-actions";
  const what = calibGroupTitle(group);
  const buttons = [
    ["apply", `Apply ${what} to the board (RAM only)`, CALIB_ICONS.apply],
    ["save", "Save the board settings to STM flash (the whole RAM image is persisted)", CALIB_ICONS.save],
    ["restore", `Restore ${what} from the board (discard its local edits)`, CALIB_ICONS.restore],
  ];
  for (const [action, tooltip, icon] of buttons) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = `icon-btn ${action}`;
    button.dataset.group = group;
    button.dataset.action = action;
    button.title = tooltip;
    button.innerHTML = icon;
    actions.appendChild(button);
  }
  head.appendChild(actions);
  wrap.appendChild(head);
  const body = document.createElement("div");
  body.className = "calib-group-body";
  const ordered = group === "flags"
    ? [...registers].sort((a, b) => CALIB_FLAG_ORDER.indexOf(a.key) - CALIB_FLAG_ORDER.indexOf(b.key))
    : registers;
  for (const register of ordered) {
    body.appendChild(register.bits.length
      ? buildCalibBitBlock(register)
      : buildCalibNumberField(register));
  }
  wrap.appendChild(body);
  return wrap;
}

function renderCalibForm() {
  if (!app.calib || !els.calibForm) return;
  els.calibForm.replaceChildren();
  for (const [group, registers] of calibGroupList()) {
    els.calibForm.appendChild(buildCalibGroup(group, registers));
  }
}

/** Rebuild one group from the last board read (per-group Restore). */
function renderCalibGroup(group) {
  if (!app.calib || !els.calibForm) return;
  const entry = calibGroupList().find(([name]) => name === group);
  if (!entry) return;
  const rebuilt = buildCalibGroup(group, entry[1]);
  const current = els.calibForm.querySelector(`.calib-group[data-group="${group}"]`);
  if (current) current.replaceWith(rebuilt);
  else els.calibForm.appendChild(rebuilt);
}

function calibFieldValue(key) {
  const input = els.calibForm.querySelector(`input[data-key="${key}"]`);
  return input ? Number(input.value) : null;
}

/** Engineering values of one group as they currently stand in its controls:
 *  numeric inputs by key, mask registers re-assembled from the 0/1 columns. */
function calibCollectGroup(group) {
  const block = els.calibForm.querySelector(`.calib-group[data-group="${group}"]`);
  const values = {};
  if (!block) return values;
  block.querySelectorAll("input[data-key]").forEach((input) => {
    const value = Number(input.value);
    if (Number.isFinite(value)) values[input.dataset.key] = value;
  });
  block.querySelectorAll(".calib-bits-block").forEach((bitsBlock) => {
    let mask = 0;
    bitsBlock.querySelectorAll("select[data-bit]").forEach((select) => {
      if (select.value === "1") mask |= 1 << Number(select.dataset.bit);
    });
    values[bitsBlock.dataset.key] = mask;
  });
  return values;
}

/** True when a group's controls differ from the values last read from the board. */
function calibGroupDirty(group) {
  const current = calibCollectGroup(group);
  return Object.keys(current).some((key) => Number(current[key]) !== calibBoardValue(key));
}

function calibMarkDirty(target) {
  const block = target.closest ? target.closest(".calib-group") : null;
  if (block) block.classList.toggle("dirty", calibGroupDirty(block.dataset.group));
}

async function calibApplyGroup(group) {
  const values = calibCollectGroup(group);
  if (!Object.keys(values).length) return;
  try {
    const result = await api("/config/robot", "POST", { values });
    await calibLoad(); // the board is the source of truth after a write
    const count = Object.keys(result.applied || values).length;
    calibSetNote(`${calibGroupTitle(group)}: applied ${count} register(s) to the board RAM - press Save to flash to keep`);
    logLine("info", `calibration: ${calibGroupTitle(group)} applied to the board (RAM)`);
  } catch (error) {
    calibSetNote(`${calibGroupTitle(group)}: apply failed - ${error.message}`);
    logLine("error", `calibration: ${error.message}`);
  }
}

async function calibSaveFlash(group) {
  try {
    await api("/config/robot/save", "POST");
    await calibLoad();
    calibSetNote(`saved to the STM flash (from ${calibGroupTitle(group)}) - the board reloads these values at every boot`);
    logLine("info", "calibration: board settings saved to flash");
  } catch (error) {
    calibSetNote(`save failed - ${error.message}`);
    logLine("error", `calibration: ${error.message}`);
  }
}

els.calibForm.addEventListener("input", (event) => calibMarkDirty(event.target));
els.calibForm.addEventListener("change", (event) => calibMarkDirty(event.target));
els.calibForm.addEventListener("click", (event) => {
  const button = event.target.closest("button[data-action]");
  if (!button) return;
  const group = button.dataset.group;
  if (button.dataset.action === "apply") calibApplyGroup(group);
  else if (button.dataset.action === "save") calibSaveFlash(group);
  else if (button.dataset.action === "restore") {
    renderCalibGroup(group);
    calibSetNote(`${calibGroupTitle(group)}: fields restored from the board (local edits discarded)`);
  }
});

// the calibration tests watch the motion state on the telemetry stream
let motionWatch = null;

function waitBoardMotionDone(timeoutMs = 40000) {
  return new Promise((resolve) => {
    finishMotionWatch();
    motionWatch = { seenMoving: false, resolve, deadline: Date.now() + timeoutMs };
    // the state transitions are caught on the telemetry stream (WebSocket
    // messages are not throttled like timers are in a background tab); the
    // interval is only the timeout safety net
    motionWatch.timer = setInterval(() => {
      if (motionWatch && Date.now() > motionWatch.deadline) finishMotionWatch();
    }, 1000);
    motionCheck();
  });
}

function motionCheck() {
  if (!motionWatch) return;
  const telemetry = app.telemetry;
  if (telemetry && telemetry.state !== 0) motionWatch.seenMoving = true;
  if (motionWatch.seenMoving && telemetry && telemetry.state === 0) finishMotionWatch();
}

function finishMotionWatch() {
  if (!motionWatch) return;
  clearInterval(motionWatch.timer);
  const resolve = motionWatch.resolve;
  const seen = motionWatch.seenMoving;
  motionWatch = null;
  resolve(seen);
}

el("btn-calib-load").addEventListener("click", async () => {
  try {
    await api("/config/robot/load", "POST");
    await calibLoad();
    calibSetNote("reloaded from the STM flash - the board RAM now matches the saved record");
    logLine("info", "calibration: settings reloaded from the board flash");
  } catch (error) { calibSetNote(`reload failed - ${error.message}`); logLine("error", `calibration: ${error.message}`); }
});

el("btn-calib-defaults").addEventListener("click", async () => {
  try {
    await api("/config/robot/defaults", "POST");
    await calibLoad();
    calibSetNote("factory defaults restored in RAM (not saved - press Save to flash to persist)");
    logLine("warn", "calibration: factory defaults restored (press Save to flash to persist)");
  } catch (error) { calibSetNote(`defaults failed - ${error.message}`); logLine("error", `calibration: ${error.message}`); }
});

async function calibFix(key, factor, note) {
  const old = calibFieldValue(key);
  if (old === null) return;
  const newValue = Math.round(old * factor * 10) / 10;
  await api("/config/robot", "POST", { values: { [key]: newValue } });
  await calibLoad();
  logLine("info", `calibration: ${key} ${old} -> ${newValue} ${note}`);
  calibSetNote(`${key}: ${old} -> ${newValue} ${note} - applied to the board RAM, press Save to flash to keep`);
}

async function calibFixSafe(key, factor, note) {
  try { await calibFix(key, factor, note); }
  catch (error) { calibSetNote(`correction failed - ${error.message}`); logLine("error", `calibration: ${error.message}`); }
}

el("btn-calib-drive").addEventListener("click", async () => {
  const distance = Number(el("calib-drive-dist").value);
  const speed = Number(el("calib-drive-speed").value);
  const start = app.telemetry ? { x: app.telemetry.x_mm, y: app.telemetry.y_mm } : null;
  app.calibDriveOdom = null;
  try {
    await api("/move/drive", "POST", { distance_mm: distance, speed_mm_s: speed });
    logLine("info", `drive test: ${distance} mm at ${speed} mm/s - measure the real distance`);
    await waitBoardMotionDone();
    const end = app.telemetry;
    if (start && end) {
      app.calibDriveOdom = Math.hypot(end.x_mm - start.x, end.y_mm - start.y);
      el("calib-drive-info").textContent = `board odometry: moved ${app.calibDriveOdom.toFixed(1)} mm (commanded ${distance} mm)`;
    } else {
      el("calib-drive-info").textContent = "board odometry unavailable";
    }
  } catch (error) { logLine("error", `drive test: ${error.message}`); }
});

el("btn-calib-turn").addEventListener("click", async () => {
  const angle = Number(el("calib-turn-angle").value);
  const speed = Number(el("calib-turn-speed").value);
  const start = app.telemetry ? app.telemetry.theta_deg : null;
  app.calibTurnOdom = null;
  try {
    await api("/move/turn", "POST", { angle_deg: angle, speed_mm_s: speed });
    logLine("info", `turn test: ${angle} deg at ${speed} mm/s - measure the real angle`);
    await waitBoardMotionDone();
    const end = app.telemetry;
    if (start !== null && end) {
      app.calibTurnOdom = Math.abs(norm180(end.theta_deg - start));
      el("calib-turn-info").textContent = `board odometry: turned ${app.calibTurnOdom.toFixed(1)} deg (commanded ${angle} deg)`;
    } else {
      el("calib-turn-info").textContent = "board odometry unavailable";
    }
  } catch (error) { logLine("error", `turn test: ${error.message}`); }
});

el("btn-calib-fix-wheel").addEventListener("click", () => {
  const commanded = Number(el("calib-drive-dist").value);
  const measured = Number(el("calib-drive-actual").value);
  calibFixSafe("wheel_diameter_mm", measured / commanded, "(wheel size)");
});
el("btn-calib-fix-odom-circ").addEventListener("click", () => {
  const measured = Number(el("calib-drive-actual").value);
  const board = app.calibDriveOdom;
  if (!board || board < 1) { calibSetNote("run the drive test first (board distance unknown)"); return; }
  calibFixSafe("odom_wheel_circumference_mm", measured / board, "(odom wheel)");
});
el("btn-calib-fix-track").addEventListener("click", () => {
  const commanded = Number(el("calib-turn-angle").value);
  const measured = Number(el("calib-turn-actual").value);
  calibFixSafe("robot_track_mm", commanded / measured, "(drive track)");
});
el("btn-calib-fix-odom-track").addEventListener("click", () => {
  const measured = Number(el("calib-turn-actual").value);
  const board = app.calibTurnOdom;
  if (!board || board < 1) { calibSetNote("run the turn test first (board angle unknown)"); return; }
  calibFixSafe("odom_track_mm", board / measured, "(odom track)");
});

// ------------------------------------------------------ IMU calibration --
/** Live IMU / gyro health + the hand-spin scale, bias and noise helpers.
 *  The values ride the telemetry snapshot (imu / gyro_stream / loc / board
 *  blocks the service already publishes); the actions call /sensors/imu/*.
 */

function imuSetNote(message) {
  if (els.imuNote) els.imuNote.textContent = message;
}

/** Badge one readout value: "ok" (green), "warn" (amber) or "off" (grey). */
function imuBadge(node, state) {
  if (!node) return;
  node.className = state;
}

function updateImuCard(data) {
  if (!els.imuStatus || !data || !data.imu) return;
  const imu = data.imu;
  const stream = data.gyro_stream || {};
  const loc = data.loc || {};
  const board = data.board || {};
  const ready = imu.state === "ready";
  els.imuStatus.textContent = `- ${imu.state}${imu.present ? `, ${imu.samples} samples` : ""}`;
  els.imuMode.textContent = loc.mode || "-";
  els.imuState.textContent = imu.state + (imu.calibrating ? " (calibrating)" : "");
  imuBadge(els.imuState, ready ? "ok" : (imu.state === "off" ? "off" : "warn"));
  const tilt = imu.mount ? ` - tilt ${imu.mount.tilt_deg} deg` : "";
  els.imuAxis.textContent = imu.present ? `${imu.yaw_axis}${tilt}` : "-";
  imuBadge(els.imuAxis, imu.present ? "ok" : "off");
  els.imuBias.textContent = imu.present ? `${imu.bias_deg_s.toFixed(3)} deg/s` : "-";
  els.imuIntegral.textContent = imu.present ? `${imu.integrated_deg.toFixed(1)} deg` : "-";
  const errs = stream.errors ? `, ${stream.errors} err` : "";
  els.imuStream.textContent = stream.state === "active"
    ? `active - ${stream.frames} frame(s)${errs}`
    : (stream.state || "-");
  imuBadge(els.imuStream, stream.state === "active" ? "ok" : (stream.state === "unsupported" ? "warn" : "off"));
  els.imuBoard.textContent = board.firmware
    ? (board.gyro_fresh ? "fresh stream" : "wheels only")
    : "-";
  imuBadge(els.imuBoard, board.firmware ? (board.gyro_fresh ? "ok" : "off") : "off");
  els.imuScale.textContent = imu.present ? imu.scale.toPrecision(5) : "-";
  const toggle = els.imuToggle;
  if (toggle) {
    const calibrating = Boolean(imu.calibrating);
    const canStop = ready && !calibrating;
    const canStart = data.transport === "i2c" && !calibrating &&
      (imu.state === "stopped" || imu.state === "absent" || imu.state === "lost");
    toggle.disabled = !(canStop || canStart);
    toggle.textContent = canStop ? "Stop IMU (bench test)" : "Start IMU";
    toggle.title = canStop
      ? "Simulate an IMU loss (safe while moving): the sampler and the gyro stream stop, the board falls back to wheel heading within ~200 ms, never a fault"
      : "Re-probe the IMU and re-average the bias (the robot must stand still)";
  }
}

el("btn-imu-reset").addEventListener("click", async () => {
  try {
    await api("/sensors/imu/reset", "POST", {});
    imuSetNote("yaw counter reset - now rotate the robot N complete turns by hand, then press Compute & apply scale");
    logLine("info", "IMU calibration: yaw integral reset");
  } catch (error) { imuSetNote(`reset failed - ${error.message}`); logLine("error", `IMU calibration: ${error.message}`); }
});

el("btn-imu-apply").addEventListener("click", async () => {
  const turns = Number(el("imu-turns").value);
  try {
    const result = await api("/sensors/imu/scale", "POST", { turns_done: turns });
    imuSetNote(
      `gyro scale ${result.previous.toPrecision(5)} -> ${result.applied.toPrecision(5)} applied live - ` +
      `add "${result.env_line}" to docker/.env to keep it across restarts`);
    logLine("info", `IMU calibration: scale -> ${result.applied.toPrecision(5)} (${turns} turn(s), integrated ${result.integrated_deg.toFixed(1)} deg)`);
  } catch (error) { imuSetNote(`scale failed - ${error.message}`); logLine("error", `IMU calibration: ${error.message}`); }
});

el("btn-imu-bias").addEventListener("click", async () => {
  try {
    const result = await api("/sensors/imu/bias", "POST", {});
    imuSetNote(`bias re-averaged: ${result.bias_deg_s.toFixed(3)} deg/s (robot must stand still - re-check after any remount)`);
    logLine("info", `IMU calibration: bias ${result.bias_deg_s.toFixed(3)} deg/s`);
  } catch (error) { imuSetNote(`bias failed - ${error.message}`); logLine("error", `IMU calibration: ${error.message}`); }
});

el("btn-imu-noise").addEventListener("click", async () => {
  try {
    const result = await api("/sensors/imu/noise", "POST", { seconds: 2 });
    imuSetNote(
      `noise: mean ${result.mean_deg_s.toFixed(3)} deg/s over ${result.samples} samples - ` +
      `add "${result.env_line}" to docker/.env to tune the fusion sigma`);
    logLine("info", `IMU calibration: noise sigma ${result.sigma_deg_s.toFixed(3)} deg/s (mean ${result.mean_deg_s.toFixed(3)})`);
  } catch (error) { imuSetNote(`noise failed - ${error.message}`); logLine("error", `IMU calibration: ${error.message}`); }
});

el("btn-imu-toggle").addEventListener("click", async () => {
  const imu = (app.telemetry && app.telemetry.imu) || {};
  const stopping = imu.state === "ready";
  try {
    const result = await api(stopping ? "/sensors/imu/stop" : "/sensors/imu/start", "POST", {});
    if (stopping) {
      imuSetNote("IMU stopped - the board falls back to wheel heading within ~200 ms (no fault); press Start IMU when the robot stands still to resume");
      logLine("warn", "IMU stopped on purpose (bench fallback test)");
    } else {
      imuSetNote(`IMU restarted: bias re-averaged (${Number(result.bias_deg_s).toFixed(3)} deg/s) - the gyro stream resumes with the next motion`);
      logLine("info", `IMU restarted (bias ${Number(result.bias_deg_s).toFixed(3)} deg/s)`);
    }
  } catch (error) {
    imuSetNote(`${stopping ? "stop" : "start"} failed - ${error.message}`);
    logLine("error", `IMU calibration: ${error.message}`);
  }
});

// --------------------------------------- acceptance & readiness tests --
/** One-click self-tests (server-side evaluation, Docs/05 section 5). Each
 *  block on the card explains itself with steps; a run disables the card,
 *  renders the result in place and leaves one log line. Motion tests are
 *  refused while a mission executes and on latched faults. */

const SELFTEST_BUTTONS = [
  "btn-test-readiness", "btn-test-square", "btn-test-arc", "btn-test-chain", "btn-test-drift",
  "btn-test-turn", "btn-test-loop", "btn-test-ramp", "btn-test-enc",
  "btn-test-drill-estop", "btn-test-drill-deadman",
];

function selftestSetBusy(busy, message) {
  for (const id of SELFTEST_BUTTONS) {
    const button = el(id);
    if (button) button.disabled = busy;
  }
  const status = el("selftest-status");
  if (status) status.textContent = busy ? `- ${message}` : "- idle";
}

function selftestClear(resultId) {
  const node = el(resultId);
  if (node) node.replaceChildren();
}

/** Append one result line; kind is "pass" | "warn" | "fail" | "" (info). */
function selftestLine(resultId, kind, text) {
  const node = el(resultId);
  if (!node) return;
  const line = document.createElement("div");
  line.className = `selftest-check ${kind}`;
  line.textContent = text;
  node.appendChild(line);
}

/** Run one self-test endpoint and render its result document. */
async function selftestRun(resultId, path, body, render) {
  selftestSetBusy(true, "running - the robot may move");
  selftestClear(resultId);
  const started = performance.now();
  try {
    const result = await api(path, "POST", body);
    render(result, resultId);
    const seconds = ((performance.now() - started) / 1000).toFixed(1);
    logLine(result.passed ? "info" : "warn",
      `self-test ${result.test}: ${result.passed ? "PASS" : "FAIL"} (${seconds} s, ${result.transport})`);
  } catch (error) {
    selftestLine(resultId, "fail", `FAIL - ${error.message}`);
    logLine("error", `self-test: ${error.message}`);
  } finally {
    selftestSetBusy(false, "");
  }
}

/** " - MOCK (simulated)" suffix so a simulated pass is never mistaken. */
function selftestMock(result) {
  return result.transport === "mock" ? " - MOCK (simulated)" : "";
}

function renderReadiness(result, resultId) {
  selftestLine(resultId, result.passed ? "pass" : "fail",
    `${result.passed ? "PASS" : "FAIL"} - ${result.summary} (${result.t})${selftestMock(result)}`);
  for (const check of result.checks) {
    selftestLine(resultId, check.status,
      `${check.status.toUpperCase()} ${check.name} - ${check.detail}`);
  }
}

function renderSquare(result, resultId) {
  const metrics = result.metrics;
  selftestLine(resultId, result.passed ? "pass" : "fail",
    `${result.passed ? "PASS" : "FAIL"} - returned ${metrics.position_error_mm} mm / ${metrics.heading_error_deg} deg ` +
    `(limit ${result.thresholds.pos_mm} mm / ${result.thresholds.heading_deg} deg), ${result.duration_s} s` +
    selftestMock(result));
  if (metrics.board_position_error_mm !== undefined) {
    selftestLine(resultId, "",
      `board odometry: ${metrics.board_position_error_mm} mm / ${metrics.board_heading_error_deg} deg`);
  }
  if (result.reason) selftestLine(resultId, "warn", result.reason);
}

function renderArc(result, resultId) {
  selftestLine(resultId, result.passed ? "pass" : "fail",
    `${result.passed ? "PASS" : "FAIL"} - quarter arcs ${result.metrics.radius_mm} mm left and right, ` +
    `${result.duration_s} s${selftestMock(result)}`);
  for (const arc of result.metrics.arcs) {
    selftestLine(resultId, arc.passed ? "pass" : "fail",
      `${arc.passed ? "PASS" : "FAIL"} ${arc.name}: ${arc.position_error_mm} mm / ${arc.heading_error_deg} deg` +
      (arc.reason ? ` - ${arc.reason}` : ""));
  }
  if (result.reason) selftestLine(resultId, "warn", result.reason);
}

function renderChain(result, resultId) {
  const metrics = result.metrics;
  selftestLine(resultId, result.passed ? "pass" : "fail",
    `${result.passed ? "PASS" : "FAIL"} - ${metrics.stop_count} junction stop(s), closing error ` +
    `${metrics.closing_error_mm} mm, ${result.duration_s} s${selftestMock(result)}`);
  selftestLine(resultId, "",
    `segments ${metrics.segments}, max speed ${metrics.max_speed_mm_s} mm/s, stops: ` +
    (metrics.junction_stops.length ? metrics.junction_stops.join(", ") + " s" : "none"));
  if (result.reason) selftestLine(resultId, "warn", result.reason);
}

function renderDrift(result, resultId) {
  const metrics = result.metrics;
  selftestLine(resultId, result.passed ? "pass" : "fail",
    `${result.passed ? "PASS" : "FAIL"} - board drift ${metrics.board_max_pos_mm} mm / ` +
    `${metrics.board_max_heading_deg} deg over ${metrics.seconds} s${selftestMock(result)}`);
  selftestLine(resultId, "",
    `fused drift ${metrics.fused_max_pos_mm} mm / ${metrics.fused_max_heading_deg} deg, ` +
    `encoder tick noise ${metrics.tick_noise_counts} counts, gyro sigma ${metrics.gyro_sigma_deg_s} deg/s ` +
    `(${metrics.gyro_samples} samples)`);
  if (result.reason) selftestLine(resultId, "warn", result.reason);
}

function renderTurn(result, resultId) {
  const metrics = result.metrics;
  selftestLine(resultId, result.passed ? "pass" : "fail",
    `${result.passed ? "PASS" : "FAIL"} - ${metrics.count_per_dir} turns of ${metrics.angle_deg} deg ` +
    `at ${metrics.speed_mm_s} mm/s, ${result.duration_s} s${selftestMock(result)}`);
  for (const record of metrics.directions) {
    selftestLine(resultId, record.passed ? "pass" : "fail",
      `${record.passed ? "PASS" : "FAIL"} ${record.direction}: fused mean ${record.fused_mean_deg} deg, ` +
      `sigma ${record.fused_sigma_deg} deg, max ${record.fused_max_deg} deg ` +
      `(board mean ${record.board_mean_deg} deg, scrub ${record.scrub_mean_deg} deg)`);
  }
  if (result.reason) selftestLine(resultId, "warn", result.reason);
}

function renderLoopDrift(result, resultId) {
  const metrics = result.metrics;
  const last = metrics.records[metrics.records.length - 1];
  selftestLine(resultId, result.passed ? "pass" : "fail",
    `${result.passed ? "PASS" : "FAIL"} - ${metrics.loops} x ${metrics.side_mm} mm loop, ` +
    `fused drift ${last.fused_pos_mm} mm / ${last.fused_heading_deg} deg, ${result.duration_s} s` +
    selftestMock(result));
  for (const record of metrics.records) {
    selftestLine(resultId, record.loop === metrics.loops && !result.passed ? "fail" : "",
      `loop ${record.loop}: fused ${record.fused_pos_mm} mm / ${record.fused_heading_deg} deg, ` +
      `board ${record.board_pos_mm} mm / ${record.board_heading_deg} deg, ` +
      `tick scale ${record.tick_ratio}`);
  }
  if (result.reason) selftestLine(resultId, "warn", result.reason);
}

function renderRamp(result, resultId) {
  const metrics = result.metrics;
  selftestLine(resultId, result.passed ? "pass" : "fail",
    `${result.passed ? "PASS" : "FAIL"} - peak ${metrics.peak_speed_mm_s} mm/s ` +
    `(command ${metrics.commanded_speed_mm_s}), cruise ${metrics.cruise_speed_mm_s} mm/s, ` +
    `t95 ${metrics.time_to_95pct_s === null ? "-" : metrics.time_to_95pct_s + " s"}${selftestMock(result)}`);
  selftestLine(resultId, "",
    `distance ${metrics.board_distance_mm} mm of ${metrics.commanded_distance_mm} mm ` +
    `(error ${metrics.distance_error_mm} mm), fused ${metrics.fused_distance_mm} mm, ` +
    `${metrics.samples} samples`);
  if (result.reason) selftestLine(resultId, "warn", result.reason);
}

function renderEncoders(result, resultId) {
  const metrics = result.metrics;
  selftestLine(resultId, result.passed ? "pass" : "fail",
    `${result.passed ? "PASS" : "FAIL"} - ${metrics.distance_mm} mm forward/back + 90 deg turn, ` +
    `${result.duration_s} s${selftestMock(result)}`);
  selftestLine(resultId, "",
    `ticks fwd [${metrics.forward_ticks[0]}, ${metrics.forward_ticks[1]}], ` +
    `back [${metrics.backward_ticks[0]}, ${metrics.backward_ticks[1]}], ` +
    `turn [${metrics.turn_ticks[0]}, ${metrics.turn_ticks[1]}] (expect +/-${metrics.expected_ticks})`);
  const failed = Object.entries(metrics.checks).filter(([, ok]) => !ok).map(([name]) => name);
  if (failed.length) selftestLine(resultId, "fail", `failed checks: ${failed.join(", ")}`);
  if (result.reason) selftestLine(resultId, "warn", result.reason);
}

function renderDrill(result, resultId) {
  const metrics = result.metrics;
  if (metrics.kind === "estop") {
    selftestLine(resultId, result.passed ? "pass" : "fail",
      `${result.passed ? "PASS" : "FAIL"} - e-stop latched (${metrics.latched_faults.join(", ") || "none"}), ` +
      `recovered: ${metrics.recovered ? "yes" : "NO"}${selftestMock(result)}`);
  } else {
    selftestLine(resultId, result.passed ? "pass" : "fail",
      `${result.passed ? "PASS" : "FAIL"} - deadman ${metrics.deadman_ms} ms: stopped at ` +
      `${metrics.displacement_mm} mm of ${metrics.commanded_distance_mm} mm, ` +
      `faults [${metrics.latched_faults.join(", ") || "none"}], quiet: ${metrics.quiet ? "yes" : "NO"}` +
      selftestMock(result));
    selftestLine(resultId, "",
      `stopped short of the target: ${metrics.stopped_short ? "yes" : "no"} - ` +
      `the watchdog fired while moving at ${metrics.speed_mm_s} mm/s`);
  }
  if (result.reason) selftestLine(resultId, "warn", result.reason);
}

el("btn-test-readiness").addEventListener("click", () =>
  selftestRun("test-readiness-result", "/selftest/readiness", {}, renderReadiness));

el("btn-test-square").addEventListener("click", () =>
  selftestRun("test-square-result", "/selftest/square", {
    side_mm: Number(el("test-square-side").value),
    speed_mm_s: Number(el("test-square-speed").value),
  }, renderSquare));

el("btn-test-arc").addEventListener("click", () =>
  selftestRun("test-arc-result", "/selftest/arc", {
    radius_mm: Number(el("test-arc-radius").value),
    speed_mm_s: Number(el("test-arc-speed").value),
  }, renderArc));

el("btn-test-chain").addEventListener("click", () =>
  selftestRun("test-chain-result", "/selftest/chain", {
    length_mm: Number(el("test-chain-length").value),
    radius_mm: Number(el("test-chain-radius").value),
    speed_mm_s: Number(el("test-chain-speed").value),
  }, renderChain));

el("btn-test-drift").addEventListener("click", () =>
  selftestRun("test-drift-result", "/selftest/rest-drift", {
    seconds: Number(el("test-drift-seconds").value),
  }, renderDrift));

el("btn-test-turn").addEventListener("click", () =>
  selftestRun("test-turn-result", "/selftest/turn-repeat", {
    count_per_dir: Number(el("test-turn-count").value),
    angle_deg: Number(el("test-turn-angle").value),
    speed_mm_s: Number(el("test-turn-speed").value),
  }, renderTurn));

el("btn-test-loop").addEventListener("click", () =>
  selftestRun("test-loop-result", "/selftest/loop-drift", {
    loops: Number(el("test-loop-count").value),
    side_mm: Number(el("test-loop-side").value),
    speed_mm_s: Number(el("test-loop-speed").value),
  }, renderLoopDrift));

el("btn-test-ramp").addEventListener("click", () =>
  selftestRun("test-ramp-result", "/selftest/speed-ramp", {
    distance_mm: Number(el("test-ramp-distance").value),
    speed_mm_s: Number(el("test-ramp-speed").value),
  }, renderRamp));

el("btn-test-enc").addEventListener("click", () =>
  selftestRun("test-enc-result", "/selftest/encoders", {
    distance_mm: Number(el("test-enc-distance").value),
    speed_mm_s: Number(el("test-enc-speed").value),
  }, renderEncoders));

el("btn-test-drill-estop").addEventListener("click", () =>
  selftestRun("test-drill-result", "/selftest/fault-drill", { kind: "estop" }, renderDrill));

el("btn-test-drill-deadman").addEventListener("click", () =>
  selftestRun("test-drill-result", "/selftest/fault-drill", {
    kind: "deadman",
    distance_mm: Number(el("test-drill-distance").value),
    speed_mm_s: Number(el("test-drill-speed").value),
  }, renderDrill));

el("btn-errors-refresh").addEventListener("click", async () => {
  try {
    const result = await api("/errors");
    renderBoardErrors(result.events);
  } catch (error) { logLine("error", `board errors: ${error.message}`); }
});

function applyBoardTelemetry(data) {
  if (data.board) {
    const board = data.board;
    els.boardStatus.textContent = `- fw ${board.firmware}, reset: ${board.reset_cause}, last: ${board.last_event}`;
    if (els.fwVersion && board.firmware) els.fwVersion.textContent = board.firmware;
    if (app.calib === null) els.calibStatus.textContent = `- board fw ${board.firmware}`;
  }
  if (data.board_events) renderBoardErrors(data.board_events);
}

function renderBoardErrors(events) {
  if (!els.boardErrors) return;
  els.boardErrors.replaceChildren();
  const list = [...events].reverse();
  for (const event of list) {
    const line = document.createElement("div");
    line.className = `errline ${event.severity || "info"}`;
    const index = event.index !== undefined ? `[${event.index}] ` : "";
    const code = event.code !== undefined ? `#${event.code} ` : "";
    const aux = event.aux !== undefined && event.aux !== 0 ? ` aux=${event.aux}` : "";
    line.textContent = `${index}${code}${event.name || "EVENT"}${aux}`;
    els.boardErrors.appendChild(line);
  }
  if (!list.length) {
    const line = document.createElement("div");
    line.className = "errline";
    line.textContent = "no board events recorded";
    els.boardErrors.appendChild(line);
  }
}

function updateRobotInfo() {
  const w = Number(el("robot-w").value) || 300;
  const h = Number(el("robot-h").value) || 300;
  const margin = Math.max(0, Number(el("robot-margin").value) || 0);
  const wpMargin = Math.max(0, Number(el("robot-wp-margin").value) || 0);
  const halfDiagonal = Math.hypot(w, h) / 2;
  const wpInset = Math.max(w, h) / 2 + wpMargin;
  el("robot-info").textContent =
    `Half-diagonal ${halfDiagonal.toFixed(0)} mm -> the path keeps the body inside the border ` +
    `(${halfDiagonal.toFixed(0)} mm) and clears obstacles by ${(halfDiagonal + margin).toFixed(0)} mm ` +
    `(body ${Math.min(w, h).toFixed(0)}-${Math.max(w, h).toFixed(0)} mm + ${margin.toFixed(0)} mm margin). ` +
    `Waypoints stay ${wpInset.toFixed(0)} mm from obstacles / the border.`;
}
["robot-w", "robot-h", "robot-margin", "robot-wp-margin"].forEach((id) => el(id).addEventListener("input", updateRobotInfo));
updateRobotInfo();

// --------------------------------------- opponent robots (camera role) ----
/** Config card: how many opponent robots the console reports as the camera
 *  and where they stand, 300x300 mm each. The canvas draws them; dragging on
 *  the map and editing here are the same thing (the fields mirror live).
 *  Default: 1 robot at (2700, 1790) mm. When a REAL camera streams on
 *  /ws/vision the console yields (applyCameraRole) and reports nothing. */
const robotFieldRefs = []; // {robot, x, y} - the inputs mirroring app.robots

function updateRobotsInfo() {
  const ids = app.robots.map((r) => `${r.id} (${Math.round(r.cx)}, ${Math.round(r.cy)})`).join(" - ");
  el("robots-status").textContent = app.cameraLive ? "- REAL camera stream (console paused)" : "- console camera role";
  el("robots-info").textContent = app.cameraLive
    ? `A real camera streams on /ws/vision: the console robots${ids ? ` (${ids})` : ""} are dimmed and NOT reported - its input is used. They are reported again when that stream ends.`
    : `This console reports ${app.robots.length} robot(s) to the service like a camera: ${ids || "none"}. Default: 1 robot at (2700, 1790) mm, 300x300 mm. Drag them on the map anytime - these fields follow.`;
}

/** Rebuild the per-robot X / Y rows from app.robots. */
function renderRobotRows() {
  const list = el("robots-list");
  list.textContent = "";
  robotFieldRefs.length = 0;
  for (const robot of app.robots) {
    const row = document.createElement("div");
    row.className = "row robot-row";
    const tag = document.createElement("span");
    tag.className = "robot-tag";
    tag.textContent = robot.id;
    row.appendChild(tag);
    const fields = { robot };
    for (const [key, axis] of [["cx", "X"], ["cy", "Y"]]) {
      const wrap = document.createElement("label");
      wrap.appendChild(document.createTextNode(axis));
      const input = document.createElement("input");
      input.type = "number";
      input.step = "10";
      input.value = String(Math.round(robot[key]));
      input.addEventListener("input", () => {
        const value = Number(input.value);
        if (!Number.isFinite(value)) return;
        const half = key === "cx" ? robot.w / 2 : robot.h / 2;
        const limit = key === "cx" ? app.field.width_mm : app.field.height_mm;
        robot[key] = clamp(value, half, limit - half);
        pushVision(true);
        updateRobotsInfo();
      });
      input.addEventListener("change", () => { input.value = String(Math.round(robot[key])); });
      wrap.appendChild(input);
      row.appendChild(wrap);
      fields[key === "cx" ? "x" : "y"] = input;
    }
    list.appendChild(row);
    robotFieldRefs.push(fields);
  }
}

/** Mirror app.robots into the config fields (after a drag on the map). */
function syncRobotFields() {
  for (const fields of robotFieldRefs) {
    if (document.activeElement !== fields.x) fields.x.value = String(Math.round(fields.robot.cx));
    if (document.activeElement !== fields.y) fields.y.value = String(Math.round(fields.robot.cy));
  }
  updateRobotsInfo();
}

/** Set how many opponent robots the console reports (0...MAX_OPPONENTS). */
function setRobotCount(count) {
  const wanted = clamp(Math.round(Number(count)) || 0, 0, MAX_OPPONENTS);
  while (app.robots.length > wanted) app.robots.pop();
  while (app.robots.length < wanted) {
    const index = app.robots.length;
    const [cx, cy] = OPPONENT_SPOTS[index] || OPPONENT_SPOTS[OPPONENT_SPOTS.length - 1];
    app.robots.push({ id: `R${index + 1}`, cx, cy, w: OPPONENT_SIZE_MM, h: OPPONENT_SIZE_MM });
  }
  el("robots-count").value = String(wanted);
  renderRobotRows();
  updateRobotsInfo();
  pushVision();
  logLine("info", wanted
    ? `opponent robots: ${wanted} (${app.robots.map((r) => r.id).join(", ")}) reported as camera obstacles`
    : "opponent robots: none reported");
}

el("robots-count").addEventListener("change", () => setRobotCount(el("robots-count").value));
el("btn-robots-reset").addEventListener("click", () => {
  app.robots = defaultRobots(1);
  el("robots-count").value = "1";
  renderRobotRows();
  updateRobotsInfo();
  pushVision();
  logLine("info", "opponent robots reset to the default: 1 robot at (2700, 1790) mm");
});
renderRobotRows();

// ------------------------------------------------- robot / opponent photos --
/** Read an image file and downscale it to ``maxSide`` px (JPEG) before it is
 *  stored/drawn: uploads stay small in browser storage and cheap to draw. */
function loadScaledPhoto(file, maxSide, done) {
  const reader = new FileReader();
  reader.onload = () => {
    const source = new Image();
    source.onload = () => {
      const longest = Math.max(source.naturalWidth, source.naturalHeight) || 1;
      const scale = Math.min(1, maxSide / longest);
      if (scale >= 1) {
        done(String(reader.result || "")); // already small enough
        return;
      }
      const canvas = document.createElement("canvas");
      canvas.width = Math.max(1, Math.round(source.naturalWidth * scale));
      canvas.height = Math.max(1, Math.round(source.naturalHeight * scale));
      canvas.getContext("2d").drawImage(source, 0, 0, canvas.width, canvas.height);
      done(canvas.toDataURL("image/jpeg", 0.85));
    };
    source.src = String(reader.result || "");
  };
  reader.readAsDataURL(file);
}

/** Mirror a photo slot into its small preview next to the file picker. */
function syncPhotoPreview(inputId, slot) {
  const preview = el(`${inputId}-preview`);
  if (!preview) return;
  preview.hidden = !slot.url;
  if (slot.url) preview.src = slot.url;
}

/** Wire a file picker + clear button to a photo slot (data URL in storage). */
function wirePhoto(inputId, clearId, key, slot, note) {
  const input = el(inputId);
  const clear = el(clearId);
  if (!input || !clear) return;
  input.addEventListener("change", () => {
    const file = input.files && input.files[0];
    input.value = ""; // allow re-picking the same file later
    if (!file) return;
    loadScaledPhoto(file, 512, (url) => {
      const image = new Image();
      image.src = url;
      slot.image = image;
      slot.url = url;
      syncPhotoPreview(inputId, slot);
      try {
        localStorage.setItem(key, url);
      } catch (error) {
        logLine("warn", "photo kept for this session only - the browser storage is full or blocked");
      }
      logLine("info", note);
    });
  });
  clear.addEventListener("click", () => {
    slot.image = null;
    slot.url = "";
    syncPhotoPreview(inputId, slot);
    try { localStorage.removeItem(key); } catch (error) { /* private mode */ }
    logLine("info", "photo cleared");
  });
}

wirePhoto("robot-photo", "btn-robot-photo-clear", ROBOT_PHOTO_KEY, app.robotPhoto,
  "robot photo updated - drawn inside the robot square");
wirePhoto("opponent-photo", "btn-opponent-photo-clear", OPPONENT_PHOTO_KEY, app.opponentPhoto,
  "opponent photo updated - drawn inside the opponent squares");
syncPhotoPreview("robot-photo", app.robotPhoto);
syncPhotoPreview("opponent-photo", app.opponentPhoto);
updateRobotsInfo();

// ------------------------------------------------------------ colours -----
const COLOR_INPUTS = {
  "color-stroke": "stroke", "color-path": "path", "color-ref": "ref",
  "color-actions": "actions", "color-waypoint": "waypoint",
  "color-robot": "robot", "color-opponent": "opponent",
  "color-obstacle": "obstacle", "color-goal": "goal",
  "color-measure": "measure", "color-draft": "draft",
};

function colorHexLabel(id, value) {
  const label = document.querySelector(`.color-hex[data-hex="${id}"]`);
  if (label) label.textContent = String(value).toUpperCase();
}

function syncColorInputs() {
  Object.entries(COLOR_INPUTS).forEach(([id, key]) => {
    const input = el(id);
    if (!input) return;
    const value = app.colors[key] || COLOR_DEFAULTS[key];
    input.value = value;
    colorHexLabel(id, value);
  });
}

Object.entries(COLOR_INPUTS).forEach(([id, key]) => {
  const input = el(id);
  if (!input) return;
  input.addEventListener("input", () => {
    app.colors[key] = input.value;
    colorHexLabel(id, input.value);
    saveColors();
  });
});

el("btn-colors-reset").addEventListener("click", () => {
  app.colors = { ...COLOR_DEFAULTS };
  saveColors();
  syncColorInputs();
  logLine("info", "map colours reset to the defaults");
});
syncColorInputs();

// -------------------------------------------------------------- theme -----
const THEME_ICONS = {
  // the header button shows the mode you switch TO
  moon: '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M20.2 14.4A8.2 8.2 0 0 1 9.6 3.8a8.2 8.2 0 1 0 10.6 10.6z"/></svg>',
  sun: '<svg viewBox="0 0 24 24" aria-hidden="true"><circle cx="12" cy="12" r="4.2"/><path d="M12 2.5v2.2M12 19.3v2.2M2.5 12h2.2M19.3 12h2.2M5.2 5.2l1.6 1.6M17.2 17.2l1.6 1.6M18.8 5.2l-1.6 1.6M6.8 17.2l-1.6 1.6"/></svg>',
};
const themeMedia = window.matchMedia ? window.matchMedia("(prefers-color-scheme: dark)") : null;

function effectiveTheme(mode) {
  if (mode !== "system") return mode === "dark" ? "dark" : "light";
  return themeMedia && themeMedia.matches ? "dark" : "light";
}

/** Applies a theme to the document and top-bar/dock controls. The canvas
 *  reads data-theme on every frame, so no explicit redraw is needed. */
function applyTheme(mode, { persist = true } = {}) {
  app.theme = THEME_CHOICES.includes(mode) ? mode : "light";
  const effective = effectiveTheme(app.theme);
  document.documentElement.dataset.theme = effective;
  if (persist) {
    try { localStorage.setItem(THEME_KEY, app.theme); } catch (error) { /* private mode */ }
  }
  const dark = effective === "dark";
  const button = el("btn-theme");
  if (button) {
    button.innerHTML = dark ? THEME_ICONS.sun : THEME_ICONS.moon;
    const hint = dark ? "Switch to light mode" : "Switch to dark mode";
    button.title = hint;
    button.setAttribute("aria-label", hint);
    button.setAttribute("aria-pressed", dark ? "true" : "false");
    button.classList.toggle("active", dark);
  }
  document.querySelectorAll("[data-theme-choice]").forEach((choice) => {
    const active = choice.dataset.themeChoice === app.theme;
    choice.classList.toggle("active", active);
    choice.setAttribute("aria-pressed", active ? "true" : "false");
  });
}

document.querySelectorAll("[data-theme-choice]").forEach((choice) => {
  choice.addEventListener("click", () => applyTheme(choice.dataset.themeChoice));
});
if (el("btn-theme")) {
  el("btn-theme").addEventListener("click", () => applyTheme(isDarkTheme() ? "light" : "dark"));
}
if (themeMedia) {
  const onSchemeChange = () => {
    if (app.theme === "system") applyTheme("system", { persist: false });
  };
  if (themeMedia.addEventListener) themeMedia.addEventListener("change", onSchemeChange);
  else if (themeMedia.addListener) themeMedia.addListener(onSchemeChange);
}
applyTheme(app.theme, { persist: false });

// ------------------------------------------------------------------- init --
async function loadBackground() {
  for (const src of ["field/table.png", "field/table.jpg", "field/field-default.svg"]) {
    const loaded = await new Promise((resolve) => {
      const image = new Image();
      image.onload = () => resolve(image);
      image.onerror = () => resolve(null);
      image.src = src;
    });
    if (loaded) {
      app.bg = { image: loaded, ok: true };
      logLine("info", `field background: ${src}`);
      return;
    }
  }
  logLine("warn", "no field background found, using built-in layout");
}

async function refreshVersions() {
  try {
    const health = await api("/health");
    if (els.swVersion && health.version) {
      els.swVersion.textContent = health.build
        ? `${health.version}+${health.build}`
        : health.version;
      els.swVersion.parentElement.title = health.build
        ? `Console software version (RPi service + UI) - build ${health.build}`
        : "Console software version (RPi service + UI)";
    }
    if (els.fwVersion && health.firmware && health.firmware !== "unknown") {
      els.fwVersion.textContent = health.firmware;
    }
  } catch (error) { logLine("warn", `version info unavailable (${error.message})`); }
}

async function init() {
  resizeCanvas();
  window.addEventListener("resize", () => { resizeCanvas(); });
  // the stage also resizes without a window resize (settings drawer, I2C
  // monitor): keep the drawing buffer in sync with the CSS size
  if (window.ResizeObserver) {
    new ResizeObserver(() => resizeCanvas()).observe(canvas.parentElement);
  }
  try {
    const field = await api("/field");
    app.field = { width_mm: field.width_mm, height_mm: field.height_mm };
    if (field.defaults) {
      el("opt-speed").value = field.defaults.speed_mm_s;
      el("opt-step").value = field.defaults.goto_step_mm;
      el("opt-fit").value = field.defaults.fit_tol_mm;
      if (field.defaults.robot_width_mm) el("robot-w").value = field.defaults.robot_width_mm;
      if (field.defaults.robot_height_mm) el("robot-h").value = field.defaults.robot_height_mm;
      if (field.defaults.obstacle_margin_mm !== undefined) el("robot-margin").value = field.defaults.obstacle_margin_mm;
      if (field.defaults.waypoint_margin_mm !== undefined) el("robot-wp-margin").value = field.defaults.waypoint_margin_mm;
      updateRobotInfo();
      document.querySelectorAll(".card input[type=range]").forEach((input) => input.dispatchEvent(new Event("input")));
    }
  } catch (error) {
    logLine("warn", `field config unavailable (${error.message})`);
  }
  el("field-w").value = app.field.width_mm;
  el("field-h").value = app.field.height_mm;
  try {
    applyTeam(await api("/team"));
    applyTeamScenario(null, app.team);
  } catch (error) {
    logLine("warn", `team config unavailable (${error.message})`);
  }
  fitView();
  renderSequence();
  applyI2cChrome();
  await loadBackground();
  try {
    const history = await api("/trace?after=0&limit=200");
    for (const entry of history.entries) i2cAppend(entry, false);
    app.i2c.count = history.count;
    els.i2cCount.textContent = `${history.count} frames`;
  } catch (error) { logLine("warn", `I2C trace unavailable (${error.message})`); }
  await refreshVersions();
  updateLinkPills();
  updateImuPill(null);
  connectWs();
  connectVision();
  setInterval(() => pushVision(), VISION_KEEPALIVE_MS);
  // coming back from a throttled background tab: refresh at once so the
  // report is never seen as stale (which would schedule a re-plan)
  document.addEventListener("visibilitychange", () => { if (!document.hidden) pushVision(); });
  window.addEventListener("focus", () => pushVision());
  calibLoad().catch(() => {}); // first read of the board settings when idle
  renderLoop();
  logLine("info", "console ready");
}

init();
