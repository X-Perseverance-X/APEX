/* APEX Kinematics Lab v3.1 - firmware-aligned semi-digital twin */
(() => {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const LEG_ORDER = ["LF", "LM", "LR", "RF", "RM", "RR"];
  const AXES = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"];
  const GAITS = [
    "tripod",
    "alternative_tripod",
    "ripple",
    "wave",
    "tetrapod",
    "metachronal",
    "crawl_slow",
    "custom",
  ];
  const CURVES = [
    "linear",
    "smoothstep",
    "cosine",
    "cubic_bezier",
    "minimum_jerk",
    "trapezoid",
    "s_curve",
  ];
  const SENSOR_ORDER = ["imu", "lidar", "camera", "tof", "gps"];
  const FIRMWARE_LEG_ORDER = ["RF", "RM", "RR", "LF", "LM", "LR"];
  const FIRMWARE_PHASE = { RF: 0, RM: 0.5, RR: 0, LF: 0.5, LM: 0, LR: 0.5 };
  const JOINT_ORDER = ["coxa", "femur", "tibia"];
  const LIVE_MODES = ["simulation", "live_overlay", "live_drive"];
  const C = {
    red: 0xe60000,
    cyan: 0x00e5b0,
    orange: 0xffa500,
    green: 0x00ff66,
    violet: 0x9d4edd,
    white: 0xffffff,
    grey: 0x666666,
    dark: 0x151515,
    blue: 0x00aaff,
  };
  const Tip = {
    el: null,
    show(t, e) {
      if (!this.el) this.el = $("tipbox");
      this.el.textContent = t;
      this.el.style.display = "block";
      this.move(e);
    },
    move(e) {
      if (!this.el || this.el.style.display === "none") return;
      this.el.style.left = e.clientX + 14 + "px";
      this.el.style.top = e.clientY + 14 + "px";
    },
    hide() {
      if (this.el) this.el.style.display = "none";
    },
  };
  let cfg = null;
  const State = {
    scene: null,
    camera: null,
    renderer: null,
    robotGroup: null,
    liveGroup: null,
    sensorGroup: null,
    helperGroup: null,
    gridHelper: null,
    clock: 0,
    playing: false,
    lastTs: 0,
    legResults: {},
    activeSection: "live",
    sectionPinned: null,
    menuSwitchUntil: 0,
    loopStarted: false,
    cameraCtl: null,
    lastSceneRenderTs: 0,
    lastCurveRenderTs: 0,
    live: {
      online: false,
      telemetry: null,
      legStatus: null,
      lastSnapshotTs: 0,
      lastPollOkTs: 0,
      contactSource: "simulation",
      contacts: {},
      movingLegs: {},
      previousAngles: {},
      targetAngles: {},
      renderAngles: {},
      yawZero: 0,
      history: [],
      connectedAt: 0,
      lastPhase: null,
      lastPhaseTs: 0,
      lastWrapTs: 0,
      cycles: [],
      phaseError: 0,
      contactMismatch: 0,
      polling: false,
      error: "",
    },
  };

  function deepClone(o) {
    return JSON.parse(JSON.stringify(o));
  }
  function rad(d) {
    return (d * Math.PI) / 180;
  }
  function deg(r) {
    return (r * 180) / Math.PI;
  }
  function clamp(v, a, b) {
    return Math.max(a, Math.min(b, Number(v)));
  }
  function frac(v) {
    return ((Number(v) % 1) + 1) % 1;
  }
  function phaseDelta(a, b) {
    let d = frac(Number(a) - Number(b));
    if (d > 0.5) d -= 1;
    return d;
  }
  function finite(v) {
    return Number.isFinite(Number(v));
  }
  function fmt(v, n = 1) {
    return Number(v).toFixed(n);
  }
  function liveImuValue(axis, telemetry = State.live.telemetry || {}) {
    const keyMap = { roll: "mpuR", pitch: "mpuP", yaw: "mpuY" };
    const signKey = { roll: "imuRollSign", pitch: "imuPitchSign", yaw: "imuYawSign" };
    const raw = Number(telemetry?.[keyMap[axis]]);
    if (!Number.isFinite(raw)) return null;
    const sign = Number(cfg?.live?.[signKey[axis]] ?? 1);
    const zero = axis === "yaw" ? Number(State.live.yawZero || 0) : 0;
    return (raw - zero) * (Number.isFinite(sign) ? sign : 1);
  }
  function vec(x = 0, y = 0, z = 0) {
    return { x: Number(x) || 0, y: Number(y) || 0, z: Number(z) || 0 };
  }
  function getPath(path) {
    return path.split(".").reduce((o, k) => o?.[k], cfg);
  }
  function setPath(path, val) {
    const parts = path.split(".");
    let o = cfg;
    for (let i = 0; i < parts.length - 1; i++) o = o[parts[i]];
    const key = parts.at(-1);
    const old = o[key];
    o[key] = typeof old === "number" ? Number(val) : val;
  }
  function robotToThree(v) {
    return new THREE.Vector3(v.x, v.z, -v.y);
  }
  function axisVector(axis) {
    return (
      {
        "+X": vec(1, 0, 0),
        "-X": vec(-1, 0, 0),
        "+Y": vec(0, 1, 0),
        "-Y": vec(0, -1, 0),
        "+Z": vec(0, 0, 1),
        "-Z": vec(0, 0, -1),
      }[axis] || vec(0, 0, 1)
    );
  }
  function eulerMatrix(roll, pitch, yaw) {
    const cr = Math.cos(rad(roll)),
      sr = Math.sin(rad(roll)),
      cp = Math.cos(rad(pitch)),
      sp = Math.sin(rad(pitch)),
      cy = Math.cos(rad(yaw)),
      sy = Math.sin(rad(yaw));
    return [
      [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
      [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
      [-sp, cp * sr, cp * cr],
    ];
  }
  function matTmulVec(M, v) {
    return vec(
      M[0][0] * v.x + M[1][0] * v.y + M[2][0] * v.z,
      M[0][1] * v.x + M[1][1] * v.y + M[2][1] * v.z,
      M[0][2] * v.x + M[1][2] * v.y + M[2][2] * v.z,
    );
  }
  function matMulVec(M, v) {
    return vec(
      M[0][0] * v.x + M[0][1] * v.y + M[0][2] * v.z,
      M[1][0] * v.x + M[1][1] * v.y + M[1][2] * v.z,
      M[2][0] * v.x + M[2][1] * v.y + M[2][2] * v.z,
    );
  }
  function bodyToWorld(p, bodyPos, bodyRotation) {
    return add(bodyPos, matMulVec(bodyRotation, p));
  }
  function setRobotRotation(object, robotRotation) {
    // Three frame is (robot X, robot Z, -robot Y). Build the exact basis
    // instead of relying on a fragile Euler-axis permutation.
    const basisX = robotToThree(matMulVec(robotRotation, vec(1, 0, 0)));
    const basisY = robotToThree(matMulVec(robotRotation, vec(0, 0, 1)));
    const basisZ = robotToThree(matMulVec(robotRotation, vec(0, -1, 0)));
    const matrix = new THREE.Matrix4().makeBasis(basisX, basisY, basisZ);
    object.quaternion.setFromRotationMatrix(matrix);
  }
  function sub(a, b) {
    return vec(a.x - b.x, a.y - b.y, a.z - b.z);
  }
  function add(a, b) {
    return vec(a.x + b.x, a.y + b.y, a.z + b.z);
  }
  function mul(a, s) {
    return vec(a.x * s, a.y * s, a.z * s);
  }
  function len2(a) {
    return Math.hypot(a.x, a.y);
  }

  function material(color, opacity = 1) {
    return new THREE.MeshStandardMaterial({
      color,
      transparent: opacity < 1,
      opacity,
      roughness: 0.62,
      metalness: 0.12,
    });
  }
  function clear(g) {
    while (g.children.length) {
      const c = g.children.pop();
      if (c.geometry) c.geometry.dispose();
      if (c.material) c.material.dispose?.();
    }
  }
  function sphere(group, p, r, color, opacity = 1) {
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(r, 18, 12),
      material(color, opacity),
    );
    m.position.copy(robotToThree(p));
    group.add(m);
    return m;
  }
  function box(group, p, size, color, opacity = 1) {
    const m = new THREE.Mesh(
      new THREE.BoxGeometry(size.x, size.z, size.y),
      material(color, opacity),
    );
    m.position.copy(robotToThree(p));
    group.add(m);
    return m;
  }
  function cyl(group, a, b, r, color, opacity = 1) {
    const A = robotToThree(a),
      B = robotToThree(b),
      dir = new THREE.Vector3().subVectors(B, A),
      L = dir.length();
    if (L < 0.01) return null;
    const mesh = new THREE.Mesh(
      new THREE.CylinderGeometry(r, r, L, 14),
      material(color, opacity),
    );
    mesh.position.copy(A).add(B).multiplyScalar(0.5);
    mesh.quaternion.setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      dir.clone().normalize(),
    );
    group.add(mesh);
    return mesh;
  }
  function line(group, pts, color, opacity = 1) {
    const verts = [];
    pts.forEach((p) => {
      const t = robotToThree(p);
      verts.push(t.x, t.y, t.z);
    });
    const geom = new THREE.BufferGeometry();
    geom.setAttribute("position", new THREE.Float32BufferAttribute(verts, 3));
    const mat = new THREE.LineBasicMaterial({
      color,
      transparent: opacity < 1,
      opacity,
    });
    const l = new THREE.Line(geom, mat);
    group.add(l);
    return l;
  }
  function label(group, text, p, color = "#fff") {
    if (!cfg.visual.showLabels) return;
    const canvas = document.createElement("canvas");
    canvas.width = 256;
    canvas.height = 64;
    const ctx = canvas.getContext("2d");
    ctx.fillStyle = "rgba(0,0,0,.55)";
    ctx.fillRect(0, 0, 256, 64);
    ctx.font = "26px Share Tech Mono";
    ctx.fillStyle = color;
    ctx.fillText(text, 12, 40);
    const tex = new THREE.CanvasTexture(canvas);
    const sp = new THREE.Sprite(
      new THREE.SpriteMaterial({ map: tex, transparent: true }),
    );
    sp.scale.set(60, 15, 1);
    sp.position.copy(robotToThree(p));
    group.add(sp);
  }
  function arrow(group, p, axis, length, color, labelText) {
    const v = axisVector(axis);
    return arrowVector(group, p, v, length, color, labelText);
  }
  function arrowVector(group, p, direction, length, color, labelText) {
    const norm = Math.hypot(direction.x, direction.y, direction.z) || 1;
    const v = mul(direction, 1 / norm);
    const q = add(p, mul(v, length));
    cyl(group, p, q, 3, color, 0.95);
    sphere(group, q, 8, color, 0.95);
    if (labelText) label(group, labelText, q, "#e6e6e6");
  }

  function initScene() {
    const container = $("scene3d");
    if (typeof THREE === "undefined") {
      container.innerHTML =
        '<div style="height:100%;display:flex;align-items:center;justify-content:center;color:#888">THREE.JS YÜKLENMEDİ — internet/CDN erişimini kontrol et.</div>';
      return;
    }
    State.scene = new THREE.Scene();
    State.scene.background = new THREE.Color(0x020202);
    State.camera = new THREE.PerspectiveCamera(
      52,
      container.clientWidth / container.clientHeight,
      1,
      9000,
    );
    State.camera.position.set(520, 360, 520);
    State.camera.lookAt(0, 70, 0);
    State.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    State.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    State.renderer.setSize(container.clientWidth, container.clientHeight);
    container.innerHTML = "";
    container.appendChild(State.renderer.domElement);
    State.robotGroup = new THREE.Group();
    State.liveGroup = new THREE.Group();
    State.sensorGroup = new THREE.Group();
    State.helperGroup = new THREE.Group();
    State.scene.add(
      State.robotGroup,
      State.liveGroup,
      State.sensorGroup,
      State.helperGroup,
    );
    State.scene.add(new THREE.AmbientLight(0xffffff, 0.42));
    const key = new THREE.DirectionalLight(0xffffff, 0.9);
    key.position.set(350, 500, 260);
    State.scene.add(key);
    const rim = new THREE.DirectionalLight(C.red, 0.35);
    rim.position.set(-500, 240, -400);
    State.scene.add(rim);
    const grid = new THREE.GridHelper(1100, 22, 0x333333, 0x151515);
    grid.position.y = Number(cfg?.contacts?.groundZ || 0);
    State.gridHelper = grid;
    State.scene.add(grid);
    const axes = new THREE.Group();
    State.scene.add(axes);
    arrow(axes, vec(0, 0, 0), "+X", 120, 0xff3333, "+X");
    arrow(axes, vec(0, 0, 0), "+Y", 120, 0x00ff66, "+Y");
    arrow(axes, vec(0, 0, 0), "+Z", 120, 0x33aaff, "+Z");
    // Full free camera controller: left-drag orbit, right/shift-drag pan, wheel zoom.
    const canvas = State.renderer.domElement;
    canvas.tabIndex = 0;
    canvas.style.outline = "none";
    canvas.addEventListener("contextmenu", (e) => e.preventDefault());
    const target = new THREE.Vector3(0, 70, 0);
    const spherical = new THREE.Spherical().setFromVector3(
      State.camera.position.clone().sub(target),
    );
    State.cameraCtl = {
      target,
      spherical,
      dragging: false,
      mode: "rotate",
      lastX: 0,
      lastY: 0,
    };
    const updateFreeCamera = () => {
      if (!State.cameraCtl) return;
      const ctl = State.cameraCtl;
      ctl.spherical.radius = clamp(ctl.spherical.radius, 90, 2600);
      ctl.spherical.phi = clamp(ctl.spherical.phi, 0.025, Math.PI - 0.025);
      const offset = new THREE.Vector3().setFromSpherical(ctl.spherical);
      State.camera.position.copy(ctl.target).add(offset);
      State.camera.lookAt(ctl.target);
    };
    State.updateFreeCamera = updateFreeCamera;
    canvas.addEventListener("wheel", (e) => {
      e.preventDefault();
      State.cameraCtl.spherical.radius *= Math.exp(e.deltaY * 0.001);
      updateFreeCamera();
    }, { passive: false });
    canvas.addEventListener("pointerdown", (e) => {
      const ctl = State.cameraCtl;
      ctl.dragging = true;
      ctl.mode = (e.button === 2 || e.button === 1 || e.shiftKey) ? "pan" : "rotate";
      ctl.lastX = e.clientX;
      ctl.lastY = e.clientY;
      canvas.focus({ preventScroll: true });
      canvas.setPointerCapture(e.pointerId);
    });
    const endDrag = () => { if (State.cameraCtl) State.cameraCtl.dragging = false; };
    canvas.addEventListener("pointerup", endDrag);
    canvas.addEventListener("pointercancel", endDrag);
    canvas.addEventListener("pointermove", (e) => {
      const ctl = State.cameraCtl;
      if (!ctl?.dragging) return;
      const dx = e.clientX - ctl.lastX;
      const dy = e.clientY - ctl.lastY;
      ctl.lastX = e.clientX;
      ctl.lastY = e.clientY;
      if (ctl.mode === "rotate") {
        ctl.spherical.theta -= dx * 0.0065;
        ctl.spherical.phi -= dy * 0.0065;
      } else {
        const scale = ctl.spherical.radius * 0.0015;
        State.camera.updateMatrixWorld();
        const right = new THREE.Vector3().setFromMatrixColumn(State.camera.matrixWorld, 0);
        const up = new THREE.Vector3().setFromMatrixColumn(State.camera.matrixWorld, 1);
        ctl.target.addScaledVector(right, -dx * scale);
        ctl.target.addScaledVector(up, dy * scale);
      }
      updateFreeCamera();
    });
    canvas.addEventListener("keydown", (e) => {
      const ctl = State.cameraCtl;
      if (!ctl) return;
      const step = Math.max(4, ctl.spherical.radius * 0.025);
      State.camera.updateMatrixWorld();
      const right = new THREE.Vector3().setFromMatrixColumn(State.camera.matrixWorld, 0);
      const up = new THREE.Vector3().setFromMatrixColumn(State.camera.matrixWorld, 1);
      const forward = ctl.target.clone().sub(State.camera.position).normalize();
      let handled = true;
      if (e.key === "a" || e.key === "A") ctl.target.addScaledVector(right, -step);
      else if (e.key === "d" || e.key === "D") ctl.target.addScaledVector(right, step);
      else if (e.key === "w" || e.key === "W") ctl.target.addScaledVector(forward, step);
      else if (e.key === "s" || e.key === "S") ctl.target.addScaledVector(forward, -step);
      else if (e.key === "q" || e.key === "Q") ctl.target.addScaledVector(up, -step);
      else if (e.key === "e" || e.key === "E") ctl.target.addScaledVector(up, step);
      else handled = false;
      if (handled) { e.preventDefault(); updateFreeCamera(); }
    });
    updateFreeCamera();
    window.addEventListener("resize", resizeScene);
    $("sceneState").textContent = "READY";
  }
  function resizeScene() {
    const container = $("scene3d");
    if (!State.renderer || !container) return;
    State.camera.aspect = container.clientWidth / container.clientHeight;
    State.camera.updateProjectionMatrix();
    State.renderer.setSize(container.clientWidth, container.clientHeight);
  }

  // Firmware contract: servo = zero + dir * (geometricAngle - 90deg).
  // Geometric angles are not URDF / serial-chain joint angles. In particular,
  // the tibia geometric angle is an interior linkage angle near 130 degrees at
  // stand, while the serial-chain knee angle is about -130 degrees.
  function servoToGeometry(legName, joint, servoDeg) {
    const j = cfg.legs[legName].joints[joint];
    return 90 + (Number(servoDeg) - Number(j.zero)) * Number(j.dir || 1);
  }
  function geometryToServoRaw(legName, joint, geometryDeg) {
    const j = cfg.legs[legName].joints[joint];
    return Number(j.zero) +
      (Number(geometryDeg) - 90) * Number(j.dir || 1);
  }
  function geometryToServo(legName, joint, geometryDeg) {
    const j = cfg.legs[legName].joints[joint];
    return clamp(
      geometryToServoRaw(legName, joint, geometryDeg),
      Number(j.min),
      Number(j.max),
    );
  }
  function gaitPhaseForLeg(name) {
    if (cfg.gait.type === "custom") return cfg.gait.phases[name] ?? 0;
    const map = {
      tripod: { RF: 0, RR: 0, LM: 0, RM: 0.5, LF: 0.5, LR: 0.5 },
      alternative_tripod: { LF: 0, LR: 0, RM: 0, RF: 0.5, RR: 0.5, LM: 0.5 },
      ripple: { LF: 0, RM: 1 / 6, LR: 2 / 6, RF: 3 / 6, LM: 4 / 6, RR: 5 / 6 },
      wave: { LF: 0, LM: 1 / 6, LR: 2 / 6, RF: 3 / 6, RM: 4 / 6, RR: 5 / 6 },
      tetrapod: { LF: 0, RR: 0, LM: 1 / 3, RF: 1 / 3, LR: 2 / 3, RM: 2 / 3 },
      metachronal: { LF: 0, LM: 0.16, LR: 0.32, RR: 0.5, RM: 0.66, RF: 0.82 },
      crawl_slow: { LF: 0, RM: 0.17, LR: 0.34, RF: 0.51, LM: 0.68, RR: 0.85 },
    };
    return map[cfg.gait.type]?.[name] ?? 0;
  }
  function liveModeActive() {
    return cfg?.live?.mode && cfg.live.mode !== "simulation";
  }
  function simBasePhase() {
    return frac(State.clock * Number(cfg.gait.frequency || 0.6));
  }
  function activeBasePhase() {
    const gp = State.live.telemetry?.gaitPhase;
    if (
      liveModeActive() &&
      cfg.live.phaseSync &&
      State.live.online &&
      finite(gp)
    ) {
      return frac(gp);
    }
    return simBasePhase();
  }
  function liveMotionActive() {
    const mode = String(State.live.telemetry?.mode || "").toUpperCase();
    return /YUR|WALK|GAIT|BEZIER/.test(mode);
  }
  function getLiveLegEntry(name) {
    const idx = FIRMWARE_LEG_ORDER.indexOf(name);
    const legs = State.live.legStatus?.legs;
    if (!Array.isArray(legs) || idx < 0) return null;
    return (
      legs.find(
        (x) =>
          Number(x?.leg) === idx ||
          String(x?.name || "").toUpperCase() === name,
      ) ||
      legs[idx] ||
      null
    );
  }
  function getRawLiveAngles(name) {
    const entry = getLiveLegEntry(name);
    if (!entry || !Array.isArray(entry.angle) || entry.angle.length < 3)
      return null;
    const vals = entry.angle.slice(0, 3).map(Number);
    if (!vals.every(Number.isFinite)) return null;
    return { coxa: vals[0], femur: vals[1], tibia: vals[2] };
  }
  function getLiveAngles(name) {
    return State.live.renderAngles[name] || getRawLiveAngles(name);
  }
  function advanceLiveInterpolation(dt) {
    if (!liveModeActive() || !State.live.online) return;
    const tau = Math.max(0.025, Number(cfg?.live?.interpolationTauMs || 70) / 1000);
    const alpha = 1 - Math.exp(-Math.max(0, dt) / tau);
    FIRMWARE_LEG_ORDER.forEach((name) => {
      const target = State.live.targetAngles[name];
      if (!target) return;
      const cur = State.live.renderAngles[name];
      if (!cur) {
        State.live.renderAngles[name] = { ...target };
        return;
      }
      JOINT_ORDER.forEach((j) => {
        cur[j] += (target[j] - cur[j]) * alpha;
      });
    });
  }
  function extractPhysicalContacts(t) {
    if (!t || typeof t !== "object") return null;
    const keys = [
      "contacts",
      "footContact",
      "groundContact",
      "legContacts",
      "contact",
    ];
    for (const key of keys) {
      const value = t[key];
      if (Array.isArray(value) && value.length >= 6) {
        const out = {};
        FIRMWARE_LEG_ORDER.forEach((n, i) => (out[n] = !!value[i]));
        return out;
      }
      if (value && typeof value === "object") {
        const out = {};
        let hits = 0;
        FIRMWARE_LEG_ORDER.forEach((n) => {
          if (Object.prototype.hasOwnProperty.call(value, n)) {
            out[n] = !!value[n];
            hits++;
          } else if (
            Object.prototype.hasOwnProperty.call(value, n.toLowerCase())
          ) {
            out[n] = !!value[n.toLowerCase()];
            hits++;
          }
        });
        if (hits === 6) return out;
      }
    }
    const direct = {};
    let hits = 0;
    FIRMWARE_LEG_ORDER.forEach((n) => {
      for (const key of [`contact${n}`, `contact_${n}`, `${n}Contact`]) {
        if (Object.prototype.hasOwnProperty.call(t, key)) {
          direct[n] = !!t[key];
          hits++;
          break;
        }
      }
    });
    return hits === 6 ? direct : null;
  }
  function phaseEstimatedContacts(phase) {
    const out = {};
    FIRMWARE_LEG_ORDER.forEach((n) => {
      // Firmware: t_leg < 0.5 SWING, aksi STANCE.
      out[n] = frac(phase + FIRMWARE_PHASE[n]) >= 0.5;
    });
    return out;
  }
  function simulationCycleMs() {
    return 1000 / Math.max(0.01, Number(cfg.gait.frequency || 0.6));
  }
  function liveCycleMs() {
    if (!State.live.cycles.length) return null;
    return (
      State.live.cycles.reduce((a, b) => a + b, 0) / State.live.cycles.length
    );
  }
  function computeSimServoAtPhase(name, basePhase) {
    const target = footTarget(name, false, basePhase);
    return solveIK(name, correctedTarget(name, target)).servo;
  }
  function selectedRmse() {
    const leg = cfg.live.compareLeg;
    const joint = cfg.live.compareJoint;
    const pts = State.live.history.filter(
      (x) =>
        x.leg === leg &&
        x.joint === joint &&
        finite(x.model) &&
        finite(x.robot),
    );
    if (!pts.length) return null;
    const mse =
      pts.reduce((sum, x) => sum + (x.model - x.robot) ** 2, 0) / pts.length;
    return Math.sqrt(mse);
  }
  function recordComparisonSample(nowSec) {
    if (!State.live.online) return;
    const leg = cfg.live.compareLeg;
    const joint = cfg.live.compareJoint;
    const angles = getLiveAngles(leg);
    if (!angles || !finite(angles[joint])) return;
    const phase = finite(State.live.telemetry?.gaitPhase)
      ? frac(State.live.telemetry.gaitPhase)
      : activeBasePhase();
    const model = computeSimServoAtPhase(leg, phase)[joint];
    State.live.history.push({
      ts: nowSec,
      phase,
      leg,
      joint,
      model,
      robot: angles[joint],
    });
    const cutoff = nowSec - 12;
    State.live.history = State.live.history.filter(
      (x) => x.ts >= cutoff && x.leg === leg && x.joint === joint,
    );
  }
  function processLiveSnapshot(snapshot) {
    const nowSec = performance.now() / 1000;
    const wasOnline = State.live.online;
    State.live.online = !!snapshot?.online && !!snapshot?.telemetry;
    State.live.error = snapshot?.error || "";
    State.live.lastSnapshotTs = Number(snapshot?.serverTs || Date.now() / 1000);
    if (!State.live.online) return;
    if (!wasOnline || !State.live.connectedAt) State.live.connectedAt = nowSec;
    State.live.lastPollOkTs = nowSec;
    State.live.telemetry = snapshot.telemetry || {};
    State.live.legStatus = snapshot.legStatus || null;

    const moving = {};
    FIRMWARE_LEG_ORDER.forEach((name) => {
      const cur = getRawLiveAngles(name);
      const prev = State.live.previousAngles[name];
      moving[name] = !!(
        cur &&
        prev &&
        JOINT_ORDER.some((j) => Math.abs(cur[j] - prev[j]) >= 0.25)
      );
      if (cur) {
        State.live.targetAngles[name] = { ...cur };
        if (!State.live.renderAngles[name]) State.live.renderAngles[name] = { ...cur };
        State.live.previousAngles[name] = { ...cur };
      }
    });
    State.live.movingLegs = moving;

    const t = State.live.telemetry;
    const physical = extractPhysicalContacts(t);
    const gp = finite(t.gaitPhase) ? frac(t.gaitPhase) : null;
    if (physical) {
      State.live.contacts = physical;
      State.live.contactSource = "LIVE SENSOR";
      if (gp !== null) {
        const estimated = phaseEstimatedContacts(gp);
        State.live.contactMismatch = FIRMWARE_LEG_ORDER.filter(
          (n) => estimated[n] !== physical[n],
        ).length;
      }
    } else if (gp !== null) {
      State.live.contacts = phaseEstimatedContacts(gp);
      State.live.contactSource = "PHASE-ESTIMATED";
      State.live.contactMismatch = 0;
    } else {
      State.live.contacts = {};
      State.live.contactSource = "SIMULATION";
    }

    if (gp !== null) {
      if (!wasOnline)
        State.clock = gp / Math.max(0.01, Number(cfg.gait.frequency || 0.6));
      const nowMs = performance.now();
      if (State.live.lastPhase !== null && gp < State.live.lastPhase - 0.5) {
        if (State.live.lastWrapTs) {
          const cycle = nowMs - State.live.lastWrapTs;
          if (cycle > 150 && cycle < 15000) {
            State.live.cycles.push(cycle);
            if (State.live.cycles.length > 8) State.live.cycles.shift();
          }
        }
        State.live.lastWrapTs = nowMs;
      }
      State.live.phaseError = phaseDelta(simBasePhase(), gp);
      State.live.lastPhase = gp;
      State.live.lastPhaseTs = nowMs;
    }
    recordComparisonSample(nowSec);
  }
  function updateLiveWidgets() {
    const t = State.live.telemetry || {};
    const setText = (id, value, cls = "") => {
      const el = $(id);
      if (!el) return;
      el.textContent = value;
      if (cls) el.className = cls;
    };
    setText(
      "liveLink",
      State.live.online ? "ONLINE" : "OFFLINE",
      State.live.online ? "live-on" : "live-off",
    );
    setText("liveRobotMode", t.mode || "--");
    setText(
      "liveRealCycle",
      liveCycleMs() === null ? "--" : `${fmt(liveCycleMs(), 0)} ms`,
    );
    setText("liveSimCycle", `${fmt(simulationCycleMs(), 0)} ms`);
    setText(
      "livePhaseErr",
      finite(t.gaitPhase) ? `${fmt(State.live.phaseError * 100, 1)} %` : "--",
    );
    const rmse = selectedRmse();
    setText("liveRmse", rmse === null ? "--" : `${fmt(rmse, 2)}°`);
    const elapsed = State.live.connectedAt
      ? performance.now() / 1000 - State.live.connectedAt
      : 0;
    setText("liveElapsed", `${fmt(elapsed, 1)} s`);
    setText("liveContactSource", State.live.contactSource);
    setText("liveTof", t.tofOnline ? `${t.tofMm} mm` : "OFFLINE");
    setText(
      "liveBattery",
      finite(t.batR) && finite(t.batL)
        ? `R ${fmt(t.batR, 2)}V / L ${fmt(t.batL, 2)}V`
        : "--",
    );
    FIRMWARE_LEG_ORDER.forEach((n) => {
      const el = $(`liveContact${n}`);
      if (!el) return;
      const on = !!State.live.contacts[n];
      const moving = !!State.live.movingLegs[n];
      el.className = `contact-live ${on ? "on" : ""} ${moving ? "moving" : ""}`;
      el.textContent = `${n} ${on ? "STANCE" : "SWING"}${moving ? " • MOVE" : ""}`;
    });
    const liveTicker = $("tickerLive");
    if (liveTicker) {
      liveTicker.textContent = State.live.online ? "ONLINE" : "OFFLINE";
      liveTicker.style.color = State.live.online ? "var(--c-cyan)" : "#777";
    }
    const contactTicker = $("tickerContact");
    if (contactTicker) contactTicker.textContent = State.live.contactSource;
  }
  async function pollLive() {
    if (State.live.polling || !cfg) return;
    State.live.polling = true;
    try {
      const r = await fetch(`/api/live?_=${Date.now()}`, { cache: "no-store" });
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      processLiveSnapshot(await r.json());
    } catch (err) {
      State.live.online = false;
      State.live.error = String(err?.message || err);
    } finally {
      State.live.polling = false;
      updateLiveWidgets();
      clearTimeout(State.live.pollTimer);
      State.live.pollTimer = setTimeout(
        pollLive,
        clamp(cfg?.live?.pollMs || 80, 50, 1000),
      );
    }
  }
  function startLivePolling() {
    clearTimeout(State.live.pollTimer);
    State.live.pollTimer = setTimeout(pollLive, 60);
  }

  function footTarget(name, nominal = false, basePhaseOverride = null) {
    const leg = cfg.legs[name];
    let p = vec(leg.target.x, leg.target.y, leg.target.z);
    const liveDynamic =
      liveModeActive() && State.live.online && liveMotionActive();
    const dynamic =
      !nominal && (State.playing || liveDynamic || basePhaseOverride !== null);
    if (dynamic) {
      const base =
        basePhaseOverride === null
          ? activeBasePhase()
          : frac(basePhaseOverride);
      const ph = frac(base + gaitPhaseForLeg(name));
      const duty = clamp(cfg.gait.dutyFactor, 0.12, 0.88);
      if (ph > duty) {
        const u = (ph - duty) / (1 - duty);
        const swing = Math.sin(Math.PI * u);
        p.x += cfg.gait.stepLength * (u - 0.5);
        p.z += cfg.gait.stepHeight * swing;
        p.y +=
          cfg.gait.stepWidth *
          (leg.side === "left" ? 1 : -1) *
          Math.sin(Math.PI * u);
      } else {
        const u = ph / duty;
        p.x += cfg.gait.stepLength * (0.5 - u);
      }
    }
    return p;
  }
  function correctedTarget(name, p) {
    if (!cfg.bodyIK.enabled) return p;
    const telemetry = State.live.telemetry || {};
    const liveAct = liveModeActive() && cfg.live.followImu && State.live.online;
    const act = {
      roll:
        liveAct && liveImuValue("roll", telemetry) !== null
          ? liveImuValue("roll", telemetry)
          : Number(cfg.bodyIK.actual.roll),
      pitch:
        liveAct && liveImuValue("pitch", telemetry) !== null
          ? liveImuValue("pitch", telemetry)
          : Number(cfg.bodyIK.actual.pitch),
      yaw:
        liveAct && liveImuValue("yaw", telemetry) !== null
          ? liveImuValue("yaw", telemetry)
          : Number(cfg.bodyIK.actual.yaw),
    };
    const des = cfg.bodyIK.desired;
    const Ra = eulerMatrix(act.roll, act.pitch, act.yaw);
    const Rd = eulerMatrix(des.roll, des.pitch, des.yaw);
    const desiredWorld = matMulVec(Rd, p);
    const corr = matTmulVec(Ra, desiredWorld);
    const dz = clamp(
      (corr.z - p.z) * cfg.bodyIK.gain,
      -cfg.bodyIK.maxZCorrection,
      cfg.bodyIK.maxZCorrection,
    );
    return vec(corr.x, corr.y, p.z + dz);
  }
  function solveIK(name, targetAbs) {
    const leg = cfg.legs[name],
      L = cfg.legCommon;
    const hip = vec(leg.hip.x, leg.hip.y, leg.hip.z);
    const rel = sub(targetAbs, hip);
    const outwardSign = leg.side === "right" ? -1 : 1;
    const ikX = rel.x;
    const ikYOut = outwardSign * rel.y;
    const ikZ = rel.z;
    const radial = Math.hypot(ikX, ikYOut);
    const rawR = radial - Number(L.coxa);
    const r = Math.max(5, rawR);
    const rawC = Math.hypot(ikZ, r);
    const cMin = Math.abs(Number(L.femur) - Number(L.tibia)) * 1.001;
    const cMax = (Number(L.femur) + Number(L.tibia)) * 0.999;
    const c = clamp(rawC, cMin, cMax);
    const cosFemur = clamp(
      (L.femur ** 2 + c ** 2 - L.tibia ** 2) / (2 * L.femur * c),
      -1,
      1,
    );
    const cosTibia = clamp(
      (L.femur ** 2 + L.tibia ** 2 - c ** 2) /
        (2 * L.femur * L.tibia),
      -1,
      1,
    );
    const geometryDeg = {
      coxa: deg(Math.atan2(ikYOut, ikX)),
      femur: deg(Math.atan2(r, -ikZ) + Math.acos(cosFemur)),
      tibia: 180 - deg(Math.acos(cosTibia)),
    };
    const rawServo = {
      coxa: geometryToServoRaw(name, "coxa", geometryDeg.coxa),
      femur: geometryToServoRaw(name, "femur", geometryDeg.femur),
      tibia: geometryToServoRaw(name, "tibia", geometryDeg.tibia),
    };
    const servo = {
      coxa: geometryToServo(name, "coxa", geometryDeg.coxa),
      femur: geometryToServo(name, "femur", geometryDeg.femur),
      tibia: geometryToServo(name, "tibia", geometryDeg.tibia),
    };
    const fk = fkFromServo(name, servo);
    const err = Math.hypot(
      fk.foot.x - targetAbs.x,
      fk.foot.y - targetAbs.y,
      fk.foot.z - targetAbs.z,
    );
    const limit = ["coxa", "femur", "tibia"].some(
      (j) =>
        rawServo[j] <= Number(leg.joints[j].min) + 0.5 ||
        rawServo[j] >= Number(leg.joints[j].max) - 0.5,
    );
    const reachable = rawR >= 5 && rawC >= cMin && rawC <= cMax;
    return {
      reachable,
      servo,
      rawServo,
      geometryDeg,
      jointDeg: {
        coxa: geometryDeg.coxa - 90,
        femur: geometryDeg.femur - 90,
        tibia: -geometryDeg.tibia,
      },
      err,
      limit,
      points: fk.points,
      foot: fk.foot,
      target: targetAbs,
    };
  }
  function fkFromServo(name, servo) {
    const leg = cfg.legs[name],
      L = cfg.legCommon;
    const hip = vec(leg.hip.x, leg.hip.y, leg.hip.z);
    const thetaCoxa = rad(servoToGeometry(name, "coxa", servo.coxa));
    const q2 = rad(servoToGeometry(name, "femur", servo.femur) - 90);
    const q3 = rad(-servoToGeometry(name, "tibia", servo.tibia));
    const outwardSign = leg.side === "right" ? -1 : 1;
    const u = vec(
      Math.cos(thetaCoxa),
      outwardSign * Math.sin(thetaCoxa),
      0,
    );
    const coxaEnd = add(hip, mul(u, L.coxa));
    const knee = add(
      coxaEnd,
      add(mul(u, L.femur * Math.cos(q2)), vec(0, 0, L.femur * Math.sin(q2))),
    );
    const foot = add(
      knee,
      add(
        mul(u, L.tibia * Math.cos(q2 + q3)),
        vec(0, 0, L.tibia * Math.sin(q2 + q3)),
      ),
    );
    return { points: [hip, coxaEnd, knee, foot], foot };
  }
  function contactForLeg(name) {
    if (
      liveModeActive() &&
      State.live.online &&
      Object.prototype.hasOwnProperty.call(State.live.contacts, name)
    ) {
      return !!State.live.contacts[name];
    }
    if (cfg.contacts.mode === "manual") return !!cfg.contacts.manual[name];
    const ph = frac(activeBasePhase() + gaitPhaseForLeg(name));
    return ph < cfg.gait.dutyFactor;
  }

  function renderScene() {
    if (!State.scene) return;
    if (State.gridHelper) {
      State.gridHelper.visible = cfg.visual.showGrid !== false;
      State.gridHelper.position.y = Number(cfg.contacts.groundZ || 0);
    }
    clear(State.robotGroup);
    clear(State.liveGroup);
    clear(State.sensorGroup);
    clear(State.helperGroup);
    State.legResults = {};

    const b = cfg.body;
    const bodyPos = vec(b.pose.x, b.pose.y, b.pose.z);
    const telemetry = State.live.telemetry || {};
    const useLiveImu =
      liveModeActive() && cfg.live.followImu && State.live.online;
    const displayRoll =
      useLiveImu && liveImuValue("roll", telemetry) !== null
        ? liveImuValue("roll", telemetry)
        : Number(b.pose.roll);
    // META-1'in Body IK pitch işareti ile ekrandaki REP-103 gövde pitch yönü
    // ters eksen sözleşmeleridir. Kinematik işareti değiştirmeden yalnızca
    // dünya sahnesindeki gövde/overlay yönünü burada çevir.
    const displayPitch =
      useLiveImu && liveImuValue("pitch", telemetry) !== null
        ? -liveImuValue("pitch", telemetry)
        : Number(b.pose.pitch);
    const displayYaw =
      useLiveImu && liveImuValue("yaw", telemetry) !== null
        ? liveImuValue("yaw", telemetry)
        : Number(b.pose.yaw);
    const bodyRotation = eulerMatrix(displayRoll, displayPitch, displayYaw);
    const toWorld = (p) => bodyToWorld(p, bodyPos, bodyRotation);

    const bg = new THREE.Group();
    State.robotGroup.add(bg);
    const body = box(
      bg,
      bodyPos,
      vec(b.size.length, b.size.width, b.size.height),
      C.dark,
      0.95,
    );
    const edges = new THREE.EdgesGeometry(body.geometry);
    body.add(
      new THREE.LineSegments(
        edges,
        new THREE.LineBasicMaterial({ color: C.red }),
      ),
    );
    setRobotRotation(body, bodyRotation);
    const comWorld = toWorld(vec(b.com.x, b.com.y, b.com.z));
    sphere(bg, comWorld, 10, C.orange, 1);
    label(
      bg,
      "COM",
      add(comWorld, matMulVec(bodyRotation, vec(0, 0, 18))),
      "#ffa500",
    );
    if (cfg.visual.showAxes) {
      arrowVector(bg, bodyPos, matMulVec(bodyRotation, vec(1, 0, 0)), 90, 0xff3333, "BX");
      arrowVector(bg, bodyPos, matMulVec(bodyRotation, vec(0, 1, 0)), 90, 0x00ff66, "BY");
      arrowVector(bg, bodyPos, matMulVec(bodyRotation, vec(0, 0, 1)), 90, 0x33aaff, "BZ");
    }

    const support = [];
    let okCount = 0,
      limitCount = 0,
      errMax = 0;
    LEG_ORDER.forEach((name) => {
      const leg = cfg.legs[name];
      const nominal = footTarget(name, true);
      const tgt = footTarget(name, false);
      const corr = correctedTarget(name, tgt);
      const modelRes = leg.useIK
        ? solveIK(name, corr)
        : (() => {
            const fk = fkFromServo(name, leg.manualAngles);
            return {
              ...fk,
              servo: { ...leg.manualAngles },
              reachable: true,
              err: 0,
              limit: false,
              target: corr,
              points: fk.points,
            };
          })();

      const liveServo = getLiveAngles(name);
      let displayRes = modelRes;
      if (cfg.live.mode === "live_drive" && liveServo) {
        const fk = fkFromServo(name, liveServo);
        displayRes = {
          ...modelRes,
          ...fk,
          servo: liveServo,
          points: fk.points,
          foot: fk.foot,
          liveDriven: true,
        };
      }
      State.legResults[name] = { ...modelRes, displayRes, liveServo };
      if (modelRes.reachable && !modelRes.limit) okCount++;
      if (modelRes.limit) limitCount++;
      errMax = Math.max(errMax, modelRes.err || 0);

      const pts = displayRes.points.map(toWorld);
      const legColor = leg.side === "left" ? C.cyan : C.orange;
      cyl(State.robotGroup, pts[0], pts[1], 8, legColor, 0.95);
      cyl(State.robotGroup, pts[1], pts[2], 7, 0xdddddd, 0.9);
      cyl(State.robotGroup, pts[2], pts[3], 7, C.red, 0.9);
      pts.forEach((p, i) =>
        sphere(
          State.robotGroup,
          p,
          i === 3 ? 8 : 7,
          i === 3 ? C.green : C.white,
          0.95,
        ),
      );
      label(State.robotGroup, name, add(pts[0], vec(0, 0, 22)), "#e60000");

      const nominalWorld = toWorld(nominal);
      const targetWorld = toWorld(tgt);
      const correctedWorld = toWorld(corr);
      sphere(State.helperGroup, nominalWorld, 5, C.grey, 0.45);
      sphere(State.helperGroup, targetWorld, 6, C.cyan, 0.65);
      if (cfg.visual.showCorrectedTargets) {
        sphere(State.helperGroup, correctedWorld, 7, C.violet, 0.8);
        line(State.helperGroup, [targetWorld, correctedWorld], C.violet, 0.7);
      }
      if (cfg.visual.showJointAxes) {
        const localU = sub(displayRes.points[1], displayRes.points[0]);
        const uLen = Math.hypot(localU.x, localU.y) || 1;
        const pitchAxisLocal = vec(localU.y / uLen, -localU.x / uLen, 0);
        const pitchAxisWorld = matMulVec(bodyRotation, pitchAxisLocal);
        arrowVector(
          State.helperGroup,
          pts[0],
          matMulVec(bodyRotation, vec(0, 0, 1)),
          38,
          C.red,
          "C+Z",
        );
        arrowVector(
          State.helperGroup,
          pts[1],
          pitchAxisWorld,
          34,
          C.cyan,
          "F",
        );
        arrowVector(
          State.helperGroup,
          pts[2],
          pitchAxisWorld,
          34,
          C.orange,
          "T",
        );
      }
      if (cfg.visual.showTrajectories) {
        const tr = [];
        for (let k = 0; k <= 32; k++) tr.push(footTarget(name, false, k / 32));
        line(State.helperGroup, tr.map(toWorld), legColor, 0.32);
      }

      if (cfg.live.mode === "live_overlay" && liveServo) {
        const liveFk = fkFromServo(name, liveServo);
        const livePoints = liveFk.points.map(toWorld);
        const alpha = State.live.movingLegs[name] ? 0.95 : 0.58;
        cyl(
          State.liveGroup,
          livePoints[0],
          livePoints[1],
          4.5,
          C.blue,
          alpha,
        );
        cyl(
          State.liveGroup,
          livePoints[1],
          livePoints[2],
          4.5,
          C.blue,
          alpha,
        );
        cyl(
          State.liveGroup,
          livePoints[2],
          livePoints[3],
          4.5,
          C.blue,
          alpha,
        );
        sphere(State.liveGroup, livePoints[3], 7, C.blue, alpha);
        line(State.liveGroup, [toWorld(modelRes.foot), livePoints[3]], C.blue, 0.5);
      }

      const contact = contactForLeg(name);
      if (contact) {
        support.push(pts[3]);
        sphere(
          State.helperGroup,
          add(pts[3], vec(0, 0, 4)),
          10,
          C.green,
          0.75,
        );
      }
    });

    if (cfg.visual.showSupportPolygon && support.length >= 3) {
      const cx = support.reduce((s, p) => s + p.x, 0) / support.length;
      const cy = support.reduce((s, p) => s + p.y, 0) / support.length;
      const sorted = support
        .slice()
        .sort(
          (a, b) =>
            Math.atan2(a.y - cy, a.x - cx) - Math.atan2(b.y - cy, b.x - cx),
        );
      sorted.push(sorted[0]);
      line(State.helperGroup, sorted, C.green, 0.9);
    }
    renderSensors(bodyPos, bodyRotation);
    updateStatus(okCount, limitCount, errMax);
    updateLiveWidgets();
    if (State.renderer) State.renderer.render(State.scene, State.camera);
  }
  function renderSensors(bodyPos, bodyRotation) {
    if (!cfg.visual.showSensors) return;
    SENSOR_ORDER.forEach((id) => {
      const s = cfg.sensors[id];
      if (!s?.visible) return;
      const p = bodyToWorld(
        vec(s.xyz.x, s.xyz.y, s.xyz.z),
        bodyPos,
        bodyRotation,
      );
      const color =
        s.type === "imu"
          ? C.orange
          : s.type === "lidar"
            ? C.cyan
            : s.type === "camera"
              ? C.blue
              : s.type === "tof"
                ? C.violet
                : C.green;
      sphere(State.sensorGroup, p, 10, color, 0.9);
      label(
        State.sensorGroup,
        id.toUpperCase(),
        add(p, vec(0, 0, 22)),
        "#ffffff",
      );
      arrow(State.sensorGroup, p, "+X", 42, color);
      if (s.type === "lidar") {
        const pts = [];
        for (let a = 0; a <= 360; a += 15)
          pts.push(
            add(p, vec(Math.cos(rad(a)) * 80, Math.sin(rad(a)) * 80, 0)),
          );
        pts.push(pts[0]);
        line(State.sensorGroup, pts, color, 0.45);
      }
      if (s.type === "camera") {
        line(
          State.sensorGroup,
          [
            p,
            add(p, vec(s.range || 520, 180, 90)),
            add(p, vec(s.range || 520, -180, 90)),
            p,
          ],
          color,
          0.55,
        );
        line(
          State.sensorGroup,
          [
            p,
            add(p, vec(s.range || 520, 180, -90)),
            add(p, vec(s.range || 520, -180, -90)),
            p,
          ],
          color,
          0.55,
        );
      }
      if (s.type === "tof") {
        line(
          State.sensorGroup,
          [p, add(p, vec(s.range || 700, 0, 0))],
          color,
          0.8,
        );
      }
    });
  }

  function input(label, path, type = "number", tip = "", opts = {}) {
    const id = "i_" + path.replace(/[^a-zA-Z0-9]/g, "_");
    const val = getPath(path);
    const extra =
      type === "range"
        ? `min="${opts.min ?? -250}" max="${opts.max ?? 250}" step="${opts.step ?? 1}"`
        : "";
    return `<div class="row"><label><span class="lbl">${label}${tip ? `<span class="hint" data-tip="${tip}">?</span>` : ""}</span>${type === "range" ? `<input id="${id}" data-path="${path}" type="range" value="${val}" ${extra}>` : `<input id="${id}" data-path="${path}" type="${type}" value="${val}">`}</label><input data-path="${path}" type="number" value="${val}"></div>`;
  }
  function select(label, path, options, tip = "") {
    const val = getPath(path);
    return `<div class="row"><label><span class="lbl">${label}${tip ? `<span class="hint" data-tip="${tip}">?</span>` : ""}</span><select data-path="${path}">${options.map((o) => `<option value="${o}" ${o == val ? "selected" : ""}>${o}</option>`).join("")}</select></label><span></span></div>`;
  }
  function section(id, title, summary, html) {
    return `<div class="control-section ${State.activeSection === id ? "active" : ""}" data-section="${id}"><div class="control-title"><span>${title}</span><small>${summary}</small></div><div class="control-content">${html}</div></div>`;
  }
  function infoGrid(items) {
    return `<div class="info-grid">${items.map(([k, v]) => `<div class="info-cell"><span>${k}</span><b>${v}</b></div>`).join("")}</div>`;
  }
  function layerButton(label, key) {
    return `<button onclick="App.toggleLayer('${key}')">${label}: ${cfg.visual[key] ? "ON" : "OFF"}</button>`;
  }
  function renderControls() {
    const leg = cfg.selected.leg,
      sensor = cfg.selected.sensor;
    const L = cfg.legs[leg],
      S = cfg.sensors[sensor];
    const sensorExtras =
      S.type === "lidar"
        ? `${input("Range mm", `sensors.${sensor}.range`, "number", "LiDAR görsel menzil sınırı.")}${input("Scan Angle", `sensors.${sensor}.angle`, "number", "LiDAR tarama açısı.")}`
        : S.type === "camera"
          ? `${input("Range mm", `sensors.${sensor}.range`, "number", "Kamera frustum görsel menzili.")}${input("FOV H", `sensors.${sensor}.fovH`, "number", "Yatay görüş açısı.")}${input("FOV V", `sensors.${sensor}.fovV`, "number", "Dikey görüş açısı.")}`
          : S.type === "tof"
            ? `${input("Range mm", `sensors.${sensor}.range`, "number", "TOF görsel ölçüm menzili.")}`
            : "";
    $("controlDeck").innerHTML = [
      section(
        "live",
        "00 LIVE TWIN",
        "SIM ↔ ROBOT",
        `<div class="group full-span"><div class="group-title">LIVE SOURCE / PROVENANCE</div><div class="group-subtitle">Lab, 8080 Command Center proxy'sinden önbellekli veri okur. ESP32'ye ek browser yükü bindirmez.</div>${select("Mode", "live.mode", LIVE_MODES, "simulation: tamamen model; live_overlay: model + mavi robot command-state; live_drive: 3D bacakları firmware command-state açıları sürer.")}${select("Compare Leg", "live.compareLeg", FIRMWARE_LEG_ORDER, "Eğri karşılaştırmasında izlenecek bacak.")}${select("Compare Joint", "live.compareJoint", JOINT_ORDER, "Model ve firmware command-state arasında karşılaştırılacak eklem.")}${input("Poll ms", "live.pollMs", "range", "Live Twin sorgu aralığı. 50–150 ms arası dinamik kullanım içindir.", { min: 50, max: 500, step: 10 })}${input("Interpolation ms", "live.interpolationTauMs", "range", "Yeni command-state örnekleri arasında 3D hareket yumuşatma zamanı.", { min: 25, max: 250, step: 5 })}<div class="row3">${select("Pitch Sign", "live.imuPitchSign", [-1, 1], "META-1 Body IK/servo kinematik pitch işareti -1'dir; görsel gövde yönü sahne dönüşümünde ayrıca eşlenir.").replace('class="row"', 'class="row wide"')}${select("Roll Sign", "live.imuRollSign", [-1, 1], "Roll yönü doğru olduğu için +1.").replace('class="row"', 'class="row wide"')}${select("Yaw Sign", "live.imuYawSign", [-1, 1], "MPU gyro-Z relatif yaw işareti.").replace('class="row"', 'class="row wide"')}</div><div class="row3"><button class="btn-cyan" onclick="App.toggleLiveOption('phaseSync')">PHASE SYNC: ${cfg.live.phaseSync ? "ON" : "OFF"}</button><button class="btn-orange" onclick="App.toggleLiveOption('followImu')">FOLLOW IMU: ${cfg.live.followImu ? "ON" : "OFF"}</button><button onclick="App.zeroLiveYaw()">ZERO LIVE YAW</button></div></div>
        <div class="group full-span"><div class="group-title">LIVE METRICS</div><div class="live-grid"><div class="live-card"><span>LINK</span><b id="liveLink">--</b></div><div class="live-card"><span>ROBOT MODE</span><b id="liveRobotMode">--</b></div><div class="live-card"><span>REAL CYCLE</span><b id="liveRealCycle">--</b></div><div class="live-card"><span>SIM CYCLE</span><b id="liveSimCycle">--</b></div><div class="live-card"><span>PHASE Δ</span><b id="livePhaseErr">--</b></div><div class="live-card"><span>ANGLE RMSE</span><b id="liveRmse">--</b></div><div class="live-card"><span>ELAPSED</span><b id="liveElapsed">--</b></div><div class="live-card"><span>CONTACT SOURCE</span><b id="liveContactSource">--</b></div><div class="live-card"><span>TOF</span><b id="liveTof">--</b></div><div class="live-card"><span>BATTERIES</span><b id="liveBattery">--</b></div></div><div class="contact-live-grid">${FIRMWARE_LEG_ORDER.map((n) => `<div class="contact-live" id="liveContact${n}">${n} --</div>`).join("")}</div><div class="row2"><button onclick="App.resetLiveMetrics()">RESET TIMING</button><button onclick="App.clearLiveHistory()">CLEAR CURVE HISTORY</button></div></div>
        <div class="group full-span"><div class="source-note"><strong>VERİ KAYNAĞI:</strong> IMU / TOF / batarya = gerçek sensör telemetrisi. Gait phase = firmware kontrol durumu. Servo angle = servo encoder ölçümü değil; firmware'in smoothing sonrası command-state açısı. Fiziksel foot-switch alanı telemetride bulunursa otomatik kullanılır; bulunmazsa temas <strong>PHASE-ESTIMATED</strong> olarak açıkça etiketlenir.</div></div>`,
      ),
      section(
        "body",
        "01 BODY / FRAME",
        "XYZ + RPY",
        `<div class="group"><div class="group-title">BODY POSE</div>${input("Body X", "body.pose.x", "range", "Gövdenin dünya X konumu.", { min: -300, max: 300 })}${input("Body Y", "body.pose.y", "range", "Gövdenin dünya Y konumu.", { min: -300, max: 300 })}${input("Body Z", "body.pose.z", "range", "Gövde yüksekliği.", { min: 20, max: 320 })}${input("Roll", "body.pose.roll", "range", "Sağa/sola yatma.", { min: -45, max: 45 })}${input("Pitch", "body.pose.pitch", "range", "Öne/arkaya eğilme.", { min: -45, max: 45 })}${input("Yaw", "body.pose.yaw", "range", "Gövdenin dönüşü.", { min: -180, max: 180 })}</div><div class="group"><div class="group-title">BODY SIZE / COM</div><div class="group-subtitle">Gövde ölçüsü, kütle ve ağırlık merkezi referansı. Dünya zemini robot Z=0 düzlemidir.</div><div class="row3">${input("L", "body.size.length").replace('class="row"', 'class="row wide"')}${input("W", "body.size.width").replace('class="row"', 'class="row wide"')}${input("H", "body.size.height").replace('class="row"', 'class="row wide"')}</div>${input("Body Mass", "body.mass", "number", "Gövde kütlesi; ileride gerçek dinamik modele aktarılabilir.")}<div class="row3">${input("COM X", "body.com.x").replace('class="row"', 'class="row wide"')}${input("COM Y", "body.com.y").replace('class="row"', 'class="row wide"')}${input("COM Z", "body.com.z").replace('class="row"', 'class="row wide"')}</div><div class="frame-note"><strong>WORLD FRAME:</strong> +X ileri · +Y sol · +Z yukarı. Grid, ground Z değerine bağlı gerçek XY zemin düzlemidir.</div></div>`,
      ),
      section(
        "legs",
        "02 LEG GEOMETRY",
        "LINK + AXIS",
        `<div class="group"><div class="group-title">COMMON LEG GEOMETRY</div>${infoGrid(
          [
            ["COXA", fmt(cfg.legCommon.coxa, 2) + " mm"],
            ["FEMUR", fmt(cfg.legCommon.femur, 2) + " mm"],
            ["TIBIA", fmt(cfg.legCommon.tibia, 2) + " mm"],
          ],
        )}${input("Coxa Length", "legCommon.coxa", "range", "Hip yaw ekseni ile femur ekseni arası.", { min: 10, max: 120 })}${input("Femur Length", "legCommon.femur", "range", "Üst bacak link uzunluğu.", { min: 40, max: 240 })}${input("Tibia Length", "legCommon.tibia", "range", "Alt bacak link uzunluğu.", { min: 50, max: 280 })}${select("Femur/Tibia Bend", "legCommon.bend", ["outward", "inward"], "Femur/tibia çözümlerinin dışa mı içe mi katlanacağını belirler.")}</div><div class="group"><div class="group-title">SELECTED LEG</div>${select("Leg", "selected.leg", LEG_ORDER, "Ayar yapılacak bacağı seç.")}${input("Hip X", `legs.${leg}.hip.x`, "number", "Bacağın gövdeye bağlandığı X noktası.")}${input("Hip Y", `legs.${leg}.hip.y`, "number", "Bacağın gövdeye bağlandığı Y noktası.")}${input("Hip Z", `legs.${leg}.hip.z`, "number", "Bacağın gövdeye bağlandığı Z noktası.")}<div class="row3">${select("C axis", `legs.${leg}.joints.coxa.axis`, AXES, "Coxa servo fiziksel dönüş ekseni.").replace('class="row"', 'class="row wide"')}${select("F axis", `legs.${leg}.joints.femur.axis`, AXES, "Femur servo fiziksel dönüş ekseni.").replace('class="row"', 'class="row wide"')}${select("T axis", `legs.${leg}.joints.tibia.axis`, AXES, "Tibia servo fiziksel dönüş ekseni.").replace('class="row"', 'class="row wide"')}</div><div class="row3">${input("C dir", `legs.${leg}.joints.coxa.dir`).replace('class="row"', 'class="row wide"')}${input("F dir", `legs.${leg}.joints.femur.dir`).replace('class="row"', 'class="row wide"')}${input("T dir", `legs.${leg}.joints.tibia.dir`).replace('class="row"', 'class="row wide"')}</div><div class="row3">${input("C zero", `legs.${leg}.joints.coxa.zero`).replace('class="row"', 'class="row wide"')}${input("F zero", `legs.${leg}.joints.femur.zero`).replace('class="row"', 'class="row wide"')}${input("T zero", `legs.${leg}.joints.tibia.zero`).replace('class="row"', 'class="row wide"')}</div><div class="group-subtitle">Servo güvenli limitleri 5°–175° tutulur; yön ve zero değerleri IK açısını gerçek servo komutuna dönüştürür.</div><div class="row3">${input("C min", `legs.${leg}.joints.coxa.min`).replace('class="row"', 'class="row wide"')}${input("F min", `legs.${leg}.joints.femur.min`).replace('class="row"', 'class="row wide"')}${input("T min", `legs.${leg}.joints.tibia.min`).replace('class="row"', 'class="row wide"')}</div><div class="row3">${input("C max", `legs.${leg}.joints.coxa.max`).replace('class="row"', 'class="row wide"')}${input("F max", `legs.${leg}.joints.femur.max`).replace('class="row"', 'class="row wide"')}${input("T max", `legs.${leg}.joints.tibia.max`).replace('class="row"', 'class="row wide"')}</div></div>`,
      ),
      section(
        "ik",
        "03 IK TEST",
        "TARGET XYZ",
        `<div class="group"><div class="group-title">FOOT TARGET — ${leg}</div>${input("Target X", `legs.${leg}.target.x`, "range", "IK hedefinin gövde frame X değeri.", { min: -320, max: 320 })}${input("Target Y", `legs.${leg}.target.y`, "range", "IK hedefinin gövde frame Y değeri.", { min: -300, max: 300 })}${input("Target Z", `legs.${leg}.target.z`, "range", "IK hedefinin gövde frame Z değeri.", { min: -280, max: -20 })}<div class="row2"><button class="btn-cyan" onclick="App.copyTargetMirror()">MIRROR TO OTHER SIDE</button><button class="btn-orange" onclick="App.homeTargets()">RESET STANCE</button></div></div><div class="group"><div class="group-title">MANUAL SERVO CHECK</div><div class="row3">${input("C angle", `legs.${leg}.manualAngles.coxa`, "range", "Manuel coxa servo komutu.", { min: 0, max: 180 }).replace('class="row"', 'class="row wide"')}${input("F angle", `legs.${leg}.manualAngles.femur`, "range", "Manuel femur servo komutu.", { min: 0, max: 180 }).replace('class="row"', 'class="row wide"')}${input("T angle", `legs.${leg}.manualAngles.tibia`, "range", "Manuel tibia servo komutu.", { min: 0, max: 180 }).replace('class="row"', 'class="row wide"')}</div><div class="row2"><button onclick="App.toggleLegIK()">IK / MANUAL TOGGLE</button><button onclick="App.selectedLegHome()">SELECTED HOME</button></div></div>`,
      ),
      section(
        "gait",
        "04 GAIT STUDIO",
        "MULTI GAIT",
        `<div class="group"><div class="group-title">GAIT GENERATOR</div>${select("Gait Type", "gait.type", GAITS, "Tripod/ripple/wave/tetrapod/metachronal/crawl/custom seçimi.")}${input("Frequency", "gait.frequency", "range", "Yürüyüş frekansı.", { min: 0.1, max: 2.5, step: 0.05 })}${input("Step Length", "gait.stepLength", "range", "Adım uzunluğu.", { min: 0, max: 180 })}${input("Step Width", "gait.stepWidth", "range", "Yanal adım genişliği.", { min: -80, max: 80 })}${input("Step Height", "gait.stepHeight", "range", "Swing fazında ayak kalkma yüksekliği.", { min: 0, max: 120 })}${input("Duty Factor", "gait.dutyFactor", "range", "Ayağın yerde kalma oranı.", { min: 0.2, max: 0.85, step: 0.01 })}${input("Body Height", "gait.bodyHeight", "range", "Gait üreticisindeki gövde yüksekliği referansı.", { min: 60, max: 260 })}${input("Turn Rate", "gait.turnRate", "range", "Dönüş komutu / diferansiyel adım etkisi.", { min: -1, max: 1, step: 0.01 })}<div class="row2"><button class="btn-red" onclick="App.togglePlay()">PLAY / PAUSE</button><button onclick="App.stepFrame()">STEP FRAME</button></div></div><div class="group"><div class="group-title">CUSTOM PHASES</div><div class="row3">${LEG_ORDER.map((n) => input(n, `gait.phases.${n}`, "number", "0–1 arası faz offseti.").replace('class="row"', 'class="row wide"')).join("")}</div><button class="btn-cyan" onclick="App.applyClassicTripod()">FIRMWARE TRIPOD: RF+RR+LM</button></div>`,
      ),
      section(
        "motion",
        "05 SERVO CURVE",
        "PROFILE + LIVE COMPARE",
        `<div class="group"><div class="group-title">MOTION PROFILE</div>${select("Curve", "servoMotion.curve", CURVES, "Servo hedeflerine giderken kullanılacak hareket eğrisi.")}${input("Duration ms", "servoMotion.durationMs", "range", "Hareket geçiş süresi.", { min: 80, max: 2500 })}${input("Max Speed deg/s", "servoMotion.maxSpeedDegS", "range", "Servo hız sınırı.", { min: 40, max: 900 })}${input("Accel Limit", "servoMotion.accelLimit", "range", "İvme sınırı proxy değeri.", { min: 50, max: 2500 })}${input("Stiffness", "servoMotion.stiffness", "range", "Sertlik arttıkça hızlı cevap ama daha yüksek pik akım beklenir.", { min: 0, max: 1, step: 0.01 })}${input("Smoothness", "servoMotion.smoothness", "range", "Yumuşaklık arttıkça jerk ve akım piki düşer.", { min: 0, max: 1, step: 0.01 })}${input("Deadband deg", "servoMotion.deadbandDeg", "range", "Küçük hedef farklarında hareketi bastıran ölü bant proxy'si.", { min: 0, max: 8, step: 0.1 })}</div>
        <div class="group"><div class="group-title">CURRENT / ENERGY PROXY WEIGHTS</div>${input("Current Weight", "servoMotion.currentWeight", "range", "Açı/hız kaynaklı akım proxy ağırlığı.", { min: 0.1, max: 3, step: 0.05 })}${input("Jerk Weight", "servoMotion.jerkWeight", "range", "Eğri jerk etkisinin enerji proxy ağırlığı.", { min: 0, max: 3, step: 0.05 })}${input("Hold Torque Weight", "servoMotion.holdTorqueWeight", "range", "Gövde yüksekliği ve adım yüksekliği kaynaklı tutma torku proxy ağırlığı.", { min: 0, max: 3, step: 0.05 })}${infoGrid(
          [
            ["PROFILE", cfg.servoMotion.preset || "custom"],
            ["CURVE", cfg.servoMotion.curve],
            ["ENERGY", fmt(energyIndex(), 2)],
          ],
        )}<div class="row4"><button onclick="App.motionPreset('low_current')">LOW CURRENT</button><button onclick="App.motionPreset('smooth_walk')">SMOOTH</button><button onclick="App.motionPreset('fast_response')">FAST</button><button onclick="App.motionPreset('stiff_test')">STIFF</button></div></div>
        <div class="group full-span"><div class="group-title">CURVE LAYERS</div><div class="group-subtitle">Üst grafikte position / velocity / acceleration / jerk; alt grafikte seçili eklem için model ve robot command-state karşılaştırması.</div><div class="curve-layer-grid"><label><input type="checkbox" data-curve-layer="showPosition" ${cfg.servoMotion.showPosition ? "checked" : ""}>POSITION</label><label><input type="checkbox" data-curve-layer="showVelocity" ${cfg.servoMotion.showVelocity ? "checked" : ""}>VELOCITY</label><label><input type="checkbox" data-curve-layer="showAcceleration" ${cfg.servoMotion.showAcceleration ? "checked" : ""}>ACCEL</label><label><input type="checkbox" data-curve-layer="showJerk" ${cfg.servoMotion.showJerk ? "checked" : ""}>JERK</label><label><input type="checkbox" data-curve-layer="showLiveOverlay" ${cfg.servoMotion.showLiveOverlay ? "checked" : ""}>LIVE COMPARE</label></div></div>`,
      ),
      section(
        "bodyik",
        "06 BODY IK",
        "ORIENTATION COMP",
        `<div class="group"><div class="group-title">ACTUAL ORIENTATION</div><div class="group-subtitle">IMU'dan geliyormuş gibi kullanılan gerçek gövde yönelimi.</div>${input("Actual Roll", "bodyIK.actual.roll", "range", "IMU roll değeri gibi düşün.", { min: -35, max: 35 })}${input("Actual Pitch", "bodyIK.actual.pitch", "range", "IMU pitch değeri gibi düşün.", { min: -35, max: 35 })}${input("Actual Yaw", "bodyIK.actual.yaw", "range", "Yaw düzeltme testi.", { min: -90, max: 90 })}</div><div class="group"><div class="group-title">DESIRED ORIENTATION</div><div class="group-subtitle">Ayaklar dünya frame'inde sabit tutulurken hedeflenen gövde yönelimi.</div>${input("Desired Roll", "bodyIK.desired.roll", "range", "Hedef roll.", { min: -35, max: 35 })}${input("Desired Pitch", "bodyIK.desired.pitch", "range", "Hedef pitch.", { min: -35, max: 35 })}${input("Desired Yaw", "bodyIK.desired.yaw", "range", "Hedef yaw.", { min: -90, max: 90 })}</div><div class="group full-span"><div class="group-title">COMPENSATION</div>${input("Gain", "bodyIK.gain", "range", "0 kapalı, 1 tam düzeltme.", { min: 0, max: 1.5, step: 0.01 })}${input("Max Z Correction", "bodyIK.maxZCorrection", "range", "Ayak başına izin verilen maksimum z düzeltmesi.", { min: 0, max: 120 })}<div class="row2"><button onclick="App.toggleBodyIK()">BODY IK ON/OFF</button><button onclick="App.zeroBodyIK()">ZERO ORIENTATION</button></div></div>`,
      ),
      section(
        "sensors",
        "07 SENSOR MOUNTS",
        "IMU/LIDAR/CAM",
        `<div class="group"><div class="group-title">SENSOR SELECT</div>${select("Sensor", "selected.sensor", SENSOR_ORDER, "Yerleşimi ayarlanacak sensör.")}${input("X", `sensors.${sensor}.xyz.x`, "number", "Sensör x montaj konumu.")}${input("Y", `sensors.${sensor}.xyz.y`, "number", "Sensör y montaj konumu.")}${input("Z", `sensors.${sensor}.xyz.z`, "number", "Sensör z montaj konumu.")}<div class="row3">${input("Roll", `sensors.${sensor}.rpy.roll`).replace('class="row"', 'class="row wide"')}${input("Pitch", `sensors.${sensor}.rpy.pitch`).replace('class="row"', 'class="row wide"')}${input("Yaw", `sensors.${sensor}.rpy.yaw`).replace('class="row"', 'class="row wide"')}</div>${sensorExtras}<button onclick="App.toggleSensorVisible()">VISIBLE TOGGLE</button></div><div class="group"><div class="group-title">SCENE LAYERS</div><div class="group-subtitle">Sensörleri ve mekanik yardımcı katmanları sahnede ayrı ayrı aç/kapat.</div><div class="row2">${layerButton("SENSORS", "showSensors")}${layerButton("JOINT AXES", "showJointAxes")}</div><div class="row2">${layerButton("LABELS", "showLabels")}${layerButton("TARGETS", "showCorrectedTargets")}</div></div>`,
      ),
      section(
        "contact",
        "08 CONTACT / SWITCH",
        "MICRO SWITCH",
        `<div class="group"><div class="group-title">CONTACT SIMULATION</div>${select("Mode", "contacts.mode", ["gait_phase", "manual"], "Mikro switch temasını gait fazından mı manuel mi alalım?")}${input("Ground Z", "contacts.groundZ", "number", "Yer seviyesi referansı.")}${input("Switch Trigger", "contacts.switchTrigger", "number", "Switch tetikleme toleransı.")}<div class="contact-grid">${LEG_ORDER.map((n) => `<div class="contact ${cfg.contacts.manual[n] ? "on" : ""}" data-contact="${n}">${n}</div>`).join("")}</div></div><div class="group"><div class="group-title">GROUND / SUPPORT VISUALS</div><div class="group-subtitle">Grid artık robot Z=0 dünya zeminidir; Ground Z değiştirilirse grid de aynı düzleme taşınır.</div><div class="row2">${layerButton("WORLD GRID", "showGrid")}${layerButton("SUPPORT POLYGON", "showSupportPolygon")}</div><div class="row2">${layerButton("TRAJECTORIES", "showTrajectories")}${layerButton("BODY AXES", "showAxes")}</div></div>`,
      ),
      section(
        "export",
        "09 EXPORT / SAVE",
        "JSON CONFIG",
        `<div class="group"><div class="group-title">CONFIG OUTPUT</div>${infoGrid(
          [
            ["ROBOT", cfg.meta.name || "META-1"],
            ["VERSION", cfg.meta.version || "--"],
            ["UNIT", cfg.meta.unit || "mm"],
          ],
        )}<div class="row2"><button class="btn-cyan" onclick="App.saveConfig()">SAVE SERVER</button><button class="btn-orange" onclick="App.downloadConfig()">DOWNLOAD JSON</button></div><button onclick="App.copyConfig()">COPY JSON TO CLIPBOARD</button><button class="btn-red" onclick="App.reloadConfig()">RELOAD DEFAULT</button><div class="frame-note"><strong>NOT:</strong> Export; body, leg geometry, servo axes/directions, gait, Body IK, contact ve sensor mount ayarlarının tamamını içerir.</div></div>`,
      ),
    ].join("");
    bindControls();
  }
  function activateSection(id, pin = false) {
    State.activeSection = id;
    if (pin) State.sectionPinned = State.sectionPinned === id ? null : id;
    State.menuSwitchUntil = performance.now() + 220;
    document.querySelectorAll(".control-section").forEach((sec) => {
      sec.classList.toggle("active", sec.dataset.section === id);
      sec.classList.toggle(
        "pinned",
        sec.dataset.section === State.sectionPinned,
      );
    });
  }
  function bindControls() {
    const deck = $("controlDeck");
    if (deck) {
      deck.addEventListener("pointermove", (e) => {
        State.lastPointer = { x: e.clientX, y: e.clientY };
      });
    }
    document.querySelectorAll(".control-section").forEach((sec) => {
      sec.addEventListener("pointerenter", (e) => {
        State.lastPointer = { x: e.clientX, y: e.clientY };
        clearTimeout(State.hoverTimer);
        if (State.sectionPinned || performance.now() < State.menuSwitchUntil)
          return;
        State.hoverTimer = setTimeout(() => {
          if (State.sectionPinned || !State.lastPointer) return;
          const under = document
            .elementFromPoint(State.lastPointer.x, State.lastPointer.y)
            ?.closest(".control-section");
          if (under === sec) activateSection(sec.dataset.section, false);
        }, 180);
      });
      sec.addEventListener("pointerleave", () =>
        clearTimeout(State.hoverTimer),
      );
      sec.querySelector(".control-title")?.addEventListener("click", (e) => {
        e.preventDefault();
        activateSection(sec.dataset.section, true);
      });
    });

    document.querySelectorAll("[data-path]").forEach((el) => {
      el.addEventListener("input", () => {
        setPath(el.dataset.path, el.value);
        syncSamePath(el);
        const rerenderKeys = [
          "selected.leg",
          "selected.sensor",
          "legCommon.bend",
          "gait.type",
          "live.mode",
          "live.compareLeg",
          "live.compareJoint",
        ];
        if (rerenderKeys.includes(el.dataset.path)) {
          State.live.history = [];
          renderControls();
        }
        renderScene();
        drawCurve();
      });
    });
    document.querySelectorAll("[data-curve-layer]").forEach((el) => {
      el.addEventListener("change", () => {
        cfg.servoMotion[el.dataset.curveLayer] = !!el.checked;
        drawCurve();
      });
    });
    document.querySelectorAll("[data-tip]").forEach((h) => {
      h.addEventListener("mouseenter", (e) => Tip.show(h.dataset.tip, e));
      h.addEventListener("mousemove", (e) => Tip.move(e));
      h.addEventListener("mouseleave", () => Tip.hide());
    });
    document.querySelectorAll("[data-contact]").forEach((el) =>
      el.addEventListener("click", () => {
        const n = el.dataset.contact;
        cfg.contacts.manual[n] = !cfg.contacts.manual[n];
        el.classList.toggle("on", cfg.contacts.manual[n]);
        renderScene();
      }),
    );
    updateLiveWidgets();
  }
  function syncSamePath(src) {
    document
      .querySelectorAll(`[data-path="${src.dataset.path}"]`)
      .forEach((e) => {
        if (e !== src) e.value = src.value;
      });
  }
  function updateStatus(okCount, limitCount, errMax) {
    $("tickerGait").textContent = cfg.gait.type.toUpperCase();
    $("tickerMode").textContent = liveModeActive()
      ? cfg.live.mode.toUpperCase()
      : State.playing
        ? "DYNAMIC"
        : "STATIC";
    $("tickerIK").textContent = `${okCount}/6 OK`;
    $("tickerEnergy").textContent = fmt(energyIndex(), 2);
    const t = State.live.telemetry || {};
    const phaseTicker = $("tickerPhase");
    if (phaseTicker)
      phaseTicker.textContent = finite(t.gaitPhase)
        ? `${fmt(State.live.phaseError * 100, 1)}%`
        : "--";

    const lines = [];
    LEG_ORDER.forEach((n) => {
      const r = State.legResults[n];
      if (!r) return;
      const m = r.servo;
      const live = r.liveServo;
      const liveText = live
        ? ` | CMD C:${fmt(live.coxa, 1)} F:${fmt(live.femur, 1)} T:${fmt(live.tibia, 1)}`
        : "";
      const move = State.live.movingLegs[n] ? " MOVE" : "";
      lines.push(
        `${n} MODEL C:${fmt(m.coxa, 1)} F:${fmt(m.femur, 1)} T:${fmt(m.tibia, 1)}${liveText} | err:${fmt(r.err || 0, 1)}mm | ${r.reachable ? "OK" : "UNREACH"}${r.limit ? " LIMIT" : ""}${move}`,
      );
    });
    $("ikOutput").textContent = lines.join("\n");

    const warn = [];
    if (limitCount) warn.push(`${limitCount} bacak servo limitine yakın.`);
    if (errMax > 30) warn.push(`IK/FK hata yüksek: ${fmt(errMax, 1)} mm.`);
    if (!cfg.bodyIK.enabled) warn.push("Body IK compensation kapalı.");
    if (liveModeActive() && !State.live.online)
      warn.push(
        `LIVE OFFLINE: ${State.live.error || "8080 telemetri bekleniyor"}`,
      );
    if (State.live.online) {
      const realCycle = liveCycleMs();
      const simCycle = simulationCycleMs();
      warn.push(
        `LIVE ${t.mode || "--"} | elapsed ${fmt(performance.now() / 1000 - State.live.connectedAt, 1)}s | gaitPhase ${finite(t.gaitPhase) ? fmt(t.gaitPhase, 3) : "--"}`,
      );
      if (realCycle !== null)
        warn.push(
          `Cycle real ${fmt(realCycle, 0)} ms vs sim ${fmt(simCycle, 0)} ms | Δ ${fmt(((realCycle - simCycle) / simCycle) * 100, 1)}%`,
        );
      if (finite(t.gaitPhase))
        warn.push(
          `Phase Δ ${fmt(State.live.phaseError * 100, 1)}% ≈ ${fmt(State.live.phaseError * simCycle, 0)} ms`,
        );
      const rmse = selectedRmse();
      if (rmse !== null)
        warn.push(
          `${cfg.live.compareLeg}/${cfg.live.compareJoint.toUpperCase()} model ↔ command-state RMSE: ${fmt(rmse, 2)}°`,
        );
      warn.push(
        `CONTACT SOURCE: ${State.live.contactSource}${State.live.contactSource === "PHASE-ESTIMATED" ? " (fiziksel switch ölçümü değildir)" : ""}`,
      );
      if (State.live.contactSource === "LIVE SENSOR")
        warn.push(
          `Physical contact ↔ phase mismatch: ${State.live.contactMismatch}/6`,
        );
      warn.push(
        "SERVO CMD: firmware command-state; fiziksel encoder feedback değildir.",
      );
    }
    $("warnOutput").textContent = warn.length ? warn.join("\n") : "Uyarı yok.";
  }
  function curveValue(t) {
    t = clamp(t, 0, 1);
    switch (cfg.servoMotion.curve) {
      case "linear":
        return t;
      case "smoothstep":
        return t * t * (3 - 2 * t);
      case "cosine":
        return 0.5 - 0.5 * Math.cos(Math.PI * t);
      case "cubic_bezier":
        return 3 * (1 - t) * t * t * 0.82 + t * t * t;
      case "minimum_jerk":
        return 10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5;
      case "trapezoid": {
        const a = 0.25,
          v = 1 / (1 - a),
          k = v / (2 * a);
        return t < a
          ? k * t * t
          : t > 1 - a
            ? 1 - k * (1 - t) * (1 - t)
            : k * a * a + v * (t - a);
      }
      case "s_curve": {
        const f = (x) => 1 / (1 + Math.exp(-10 * (x - 0.5)));
        const a = f(0),
          b = f(1);
        return (f(t) - a) / (b - a);
      }
      default:
        return t;
    }
  }
  function curveJerkFactor(name) {
    return (
      {
        linear: 1.35,
        trapezoid: 1.05,
        cubic_bezier: 0.82,
        cosine: 0.72,
        smoothstep: 0.68,
        s_curve: 0.62,
        minimum_jerk: 0.52,
      }[name] || 0.85
    );
  }
  function energyIndex() {
    const s = cfg.servoMotion;
    const motion =
      (Number(s.maxSpeedDegS || 260) / 260) * Number(s.currentWeight || 1);
    const jerk =
      curveJerkFactor(s.curve) *
      Number(s.jerkWeight || 0.8) *
      (1.25 - Number(s.smoothness || 0.7) * 0.45);
    const accel =
      (Number(s.accelLimit || 800) / 800) * (0.25 + Number(s.stiffness || 0.5));
    const hold =
      (Number(cfg.gait.stepHeight || 45) / 45 +
        Number(cfg.gait.bodyHeight || 145) / 145) *
      Number(s.holdTorqueWeight || 0.6);
    const deadbandRelief = 1 - clamp(Number(s.deadbandDeg || 1) / 20, 0, 0.25);
    return (motion + jerk + accel * 0.4 + hold * 0.5) * deadbandRelief;
  }
  function derivativeSeries(values, dt) {
    const out = new Array(values.length).fill(0);
    for (let i = 1; i < values.length - 1; i++)
      out[i] = (values[i + 1] - values[i - 1]) / (2 * dt);
    out[0] = (values[1] - values[0]) / dt;
    out[out.length - 1] = (values.at(-1) - values.at(-2)) / dt;
    return out;
  }
  function normalizeSeries(values) {
    const max = Math.max(1e-9, ...values.map((x) => Math.abs(x)));
    return values.map((x) => x / max);
  }
  function drawTrace(
    ctx,
    values,
    x0,
    y0,
    w,
    h,
    color,
    signed = false,
    width = 2,
  ) {
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.beginPath();
    values.forEach((v, i) => {
      const x = x0 + (i / (values.length - 1)) * w;
      const y = signed
        ? y0 + h * 0.5 - v * h * 0.42
        : y0 + h - clamp(v, 0, 1) * h;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
  function drawCurve() {
    const can = $("curveCanvas");
    if (!can || !cfg) return;
    const ctx = can.getContext("2d"),
      w = can.width,
      h = can.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#050505";
    ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = "#1f1f1f";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 10; i++) {
      const x = (i / 10) * w;
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    for (let i = 0; i <= 8; i++) {
      const y = (i / 8) * h;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    const N = 220,
      dt = 1 / N;
    const pos = Array.from({ length: N + 1 }, (_, i) => curveValue(i / N));
    const velRaw = derivativeSeries(pos, dt);
    const accRaw = derivativeSeries(velRaw, dt);
    const jerkRaw = derivativeSeries(accRaw, dt);
    const vel = normalizeSeries(velRaw),
      acc = normalizeSeries(accRaw),
      jerk = normalizeSeries(jerkRaw);
    const topY = 35,
      topH = h * 0.52;
    if (cfg.servoMotion.showPosition)
      drawTrace(ctx, pos, 24, topY, w - 48, topH, "#e60000", false, 3);
    if (cfg.servoMotion.showVelocity)
      drawTrace(ctx, vel, 24, topY, w - 48, topH, "#00e5b0", true, 2);
    if (cfg.servoMotion.showAcceleration)
      drawTrace(ctx, acc, 24, topY, w - 48, topH, "#ffa500", true, 2);
    if (cfg.servoMotion.showJerk)
      drawTrace(ctx, jerk, 24, topY, w - 48, topH, "#9d4edd", true, 2);

    ctx.fillStyle = "#bbb";
    ctx.font = "16px Share Tech Mono";
    ctx.fillText(
      `PROFILE ${cfg.servoMotion.curve} | duration ${cfg.servoMotion.durationMs}ms | energy ${fmt(energyIndex(), 2)}`,
      18,
      21,
    );
    ctx.fillStyle = "#e60000";
    ctx.fillText("POS", w - 330, 21);
    ctx.fillStyle = "#00e5b0";
    ctx.fillText("VEL", w - 285, 21);
    ctx.fillStyle = "#ffa500";
    ctx.fillText("ACC", w - 240, 21);
    ctx.fillStyle = "#9d4edd";
    ctx.fillText("JERK", w - 195, 21);

    const lowerY = Math.floor(h * 0.62),
      lowerH = h - lowerY - 24;
    ctx.strokeStyle = "#444";
    ctx.beginPath();
    ctx.moveTo(0, lowerY - 10);
    ctx.lineTo(w, lowerY - 10);
    ctx.stroke();
    const hist = State.live.history;
    const jointCfg =
      cfg.legs[cfg.live.compareLeg]?.joints?.[cfg.live.compareJoint];
    const lo = Number(jointCfg?.min ?? 5),
      hi = Number(jointCfg?.max ?? 175),
      span = Math.max(1, hi - lo);
    if (cfg.servoMotion.showLiveOverlay && hist.length >= 2) {
      const t0 = hist[0].ts,
        t1 = Math.max(t0 + 0.01, hist.at(-1).ts);
      const drawHistory = (key, color, width) => {
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.beginPath();
        hist.forEach((p, i) => {
          const x = 24 + ((p.ts - t0) / (t1 - t0)) * (w - 48);
          const y =
            lowerY + lowerH - clamp((p[key] - lo) / span, 0, 1) * lowerH;
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.stroke();
      };
      drawHistory("model", "#ffa500", 2.5);
      drawHistory("robot", "#00ff66", 2.5);
    }
    const rmse = selectedRmse();
    ctx.fillStyle = "#bbb";
    ctx.fillText(
      `LIVE COMPARE ${cfg.live.compareLeg}/${cfg.live.compareJoint.toUpperCase()} | MODEL orange | ROBOT CMD green | RMSE ${rmse === null ? "--" : fmt(rmse, 2) + "°"}`,
      18,
      lowerY + 12,
    );
    if (!hist.length) {
      ctx.fillStyle = "#666";
      ctx.fillText("Live history bekleniyor...", 18, lowerY + 40);
    }
  }
  function animate(ts) {
    requestAnimationFrame(animate);
    const dt = State.lastTs ? Math.min(0.1, (ts - State.lastTs) / 1000) : 0;
    State.lastTs = ts;
    if (State.playing || liveModeActive())
      State.clock = (State.clock + dt) % 10000;
    advanceLiveInterpolation(dt);

    const dynamicScene = State.playing || liveModeActive();
    if (dynamicScene && ts - State.lastSceneRenderTs >= 50) {
      State.lastSceneRenderTs = ts;
      renderScene();
    } else if (State.renderer && State.scene) {
      State.renderer.render(State.scene, State.camera);
    }
    if (liveModeActive() && ts - State.lastCurveRenderTs >= 120) {
      State.lastCurveRenderTs = ts;
      drawCurve();
    }
  }
  async function loadConfig() {
    const r = await fetch("/api/config", { cache: "no-store" });
    cfg = await r.json();
    normalizeConfig();
    if (!State.scene) initScene();
    renderControls();
    renderScene();
    drawCurve();
    startLivePolling();
    if (!State.loopStarted) {
      State.loopStarted = true;
      requestAnimationFrame(animate);
    }
  }
  function normalizeConfig() {
    cfg.body ??= {};
    cfg.body.mass ??= 8;
    cfg.visual ??= {};
    [
      "showAxes",
      "showJointAxes",
      "showLabels",
      "showSensors",
      "showSupportPolygon",
      "showTrajectories",
      "showCorrectedTargets",
      "showGrid",
    ].forEach((k) => {
      if (cfg.visual[k] === undefined) cfg.visual[k] = true;
    });
    cfg.live = {
      // Dijital ikizin ana modeli varsayılan olarak firmware command-state
      // açılarıyla gerçek robot pozunu izlesin. Ayrık sim karşılaştırması
      // gerektiğinde operatör LIVE_OVERLAY modunu seçebilir.
      mode: "live_drive",
      phaseSync: true,
      followImu: true,
      compareLeg: "RF",
      compareJoint: "femur",
      pollMs: 80,
      interpolationTauMs: 70,
      imuPitchSign: -1,
      imuRollSign: 1,
      imuYawSign: 1,
      ...(cfg.live || {}),
    };
    if (!LIVE_MODES.includes(cfg.live.mode)) cfg.live.mode = "live_drive";
    if (!FIRMWARE_LEG_ORDER.includes(cfg.live.compareLeg))
      cfg.live.compareLeg = "RF";
    if (!JOINT_ORDER.includes(cfg.live.compareJoint))
      cfg.live.compareJoint = "femur";
    if (Number(cfg.live.pollMs) === 300) cfg.live.pollMs = 80;
    cfg.live.pollMs = clamp(Number(cfg.live.pollMs || 80), 50, 1000);
    cfg.live.interpolationTauMs = clamp(Number(cfg.live.interpolationTauMs || 70), 25, 250);
    cfg.live.imuPitchSign = Number(cfg.live.imuPitchSign ?? -1);
    cfg.live.imuRollSign = Number(cfg.live.imuRollSign ?? 1);
    cfg.live.imuYawSign = Number(cfg.live.imuYawSign ?? 1);
    cfg.servoMotion ??= {};
    if (cfg.servoMotion.curve === "min_jerk")
      cfg.servoMotion.curve = "minimum_jerk";
    if (
      cfg.servoMotion.maxSpeedDegS === undefined &&
      cfg.servoMotion.maxSpeedDps !== undefined
    )
      cfg.servoMotion.maxSpeedDegS = cfg.servoMotion.maxSpeedDps;
    const smDefaults = {
      curve: "minimum_jerk",
      durationMs: 520,
      maxSpeedDegS: 260,
      accelLimit: 800,
      stiffness: 0.55,
      smoothness: 0.78,
      deadbandDeg: 1,
      currentWeight: 1,
      jerkWeight: 0.8,
      holdTorqueWeight: 0.6,
      preset: "smooth_walk",
      showPosition: true,
      showVelocity: true,
      showAcceleration: false,
      showJerk: false,
      showLiveOverlay: true,
    };
    Object.entries(smDefaults).forEach(([k, v]) => {
      if (cfg.servoMotion[k] === undefined) cfg.servoMotion[k] = v;
    });
    cfg.contacts ??= {
      mode: "gait_phase",
      groundZ: 0,
      switchTrigger: 4,
      manual: {},
    };
    cfg.contacts.manual ??= {};
    LEG_ORDER.forEach((n) => {
      cfg.contacts.manual[n] ??= false;
      const l = cfg.legs[n];
      if (!l.joints.coxa.axis) l.joints.coxa.axis = "+Z";
      if (!l.joints.femur.axis)
        l.joints.femur.axis = l.side === "left" ? "+X" : "-X";
      if (!l.joints.tibia.axis)
        l.joints.tibia.axis = l.side === "left" ? "+X" : "-X";
    });
    State.activeSection ||= "live";
  }
  window.App = {
    resetCamera() {
      if (State.cameraCtl) {
        State.cameraCtl.target.set(0, 70, 0);
        State.cameraCtl.spherical.setFromVector3(new THREE.Vector3(520, 290, 520));
        State.updateFreeCamera?.();
      } else {
        State.camera.position.set(520, 360, 520);
        State.camera.lookAt(0, 70, 0);
      }
      renderScene();
    },
    centerRobot() {
      if (State.cameraCtl) {
        State.cameraCtl.target.set(Number(cfg.body.pose.x || 0), Number(cfg.body.pose.z || 120), -Number(cfg.body.pose.y || 0));
        State.updateFreeCamera?.();
      } else {
        State.camera.lookAt(0, cfg.body.pose.z, 0);
      }
      renderScene();
    },
    zeroLiveYaw() {
      const raw = Number(State.live.telemetry?.mpuY);
      if (Number.isFinite(raw)) State.live.yawZero = raw;
      renderScene();
    },
    togglePlay() {
      State.playing = !State.playing;
      cfg.gait.playing = State.playing;
      const btn = $("playBtn");
      if (btn) btn.textContent = State.playing ? "PAUSE" : "PLAY";
      renderScene();
    },
    stepFrame() {
      State.clock += 1 / (Math.max(0.05, cfg.gait.frequency) * 24);
      renderScene();
    },
    toggleLayer(k) {
      cfg.visual[k] = !cfg.visual[k];
      renderControls();
      renderScene();
    },
    toggleCurveLayer(k) {
      cfg.servoMotion[k] = !cfg.servoMotion[k];
      renderControls();
      drawCurve();
    },
    toggleLiveOption(k) {
      cfg.live[k] = !cfg.live[k];
      renderControls();
      renderScene();
    },
    resetLiveMetrics() {
      State.live.cycles = [];
      State.live.lastPhase = null;
      State.live.lastWrapTs = 0;
      State.live.phaseError = 0;
      State.live.connectedAt = State.live.online ? performance.now() / 1000 : 0;
      updateLiveWidgets();
      updateStatus(0, 0, 0);
    },
    clearLiveHistory() {
      State.live.history = [];
      drawCurve();
      updateLiveWidgets();
    },
    applyClassicTripod() {
      cfg.gait.type = "tripod";
      cfg.gait.phases = { RF: 0, RR: 0, LM: 0, RM: 0.5, LF: 0.5, LR: 0.5 };
      renderControls();
      renderScene();
    },
    motionPreset(p) {
      const presets = {
        low_current: {
          curve: "minimum_jerk",
          durationMs: 900,
          maxSpeedDegS: 160,
          accelLimit: 420,
          stiffness: 0.28,
          smoothness: 0.95,
          deadbandDeg: 1.5,
          currentWeight: 0.75,
          jerkWeight: 0.45,
          holdTorqueWeight: 0.55,
        },
        smooth_walk: {
          curve: "s_curve",
          durationMs: 650,
          maxSpeedDegS: 220,
          accelLimit: 650,
          stiffness: 0.42,
          smoothness: 0.88,
          deadbandDeg: 1,
          currentWeight: 1,
          jerkWeight: 0.65,
          holdTorqueWeight: 0.7,
        },
        fast_response: {
          curve: "trapezoid",
          durationMs: 260,
          maxSpeedDegS: 520,
          accelLimit: 1800,
          stiffness: 0.82,
          smoothness: 0.32,
          deadbandDeg: 0.6,
          currentWeight: 1.3,
          jerkWeight: 1.2,
          holdTorqueWeight: 0.7,
        },
        stiff_test: {
          curve: "linear",
          durationMs: 180,
          maxSpeedDegS: 650,
          accelLimit: 2300,
          stiffness: 1,
          smoothness: 0.1,
          deadbandDeg: 0.2,
          currentWeight: 1.6,
          jerkWeight: 1.6,
          holdTorqueWeight: 1,
        },
      };
      if (!presets[p]) return;
      Object.assign(cfg.servoMotion, presets[p], { preset: p });
      renderControls();
      drawCurve();
      renderScene();
    },
    toggleLegIK() {
      const l = cfg.legs[cfg.selected.leg];
      l.useIK = !l.useIK;
      renderScene();
    },
    selectedLegHome() {
      cfg.legs[cfg.selected.leg].manualAngles = {
        ...cfg.legs[cfg.selected.leg].homeAngles,
      };
      renderControls();
      renderScene();
    },
    homeTargets() {
      LEG_ORDER.forEach((n) => {
        const h = cfg.legs[n].hip;
        cfg.legs[n].target = {
          x: h.x,
          y: h.y + (cfg.legs[n].side === "left" ? 150 : -150),
          z: cfg.legCommon.stanceZ,
        };
      });
      renderControls();
      renderScene();
    },
    copyTargetMirror() {
      const n = cfg.selected.leg;
      const pair = {
        LF: "RF",
        LM: "RM",
        LR: "RR",
        RF: "LF",
        RM: "LM",
        RR: "LR",
      }[n];
      const t = cfg.legs[n].target;
      cfg.legs[pair].target = { x: t.x, y: -t.y, z: t.z };
      renderScene();
    },
    toggleBodyIK() {
      cfg.bodyIK.enabled = !cfg.bodyIK.enabled;
      renderScene();
    },
    zeroBodyIK() {
      cfg.bodyIK.actual = { roll: 0, pitch: 0, yaw: 0 };
      cfg.bodyIK.desired = { roll: 0, pitch: 0, yaw: 0 };
      renderControls();
      renderScene();
    },
    toggleSensorVisible() {
      cfg.sensors[cfg.selected.sensor].visible =
        !cfg.sensors[cfg.selected.sensor].visible;
      renderScene();
    },
    async saveConfig() {
      await fetch("/api/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(cfg),
      });
      $("warnOutput").textContent = "Config kaydedildi.";
    },
    downloadConfig() {
      const a = document.createElement("a");
      a.href = URL.createObjectURL(
        new Blob([JSON.stringify(cfg, null, 2)], { type: "application/json" }),
      );
      a.download = "meta1_robot_parameters.json";
      a.click();
    },
    copyConfig() {
      navigator.clipboard?.writeText(JSON.stringify(cfg, null, 2));
      $("warnOutput").textContent = "JSON panoya kopyalandı.";
    },
    reloadConfig() {
      loadConfig();
    },
  };
  document.addEventListener("DOMContentLoaded", loadConfig);
})();
