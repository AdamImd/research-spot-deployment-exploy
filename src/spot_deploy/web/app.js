/* Read-only viewer. All code, meshes and textures are served locally. */
import { RobotScene } from "./robot_scene.js";
"use strict";
const $ = (id) => document.getElementById(id),
  colors = { measured: "#6bddc3", target: "#f5c454", command: "#b29bf4" };
const ui = {
  frames: [],
  index: 0,
  playing: false,
  follow: true,
  last: 0,
  model: null,
  meta: null,
  selected: 0,
  connected: false,
  pollAt: 0,
  autoStarted: false,
};
const camera = { yaw: 0.95, pitch: 0.5, zoom: 1 };
const num = (v, n = 3) => (Number.isFinite(v) ? v.toFixed(n) : "—");
const set = (id, value) => {
  $(id).textContent = value;
};
const current = () => ui.frames[ui.index];
function canvasSize(canvas) {
  let rect = canvas.getBoundingClientRect(),
    ratio = window.devicePixelRatio || 1;
  let w = rect.width,
    h = rect.height;
  if (
    canvas.width !== Math.round(w * ratio) ||
    canvas.height !== Math.round(h * ratio)
  ) {
    canvas.width = Math.round(w * ratio);
    canvas.height = Math.round(h * ratio);
  }
  let ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return { ctx, w, h };
}
function liveAge(frame) {
  if (!frame || !ui.meta) return Infinity;
  const wall = frame.recorded_at
    ? ui.meta.server_unix_s - Date.parse(frame.recorded_at) / 1000
    : Infinity;
  const receive = ui.meta.same_host
    ? ui.meta.server_monotonic_s - frame.state.received_monotonic_s
    : wall;
  return Math.max(wall, receive) + (performance.now() - ui.pollAt) / 1000;
}
function freshOverlay(frame, kind) {
  if (
    ui.follow &&
    !ui.meta?.complete &&
    !ui.meta?.demo &&
    (!ui.connected || liveAge(frame) > 0.5)
  )
    return null;
  let item = frame?.[kind];
  return item && frame.t - item.at >= -0.001 && frame.t - item.at <= 0.25
    ? item
    : null;
}
function drawRobot() {
  const frame = current();
  if (!ui.scene || !frame) return;
  const layers = {};
  for (const kind of ["measured", "target", "command"]) {
    layers[kind] = $(`show-${kind}`).checked
      ? kind === "measured" ? { positions: frame.state.positions } : freshOverlay(frame, kind)
      : null;
  }
  ui.scene.draw(frame.state.odom_quaternion_wxyz, layers, camera, ui.selected,
    $("show-meshes").checked, $("show-frames").checked);
}
function drawPlot() {
  let { ctx, w, h } = canvasSize($("plot")),
    frame = current();
  if (!frame) return;
  let start = Math.max(ui.frames[0].t, frame.t - 8),
    points = ui.frames.filter((f, i) => i <= ui.index && f.t >= start),
    values = [];
  for (const f of points) {
    values.push(f.state.positions[ui.selected]);
    for (const k of ["target", "command"]) {
      let o = freshOverlay(f, k);
      if (o) values.push(o.positions[ui.selected]);
    }
  }
  let low = Math.min(...values) - 0.06,
    high = Math.max(...values) + 0.06,
    left = 45,
    right = w - 18,
    top = 19,
    bottom = h - 25,
    span = Math.max(0.05, frame.t - start),
    x = (t) => left + ((t - start) / span) * (right - left),
    y = (v) => bottom - ((v - low) / (high - low)) * (bottom - top);
  ctx.font = "9px ui-monospace,monospace";
  for (let k = 0; k < 4; k++) {
    let value = low + ((high - low) * k) / 3,
      py = y(value);
    ctx.strokeStyle = "#2a3540";
    ctx.beginPath();
    ctx.moveTo(left, py);
    ctx.lineTo(right, py);
    ctx.stroke();
    ctx.fillStyle = "#81929f";
    ctx.fillText(value.toFixed(2), 5, py + 3);
  }
  ctx.fillText(`${num(start - ui.frames[0].t, 1)} s`, left, bottom + 18);
  ctx.fillText(
    `${num(frame.t - ui.frames[0].t, 1)} s`,
    right - 40,
    bottom + 18,
  );
  for (const kind of ["measured", "target", "command"]) {
    if (!$(`show-${kind}`).checked) continue;
    ctx.beginPath();
    ctx.strokeStyle = colors[kind];
    ctx.lineWidth = kind === "measured" ? 2 : 1.5;
    ctx.setLineDash(kind === "measured" ? [] : [4, 3]);
    let active = false;
    for (const f of points) {
      let item =
        kind === "measured"
          ? { positions: f.state.positions }
          : freshOverlay(f, kind);
      if (!item) {
        active = false;
        continue;
      }
      let px = x(f.t),
        py = y(item.positions[ui.selected]);
      if (active) ctx.lineTo(px, py);
      else ctx.moveTo(px, py);
      active = true;
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }
}
function render() {
  let f = current();
  $("empty-scene").hidden = !!f;
  if (!f) {
    drawRobot();
    return;
  }
  let q = f.state.positions[ui.selected],
    target = freshOverlay(f, "target"),
    command = freshOverlay(f, "command");
  $("timeline").max = Math.max(0, ui.frames.length - 1);
  $("timeline").value = ui.index;
  set("time-label", `${num(f.t - ui.frames[0].t, 2)} s`);
  set("play", ui.playing ? "Ⅱ" : "▶");
  $("follow").classList.toggle("active", ui.follow && !ui.meta?.complete);
  set("q-value", num(q));
  set("target-value", num(target?.positions[ui.selected]));
  set("cmd-value", num(command?.positions[ui.selected]));
  set("error-value", target ? num(target.positions[ui.selected] - q) : "—");
  set(
    "target-age",
    target
      ? `${Math.round((f.t - target.at) * 1000)} ms`
      : f.target
        ? "stale"
        : "absent",
  );
  set(
    "command-age",
    command
      ? `${Math.round((f.t - command.at) * 1000)} ms`
      : f.command
        ? "stale"
        : "absent",
  );
  set(
    "latency",
    target && Number.isFinite(target.inference_s)
      ? `${num(target.inference_s * 1000, 1)} ms`
      : "—",
  );
  for (let i = 0; i < 19; i++) {
    let row = $(`joint-${i}`);
    row.classList.toggle("selected", i === ui.selected);
    row.cells[1].textContent = num(f.state.positions[i]);
    row.cells[2].textContent = num(f.state.velocities[i]);
    row.cells[3].textContent = num(f.state.loads[i], 2);
    row.cells[4].textContent = num(target?.positions[i]);
  }
  for (let i = 0; i < 12; i++) {
    let value = target?.raw[i],
      bar = $(`action-${i}`),
      height = Number.isFinite(value) ? Math.min(1, Math.abs(value)) * 50 : 0;
    bar.style.height = `${height}%`;
    bar.style.top = `${value > 0 ? 50 - height : 50}%`;
    set(`action-value-${i}`, num(value, 2));
    set(`action-label-${i}`, ui.meta?.policy?.action_joint_names?.[i] ?? `a${i}`);
  }
  const policy = ui.meta?.policy, rates = ui.meta?.rates;
  const ratesFresh = rates && Date.now() / 1000 - rates.updated_unix_s < 2 && ui.connected;
  set("policy-rates", policy
    ? `${policy.name} · state ${ratesFresh ? num(rates.state_hz, 0) : "—"} Hz · inference ${ratesFresh ? num(rates.policy_hz, 1) : "—"} Hz · display ≈4 Hz`
    : "");
  set("policy-note", policy
    ? `${policy.note} ${policy.arm_mode}. Base command: 0 m/s, 0 rad/s; four-foot mode. ` +
      `Requested body height: ${num(policy.torso_target?.[2], 3)} m` +
      `${policy.body_height?.source ? ` (${policy.body_height.source})` : ""}.`
    : "Predicted joint targets are shown in amber. Prediction does not imply execution.");
  set("prediction-caption", policy && ui.selected >= 12 ? "Arm input" : "Predicted");
  let [w, x, y, z] = f.state.odom_quaternion_wxyz,
    roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  set(
    "attitude",
    `${num((roll * 180) / Math.PI, 1)}° / ${num((pitch * 180) / Math.PI, 1)}°`,
  );
  set("velocity", `${num(Math.hypot(...f.state.linear_velocity_odom), 2)} m/s`);
  let health = f.health,
    snapshot = ui.meta?.snapshot || {};
  set(
    "battery",
    Number.isFinite(health?.battery_percent ?? snapshot.battery_percent)
      ? `${num(health?.battery_percent ?? snapshot.battery_percent, 0)}%`
      : "—",
  );
  set("faults", health?.fault_count ?? "—");
  set(
    "health-origin",
    ui.meta?.demo
      ? "SYNTHETIC"
      : health
        ? "RECORDED HEALTH"
        : snapshot.serial
          ? "PREFLIGHT SNAPSHOT"
          : "NO HEALTH DATA",
  );
  updateStatus();
  drawRobot();
  drawPlot();
}
function updateStatus() {
  let f = current(),
    m = ui.meta;
  if (!m) return;
  let age = liveAge(f),
    live = !m.complete && !m.demo,
    stale = live && age > 0.5;
  set(
    "source-badge",
    m.demo ? "SYNTHETIC DEMO" : m.complete ? "RECORDED RUN" : "LIVE LOG",
  );
  set("source-label", m.source);
  set(
    "stream-status",
    !ui.connected
      ? "Viewer disconnected"
      : m.demo
        ? "Demo playback"
        : !f
          ? "Waiting for state"
          : live
            ? ui.follow
              ? stale
                ? "Telemetry stale"
                : "Following telemetry"
              : "Timeline paused"
            : `Recorded · ${m.complete}`,
  );
  set(
    "stream-detail",
    m.demo
      ? "Illustrative motion · no robot connected"
      : f
        ? `${ui.frames.length.toLocaleString()} samples · ${m.mode}${m.skipped ? ` · ${m.skipped} invalid lines` : ""}`
        : "No valid states received",
  );
  $("signal").style.background = !ui.connected || stale ? "#ff817c" : "#f5c454";
  let modelMismatch =
    m.snapshot?.robot_model_sha256 &&
    ui.model.sha256 &&
    m.snapshot.robot_model_sha256 !== ui.model.sha256;
  set(
    "notice",
    m.demo
      ? "SYNTHETIC DEMO — illustrative poses and predictions. No robot or E-stop is connected."
      : modelMismatch
        ? "MODEL MISMATCH — supplied URDF does not match this robot snapshot. Pose is a visual aid only."
        : live
          ? "Live log viewer · predictions are not commands. Stale overlays are hidden; all arming remains at the physical E-stop."
          : "Recorded telemetry · health and E-stop observations describe the captured run, not the robot now.",
  );
  let hw = m.hardware,
    label = "E-stop state unknown",
    detail = "No bridge status selected";
  $("estop-label").className = "";
  if (hw) {
    let hwAge = Date.now() / 1000 - hw.updated_unix_s;
    let valid =
      Number.isFinite(hwAge) && hwAge >= -0.1 && hwAge < 0.5 && ui.connected;
    if (!valid) {
      label = "Bridge status stale";
      detail = "Do not infer current E-stop state";
      $("estop-label").className = "stopped";
    } else {
      let bench = hw.mode === "bench";
      let source = hw.input_type === "linux_joystick" ? "Joystick" : "Hardware";
      label = bench ? `${source} bench · ${hw.state}` : `${source} · ${hw.state}`;
      detail = bench
        ? "USB only · not connected to Spot"
        : hw.robot_confirmed
          ? "Last check-in acknowledged by Spot"
          : "No confirmed Spot check-in";
      if (hw.input_type === "linux_joystick") {
        detail += ` · button ${hw.stop_button}: STOP · button ${hw.rearm_button}: rearm`;
      }
      $("estop-label").className =
        hw.state === "armed" && !bench ? "safe" : "stopped";
    }
  } else if (f?.health) {
    label = f.health.estop_ready
      ? "Recorded E-stop: clear"
      : "Recorded E-stop: stop";
    detail = "Captured aggregate SDK status · not current hardware state";
  }
  set("estop-label", label);
  set("estop-detail", detail);
}
async function poll() {
  try {
    let response = await fetch(`/api/frames?after=${ui.last}`, {
      cache: "no-store",
    });
    if (!response.ok) throw Error("telemetry");
    let meta = await response.json();
    ui.meta = meta;
    ui.connected = true;
    ui.pollAt = performance.now();
    for (let f of meta.frames) {
      ui.frames.push(f);
      ui.last = f.id;
    }
    if (ui.frames.length > 24000) {
      let excess = ui.frames.length - 24000;
      ui.frames.splice(0, excess);
      ui.index = Math.max(0, ui.index - excess);
    }
    if (ui.follow && !meta.complete)
      ui.index = Math.max(0, ui.frames.length - 1);
    if (meta.demo && !ui.autoStarted && ui.frames.length) {
      ui.autoStarted = true;
      ui.playing = true;
      ui.follow = false;
    }
    render();
    setTimeout(poll, meta.more ? 20 : 250);
  } catch (e) {
    ui.connected = false;
    render();
    setTimeout(poll, 1000);
  }
}
function selectJoint(i) {
  ui.selected = i;
  let label = ui.model.joint_names[i].replaceAll("_", " · ");
  set("chart-title", label);
  render();
}
function init() {
  ui.scene = new RobotScene($("robot"), ui.model, drawRobot);
  ui.scene.load().then(() => {
    set("model-label", ui.model.label);
    set("mesh-status", ui.model.missing_visuals?.length
      ? `${ui.model.missing_visuals.length} unavailable visuals; see the link list`
      : `${ui.model.visual_count ?? 0} visuals loaded · ${ui.model.links.length} links`);
    drawRobot();
  }).catch(() => {
    set("mesh-status", "Visual loading failed. Link frames remain available; refresh to retry.");
    $("show-frames").checked = true;
    drawRobot();
  });
  for (const link of ui.model.links) {
    const label = document.createElement("label"), box = document.createElement("input"),
      name = document.createElement("span"), badge = document.createElement("small");
    box.type = "checkbox";
    box.checked = true;
    box.setAttribute("aria-label", `Show link ${link.name}`);
    box.onchange = () => { ui.scene.setLinkVisible(link.name, box.checked); drawRobot(); };
    name.textContent = link.name.replace(/^visual:/, "");
    const missing = link.visuals.some(v => v.geometry.error);
    badge.textContent = missing ? "missing" : link.visuals.length ? "visual" : "frame";
    label.classList.toggle("missing-visual", missing);
    if (missing) label.title = link.visuals.filter(v => v.geometry.error).map(v => v.geometry.error).join("; ");
    label.append(box, name, badge);
    $("link-list").append(label);
  }
  set("link-count", `${ui.model.links.length} links`);
  $("show-all-links").onclick = () => {
    $("link-list").querySelectorAll("input").forEach(input => { input.checked = true; });
    ui.model.links.forEach(link => ui.scene.setLinkVisible(link.name, true));
    drawRobot();
  };
  for (let i = 0; i < 19; i++) {
    let tr = document.createElement("tr");
    tr.id = `joint-${i}`;
    tr.tabIndex = 0;
    tr.setAttribute("aria-label", `Plot ${ui.model.joint_names[i]}`);
    for (let v of [ui.model.joint_names[i], "—", "—", "—", "—"]) {
      let td = document.createElement("td");
      td.textContent = v;
      tr.append(td);
    }
    tr.onclick = () => selectJoint(i);
    tr.onkeydown = (e) => {
      if (e.key === "Enter" || e.key === " ") {
        e.preventDefault();
        selectJoint(i);
      }
    };
    $("joint-rows").append(tr);
  }
  for (let i = 0; i < 12; i++) {
    let action = document.createElement("div");
    action.className = "action";
    let track = document.createElement("div");
    track.className = "track";
    let bar = document.createElement("div");
    bar.id = `action-${i}`;
    bar.className = "bar";
    track.append(bar);
    let label = document.createElement("small");
    label.id = `action-label-${i}`;
    label.textContent = `a${i}`;
    let value = document.createElement("span");
    value.id = `action-value-${i}`;
    value.textContent = "—";
    action.append(track, value, label);
    $("action-bars").append(action);
  }
  set("model-label", ui.model.label);
  $("play").onclick = () => {
    ui.playing = !ui.playing;
    ui.follow = false;
    render();
  };
  $("follow").onclick = () => {
    ui.follow = true;
    ui.playing = false;
    ui.index = Math.max(0, ui.frames.length - 1);
    render();
  };
  $("timeline").oninput = (e) => {
    ui.index = Number(e.target.value);
    ui.playing = false;
    ui.follow = false;
    render();
  };
  for (let id of ["show-measured", "show-target", "show-command", "show-meshes", "show-frames"])
    $(id).onchange = render;
  function view(yaw, pitch, id) {
    camera.yaw = yaw;
    camera.pitch = pitch;
    camera.zoom = 1;
    for (let v of ["iso", "side", "top"])
      $(`view-${v}`).classList.toggle("active", `view-${v}` === id);
    drawRobot();
  }
  $("reset-view").onclick = () => view(0.95, 0.5, "view-iso");
  $("view-iso").onclick = $("reset-view").onclick;
  $("view-side").onclick = () => view(Math.PI / 2, 0.03, "view-side");
  $("view-top").onclick = () =>
    view(Math.PI / 2, Math.PI / 2 - 0.01, "view-top");
  let drag = null;
  $("robot").onpointerdown = (e) => {
    drag = [e.clientX, e.clientY];
    $("robot").setPointerCapture(e.pointerId);
  };
  $("robot").onpointermove = (e) => {
    if (!drag) return;
    camera.yaw -= (e.clientX - drag[0]) * 0.007;
    camera.pitch = Math.max(
      -0.1,
      Math.min(1.55, camera.pitch + (e.clientY - drag[1]) * 0.007),
    );
    drag = [e.clientX, e.clientY];
    drawRobot();
  };
  $("robot").onpointerup = $("robot").onpointercancel = () => {
    drag = null;
  };
  $("robot").addEventListener(
    "wheel",
    (e) => {
      e.preventDefault();
      camera.zoom = Math.max(
        0.45,
        Math.min(2.8, camera.zoom * Math.exp(-e.deltaY * 0.001)),
      );
      drawRobot();
    },
    { passive: false },
  );
  window.addEventListener("resize", render);
  selectJoint(0);
  poll();
  let prior = 0,
    accum = 0;
  function tick(now) {
    if (ui.playing && ui.frames.length > 1) {
      accum += Math.min(0.2, (now - prior) / 1000) * Number($("speed").value);
      let changed = false;
      while (
        ui.index < ui.frames.length - 1 &&
        accum >=
          Math.max(0.001, ui.frames[ui.index + 1].t - ui.frames[ui.index].t)
      ) {
        accum -= Math.max(
          0.001,
          ui.frames[ui.index + 1].t - ui.frames[ui.index].t,
        );
        ui.index++;
        changed = true;
      }
      if (ui.index === ui.frames.length - 1) {
        if (ui.meta.demo) {
          ui.index = 0;
          accum = 0;
        } else {
          ui.playing = false;
        }
      }
      if (changed) render();
    } else accum = 0;
    prior = now;
    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);
}
fetch("/api/model")
  .then((r) => {
    if (!r.ok) throw Error("model");
    return r.json();
  })
  .then((model) => {
    ui.model = model;
    init();
  })
  .catch(() => {
    set(
      "notice",
      "Viewer could not load the robot model. Check the local server and refresh.",
    );
  });
