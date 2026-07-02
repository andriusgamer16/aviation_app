"use strict";
/* PyAvionics primary flight display.
 *
 * Receives 20 Hz JSON frames over a WebSocket and renders a Garmin-style
 * PFD on a full-window canvas at display refresh rate, easing display
 * values toward the latest frame so the instruments move smoothly.
 */

// ---------------------------------------------------------------- units
const MPS_TO_KT = 1.94384;
const M_TO_FT = 3.28084;
const MPS_TO_FPM = 196.85;
const DEG = Math.PI / 180;

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const wrap360 = (a) => ((a % 360) + 360) % 360;
const shortestDeg = (from, to) => ((to - from + 540) % 360) - 180;
const pad3 = (n) => String(Math.round(wrap360(n))).padStart(3, "0");

// ------------------------------------------------- palette (aviation)
// Regulated PFD conventions: sky/ground horizon, white scales, yellow
// aircraft symbol, magenta for GPS-derived guidance, amber cautions,
// red failures. Every colored annunciation also carries a text label.
const C = {
  sky: "#1c66c4",
  skyHigh: "#3f87dc",
  ground: "#7c5330",
  groundLow: "#5d3f26",
  horizon: "#f5f7fa",
  white: "#f5f7fa",
  ink2: "#c4ccd6",
  inkMut: "#8d99a8",
  tapeBg: "rgba(8, 10, 13, 0.62)",
  boxBg: "#0a0c0f",
  barBg: "#0c0f13",
  frame: "#2a3340",
  yellow: "#ffe14d",
  green: "#4fd47f",
  amber: "#ffbf47",
  red: "#e0442e",
  magenta: "#ff5bff",
  cyan: "#41d8de",
};
const FONT = (px, w = 600) => `${w} ${px}px "Segoe UI", system-ui, sans-serif`;
const MONO = (px, w = 600) => `${w} ${px}px Consolas, "Cascadia Mono", monospace`;

// ----------------------------------------------------------- state
const canvas = document.getElementById("pfd");
const ctx = canvas.getContext("2d");
const connTag = document.getElementById("conn");
const zeroBtn = document.getElementById("zeroBtn");

const state = {
  frame: null,          // latest frame from the server
  lastFrameAt: 0,       // performance.now() of last frame
  connected: false,
  everConnected: false,
  disp: {               // smoothed display values
    pitch: 0, roll: 0, hdg: 0, trk: 0,
    gsKt: 0, altFt: 0, vsFpm: 0,
    slip: 0, g: 1, turn: 0,
  },
};

// ----------------------------------------------------------- websocket
function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => {
    state.connected = true;
    state.everConnected = true;
    connTag.textContent = "LINK";
    connTag.className = "tag ok";
  };
  ws.onmessage = (ev) => {
    state.frame = JSON.parse(ev.data);
    state.lastFrameAt = performance.now();
  };
  ws.onclose = () => {
    state.connected = false;
    connTag.textContent = "RECONNECTING";
    connTag.className = "tag err";
    setTimeout(connect, 1000);
  };
  ws.onerror = () => ws.close();
}
connect();

zeroBtn.addEventListener("click", async () => {
  try {
    await fetch("/api/zero", { method: "POST" });
    zeroBtn.textContent = "LEVEL ✓";
    setTimeout(() => (zeroBtn.textContent = "LEVEL"), 1200);
  } catch {
    /* server unreachable; the LINK tag already says so */
  }
});

// ----------------------------------------------------------- smoothing
function ease(current, target, dt, rate) {
  return current + (target - current) * (1 - Math.exp(-dt * rate));
}

function updateDisplay(dt) {
  const f = state.frame;
  if (!f) return;
  const d = state.disp;
  d.pitch = ease(d.pitch, f.att.pitch, dt, 12);
  d.roll = d.roll + shortestDeg(d.roll, f.att.roll) * (1 - Math.exp(-dt * 12));
  d.hdg = wrap360(d.hdg + shortestDeg(d.hdg, f.hdg.mag) * (1 - Math.exp(-dt * 8)));
  if (f.gps.trkDeg != null) {
    d.trk = wrap360(d.trk + shortestDeg(d.trk, f.gps.trkDeg) * (1 - Math.exp(-dt * 8)));
  }
  d.gsKt = ease(d.gsKt, (f.gps.gsMps ?? 0) * MPS_TO_KT, dt, 5);
  d.altFt = ease(d.altFt, (f.gps.altM ?? 0) * M_TO_FT, dt, 5);
  d.vsFpm = ease(d.vsFpm, f.vsMps * MPS_TO_FPM, dt, 4);
  d.slip = ease(d.slip, f.acc.slip, dt, 8);
  d.g = ease(d.g, f.acc.g, dt, 6);
  d.turn = ease(d.turn, f.turnRateDps, dt, 5);
}

// ----------------------------------------------------------- layout
function layout(W, H) {
  const topH = 54;
  const cx = W / 2;
  const cy = topH + (H - topH) * 0.40;
  // Lower bounds shrink with the window so instruments never overlap the
  // top bar or each other on small windows.
  const tapeH = clamp((H - topH) * 0.58, 120, 480);
  const hsiR = clamp(Math.min(W * 0.16, H * 0.20), 60, 180);
  return {
    W, H, topH, cx, cy, tapeH,
    spd: { x: 14, w: 92 },
    alt: { x: W - 14 - 36 - 6 - 92, w: 92 },
    vsi: { x: W - 14 - 36, w: 36 },
    hsi: { x: cx, y: H - hsiR - 22, r: hsiR },
    ppd: (H - topH) / 44,           // attitude pixels per degree of pitch
  };
}

// ----------------------------------------------------------- attitude
function drawAttitude(L) {
  const { cx, cy, W, H, ppd } = L;
  const d = state.disp;
  const ext = Math.max(W, H) * 2.2;

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-d.roll * DEG);
  ctx.translate(0, d.pitch * ppd);

  // Sky and ground with a subtle gradient toward the horizon.
  let g = ctx.createLinearGradient(0, -ext, 0, 0);
  g.addColorStop(0, C.skyHigh);
  g.addColorStop(1, C.sky);
  ctx.fillStyle = g;
  ctx.fillRect(-ext, -ext, ext * 2, ext);
  g = ctx.createLinearGradient(0, 0, 0, ext);
  g.addColorStop(0, C.ground);
  g.addColorStop(1, C.groundLow);
  ctx.fillStyle = g;
  ctx.fillRect(-ext, 0, ext * 2, ext);

  ctx.strokeStyle = C.horizon;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-ext, 0);
  ctx.lineTo(ext, 0);
  ctx.stroke();

  // Pitch ladder, clipped to a central window so it clears the tapes.
  ctx.save();
  ctx.rotate(d.roll * DEG);                    // clip in screen orientation
  ctx.translate(0, -d.pitch * ppd);
  ctx.beginPath();
  ctx.rect(-L.tapeH * 0.42, -L.tapeH * 0.46, L.tapeH * 0.84, L.tapeH * 0.80);
  ctx.clip();
  ctx.translate(0, d.pitch * ppd);
  ctx.rotate(-d.roll * DEG);

  ctx.strokeStyle = C.white;
  ctx.fillStyle = C.white;
  ctx.font = FONT(14);
  ctx.textBaseline = "middle";
  for (let deg = -30; deg <= 30; deg += 2.5) {
    if (deg === 0) continue;
    const y = -deg * ppd;
    const major = deg % 10 === 0;
    const half = deg % 5 === 0 && !major;
    const w = major ? 58 : half ? 34 : 16;
    ctx.lineWidth = major ? 2 : 1.4;
    ctx.beginPath();
    ctx.moveTo(-w, y);
    ctx.lineTo(w, y);
    ctx.stroke();
    if (major) {
      const label = String(Math.abs(deg));
      ctx.textAlign = "right";
      ctx.fillText(label, -w - 8, y);
      ctx.textAlign = "left";
      ctx.fillText(label, w + 8, y);
    }
  }
  ctx.restore();
  ctx.restore();
}

function drawRollScaleAndPointer(L) {
  const { cx, cy } = L;
  const R = L.tapeH * 0.52;
  const d = state.disp;

  ctx.save();
  ctx.translate(cx, cy);

  // Fixed bank scale.
  ctx.strokeStyle = C.white;
  ctx.fillStyle = C.white;
  for (const a of [-60, -45, -30, -20, -10, 10, 20, 30, 45, 60]) {
    const major = Math.abs(a) === 30 || Math.abs(a) === 60;
    const len = major ? 16 : 9;
    ctx.save();
    ctx.rotate(a * DEG);
    ctx.lineWidth = major ? 2.4 : 1.6;
    ctx.beginPath();
    ctx.moveTo(0, -R);
    ctx.lineTo(0, -R - len);
    ctx.stroke();
    ctx.restore();
  }
  // Zero-bank reference triangle (points down at the scale).
  ctx.beginPath();
  ctx.moveTo(0, -R + 2);
  ctx.lineTo(-9, -R - 14);
  ctx.lineTo(9, -R - 14);
  ctx.closePath();
  ctx.fill();

  // Moving roll pointer + slip/skid brick, aligned with the horizon.
  ctx.rotate(-d.roll * DEG);
  ctx.fillStyle = C.yellow;
  ctx.beginPath();
  ctx.moveTo(0, -R + 4);
  ctx.lineTo(-10, -R + 22);
  ctx.lineTo(10, -R + 22);
  ctx.closePath();
  ctx.fill();
  const slipPx = clamp(state.disp.slip * 180, -26, 26);
  ctx.fillRect(-10 + slipPx, -R + 26, 20, 7);
  ctx.restore();
}

function drawAircraftSymbol(L) {
  const { cx, cy } = L;
  const s = L.tapeH / 460;
  ctx.save();
  ctx.translate(cx, cy);
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#1a1a1a";
  ctx.fillStyle = C.yellow;
  for (const side of [-1, 1]) {
    ctx.beginPath();
    ctx.moveTo(side * 130 * s, -4);
    ctx.lineTo(side * 52 * s, -4);
    ctx.lineTo(side * 52 * s, 14);
    ctx.lineTo(side * 64 * s, 14);
    ctx.lineTo(side * 64 * s, 5);
    ctx.lineTo(side * 130 * s, 5);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }
  ctx.beginPath();
  ctx.arc(0, 0, 4.5, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

// ----------------------------------------------------------- tapes
function tapeBackground(x, y, w, h) {
  ctx.fillStyle = C.tapeBg;
  ctx.beginPath();
  ctx.roundRect(x, y, w, h, 6);
  ctx.fill();
  ctx.strokeStyle = C.frame;
  ctx.lineWidth = 1;
  ctx.stroke();
}

function readoutBox(x, y, w, h, tipRight, color = C.white) {
  const tip = 10;
  ctx.fillStyle = C.boxBg;
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  if (tipRight) {
    ctx.moveTo(x, y);
    ctx.lineTo(x + w, y);
    ctx.lineTo(x + w + tip, y + h / 2);
    ctx.lineTo(x + w, y + h);
    ctx.lineTo(x, y + h);
  } else {
    ctx.moveTo(x + w, y);
    ctx.lineTo(x, y);
    ctx.lineTo(x - tip, y + h / 2);
    ctx.lineTo(x, y + h);
    ctx.lineTo(x + w, y + h);
  }
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
}

function drawSpeedTape(L) {
  const { cy, tapeH } = L;
  const { x, w } = L.spd;
  const y0 = cy - tapeH / 2;
  const f = state.frame;
  const gs = state.disp.gsKt;
  const pxPerKt = tapeH / 80;
  const noData = f.gps.gsMps == null;

  tapeBackground(x, y0, w, tapeH);
  ctx.save();
  ctx.beginPath();
  ctx.rect(x, y0, w, tapeH);
  ctx.clip();

  ctx.strokeStyle = C.white;
  ctx.fillStyle = C.white;
  ctx.font = FONT(15);
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  const lo = Math.max(0, Math.floor((gs - 42) / 10) * 10);
  for (let v = lo; !noData && v <= gs + 42; v += 10) {
    const y = cy - (v - gs) * pxPerKt;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x + w - 12, y);
    ctx.lineTo(x + w, y);
    ctx.stroke();
    ctx.fillText(String(v), x + w - 18, y);
    if (v + 5 <= gs + 42) {
      const y5 = cy - (v + 5 - gs) * pxPerKt;
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(x + w - 7, y5);
      ctx.lineTo(x + w, y5);
      ctx.stroke();
    }
  }
  ctx.restore();

  readoutBox(x - 2, cy - 22, w - 4, 44, true);
  ctx.font = MONO(26, 700);
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  ctx.fillStyle = noData ? C.amber : C.white;
  ctx.fillText(noData ? "---" : String(Math.round(gs)), x + w - 12, cy + 1);

  ctx.font = FONT(12, 700);
  ctx.textAlign = "left";
  ctx.fillStyle = C.ink2;
  ctx.fillText("GS KT", x + 6, y0 - 12);
  if (noData) {
    ctx.fillStyle = C.amber;
    ctx.fillText("NO SPD SRC", x + 6, y0 + tapeH + 14);
  }
}

function drawAltTape(L) {
  const { cy, tapeH } = L;
  const { x, w } = L.alt;
  const y0 = cy - tapeH / 2;
  const f = state.frame;
  const alt = state.disp.altFt;
  const pxPerFt = tapeH / 800;
  const noData = f.gps.altM == null;

  tapeBackground(x, y0, w, tapeH);
  ctx.save();
  ctx.beginPath();
  ctx.rect(x, y0, w, tapeH);
  ctx.clip();

  ctx.strokeStyle = C.white;
  ctx.fillStyle = C.white;
  ctx.font = FONT(14);
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  const lo = Math.floor((alt - 420) / 100) * 100;
  for (let v = lo; !noData && v <= alt + 420; v += 100) {
    const y = cy - (v - alt) * pxPerFt;
    const major = v % 500 === 0;
    ctx.lineWidth = major ? 2.2 : 1.4;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + (major ? 12 : 8), y);
    ctx.stroke();
    ctx.font = FONT(major ? 15 : 12, major ? 700 : 500);
    ctx.fillText(String(v), x + 16, y);
  }
  ctx.restore();

  readoutBox(x + 6, cy - 22, w - 4, 44, false);
  ctx.font = MONO(24, 700);
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  ctx.fillStyle = noData ? C.amber : C.white;
  ctx.fillText(noData ? "----" : String(Math.round(alt)), x + w - 4, cy + 1);

  ctx.font = FONT(12, 700);
  ctx.textAlign = "left";
  ctx.fillStyle = C.ink2;
  ctx.fillText("ALT FT", x + 6, y0 - 12);
  if (noData) {
    ctx.fillStyle = C.amber;
    ctx.fillText("NO ALT SRC", x + 6, y0 + tapeH + 14);
  } else {
    ctx.fillStyle = C.magenta;
    ctx.fillText("GPS", x + 6, y0 + tapeH + 14);
  }
}

function drawVsi(L) {
  const { cy, tapeH } = L;
  const { x, w } = L.vsi;
  const h = tapeH * 0.78;
  const y0 = cy - h / 2;
  const vs = clamp(state.disp.vsFpm, -2200, 2200);
  const pxPerFpm = (h / 2) / 2200;

  ctx.fillStyle = "rgba(8, 10, 13, 0.45)";
  ctx.beginPath();
  ctx.roundRect(x, y0, w, h, 5);
  ctx.fill();

  ctx.strokeStyle = C.ink2;
  ctx.fillStyle = C.ink2;
  ctx.font = FONT(11, 700);
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  for (const v of [-2000, -1000, 1000, 2000]) {
    const y = cy - v * pxPerFpm;
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + 7, y);
    ctx.stroke();
    ctx.fillText(String(Math.abs(v) / 1000), x + 10, y);
  }
  ctx.lineWidth = 1.8;
  ctx.strokeStyle = C.white;
  ctx.beginPath();
  ctx.moveTo(x, cy);
  ctx.lineTo(x + 10, cy);
  ctx.stroke();

  // Pointer + readout.
  const y = cy - vs * pxPerFpm;
  ctx.fillStyle = C.white;
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x + 9, y - 6);
  ctx.lineTo(x + 9, y + 6);
  ctx.closePath();
  ctx.fill();
  if (Math.abs(state.disp.vsFpm) >= 100) {
    const txt = String(Math.round(state.disp.vsFpm / 50) * 50);
    ctx.font = MONO(12, 700);
    ctx.textAlign = "left";
    const ty = clamp(y, y0 + 10, y0 + h - 10);
    ctx.fillText(txt, x + 11, ty);
  }
}

// ----------------------------------------------------------- HSI
function drawHsi(L) {
  const { x, y, r } = L.hsi;
  const d = state.disp;
  const f = state.frame;

  ctx.save();
  ctx.translate(x, y);

  ctx.fillStyle = "rgba(8, 10, 13, 0.55)";
  ctx.beginPath();
  ctx.arc(0, 0, r, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = C.frame;
  ctx.lineWidth = 1;
  ctx.stroke();

  // Rotating compass rose.
  ctx.save();
  ctx.rotate(-d.hdg * DEG);
  ctx.strokeStyle = C.white;
  ctx.fillStyle = C.white;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  for (let a = 0; a < 360; a += 5) {
    const major = a % 10 === 0;
    ctx.save();
    ctx.rotate(a * DEG);
    ctx.lineWidth = major ? 1.8 : 1;
    ctx.beginPath();
    ctx.moveTo(0, -r);
    ctx.lineTo(0, -r + (major ? 12 : 7));
    ctx.stroke();
    if (a % 30 === 0) {
      const cardinal = { 0: "N", 90: "E", 180: "S", 270: "W" }[a];
      ctx.font = cardinal ? FONT(17, 700) : FONT(13, 600);
      ctx.fillStyle = cardinal ? C.white : C.ink2;
      ctx.fillText(cardinal ?? String(a / 10), 0, -r + 26);
      ctx.fillStyle = C.white;
    }
    ctx.restore();
  }
  ctx.restore();

  // GPS track diamond (magenta = GPS-derived).
  if (f.gps.trkDeg != null) {
    ctx.save();
    ctx.rotate(shortestDeg(d.hdg, d.trk) * DEG);
    ctx.fillStyle = C.magenta;
    ctx.beginPath();
    ctx.moveTo(0, -r + 2);
    ctx.lineTo(-7, -r + 12);
    ctx.lineTo(0, -r + 22);
    ctx.lineTo(7, -r + 12);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  // Turn-rate trend: arc showing predicted heading change over 6 s.
  const trend = clamp(d.turn * 6, -50, 50);
  if (Math.abs(trend) > 1) {
    ctx.strokeStyle = C.magenta;
    ctx.lineWidth = 5;
    ctx.beginPath();
    ctx.arc(0, 0, r + 7, -Math.PI / 2, -Math.PI / 2 + trend * DEG, trend < 0);
    ctx.stroke();
  }
  // Standard-rate marks (3 deg/s over 6 s = 18 deg).
  ctx.strokeStyle = C.white;
  ctx.lineWidth = 2;
  for (const a of [-18, 18]) {
    ctx.save();
    ctx.rotate(a * DEG);
    ctx.beginPath();
    ctx.moveTo(0, -r - 4);
    ctx.lineTo(0, -r - 11);
    ctx.stroke();
    ctx.restore();
  }

  // Fixed lubber line.
  ctx.fillStyle = C.white;
  ctx.beginPath();
  ctx.moveTo(0, -r + 1);
  ctx.lineTo(-7, -r - 12);
  ctx.lineTo(7, -r - 12);
  ctx.closePath();
  ctx.fill();

  // Ownship symbol.
  ctx.strokeStyle = C.white;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, -10);
  ctx.lineTo(0, 8);
  ctx.moveTo(-8, -1);
  ctx.lineTo(8, -1);
  ctx.moveTo(-5, 8);
  ctx.lineTo(5, 8);
  ctx.stroke();

  // Heading readout box.
  const hb = { w: 74, h: 30 };
  ctx.fillStyle = C.boxBg;
  ctx.strokeStyle = C.frame;
  ctx.beginPath();
  ctx.roundRect(-hb.w / 2, -r - 18 - hb.h, hb.w, hb.h, 4);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = C.white;
  ctx.font = MONO(20, 700);
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(`${pad3(d.hdg)}°`, 0, -r - 18 - hb.h / 2 + 1);
  ctx.font = FONT(10, 700);
  ctx.fillStyle = f.hdg.src === "sim" ? C.amber : C.green;
  ctx.fillText(f.hdg.src === "sim" ? "HDG SIM" : "HDG MAG", 0, -r - 12 - hb.h - 16);

  ctx.restore();
}

// ----------------------------------------------------------- top bar
function fmtCoord(v, isLat) {
  if (v == null) return "--°--.--'";
  const hemi = isLat ? (v >= 0 ? "N" : "S") : (v >= 0 ? "E" : "W");
  const abs = Math.abs(v);
  const deg = Math.floor(abs);
  const min = (abs - deg) * 60;
  const degStr = String(deg).padStart(isLat ? 2 : 3, "0");
  return `${degStr}°${min.toFixed(2).padStart(5, "0")}'${hemi}`;
}

function drawTopBar(L) {
  const { W, topH } = L;
  const f = state.frame;
  const d = state.disp;

  ctx.fillStyle = C.barBg;
  ctx.fillRect(0, 0, W, topH);
  ctx.strokeStyle = C.frame;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, topH + 0.5);
  ctx.lineTo(W, topH + 0.5);
  ctx.stroke();

  ctx.textBaseline = "middle";
  const midY = topH / 2;

  // Left: ground speed and track.
  ctx.textAlign = "left";
  ctx.font = FONT(11, 700);
  ctx.fillStyle = C.inkMut;
  ctx.fillText("GS", 16, midY - 11);
  ctx.fillText("TRK", 16, midY + 13);
  ctx.font = MONO(16, 700);
  ctx.fillStyle = C.magenta;
  ctx.fillText(
    f.gps.gsMps == null ? "--- KT" : `${String(Math.round(d.gsKt)).padStart(3)} KT`,
    48, midY - 11,
  );
  ctx.fillText(
    f.gps.trkDeg == null ? "---°" : `${pad3(d.trk)}°`,
    48, midY + 13,
  );

  // Center: position + fix quality (dropped on very narrow windows where
  // it would collide with the flanking columns).
  ctx.textAlign = "center";
  if (W >= 560) {
    ctx.font = MONO(W < 700 ? 12 : 15, 600);
    ctx.fillStyle = C.white;
    ctx.fillText(
      `${fmtCoord(f.gps.lat, true)}  ${fmtCoord(f.gps.lon, false)}`,
      W / 2, midY - 11,
    );
  }
  ctx.font = FONT(11, 700);
  if (f.gps.src === "sim") {
    ctx.fillStyle = C.amber;
    ctx.fillText("GPS SIMULATED", W / 2, midY + 13);
  } else {
    ctx.fillStyle = C.green;
    const acc = f.gps.accM != null ? ` ±${Math.round(f.gps.accM)} m` : "";
    const srcLbl = f.gps.fixSrc && f.gps.fixSrc !== "unknown"
      ? ` (${f.gps.fixSrc.toUpperCase()})` : "";
    ctx.fillText(`GPS FIX${srcLbl}${acc}`, W / 2, midY + 13);
  }

  // Right: g-load, UTC, mode chip.
  const now = new Date();
  const utc =
    `${String(now.getUTCHours()).padStart(2, "0")}:` +
    `${String(now.getUTCMinutes()).padStart(2, "0")}:` +
    `${String(now.getUTCSeconds()).padStart(2, "0")}Z`;
  ctx.textAlign = "right";
  ctx.font = MONO(15, 600);
  ctx.fillStyle = C.white;
  ctx.fillText(utc, W - 130, midY - 11);
  ctx.fillStyle = Math.abs(d.g - 1) > 0.5 ? C.amber : C.ink2;
  ctx.fillText(`G ${d.g.toFixed(2)}`, W - 130, midY + 13);

  const mode = f.status.mode.toUpperCase();
  const chipColor = mode === "LIVE" ? C.green : mode === "SIM" ? C.amber : C.cyan;
  ctx.fillStyle = chipColor;
  ctx.beginPath();
  ctx.roundRect(W - 116, midY - 12, 58, 24, 4);
  ctx.fill();
  ctx.fillStyle = "#0b0d10";
  ctx.font = FONT(12, 700);
  ctx.textAlign = "center";
  ctx.fillText(mode, W - 87, midY + 1);
}

// ------------------------------------------------------ annunciators
function drawAnnunciators(L) {
  const f = state.frame;
  const rows = [
    ["ATT", f.att.src],
    ["HDG", f.hdg.src],
    ["ACC", f.acc.src],
    ["GPS", f.gps.src === "win" ? "live" : f.gps.src],
  ];
  const x = 16;
  let y = L.H - 16 - rows.length * 20;
  ctx.font = FONT(11, 700);
  ctx.textBaseline = "middle";
  for (const [name, src] of rows) {
    const simulated = src === "sim";
    let label = simulated ? "SIM" : src.toUpperCase().slice(0, 12);
    // A connected micro:bit auto-calibrates its compass as it is rotated;
    // show the progress until headings become trustworthy.
    if (name === "HDG" && simulated && f.status.sensors.microbit) {
      const p = f.status.mbMagCal ?? 0;
      if (p < 1) label = `CAL ${Math.round(p * 100)}% — ROTATE BOARD`;
    }
    ctx.textAlign = "left";
    ctx.fillStyle = C.inkMut;
    ctx.fillText(name, x, y);
    ctx.fillStyle = simulated ? C.amber : C.green;
    ctx.fillText(label, x + 34, y);
    y += 20;
  }
}

// ----------------------------------------------------------- no data
function drawNoData(L) {
  const { W, H, topH } = L;
  ctx.fillStyle = "rgba(10, 10, 12, 0.55)";
  ctx.fillRect(0, topH, W, H - topH);
  ctx.strokeStyle = C.red;
  ctx.lineWidth = 6;
  ctx.beginPath();
  ctx.moveTo(W * 0.2, topH + 30);
  ctx.lineTo(W * 0.8, H - 30);
  ctx.moveTo(W * 0.8, topH + 30);
  ctx.lineTo(W * 0.2, H - 30);
  ctx.stroke();
  ctx.fillStyle = C.red;
  ctx.beginPath();
  ctx.roundRect(W / 2 - 110, H / 2 - 26, 220, 52, 6);
  ctx.fill();
  ctx.fillStyle = "#fff";
  ctx.font = FONT(22, 700);
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(state.everConnected ? "NO DATA" : "AWAITING LINK", W / 2, H / 2 + 1);
}

// ----------------------------------------------------------- main loop
function fit() {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(window.innerWidth * dpr);
  canvas.height = Math.round(window.innerHeight * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener("resize", fit);
fit();

let lastT = performance.now();
function render(t) {
  const dt = Math.min(0.1, (t - lastT) / 1000);
  lastT = t;
  updateDisplay(dt);

  const W = window.innerWidth;
  const H = window.innerHeight;
  const L = layout(W, H);

  ctx.clearRect(0, 0, W, H);
  const stale = !state.frame || performance.now() - state.lastFrameAt > 1500;

  if (state.frame) {
    drawAttitude(L);
    drawRollScaleAndPointer(L);
    drawAircraftSymbol(L);
    drawTopBar(L);       // before the tapes: their captions must win overlaps
    drawSpeedTape(L);
    drawAltTape(L);
    drawVsi(L);
    drawHsi(L);
    drawAnnunciators(L);
  } else {
    ctx.fillStyle = "#11151a";
    ctx.fillRect(0, 0, W, H);
  }
  if (stale) drawNoData(L);

  requestAnimationFrame(render);
}
requestAnimationFrame(render);
