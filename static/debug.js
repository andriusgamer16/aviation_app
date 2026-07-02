"use strict";
/* Live sensor debug view: polls /api/debug at 10 Hz and renders the whole
 * compass pipeline plus a field scatter plot for calibration diagnosis. */

const $ = (id) => document.getElementById(id);
const fmt = (v, digits = 0) =>
  v == null ? "--" : Number(v).toFixed(digits);

const trail = [];           // re-leveled (mx, my) history
const TRAIL_MAX = 1500;

document.getElementById("levelBtn").addEventListener("click", () =>
  fetch("/api/zero", { method: "POST" }));
document.getElementById("resetBtn").addEventListener("click", () => {
  if (confirm("Discard the learned magnetometer calibration?")) {
    trail.length = 0;
    fetch("/api/magcal/reset", { method: "POST" });
  }
});

function setVec(prefix, v, digits = 0) {
  const axes = ["x", "y", "z"];
  for (let i = 0; i < 3; i++) $(prefix + axes[i]).textContent = v ? fmt(v[i], digits) : "--";
}

function update(d) {
  const mb = d.microbit;
  $("status").textContent = mb
    ? `micro:bit LIVE (sample age ${fmt(mb.ageS, 2)} s)`
    : "micro:bit NOT CONNECTED";
  $("status").className = mb ? "ok" : "bad";

  if (mb) {
    setVec("a", mb.accelG, 3);
    $("ag").textContent = fmt(mb.gMag, 3);
    $("aage").textContent = fmt(mb.ageS, 2) + "s";
    $("a1raw").textContent = fmt(mb.a1);
    $("a1v").textContent = mb.a1Volts == null ? "--" : fmt(mb.a1Volts, 3) + " V";
  }
  if (d.airspeed) {
    $("asiGain").textContent = fmt(d.airspeed.gainKtPerV, 0);
    $("iasKt").textContent = d.airspeed.enabled
      ? fmt(d.airspeed.iasKt, 1) + " kt" : "disabled";
  }
  const m = mb && mb.mag;
  if (m) {
    setVec("mr", m.rawNt);
    setVec("mc", m.correctedNt);
    setVec("mv", m.virtNt);
    setVec("mh", m.horizNt);
    $("bmag").textContent = fmt(m.magnitudeNt) + " nT";
    $("bh").textContent = fmt(m.horizMagnitudeNt) + " nT";
    $("dip").textContent = m.dipDeg == null ? "--" : fmt(m.dipDeg, 1) + "°";
    $("hTilt").textContent =
      m.headingTiltComp == null ? "--" : String(Math.round(m.headingTiltComp)).padStart(3, "0") + "°";
    $("hFlat").textContent =
      m.headingFlat == null ? "--" : fmt(m.headingFlat, 1) + "°";
    const bad = m.magnitudeNt < 20000 || m.magnitudeNt > 90000;
    $("bmag").className = bad ? "warn" : "";
    trail.push([m.virtNt[0], m.virtNt[1]]);
    if (trail.length > TRAIL_MAX) trail.shift();
  }

  const c = d.magCal;
  // Earth's field tops out ~65000 nT, so a clean span can never exceed
  // ~130000 even with perfect tumbling; >110000 locally means a magnet
  // or ferrous object polluted the envelope.
  const SPAN_SUSPECT = 110000;
  for (let i = 0; i < 3; i++) {
    $(`e${i}min`).textContent = fmt(c.min[i]);
    $(`e${i}max`).textContent = fmt(c.max[i]);
    $(`e${i}off`).textContent = fmt(c.offset[i]);
    $(`e${i}span`).textContent = fmt(c.span[i]);
    $(`e${i}span`).className = c.span[i] > SPAN_SUSPECT ? "warn" : "";
  }
  $("warn").style.display =
    c.span.some((s) => s > SPAN_SUSPECT) ? "block" : "none";
  $("prog").textContent = Math.round(c.progress * 100) + "%" + (c.ready ? " (READY)" : "");
  $("progbar").style.width = c.progress * 100 + "%";

  drawScatter();
}

function drawScatter() {
  const cv = $("scatter");
  const ctx = cv.getContext("2d");
  const W = cv.width, H = cv.height, cx = W / 2, cy = H / 2;
  ctx.clearRect(0, 0, W, H);

  let maxR = 25000;
  for (const [x, y] of trail) maxR = Math.max(maxR, Math.hypot(x, y));
  const scale = (Math.min(W, H) / 2 - 20) / maxR;

  // Grid rings every 10000 nT + crosshair.
  ctx.strokeStyle = "#232c37";
  ctx.lineWidth = 1;
  for (let r = 10000; r <= maxR; r += 10000) {
    ctx.beginPath();
    ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.beginPath();
  ctx.moveTo(cx, 10); ctx.lineTo(cx, H - 10);
  ctx.moveTo(10, cy); ctx.lineTo(W - 10, cy);
  ctx.stroke();
  ctx.fillStyle = "#8d99a8";
  ctx.font = "11px Consolas, monospace";
  ctx.textAlign = "center";
  ctx.fillText("+Y (nose / logo edge)", cx, 18);
  ctx.textAlign = "left";
  ctx.fillText("+X (right)", W - 78, cy - 6);

  // Trail (older = dimmer). Canvas y grows downward; +my plots upward.
  for (let i = 0; i < trail.length; i++) {
    const [x, y] = trail[i];
    const a = 0.15 + 0.85 * (i / trail.length);
    ctx.fillStyle = `rgba(255, 91, 255, ${a * 0.5})`;
    ctx.fillRect(cx + x * scale - 1.5, cy - y * scale - 1.5, 3, 3);
  }
  if (trail.length) {
    const [x, y] = trail[trail.length - 1];
    ctx.fillStyle = "#ffffff";
    ctx.beginPath();
    ctx.arc(cx + x * scale, cy - y * scale, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#ff5bff";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + x * scale, cy - y * scale);
    ctx.stroke();
  }
}

async function poll() {
  try {
    const r = await fetch("/api/debug");
    update(await r.json());
    $("status").classList.remove("bad");
  } catch {
    $("status").textContent = "server unreachable";
    $("status").className = "bad";
  }
  setTimeout(poll, 100);
}
poll();
