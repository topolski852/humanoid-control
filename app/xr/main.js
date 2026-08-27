import { Hud3D } from './hud3d.js';

// Quest 3 → humanoid-control arm teleop, over WebXR.
//
// Runs in the headset's browser. Per XR frame it reads each controller's GRIP pose plus its
// gamepad buttons and posts one JSON frame to /ws/xr. The server owns every safety decision;
// this page's only jobs are to sample honestly and to stop talking when it cannot.
//
// Deliberately vanilla — no React, no three.js. An `immersive-ar` session with passthrough
// needs no scene graph (the operator looks at the real arm through the cameras), and a
// dependency-free page is one less thing between a moving robot and its stop button.
//
// THE CONTRACT THIS PAGE OWES THE SERVER:
//   * `seq` strictly increases, one per frame. It IS the liveness signal — an open socket
//     carrying nothing is precisely the failure the server cannot otherwise see.
//   * `tracked` is honest: false when the pose is missing OR merely emulated. A stale
//     extrapolated pose reported as real is how a robot keeps moving after tracking dies.
//   * When we cannot send, we STOP sending. Never buffer and flush — a burst of stale poses
//     arriving after a stall is far worse than silence, which the server already handles.

const TX_HZ = 60;              // Quest renders at 72–120; the arm loop runs at 50. 60 is
                               // plenty to detect a 200 ms stall without flooding the link.
const SEND_FAIL_LIMIT_MS = 200; // give up (and let the server's stall timer fire) past this

const $enter = document.getElementById('enter');
const $status = document.getElementById('status');

let ws = null;
let seq = 0;
let sessionId = null;
let lastSendOk = 0;
let sending = true;
let lastTx = 0;

function show(msg, cls = '') {
  $status.textContent = msg;
  $status.className = cls;
}

// ── websocket ───────────────────────────────────────────────────────────────
function wsUrl() {
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  // Same origin: this page is served by the robot's TLS listener, so the token (if the
  // server requires a password) rides along the same way the main UI passes it.
  const token = new URLSearchParams(location.search).get('token');
  return `${proto}//${location.host}/ws/xr${token ? `?token=${encodeURIComponent(token)}` : ''}`;
}

function connect() {
  return new Promise((resolve, reject) => {
    ws = new WebSocket(wsUrl());
    ws.onopen = () => { lastSendOk = performance.now(); sending = true; resolve(); };
    ws.onerror = () => reject(new Error('websocket failed — is the robot reachable?'));
    ws.onclose = () => {
      sending = false;
      show('link closed — the arm has been stopped', 'err');
      // Say so IN THE HEADSET too. A message only on the desktop is a message the operator
      // cannot read while wearing it.
      if (hudOverlayActive) {
        hud({ instruction: 'LINK LOST', note: 'the arm has been stopped', tone: 'err' });
      }
    };
    // The server drives the HUD: calibration prompts, countdowns, live readouts, warnings.
    ws.onmessage = (ev) => {
      try {
        const m = JSON.parse(ev.data);
        if (m && m.type === 'hud') hud(m);
      } catch { /* a malformed status frame must never break the send loop */ }
    };
  });
}

// ── one controller's state ──────────────────────────────────────────────────
function readController(frame, refSpace, src) {
  if (!src || !src.gripSpace) return null;
  const pose = frame.getPose(src.gripSpace, refSpace);
  const gp = src.gamepad;
  // xr-standard mapping: 0 trigger, 1 squeeze, 3 thumbstick press, 4 A/X, 5 B/Y.
  // Menu and System are reserved by the Quest runtime and never appear here — which is why
  // E-STOP is on B/Y and not on a dedicated menu button.
  const btn = (i) => (gp && gp.buttons[i] ? gp.buttons[i] : null);
  const val = (i) => { const b = btn(i); return b ? b.value : 0; };
  const on = (i) => { const b = btn(i); return !!(b && b.pressed); };

  if (!pose) {
    // Tracking lost. Report the buttons we can still read (so E-STOP keeps working) but be
    // explicit that there is no pose — the server drops the run gate on this alone.
    return { p: null, q: null, tracked: false,
             mode: src.targetRayMode || null, hasGamepad: !!gp,
             nButtons: gp ? gp.buttons.length : 0, isHand: !!src.hand,
             trigger: val(0), squeeze: val(1), stick: [0, 0],
             a: on(4), b: on(5), stickPress: on(3) };
  }
  const { position: pp, orientation: o } = pose.transform;
  return {
    // Diagnostics: a trigger stuck at 0 is ambiguous between "not pressed", "no gamepad on
    // this input source" and "this is a HAND, not a controller". Report which.
    mode: src.targetRayMode || null,
    hasGamepad: !!gp,
    nButtons: gp ? gp.buttons.length : 0,
    isHand: !!src.hand,
    p: [pp.x, pp.y, pp.z],
    q: [o.x, o.y, o.z, o.w],
    // emulatedPosition means the runtime is guessing where the controller is. Guessed poses
    // must not drive a robot arm, so they count as untracked.
    tracked: !pose.emulatedPosition,
    trigger: val(0),
    squeeze: val(1),
    stick: gp && gp.axes.length >= 4 ? [gp.axes[2], gp.axes[3]] : [0, 0],
    a: on(4), b: on(5), stickPress: on(3),
  };
}

// ── in-headset HUD ──────────────────────────────────────────────────────────
// The operator cannot see the desktop while wearing the headset, so this is the ONLY
// feedback channel during a session. Driven entirely by the server over the same websocket
// (`hud` messages), which keeps the calibration state machine in one place rather than
// splitting it across two languages.
const $hud = document.getElementById('hud');
const $hudStep = document.getElementById('hud-step');
const $hudInstr = document.getElementById('hud-instruction');
const $hudCount = document.getElementById('hud-count');
const $hudFill = document.getElementById('hud-bar-fill');
const $hudLive = document.getElementById('hud-live');
const $hudNote = document.getElementById('hud-note');

let hudOverlayActive = false;
let hud3d = null;              // WebGL head-locked panel — the channel that actually works
let hudContent = {};

function hud(c = {}) {
  // The Quest browser refuses dom-overlay for immersive-ar (measured: domOverlayState was
  // null while the server pushed happily), so the DOM path below is only useful on a flat
  // browser. The 3D panel is what the operator actually sees in the headset.
  hudContent = c;
  if (hud3d) hud3d.set(c);
  hudDom(c);
}

function hudDom({ step = '', instruction = '', count = '', progress = null,
                  live = '', note = '', tone = '' } = {}) {
  $hudStep.textContent = step;
  $hudInstr.textContent = instruction;
  $hudCount.textContent = count;
  $hudLive.textContent = live;
  $hudNote.textContent = note;
  $hudFill.style.width = progress == null ? '0%' : `${Math.max(0, Math.min(100, progress))}%`;
  $hud.className = tone;
  $hud.hidden = false;
}

// ── body tracking ───────────────────────────────────────────────────────────
// WebXR Body Tracking (https://immersive-web.github.io/body-tracking/) exposes 83 joints
// as XRBodySpace, so each carries a full 6-DOF pose rather than just a position. We send
// only what the retargeter needs: the left arm chain plus a torso reference, because the
// arm angles are meaningless except relative to the torso — lean forward and every
// world-frame shoulder angle changes while your actual posture has not.
//
// ALL-OR-NOTHING. The spec requires the UA to either emulate obscured joints or report
// null poses for ALL of them. An emulated joint is the browser guessing where your elbow
// is, so `emulated` is reported per joint and the server treats any emulation as untracked
// — a guessed elbow must never drive a robot arm.
const BODY_JOINTS = [
  'hips', 'chest',
  'left-shoulder', 'left-scapula', 'left-arm-upper', 'left-arm-lower',
  'left-hand-wrist-twist', 'left-hand-wrist',
  'right-shoulder', 'right-arm-upper', 'right-arm-lower',
  'right-hand-wrist-twist', 'right-hand-wrist',
];

let bodyWarned = false;

function readBody(frame, refSpace) {
  const body = frame.body;
  if (!body) {
    if (!bodyWarned) {
      bodyWarned = true;
      console.warn('no frame.body — body-tracking unavailable '
        + '(enable "WebXR Experiments" in chrome://flags on the headset)');
    }
    return null;
  }
  const out = {};
  let present = 0, emulated = 0;
  for (const name of BODY_JOINTS) {
    let space = null;
    try { space = body.get(name); } catch { space = null; }
    if (!space) { out[name] = null; continue; }
    const pose = frame.getPose(space, refSpace);
    if (!pose) { out[name] = null; continue; }
    present++;
    if (pose.emulatedPosition) emulated++;
    const { position: p, orientation: o } = pose.transform;
    out[name] = {
      p: [r4(p.x), r4(p.y), r4(p.z)],
      q: [r4(o.x), r4(o.y), r4(o.z), r4(o.w)],
      e: !!pose.emulatedPosition,
    };
  }
  return { joints: out, present, emulated, total: BODY_JOINTS.length };
}

// 0.1 mm resolution is far finer than the tracker's real accuracy and keeps the frame
// small — at 60 Hz the full-precision floats roughly triple the payload for no benefit.
const r4 = (v) => Math.round(v * 1e4) / 1e4;

function send(obj) {
  if (!sending || !ws || ws.readyState !== WebSocket.OPEN) return;
  // Back-pressure: if frames are queueing, the link is not keeping up. Stop rather than
  // build a backlog the server would later receive as a burst of stale poses.
  if (ws.bufferedAmount > 64 * 1024) {
    if (performance.now() - lastSendOk > SEND_FAIL_LIMIT_MS) {
      sending = false;
      show('link congested — stopped sending (arm will stop)', 'warn');
    }
    return;
  }
  try {
    ws.send(JSON.stringify(obj));
    lastSendOk = performance.now();
  } catch {
    sending = false;
    show('send failed — stopped sending (arm will stop)', 'err');
  }
}

// ── session ─────────────────────────────────────────────────────────────────
async function start() {
  $enter.disabled = true;
  try {
    await connect();
  } catch (e) {
    show(String(e.message || e), 'err');
    $enter.disabled = false;
    return;
  }

  let session;
  try {
    // immersive-ar = passthrough on Quest 3: the operator sees the real arm through the
    // headset cameras, which is the whole safety model here.
    // body-tracking is OPTIONAL on purpose. It is experimental (needs "WebXR
    // Experiments" in chrome://flags on the headset) and requesting it as required would
    // make the whole session fail on a device that does not have it, taking the working
    // controller path down with it.
    // dom-overlay renders ordinary HTML on top of the passthrough view, INSIDE the headset.
    // Without it the operator is blind: passthrough cameras cannot resolve monitor text, so
    // anything printed to the desktop is unreadable exactly when it is needed. Every prompt
    // and readout during calibration has to appear here.
    session = await navigator.xr.requestSession('immersive-ar', {
      optionalFeatures: ['local-floor', 'body-tracking', 'dom-overlay'],
      domOverlay: { root: document.getElementById('hud') },
    });
  } catch (e) {
    show(`could not start XR: ${e.message || e}`, 'err');
    $enter.disabled = false;
    return;
  }

  // A SESSION WITH NO baseLayer COMPOSITES NOTHING, and the headset shows solid black rather
  // than passthrough. We draw no scene at all — the operator is looking at the real arm — but
  // WebXR still requires a render layer to exist, and the frame loop must clear it to
  // TRANSPARENT every frame so the camera feed shows through. Clearing to opaque black (the
  // GL default) produces exactly the same black screen as having no layer.
  let gl = null;
  try {
    const canvas = document.createElement('canvas');
    gl = canvas.getContext('webgl2', { xrCompatible: true, alpha: true })
      || canvas.getContext('webgl', { xrCompatible: true, alpha: true });
    if (!gl) throw new Error('no WebGL context');
    await gl.makeXRCompatible();
    session.updateRenderState({ baseLayer: new XRWebGLLayer(session, gl) });
    // Head-locked HUD in our own layer. Built here because it needs the GL context, and
    // this is the only feedback channel the headset will actually grant us.
    hud3d = new Hud3D(gl);
    hud3d.set({ step: 'CONNECTED', instruction: 'Ready',
                note: 'hold the trigger to drive \u00b7 Y = E-STOP' });
  } catch (e) {
    show(`could not set up the XR render layer: ${e.message || e}`, 'err');
    try { await session.end(); } catch { /* already gone */ }
    $enter.disabled = false;
    return;
  }

  // Did dom-overlay actually get granted? It is optional, so a denial is silent — and a
  // silent denial means the operator gets no feedback at all inside the headset, which is
  // worse than not starting. Say so on the desktop, which is the only channel left.
  hudOverlayActive = !!(session.domOverlayState
                        && session.domOverlayState.type === 'screen');
  if (!hudOverlayActive) {
    show('note: dom-overlay unavailable (expected on Quest) — using the WebGL HUD instead.\n'
         + 'Calibration needs it. Check the Quest browser supports it for immersive-ar.', 'warn');
  } else {
    hud({ step: 'CONNECTED', instruction: 'Ready',
          note: 'hold the trigger to drive \u00b7 Y = E-STOP' });
  }

  sessionId = Math.random().toString(16).slice(2, 8);
  seq = 0;
  const refSpace = await session.requestReferenceSpace('local-floor')
    .catch(() => session.requestReferenceSpace('local'));

  // Headset removed / menu opened. The XR animation loop PAUSES when hidden, so we cannot
  // rely on the frame loop to notice — send one explicit untracked frame first, which the
  // server treats as controller-not-tracked and drops the run gate immediately.
  session.addEventListener('visibilitychange', () => {
    if (session.visibilityState !== 'visible') {
      send({ seq: ++seq, t: performance.now(), session: sessionId,
             head: null, left: { tracked: false }, right: { tracked: false } });
    }
  });

  session.addEventListener('end', () => {
    send({ seq: ++seq, t: performance.now(), session: sessionId,
           head: null, left: { tracked: false }, right: { tracked: false } });
    try { ws && ws.close(); } catch { /* already gone */ }
    $enter.disabled = false;
    $enter.textContent = 'Re-enter passthrough';
    show('session ended — arm stopped');
  });

  const onFrame = (t, frame) => {
    session.requestAnimationFrame(onFrame);

    // Clear the XR framebuffer to FULLY TRANSPARENT so the passthrough camera feed shows
    // through. This runs every frame, before the send throttle — skipping it on throttled
    // frames would flicker the view.
    const layer = session.renderState.baseLayer;
    if (gl && layer) {
      gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
      gl.clearColor(0, 0, 0, 0);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

      // Draw the HUD once per eye, into each view's own viewport, so it has correct stereo
      // depth rather than sitting flat on one eye.
      const vp = frame.getViewerPose(refSpace);
      if (hud3d && vp) {
        for (const view of vp.views) {
          const v = layer.getViewport(view);
          if (!v) continue;
          gl.viewport(v.x, v.y, v.width, v.height);
          hud3d.draw(view, vp);
        }
      }
    }

    if (t - lastTx < 1000 / TX_HZ) return;
    lastTx = t;

    const viewer = frame.getViewerPose(refSpace);
    let left = null, right = null;
    for (const src of session.inputSources) {
      const c = readController(frame, refSpace, src);
      if (!c) continue;
      if (src.handedness === 'left') left = c;
      else if (src.handedness === 'right') right = c;
    }

    send({
      seq: ++seq,
      t: Math.round(t * 10) / 10,
      session: sessionId,
      head: viewer ? {
        p: [viewer.transform.position.x, viewer.transform.position.y, viewer.transform.position.z],
        q: [viewer.transform.orientation.x, viewer.transform.orientation.y,
            viewer.transform.orientation.z, viewer.transform.orientation.w],
        tracked: true,
      } : null,
      left, right,
      body: readBody(frame, refSpace),
      // Was dom-overlay granted? Optional features fail silently, and a HUD the
      // operator cannot see is indistinguishable from a HUD the server never sent.
      overlay: hudOverlayActive,
    });

    if (seq % 30 === 0) {
      const drive = left && left.tracked ? left : (right && right.tracked ? right : null);
      show(drive
        ? `seq ${seq}   trigger ${drive.trigger.toFixed(2)}   ${
            drive.trigger >= 0.5 ? 'DRIVING' : 'idle'}\ntracked — B/Y = E-STOP`
        : `seq ${seq}\ncontroller not tracked — arm held`,
        drive ? '' : 'warn');
    }
  };
  session.requestAnimationFrame(onFrame);
  show('connected — hold the trigger to drive');
}

// ── boot ────────────────────────────────────────────────────────────────────
(async () => {
  // `navigator.xr` only exists in a SECURE CONTEXT. Plain HTTP has none, and — the part that
  // catches people out — neither does an HTTPS origin whose certificate you click-through:
  // Chromium keeps such an origin flagged and withholds powerful features from it. So a
  // self-signed cert is not reliably enough on its own.
  //
  // The dependable route is `adb reverse`, because Chromium trusts `localhost` as a secure
  // context with no certificate at all.
  const insecure = !window.isSecureContext;
  if (!navigator.xr) {
    $enter.textContent = 'WebXR unavailable';
    show(insecure
      ? 'This page is not a secure context, so the browser hides WebXR.\n\n'
        + 'On the robot:  adb reverse tcp:8000 tcp:8000\n'
        + 'On the Quest:  open http://localhost:8000/xr/\n\n'
        + 'localhost is trusted with no certificate. A self-signed HTTPS cert that you '
        + 'click through is NOT enough — Chromium keeps withholding WebXR from it.'
      : 'This browser has no WebXR at all. Use the Quest browser.', 'err');
    return;
  }
  const ok = await navigator.xr.isSessionSupported('immersive-ar').catch(() => false);
  if (!ok) {
    $enter.textContent = 'Passthrough unavailable';
    show('immersive-ar is not supported here. This page is meant for the Quest 3 browser.',
         'err');
    return;
  }
  $enter.disabled = false;
  $enter.textContent = 'Enter passthrough';
  $enter.addEventListener('click', start);
})();
