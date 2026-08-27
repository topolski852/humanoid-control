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
    ws.onclose = () => { sending = false; show('link closed — the arm has been stopped', 'err'); };
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
             trigger: val(0), squeeze: val(1), stick: [0, 0],
             a: on(4), b: on(5), stickPress: on(3) };
  }
  const { position: pp, orientation: o } = pose.transform;
  return {
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
    session = await navigator.xr.requestSession('immersive-ar', {
      optionalFeatures: ['local-floor'],
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
  } catch (e) {
    show(`could not set up the XR render layer: ${e.message || e}`, 'err');
    try { await session.end(); } catch { /* already gone */ }
    $enter.disabled = false;
    return;
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
