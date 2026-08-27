// In-headset HUD, rendered as a head-locked quad in the WebGL layer.
//
// WHY NOT dom-overlay. That was the first attempt and it is far less code, but the Quest
// browser REFUSES it for immersive-ar sessions — measured: `session.domOverlayState` came
// back null while the server was pushing HUD frames perfectly happily. Which makes sense:
// there is no 2D screen to overlay onto in a stereo view. The page received every update and
// had nowhere to draw it.
//
// So we draw into the layer we already own — the one created to clear the framebuffer
// transparent so passthrough shows through. Nobody can refuse that.
//
// The panel is anchored to the VIEWER (head) pose and transformed into each eye. Anchoring
// to each eye's own view space instead is the obvious shortcut and is WRONG: the two eyes
// have different transforms, so the panel lands in a slightly different place for each, and
// the brain fuses that disparity as a tilted panel.
//
// Text is rendered with ordinary 2D canvas `fillText` and uploaded as a texture. Laying out
// text in raw WebGL would be miserable; this way the layout stays readable and only the
// blitting is graphics code. The texture is re-uploaded ONLY when the content changes, so a
// static HUD costs one quad draw per eye per frame.

// PLACEMENT. All in metres, as an offset from the head. Tune these freely — they are the
// only things that decide where the panel sits and how big it feels.
//   X negative = left of centre      Y negative = below centre      Z negative = in front
// Parked off to the LEFT and low, so the operator's forward view of the arm stays clear:
// this is an instrument panel to glance at, not something to read through.
const PANEL_W = 0.46;      // metres wide at the panel's distance
const PANEL_H = 0.27;
const PANEL_X = -0.18;     // slightly left of centre — far enough not to cover the arm,
                           // near enough to read without turning your head
const PANEL_Y = -0.17;     // slightly below the eye line
const PANEL_Z = -1.00;     // one metre in front
const TEX_W = 1024;        // texture resolution; 2:1-ish to match the panel aspect
const TEX_H = 596;

const VERT = `
attribute vec2 aPos;
attribute vec2 aUV;
uniform mat4 uMVP;
varying vec2 vUV;
void main() {
  vUV = aUV;
  gl_Position = uMVP * vec4(aPos, 0.0, 1.0);
}`;

const FRAG = `
precision mediump float;
uniform sampler2D uTex;
varying vec2 vUV;
void main() {
  vec4 c = texture2D(uTex, vUV);
  if (c.a < 0.01) discard;
  gl_FragColor = c;
}`;

function compile(gl, type, src) {
  const s = gl.createShader(type);
  gl.shaderSource(s, src);
  gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) {
    throw new Error('HUD shader: ' + gl.getShaderInfoLog(s));
  }
  return s;
}

export class Hud3D {
  constructor(gl) {
    this.gl = gl;
    this.dirty = true;
    this.content = {};

    this.canvas = document.createElement('canvas');
    this.canvas.width = TEX_W;
    this.canvas.height = TEX_H;
    this.ctx = this.canvas.getContext('2d');

    const p = gl.createProgram();
    gl.attachShader(p, compile(gl, gl.VERTEX_SHADER, VERT));
    gl.attachShader(p, compile(gl, gl.FRAGMENT_SHADER, FRAG));
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) {
      throw new Error('HUD link: ' + gl.getProgramInfoLog(p));
    }
    this.prog = p;
    this.aPos = gl.getAttribLocation(p, 'aPos');
    this.aUV = gl.getAttribLocation(p, 'aUV');
    this.uMVP = gl.getUniformLocation(p, 'uMVP');
    this.uTex = gl.getUniformLocation(p, 'uTex');

    const hw = PANEL_W / 2, hh = PANEL_H / 2;
    this.buf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, this.buf);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([
      -hw, -hh, 0, 1,   hw, -hh, 1, 1,   -hw, hh, 0, 0,
       hw, -hh, 1, 1,   hw,  hh, 1, 0,   -hw, hh, 0, 0,
    ]), gl.STATIC_DRAW);

    this.tex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, this.tex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  }

  /** Replace the HUD content. Cheap to call every frame — only redraws when it changes. */
  set(content) {
    const same = JSON.stringify(content) === JSON.stringify(this.content);
    if (same) return;
    this.content = content || {};
    this.dirty = true;
  }

  _redraw() {
    const c = this.ctx, W = TEX_W, H = TEX_H;
    const { step = '', instruction = '', count = '', progress = null,
            live = '', note = '', tone = '' } = this.content;
    const accent = tone === 'err' ? '#f87171' : tone === 'warn' ? '#fbbf24' : '#4ade80';

    c.clearRect(0, 0, W, H);
    // Near-SOLID plate. A translucent panel over a live camera feed is exhausting to read:
    // the background moves, the contrast shifts constantly, and the eye keeps trying to
    // refocus between the text and whatever is behind it.
    c.fillStyle = 'rgba(10,12,16,0.985)';
    roundRect(c, 8, 8, W - 16, H - 16, 28);
    c.fill();
    c.strokeStyle = accent;
    c.lineWidth = 4;
    roundRect(c, 8, 8, W - 16, H - 16, 28);
    c.stroke();

    c.textAlign = 'center';
    c.textBaseline = 'middle';

    let y = 62;
    if (step) {
      c.fillStyle = '#93c5fd';
      c.font = '600 30px ui-sans-serif, system-ui, sans-serif';
      c.fillText(step, W / 2, y);
    }
    y += 62;

    if (instruction) {
      c.fillStyle = '#ffffff';
      c.font = '800 54px ui-sans-serif, system-ui, sans-serif';
      wrapText(c, instruction, W / 2, y, W - 90, 58);
      y += 58 * Math.max(1, countLines(c, instruction, W - 90));
    }

    if (count) {
      y += 20;
      c.fillStyle = accent;
      c.font = '800 76px ui-sans-serif, system-ui, sans-serif';
      c.fillText(count, W / 2, y);
      y += 56;
    }

    if (progress != null) {
      y += 22;
      const bw = W - 200, bx = 100, bh = 20;
      c.fillStyle = 'rgba(255,255,255,0.22)';
      roundRect(c, bx, y - bh / 2, bw, bh, bh / 2); c.fill();
      c.fillStyle = accent;
      const w = Math.max(0, Math.min(100, progress)) / 100 * bw;
      if (w > 2) { roundRect(c, bx, y - bh / 2, w, bh, bh / 2); c.fill(); }
      y += 40;
    }

    if (live) {
      y += 14;
      c.fillStyle = '#e5e7eb';
      c.font = '500 30px ui-monospace, SFMono-Regular, Menlo, monospace';
      for (const line of String(live).split('\n')) {
        c.fillText(line, W / 2, y);
        y += 36;
      }
    }

    if (note) {
      y += 10;
      c.fillStyle = accent;
      c.font = '600 27px ui-sans-serif, system-ui, sans-serif';
      wrapText(c, note, W / 2, Math.min(y, H - 46), W - 110, 32);
    }

    const gl = this.gl;
    gl.bindTexture(gl.TEXTURE_2D, this.tex);
    gl.pixelStorei(gl.UNPACK_PREMULTIPLY_ALPHA_WEBGL, false);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, this.canvas);
    this.dirty = false;
  }

  /**
   * Draw for one eye.
   *
   * HEAD-locked, not EYE-locked. The obvious shortcut — park the quad at a fixed offset in
   * each eye's own view space — puts it in a slightly DIFFERENT place for each eye, because
   * the two eyes have different transforms. The brain fuses that disparity as a panel that
   * is tilted or rotated, which is exactly what it looked like.
   *
   * So the panel is anchored to the VIEWER (head) pose and then transformed into each eye:
   *   mvp = projection * inverse(view) * viewer * offset
   * Both eyes then agree on where it is in the world, and the only difference between them
   * is genuine stereo parallax.
   */
  draw(view, viewerPose) {
    if (this.dirty) this._redraw();
    const gl = this.gl;

    // Fixed offset from the head (column-major translation).
    const offset = new Float32Array([
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      PANEL_X, PANEL_Y, PANEL_Z, 1,
    ]);
    const world = mul4(viewerPose.transform.matrix, offset);
    const mvp = mul4(view.projectionMatrix, mul4(view.transform.inverse.matrix, world));

    gl.useProgram(this.prog);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    // Always on top: the HUD is an instrument panel, not part of the scene, and there is no
    // scene to be occluded by anyway.
    gl.disable(gl.DEPTH_TEST);

    gl.bindBuffer(gl.ARRAY_BUFFER, this.buf);
    gl.enableVertexAttribArray(this.aPos);
    gl.vertexAttribPointer(this.aPos, 2, gl.FLOAT, false, 16, 0);
    gl.enableVertexAttribArray(this.aUV);
    gl.vertexAttribPointer(this.aUV, 2, gl.FLOAT, false, 16, 8);

    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.tex);
    gl.uniform1i(this.uTex, 0);
    gl.uniformMatrix4fv(this.uMVP, false, mvp);
    gl.drawArrays(gl.TRIANGLES, 0, 6);
  }
}

// ── small helpers ───────────────────────────────────────────────────────────
function mul4(a, b) {
  const o = new Float32Array(16);
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      o[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1]
                   + a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
    }
  }
  return o;
}

function roundRect(c, x, y, w, h, r) {
  c.beginPath();
  c.moveTo(x + r, y);
  c.arcTo(x + w, y, x + w, y + h, r);
  c.arcTo(x + w, y + h, x, y + h, r);
  c.arcTo(x, y + h, x, y, r);
  c.arcTo(x, y, x + w, y, r);
  c.closePath();
}

function splitLines(c, text, maxW) {
  const out = [];
  for (const para of String(text).split('\n')) {
    let line = '';
    for (const word of para.split(' ')) {
      const t = line ? line + ' ' + word : word;
      if (c.measureText(t).width > maxW && line) { out.push(line); line = word; }
      else line = t;
    }
    out.push(line);
  }
  return out;
}

function countLines(c, text, maxW) { return splitLines(c, text, maxW).length; }

function wrapText(c, text, cx, y, maxW, lh) {
  const lines = splitLines(c, text, maxW);
  lines.forEach((l, i) => c.fillText(l, cx, y + i * lh));
}
