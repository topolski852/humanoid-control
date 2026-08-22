// Forward kinematics + orthographic projection for the robot wireframe.
//
// Pure functions, no React — so the FK can be unit-tested against the golden vectors in
// `kinematics.test.js` without rendering anything.
//
// FRAMES. This is the whole point of the visualizer, so it is worth being explicit:
//
//   telemetry `joints[i].position`   DEVICE frame — un-mirrored, both legs share a sign
//                                    for a symmetric stance. This is what the ESCs report
//                                    and what the policy observation reads.
//   the URDF / sim                   POLICY frame — left<->right mirror-symmetric.
//
// They differ by a sign on exactly right_hip_roll, right_hip_yaw and right_ankle_roll.
// `deviceToUrdf()` below is the ONLY place that conversion happens. Getting it wrong draws
// a plausible-looking robot with the right leg swung ~6.5 cm inboard of where it really is —
// see humanoid_control/config.py POLICY_FRAME_MIRRORED_JOINTS.
//
// Geometry conventions (inherited from the URDF): +X forward, +Y left, +Z up; metres and
// radians; a child frame is T(xyz) · R(rpy) · R_axis(q).

// --- small 4x4 helpers (row-major, flat 16) ---------------------------------

export const IDENTITY4 = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]

export function matMul(a, b) {
  const out = new Array(16)
  for (let r = 0; r < 4; r++) {
    for (let c = 0; c < 4; c++) {
      out[r * 4 + c] =
        a[r * 4] * b[c] +
        a[r * 4 + 1] * b[4 + c] +
        a[r * 4 + 2] * b[8 + c] +
        a[r * 4 + 3] * b[12 + c]
    }
  }
  return out
}

/** Apply a 4x4 to a 3-vector treated as a point (w = 1). */
export function applyPoint(m, p) {
  return [
    m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3],
    m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7],
    m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11],
  ]
}

/** Apply a 4x4 to a 3-vector treated as a direction (w = 0 — translation ignored). */
export function applyDir(m, v) {
  return [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[4] * v[0] + m[5] * v[1] + m[6] * v[2],
    m[8] * v[0] + m[9] * v[1] + m[10] * v[2],
  ]
}

/** The translation column of a 4x4 — i.e. the frame's origin in world coordinates. */
export function originOf(m) {
  return [m[3], m[7], m[11]]
}

/** Build T(xyz) · R, where R is the flat row-major 3x3 precomputed in the model JSON. */
function rigid(R, xyz) {
  return [
    R[0], R[1], R[2], xyz[0],
    R[3], R[4], R[5], xyz[1],
    R[6], R[7], R[8], xyz[2],
    0, 0, 0, 1,
  ]
}

/** Rotation of `angle` about an arbitrary unit axis (Rodrigues). */
export function axisRotation(axis, angle) {
  const [x, y, z] = axis
  const c = Math.cos(angle)
  const s = Math.sin(angle)
  const t = 1 - c
  return [
    t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0,
    0, 0, 0, 1,
  ]
}

/** URDF fixed-axis roll-pitch-yaw -> flat row-major 3x3. R = Rz(yaw)·Ry(pitch)·Rx(roll). */
export function rpyToMatrix3(roll, pitch, yaw) {
  const cr = Math.cos(roll), sr = Math.sin(roll)
  const cp = Math.cos(pitch), sp = Math.sin(pitch)
  const cy = Math.cos(yaw), sy = Math.sin(yaw)
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr,
  ]
}

// --- frames -----------------------------------------------------------------

/**
 * Convert a device-frame joint vector to the URDF frame the model is drawn in.
 * `sign` is the contract's policy_frame_sign, fetched live from GET /api/contract.
 */
export function deviceToUrdf(devicePose, sign) {
  return devicePose.map((q, i) => (Number.isFinite(q) ? q * (sign[i] ?? 1) : q))
}

// --- base attitude from the IMU ---------------------------------------------

/**
 * Rotation taking unit vector `from` onto unit vector `to`, as a flat 4x4.
 * JS twin of `shortest_arc_quat()` in scripts/imu_calibrate.py, built directly as a matrix.
 *
 * "Shortest arc" is the point: it introduces no rotation about the `from`/`to` axis, so
 * using it with gravity yields tilt with ZERO yaw.
 */
export function shortestArcMatrix(from, to) {
  const nf = Math.hypot(...from)
  const nt = Math.hypot(...to)
  if (!(nf > 0) || !(nt > 0)) return IDENTITY4
  const a = from.map((v) => v / nf)
  const b = to.map((v) => v / nt)
  const d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

  if (d >= 1 - 1e-9) return IDENTITY4                  // already aligned
  if (d <= -1 + 1e-9) {
    // Antiparallel (robot exactly inverted): the arc is 180 deg and its axis is undefined,
    // so pick any perpendicular. Which one we pick is arbitrary but must be stable.
    let axis = cross(a, [1, 0, 0])
    if (Math.hypot(...axis) < 1e-6) axis = cross(a, [0, 1, 0])
    const n = Math.hypot(...axis)
    return axisRotation(axis.map((v) => v / n), Math.PI)
  }
  const axis = cross(a, b)
  const n = Math.hypot(...axis)
  return axisRotation(axis.map((v) => v / n), Math.atan2(n, d))
}

function cross(u, v) {
  return [
    u[1] * v[2] - u[2] * v[1],
    u[2] * v[0] - u[0] * v[2],
    u[0] * v[1] - u[1] * v[0],
  ]
}

/**
 * Base->world rotation from the IMU, as a flat 4x4 to seed `forwardKinematics`.
 *
 * `projected_gravity` is the gravity direction expressed in the BASE frame ([0,0,-1] when
 * upright), so the base's orientation in the world is the rotation carrying it back onto
 * world-down. Tilt only — gravity cannot observe yaw, and the IMU's own yaw origin is
 * unrelated to robot-forward, so leaving it out is correct rather than merely convenient.
 *
 * Note we use `projected_gravity` and NOT `base.quaternion`: the daemon ships the quaternion
 * in the RAW IMU frame with the mounting rotation NOT applied (witmotion_reader.hpp calls it
 * "diagnostic"), while projected_gravity is mounting-corrected. They coincide only while
 * mounting_quat is identity. Gravity is also exactly what the policy observation consumes.
 *
 * Returns IDENTITY4 (draw upright) when the IMU is absent or the vector is not a unit vector.
 */
export function baseRotationFromGravity(projectedGravity) {
  if (!Array.isArray(projectedGravity) || projectedGravity.length < 3) return IDENTITY4
  const g = projectedGravity.slice(0, 3).map(Number)
  if (!g.every(Number.isFinite)) return IDENTITY4
  const norm = Math.hypot(...g)
  // A healthy IMU reports a unit vector. Anything else is a fault, not an attitude.
  if (!(norm > 0.9 && norm < 1.1)) return IDENTITY4
  return shortestArcMatrix(g, [0, 0, -1])
}

/** Tilt away from vertical, in radians, for a status readout. 0 when upright. */
export function tiltAngle(projectedGravity) {
  if (!Array.isArray(projectedGravity) || projectedGravity.length < 3) return 0
  const [gx, gy, gz] = projectedGravity
  if (![gx, gy, gz].every(Number.isFinite)) return 0
  return Math.atan2(Math.hypot(gx, gy), -gz)
}

// --- forward kinematics -----------------------------------------------------

/**
 * Run FK over the whole chain.
 *
 * @param model      parsed viz_kinematics.json
 * @param urdfAngles (12,) joint angles in the URDF frame, in model.joint_order order
 * @param rootMat    world transform of the base link — pass the IMU attitude from
 *                   `baseRotationFromGravity()` to orient the whole body. Defaults to
 *                   upright. Seeding the root propagates through every matMul below, so
 *                   this one argument rotates the entire tree.
 * @returns {{links: Object, joints: Object}} world 4x4 per link name, and the world
 *          origin (3-vector) of every joint by joint name.
 */
export function forwardKinematics(model, urdfAngles, rootMat = IDENTITY4) {
  const links = { [model.root_link]: rootMat }
  const joints = {}
  model.joints.forEach((j, i) => {
    const parent = links[j.parent] ?? rootMat
    const fixed = matMul(parent, rigid(j.R, j.xyz))
    joints[j.name] = originOf(fixed)
    const q = Number.isFinite(urdfAngles[i]) ? urdfAngles[i] : 0
    links[j.child] = matMul(fixed, axisRotation(j.axis, q))
  })
  return { links, joints }
}

// --- projection -------------------------------------------------------------

/**
 * Orthographic projection to 2D. `azimuth` in radians: 0 looks at the robot head-on from
 * the front, so its left (+Y) lands on the viewer's right — the same handedness you get
 * standing in front of the real robot. ±π/2 gives the sagittal views.
 *
 * Returns [u, v] in metres: u is screen-right, v is screen-up.
 */
export function project(p, azimuth) {
  const s = Math.sin(azimuth)
  const c = Math.cos(azimuth)
  return [-p[0] * s + p[1] * c, p[2]]
}

/** Depth along the view direction — larger is nearer the camera. For painter's-order sorting. */
export function depth(p, azimuth) {
  return p[0] * Math.cos(azimuth) + p[1] * Math.sin(azimuth)
}

// --- shape tessellation -----------------------------------------------------

/** The 12 edges of a box, as world-space [a, b] segment pairs. */
export function boxEdges(shape, worldMat) {
  const [sx, sy, sz] = shape.size.map((v) => v / 2)
  const R = rpyToMatrix3(...(shape.rpy ?? [0, 0, 0]))
  const local = rigid(R, shape.xyz ?? [0, 0, 0])
  const m = matMul(worldMat, local)
  const corners = []
  for (const x of [-sx, sx]) {
    for (const y of [-sy, sy]) {
      for (const z of [-sz, sz]) corners.push(applyPoint(m, [x, y, z]))
    }
  }
  // Corner index bits: x=4, y=2, z=1. Edges join corners differing in exactly one bit.
  const EDGES = [
    [0, 1], [2, 3], [4, 5], [6, 7],
    [0, 2], [1, 3], [4, 6], [5, 7],
    [0, 4], [1, 5], [2, 6], [3, 7],
  ]
  return EDGES.map(([a, b]) => [corners[a], corners[b]])
}

/**
 * Half-thickness of a link, in metres, taken from its collision cylinder/capsule.
 *
 * The limbs are drawn as round-capped strokes along the joint-to-joint chain rather than as
 * separate capsule outlines — one stroke per bone instead of two, and the chain is
 * guaranteed to stay connected. The collision radius only supplies the *width*, so the
 * thickness still comes from the physics model rather than an invented constant.
 */
export function linkRadius(model, linkName, fallback = 0.02) {
  const link = model.links.find((l) => l.name === linkName)
  const round = link?.shapes?.find((s) => s.type === 'cylinder' || s.type === 'capsule')
  return round?.radius ?? fallback
}

/**
 * Sample a joint's limit arc as world-space 3D points, so it projects correctly at any
 * azimuth. Sampling in 3D rather than drawing a flat screen-space arc matters here: the hip
 * joints rotate about 45-degree canted axes, so a screen-space arc would be a lie.
 */
export function arcPoints(model, fk, joint, radius, samples = 14) {
  const parentMat = fk.links[joint.parent] ?? IDENTITY4
  const fixed = matMul(parentMat, rigid(joint.R, joint.xyz))
  const lo = joint.limit.lower
  const hi = joint.limit.upper
  const pts = []
  for (let i = 0; i <= samples; i++) {
    const q = lo + ((hi - lo) * i) / samples
    const rot = matMul(fixed, axisRotation(joint.axis, q))
    // A radius vector perpendicular to the rotation axis. The URDF's leg joints all spin
    // about local +Z, so local +X sweeps the arc.
    pts.push(applyPoint(rot, [radius, 0, 0]))
  }
  return pts
}

// --- assembly ---------------------------------------------------------------

/**
 * Turn an FK result into flat draw lists in world space. The renderer only has to project
 * and stroke these — no geometry logic in the component.
 */
export function buildDrawables(model, fk) {
  // Boxes are only the torso and the two foot pads — the shapes whose *outline* carries
  // information. Everything else reads better as a stroked bone.
  const boxes = []
  for (const link of model.links) {
    const worldMat = fk.links[link.name]
    if (!worldMat) continue
    for (const shape of link.shapes ?? []) {
      if (shape.type === 'box') {
        boxes.push({ edges: boxEdges(shape, worldMat), role: shape.role, link: link.name })
      }
    }
  }

  // One stroke per parent->child joint hop, so the chain is connected by construction even
  // where a link carries no collision primitive of its own.
  const bones = []
  for (const j of model.joints) {
    const child = model.joints.find((k) => k.parent === j.child)
    const from = fk.joints[j.name]
    if (!from) continue
    if (child) {
      bones.push({
        a: from,
        b: fk.joints[child.name],
        joint: j.name,
        side: j.name.startsWith('left') ? 'left' : 'right',
        role: shapeRoleOf(model, j.child),
        radius: linkRadius(model, j.child, 0.025),
      })
    }
    // Pelvis: the root's centreline out to each hip. The centreline point is expressed in
    // the base frame, so it must go through the root transform like everything else —
    // otherwise the pelvis stays upright while the body tilts.
    if (j.parent === model.root_link) {
      bones.push({
        a: applyPoint(fk.links[model.root_link] ?? IDENTITY4, [j.xyz[0], 0, j.xyz[2]]),
        b: from,
        joint: j.name,
        side: j.name.startsWith('left') ? 'left' : 'right',
        role: 'pelvis',
        radius: 0.03,
      })
    }
  }

  // Note the skeleton deliberately ends at the ankle_roll joint. A stub out to the foot pad
  // centre reads as a stray spur rather than a limb, and the foot's own box already shows its
  // angle clearly.
  return { boxes, bones }
}

function shapeRoleOf(model, linkName) {
  const link = model.links.find((l) => l.name === linkName)
  return link?.shapes?.[0]?.role ?? 'link'
}

/** Lowest world-Z of anything drawn — used to place the ground line under the feet. */
export function lowestZ(drawables) {
  let min = Infinity
  for (const b of drawables.boxes) {
    for (const [p, q] of b.edges) min = Math.min(min, p[2], q[2])
  }
  for (const s of drawables.bones) min = Math.min(min, s.a[2] - s.radius, s.b[2] - s.radius)
  return Number.isFinite(min) ? min : 0
}
