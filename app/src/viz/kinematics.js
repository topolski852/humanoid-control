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

// --- model subsetting -------------------------------------------------------

/**
 * Narrow the bundled superset model down to a given ordered list of joint names.
 *
 * `viz_kinematics.json` describes every limb the robot could have (22 joints). What is actually
 * attached is a runtime fact that lives in the layout, and the server sends telemetry and the
 * contract for exactly those joints. This returns a model whose `joint_order` matches that list
 * one-for-one, so every array downstream — telemetry, contract, pose, ghosts — stays
 * index-aligned without a single name lookup in the render path.
 *
 * Returns null if any requested joint is not in the model: a joint we cannot draw is a model
 * that disagrees with the server, which is exactly the case the visualizer must refuse rather
 * than paper over.
 */
export function selectModel(model, jointNames) {
  const byName = new Map(model.joints.map((j) => [j.name, j]))
  const joints = []
  for (const name of jointNames) {
    const j = byName.get(name)
    if (!j) return null
    joints.push(j)
  }
  const keptLinks = new Set([model.root_link, ...joints.map((j) => j.child)])
  const index = new Map(joints.map((j, i) => [j.name, i]))
  return {
    ...model,
    joint_order: jointNames.slice(),
    joint_sign: jointNames.map((n) => model.joint_sign[model.joint_order.indexOf(n)]),
    joints,
    links: model.links.filter((l) => keptLinks.has(l.name)),
    // Rebuilt so `poses.default` still lines up with the narrowed joint list.
    poses: {
      zero: jointNames.map(() => 0),
      default: jointNames.map((n) => model.poses.default[model.joint_order.indexOf(n)] ?? 0),
      default_known: jointNames.map(
        (n) => model.poses.default_known?.[model.joint_order.indexOf(n)] ?? false,
      ),
    },
    indexOfJoint: (n) => index.get(n) ?? -1,
  }
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
    // Joints hanging directly off the root (hips, shoulders): a strut from the body out to the
    // joint, so the limb is attached to the torso rather than floating beside it. WHERE it
    // starts comes from the model (`mount`): the centreline for hips, which reads as a pelvis,
    // and the torso SIDE WALL for shoulders, because an arm bolts to the side of the chest and
    // running its strut to the centreline draws the shoulder as if it were on the sternum.
    // The mount point is in the base frame, so it goes through the root transform like
    // everything else — otherwise it stays put while the body tilts.
    if (j.parent === model.root_link) {
      const mount = j.mount?.xyz ?? [j.xyz[0], 0, j.xyz[2]]
      bones.push({
        a: applyPoint(fk.links[model.root_link] ?? IDENTITY4, mount),
        b: from,
        joint: j.name,
        side: j.name.startsWith('left') ? 'left' : 'right',
        role: j.limb?.endsWith('_arm') ? 'clavicle' : 'pelvis',
        radius: 0.03,
      })
    }

    // Terminal stub (the hand's centre of mass, from the URDF). Without it the wrist has
    // nothing distal and its rotation would be invisible. Note the hand sits nearly ON the
    // wrist axis, so this swings only ~1.4 cm — read the wrist's angle from the table, not
    // from this. It is here so the limb ends in a hand rather than in mid-air.
    if (j.tip) {
      bones.push({
        a: from,
        b: applyPoint(fk.links[j.child] ?? IDENTITY4, j.tip.xyz),
        joint: j.name,
        side: j.name.startsWith('left') ? 'left' : 'right',
        role: 'hand',
        radius: 0.024,
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

// --- inline-twist joints ----------------------------------------------------

/**
 * Ticks that make an INLINE TWIST joint readable.
 *
 * `shoulder_yaw` and `wrist_yaw` spin the limb about its own centreline. The kinematics are
 * correct — with a straight arm, shoulder_yaw moves the wrist 0.1 cm per radian against ~28 cm
 * for pitch and roll — but a drawing made of centrelines cannot show a rotation ABOUT a
 * centreline, so the joint looks broken when it is merely axial.
 *
 * The fix is a short spur perpendicular to the rotation axis, carried in the joint's CHILD
 * frame so it sweeps as the joint turns. Two spurs, 180 deg apart, so the orientation stays
 * readable from any viewing angle instead of vanishing when one points at the camera.
 */
export function buildTwistTicks(model, fk, radius = 0.05) {
  const out = []
  for (const j of model.joints) {
    if (!j.twist) continue
    const childMat = fk.links[j.child]
    if (!childMat) continue
    const origin = fk.joints[j.name]
    // Every joint in this URDF spins about its local +Z, so local +X is perpendicular to it.
    for (const s of [1, -1]) {
      out.push({
        a: origin,
        b: applyPoint(childMat, [s * radius, 0, 0]),
        joint: j.name,
        side: j.name.startsWith('left') ? 'left' : 'right',
      })
    }
  }
  return out
}

// --- gripper ----------------------------------------------------------------

/**
 * The claw at the end of an arm — a real servo gripper, which the URDF knows nothing about.
 *
 * PROVENANCE, because half of this is measured and half is not:
 *   - REACH is measured. `configs/robot_dimensions.json` records 12 cm from the wrist pivot to
 *     the closed fingertip, and the generator splits that into the palm reach (the URDF hand
 *     link's centre of mass) and whatever jaw length makes up the difference.
 *   - OPEN SPAN is not measured. The splay is nominal until someone puts a caliper on it.
 *
 * It earns its place twice over: it gives the wrist's inline twist something visible to carry,
 * since the jaws splay ACROSS the rotation axis and so sweep as the wrist turns; and it is where
 * the gripper's real open/close state will show once there is a path to drive it.
 *
 * @param open 0 = closed, 1 = fully splayed. DISPLAY ONLY — the gripper is a hobby servo, not one
 *             of the CAN ESCs, so it is not in the robot config and the daemon cannot address it.
 */
export function buildGripper(model, fk, open = 0.35) {
  const OPEN_ANGLE = Math.PI / 4        // splay of each jaw at open = 1
  const out = []
  for (const j of model.joints) {
    const h = j.hand
    const childMat = fk.links[j.child]
    if (!h || !childMat) continue
    const t = Math.max(0, Math.min(1, open)) * OPEN_ANGLE
    const side = j.name.startsWith('left') ? 'left' : 'right'

    // Build the claw CONCENTRIC with the rotation axis. The hand's centre of mass sits ~1.4 cm
    // off the axis, and hanging the jaws off that point would partly cancel their own offset —
    // leaving the claw almost on the axis and barely sweeping, which defeats its purpose. A
    // real gripper is mounted concentric anyway.
    const [ax, ay, az] = h.axis
    const an = Math.hypot(ax, ay, az) || 1
    const A = [ax / an, ay / an, az / an]
    const reach = h.palm[0] * A[0] + h.palm[1] * A[1] + h.palm[2] * A[2]   // palm distance along the axis
    const base = A.map((v) => v * reach)
    // Any unit vector perpendicular to the axis; which one is arbitrary but must be stable.
    let P = cross(A, [0, 0, 1])
    if (Math.hypot(...P) < 1e-6) P = cross(A, [1, 0, 0])
    const pn = Math.hypot(...P)
    P = P.map((v) => v / pn)

    const jaws = []
    for (const s of [1, -1]) {
      // Knuckles sit at the wrist link's own radius: wide enough that the claw's plane — and
      // so the wrist's rotation — is legible at the drawing's scale, and never collapsing to a
      // line when closed.
      const knuckle = base.map((v, k) => v + s * h.jaw_span * P[k])
      const tip = knuckle.map(
        (v, k) => v + h.jaw_length * (Math.sin(t) * s * P[k] + Math.cos(t) * A[k]),
      )
      jaws.push({ a: applyPoint(childMat, knuckle), b: applyPoint(childMat, tip) })
    }
    out.push({
      joint: j.name,
      side,
      palm: applyPoint(childMat, h.palm),
      wrist: fk.joints[j.name],
      // The bar across the knuckles: gives the claw a plane, which is what makes the wrist's
      // rotation legible at a glance.
      knuckleBar: [jaws[0].a, jaws[1].a],
      jaws,
    })
  }
  return out
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
