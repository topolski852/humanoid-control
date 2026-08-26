// FK regression tests. Run with:  node --test app/src/viz/
//
// Uses only node:test / node:assert so it needs no test-runner dependency.
//
// The golden values below were derived INDEPENDENTLY (by FK'ing the URDF directly in Python
// with numpy) rather than captured from this implementation's own output — otherwise the
// test would only prove the code agrees with itself.
import test from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'

import {
  forwardKinematics, deviceToUrdf, project, buildDrawables, lowestZ, linkRadius, selectModel,
  buildTwistTicks, buildGripper,
  shortestArcMatrix, baseRotationFromGravity, tiltAngle, applyPoint, applyDir, IDENTITY4,
} from './kinematics.js'

// Read rather than `import ... with { type: 'json' }`: import attributes need Node >= 20.10 and
// the older `assert { type: 'json' }` spelling is deprecated in newer Node. fs works on both.
const bundle = JSON.parse(
  readFileSync(fileURLToPath(new URL('../data/viz_kinematics.json', import.meta.url)), 'utf8'),
)

// The bundled model is the SUPERSET of every limb the robot could have. The app renders the
// subset the live layout enables, so the leg goldens below are asserted against the leg subset —
// exactly what a legs-only layout draws — and `selectModel` is exercised on the way through.
const LEG_JOINTS = bundle.limbs.left_leg.concat(bundle.limbs.right_leg)
const model = selectModel(bundle, LEG_JOINTS)

const TOL = 1e-3

function close(actual, expected, tol = TOL, what = '') {
  assert.ok(
    Math.abs(actual - expected) <= tol,
    `${what}: expected ${expected}, got ${actual} (tol ${tol})`,
  )
}

function closeVec(actual, expected, tol = TOL, what = '') {
  expected.forEach((e, i) => close(actual[i], e, tol, `${what}[${i}]`))
}

test('bundle is the full-body superset in layout order', () => {
  assert.equal(bundle.joints.length, 22)
  assert.equal(bundle.joint_order.length, 22)
  assert.deepEqual(Object.keys(bundle.limbs), ['left_leg', 'right_leg', 'left_arm', 'right_arm'])
  bundle.joints.forEach((j, i) => {
    assert.equal(j.name, bundle.joint_order[i])
    assert.equal(j.index, i)
    assert.ok(j.limb, `${j.name} is tagged with its limb`)
  })
  // Device names are authoritative; the URDF calls the wrist "elbow_roll".
  const wrist = bundle.joints.find((j) => j.name === 'left_wrist_yaw_joint')
  assert.equal(wrist.urdf_name, 'arm_left_elbow_roll_joint')
})

test('selectModel narrows to a joint list and stays index-aligned', () => {
  assert.equal(model.joints.length, 12)
  assert.equal(model.joint_order.length, 12)
  model.joints.forEach((j, i) => assert.equal(j.name, model.joint_order[i]))

  // An arm-only selection is a first-class configuration, not a degenerate one.
  const arm = selectModel(bundle, bundle.limbs.left_arm)
  assert.equal(arm.joints.length, 5)
  assert.deepEqual(arm.joint_sign, [1, 1, 1, 1, 1])
  assert.equal(arm.joints[0].parent, bundle.root_link, 'shoulder hangs off the base')

  // A joint the bundle does not contain must fail loudly, not silently draw a partial robot.
  assert.equal(selectModel(bundle, ['left_hip_roll_joint', 'nope_joint']), null)
})

test('joint_sign matches the contract sign map', () => {
  // Legs must equal humanoid_control.config.LegPolicyContract.policy_frame_sign. The app also
  // checks this at runtime against GET /api/contract; this catches it at build time.
  assert.deepEqual(model.joint_sign, [1, 1, 1, 1, 1, 1, -1, -1, 1, 1, 1, -1])
  // Arms are deliberately +1: no trained arm policy means no frame to reconcile to, so they
  // are drawn raw and any disagreement with the hardware stays visible.
  const arms = bundle.limbs.left_arm.concat(bundle.limbs.right_arm)
  for (const n of arms) {
    assert.equal(bundle.joint_sign[bundle.joint_order.indexOf(n)], 1, `${n} sign is +1`)
  }
})

test('left arm FK matches independently derived URDF values', () => {
  // Goldens FK'd from humanoid.urdf directly in Python with numpy, not captured from this
  // implementation — otherwise the test would only prove the code agrees with itself.
  const arm = selectModel(bundle, bundle.limbs.left_arm)
  const zero = forwardKinematics(arm, [0, 0, 0, 0, 0])
  closeVec(zero.joints.left_shoulder_pitch_joint, [0, 0.133, 0.76403], TOL, 'shoulder_pitch')
  closeVec(zero.joints.left_shoulder_roll_joint, [0.0255, 0.176, 0.76403], TOL, 'shoulder_roll')
  closeVec(zero.joints.left_shoulder_yaw_joint, [0, 0.20783, 0.64522], TOL, 'shoulder_yaw')
  closeVec(zero.joints.left_elbow_pitch_joint, [0, 0.1953, 0.59734], TOL, 'elbow_pitch')
  closeVec(zero.joints.left_wrist_yaw_joint, [0, 0.25176, 0.48513], TOL, 'wrist_yaw')

  // Bending the elbow 90 deg must swing the wrist forward (+X) and up — if the axis or the
  // parent transform were wrong this is where it would show.
  const bent = forwardKinematics(arm, [0, 0, 0, Math.PI / 2, 0])
  closeVec(bent.joints.left_wrist_yaw_joint, [0.123, 0.21993, 0.60394], TOL, 'wrist_yaw @ elbow 90')
})

test('an arm mounts to the side of the torso, not the centreline', () => {
  // Running the shoulder strut to the centreline draws the arm as if it grew out of the
  // sternum, and the bright skeleton line then makes the shoulder look centred on the torso.
  const shoulder = bundle.joints.find((j) => j.name === 'left_shoulder_pitch_joint')
  assert.equal(shoulder.mount.source, 'torso_surface')
  assert.ok(shoulder.mount.xyz[1] > 0, 'mount stays on the arm\'s own side')
  assert.ok(shoulder.mount.xyz[1] < shoulder.xyz[1], 'mount is inboard of the joint')

  // The hips keep the centreline mount: the two struts together read as a pelvis.
  const hip = bundle.joints.find((j) => j.name === 'left_hip_roll_joint')
  assert.equal(hip.mount.source, 'centreline')
  assert.equal(hip.mount.xyz[1], 0)

  // And the strut must actually start there.
  const arm = selectModel(bundle, bundle.limbs.left_arm)
  const bones = buildDrawables(arm, forwardKinematics(arm, [0, 0, 0, 0, 0])).bones
  const clavicle = bones.find((b) => b.role === 'clavicle')
  closeVec(clavicle.a, shoulder.mount.xyz, 1e-9, 'clavicle starts at the torso wall')
})

test('measured dimensions override the URDF cosmetic geometry', () => {
  // The URDF torso is 10 cm narrower than the machine that got built. Drawn that way, the
  // shoulder floats clear of the body on a long strut and reads as mounted on the sternum.
  const torso = bundle.links.find((l) => l.name === 'base').shapes.find((s) => s.role === 'torso')
  assert.equal(torso.source, 'measured')
  close(torso.size[0], 0.25, 1e-9, 'torso width')
  close(torso.size[1], 0.14, 1e-9, 'torso depth')      // this one agreed with the URDF
  close(torso.size[2], 0.25, 1e-9, 'torso height')
  assert.deepEqual(torso.urdf_size, [0.15, 0.14, 0.23], 'what the URDF claimed is kept on record')

  // With the real width the shoulder sits just outboard of the corner, not on a long spur.
  const shoulder = bundle.joints.find((j) => j.name === 'left_shoulder_pitch_joint')
  const strut = shoulder.xyz[1] - shoulder.mount.xyz[1]
  assert.ok(strut > 0 && strut < 0.02, `clavicle strut should be ~1cm, got ${(strut * 100).toFixed(1)}cm`)

  // The claw is a real part, so its reach is measured, not chosen.
  const hand = bundle.joints.find((j) => j.name === 'left_wrist_yaw_joint').hand
  assert.equal(hand.source, 'measured')
  close(hand.length_closed, 0.12, 1e-9, 'hand length')
  const reach = hand.palm[0] * hand.axis[0] + hand.palm[1] * hand.axis[1] + hand.palm[2] * hand.axis[2]
  close(reach + hand.jaw_length, 0.12, 1e-6, 'palm reach + jaw length must equal the measured hand')
})

test('inline-twist joints are detected geometrically and their spurs sweep', () => {
  // shoulder_yaw's axis runs straight down the limb, so it reorients the arm without moving
  // it: the drawing needs a spur or the joint looks broken when it is merely axial.
  const twisters = bundle.joints.filter((j) => j.twist).map((j) => j.name)
  assert.deepEqual(twisters, ['left_shoulder_yaw_joint', 'right_shoulder_yaw_joint'])

  const arm = selectModel(bundle, bundle.limbs.left_arm)
  const ticksAt = (yaw) => buildTwistTicks(arm, forwardKinematics(arm, [0, 0, yaw, 0, 0]))
  const a = ticksAt(0)
  const b = ticksAt(Math.PI / 2)
  assert.equal(a.length, 2, 'two spurs, so orientation survives any viewing angle')
  const moved = Math.hypot(...a[0].b.map((v, i) => v - b[0].b[i]))
  assert.ok(moved > 0.02, `spur must sweep with the joint, moved ${moved}`)
  // ...while the joint origin itself stays put, which is the whole point.
  closeVec(a[0].a, b[0].a, 1e-9, 'twist joint origin is fixed')
})

test('the claw plane sweeps with the wrist and opens', () => {
  const arm = selectModel(bundle, bundle.limbs.left_arm)
  const grip = (wrist, open) =>
    buildGripper(arm, forwardKinematics(arm, [0, 0, 0, 0, wrist]), open)[0]

  const g = grip(0, 0.35)
  assert.equal(g.jaws.length, 2)

  // Rotating the wrist must swing the jaws — this is what makes the inline twist legible,
  // and it must move MORE than the hand centroid does (which sits nearly on the axis).
  const j0 = grip(0, 0.35).jaws[0].b
  const j90 = grip(Math.PI / 2, 0.35).jaws[0].b
  const jawSwing = Math.hypot(...j0.map((v, i) => v - j90[i]))
  const palmSwing = Math.hypot(...grip(0, 0.35).palm.map((v, i) => v - grip(Math.PI / 2, 0.35).palm[i]))
  assert.ok(jawSwing > 0.03, `jaw tip should swing clearly, got ${jawSwing}`)
  assert.ok(jawSwing > palmSwing * 2, 'jaws must read better than the near-axial palm point')

  // Opening splays the jaws apart; closing brings them together.
  const spread = (open) => {
    const x = grip(0, open)
    return Math.hypot(...x.jaws[0].b.map((v, i) => v - x.jaws[1].b[i]))
  }
  assert.ok(spread(1) > spread(0.35), 'open splays wider than half-open')
  assert.ok(spread(0.35) > spread(0), 'half-open splays wider than closed')

  // Every point finite in any configuration.
  for (const open of [0, 0.5, 1]) {
    for (const jw of grip(1.0, open).jaws) {
      ;[...jw.a, ...jw.b].forEach((v) => assert.ok(Number.isFinite(v)))
    }
  }
})

test('the arm ends in a hand that swings with the wrist', () => {
  const arm = selectModel(bundle, bundle.limbs.left_arm)
  const wrist = arm.joints.find((j) => j.name === 'left_wrist_yaw_joint')
  assert.ok(wrist.tip, 'wrist carries a terminal stub, else its rotation draws nothing')
  assert.equal(wrist.tip.source, 'urdf_hand_com')

  const at = (q) => {
    const bones = buildDrawables(arm, forwardKinematics(arm, [0, 0, 0, 0, q])).bones
    return bones.find((b) => b.role === 'hand').b
  }
  const a = at(0)
  const b = at(Math.PI / 2)
  const swing = Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])
  // The hand sits nearly ON the wrist axis, so the swing is real but small (~1.4 cm radius).
  // Asserting the magnitude keeps the UI's "read the wrist from the number" caveat honest.
  assert.ok(swing > 0.005 && swing < 0.05, `hand swing should be small but non-zero, got ${swing}`)
})

test('default pose FK is mirror-symmetric with both feet level', () => {
  const fk = forwardKinematics(model, model.poses.default)
  const l = fk.joints.left_ankle_roll_joint
  const r = fk.joints.right_ankle_roll_joint

  closeVec(l, [-0.0343, 0.0895, 0.1199], TOL, 'left ankle_roll')
  closeVec(r, [-0.0343, -0.0895, 0.1199], TOL, 'right ankle_roll')

  // The symmetry assertion is the real test: y must cancel, x and z must agree.
  close(l[1] + r[1], 0, 1e-6, 'ankle_roll y mirror')
  close(l[0] - r[0], 0, 1e-6, 'ankle_roll x match')
  close(l[2] - r[2], 0, 1e-6, 'ankle_roll z match')

  close(lowestZ(buildDrawables(model, fk)), 0.0666, 2e-3, 'sole height')
})

test('zero pose FK gives straight legs', () => {
  const fk = forwardKinematics(model, model.poses.zero)
  closeVec(fk.joints.left_ankle_roll_joint, [0.02, 0.057, 0.09], TOL, 'left ankle_roll')
  closeVec(fk.joints.right_ankle_roll_joint, [0.02, -0.057, 0.09], TOL, 'right ankle_roll')
  // Straight-legged stands taller than the crouched default pose.
  close(lowestZ(buildDrawables(model, fk)), 0.04, 2e-3, 'sole height')
})

test('REGRESSION: skipping the device->URDF flip breaks right-leg symmetry', () => {
  // This is the bug the visualizer exists to catch. Feed device-frame angles straight into
  // URDF FK and the right foot swings ~6.5 cm inboard, nearly crossing the centreline.
  const devicePose = model.poses.default.map((q, i) => q * model.joint_sign[i])
  const wrong = forwardKinematics(model, devicePose)
  const l = wrong.joints.left_ankle_roll_joint
  const r = wrong.joints.right_ankle_roll_joint

  close(r[1], -0.0248, 5e-3, 'unflipped right ankle_roll y')
  assert.ok(
    Math.abs(l[1] + r[1]) > 0.05,
    `unflipped pose must break symmetry, got |Ly+Ry| = ${Math.abs(l[1] + r[1])}`,
  )

  // ...and applying the flip repairs it, confirming deviceToUrdf is the inverse.
  const fixed = forwardKinematics(model, deviceToUrdf(devicePose, model.joint_sign))
  close(
    fixed.joints.left_ankle_roll_joint[1] + fixed.joints.right_ankle_roll_joint[1],
    0, 1e-6, 'flipped pose symmetry',
  )
})

test('deviceToUrdf is its own inverse and tolerates missing joints', () => {
  const pose = [0.1, -0.2, 0.3, 0.4, -0.5, 0.6, 0.7, -0.8, 0.9, 1.0, -1.1, 1.2]
  const round = deviceToUrdf(deviceToUrdf(pose, model.joint_sign), model.joint_sign)
  closeVec(round, pose, 1e-12, 'round trip')
  // Offline joints arrive as undefined/NaN; they must pass through, not become 0 silently.
  const partial = deviceToUrdf([NaN, ...pose.slice(1)], model.joint_sign)
  assert.ok(Number.isNaN(partial[0]))
})

test('projection is handed so the robot left appears on the viewer right at azimuth 0', () => {
  const fk = forwardKinematics(model, model.poses.default)
  const [lu] = project(fk.joints.left_ankle_roll_joint, 0)
  const [ru] = project(fk.joints.right_ankle_roll_joint, 0)
  assert.ok(lu > ru, `robot-left should project right of robot-right (got ${lu} vs ${ru})`)

  // At 90 degrees we are looking from the robot's left, so the two feet nearly coincide.
  const [lu90] = project(fk.joints.left_ankle_roll_joint, Math.PI / 2)
  const [ru90] = project(fk.joints.right_ankle_roll_joint, Math.PI / 2)
  close(lu90, ru90, 1e-6, 'sagittal view collapses the two legs')
})

test('FK tolerates a pose with missing angles', () => {
  const partial = new Array(12).fill(NaN)
  const fk = forwardKinematics(model, partial)
  // Missing angles fall back to 0 rather than propagating NaN into the geometry.
  for (const p of Object.values(fk.joints)) {
    p.forEach((v) => assert.ok(Number.isFinite(v), 'joint origin must stay finite'))
  }
})

test('drawables cover the torso, both feet and the limb chain', () => {
  const fk = forwardKinematics(model, model.poses.default)
  const { boxes, bones } = buildDrawables(model, fk)
  assert.ok(boxes.some((b) => b.role === 'torso'), 'torso box present')
  assert.equal(boxes.filter((b) => b.role === 'foot').length, 2, 'two foot pads')
  assert.equal(bones.filter((b) => b.role === 'thigh').length, 2, 'two thighs')
  assert.equal(bones.filter((b) => b.role === 'shin').length, 2, 'two shins')
  assert.equal(bones.filter((b) => b.role === 'pelvis').length, 2, 'two pelvis struts')
  assert.equal(bones.filter((b) => b.side === 'left').length,
               bones.filter((b) => b.side === 'right').length, 'sides balanced')

  // Every bone must be connected and thick: no NaN endpoints, no zero-width strokes.
  for (const b of bones) {
    ;[...b.a, ...b.b].forEach((v) => assert.ok(Number.isFinite(v), 'bone endpoint finite'))
    assert.ok(b.radius > 0, `bone ${b.joint} has a positive radius`)
  }
  for (const b of boxes) {
    for (const [p, q] of b.edges) {
      ;[...p, ...q].forEach((v) => assert.ok(Number.isFinite(v)))
    }
  }
})

test('limb thickness comes from the URDF collision radii, not invented constants', () => {
  // thigh cylinder r=0.05, shin r=0.04 in humanoid_biped.urdf.
  close(linkRadius(model, 'leg_left_hip_pitch'), 0.05, 1e-9, 'thigh radius')
  close(linkRadius(model, 'leg_left_knee_pitch'), 0.04, 1e-9, 'shin radius')
})

// ─── IMU base attitude ───────────────────────────────────────────────────────

const DOWN = [0, 0, -1]

test('shortestArcMatrix carries any gravity vector onto world-down', () => {
  const cases = [
    [0, 0, -1],            // upright
    [0.325, 0.010, -0.946], // the live bench reading: 19 deg forward lean
    [0.7071, 0, -0.7071],  // 45 deg forward
    [0, 0.5, -0.866],      // rolled left
    [1, 0, 0],             // face-down, 90 deg
    [0, 0, 1],             // fully inverted — the degenerate antiparallel case
    [-0.4, 0.3, 0.866],    // past horizontal
  ]
  for (const g of cases) {
    const R = shortestArcMatrix(g, DOWN)
    const out = applyDir(R, g.map((v) => v / Math.hypot(...g)))
    closeVec(out, DOWN, 1e-9, `R*${JSON.stringify(g)}`)
    // Must be a proper rotation: orthonormal columns, det +1.
    const ex = applyDir(R, [1, 0, 0]), ey = applyDir(R, [0, 1, 0]), ez = applyDir(R, [0, 0, 1])
    close(Math.hypot(...ex), 1, 1e-9, 'unit x')
    close(ex[0] * ey[0] + ex[1] * ey[1] + ex[2] * ey[2], 0, 1e-9, 'x.y orthogonal')
    const det = ex[0] * (ey[1] * ez[2] - ey[2] * ez[1])
              - ex[1] * (ey[0] * ez[2] - ey[2] * ez[0])
              + ex[2] * (ey[0] * ez[1] - ey[1] * ez[0])
    close(det, 1, 1e-9, 'det = +1 (rotation, not reflection)')
  }
})

test('the live 19-degree lean reproduces exactly, with zero induced yaw', () => {
  // Captured from the bench: GET /api/status -> base.projected_gravity, robot squatting.
  const R = baseRotationFromGravity([0.32475513219833374, 0.010175496339797974, -0.9455271679908037])
  const up = applyDir(R, [0, 0, 1])
  close(Math.atan2(up[0], up[2]) * 180 / Math.PI, 19.0, 0.1, 'forward lean degrees')
  // Yaw is the thing shortest-arc must NOT introduce: robot-forward stays in the XZ plane.
  const fwd = applyDir(R, [1, 0, 0])
  close(Math.atan2(fwd[1], fwd[0]) * 180 / Math.PI, 0, 0.2, 'induced yaw')
  close(tiltAngle([0.32475513219833374, 0.010175496339797974, -0.9455271679908037]) * 180 / Math.PI,
        19.0, 0.1, 'tiltAngle')
})

test('baseRotationFromGravity falls back to upright on a bad or missing IMU', () => {
  for (const bad of [null, undefined, [], [1, 2], [NaN, 0, -1], [0, 0, 0],
                     [0, 0, -0.5], [0, 0, -3]]) {
    assert.deepEqual(baseRotationFromGravity(bad), IDENTITY4,
      `expected upright fallback for ${JSON.stringify(bad)}`)
  }
  // A unit vector IS accepted (sanity: the guard isn't rejecting everything).
  assert.notDeepEqual(baseRotationFromGravity([0.5, 0, -0.866]), IDENTITY4)
})

test('a rotated root moves every joint but preserves the skeleton rigidly', () => {
  const pose = model.poses.default
  const upright = forwardKinematics(model, pose)
  const R = baseRotationFromGravity([0.325, 0.010, -0.946])
  const tilted = forwardKinematics(model, pose, R)

  const names = model.joint_order
  // Every joint origin actually moved (the root transform reached the whole tree)...
  for (const n of names) {
    const a = upright.joints[n], b = tilted.joints[n]
    assert.ok(Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) > 1e-4,
      `${n} should move when the base tilts`)
  }
  // ...and it is a rigid motion: all pairwise distances are unchanged. This catches a
  // transposed, scaled or non-orthonormal root matrix, which would silently deform the robot.
  for (let i = 0; i < names.length; i++) {
    for (let k = i + 1; k < names.length; k++) {
      const d = (fk, p, q) => {
        const a = fk.joints[p], b = fk.joints[q]
        return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])
      }
      close(d(tilted, names[i], names[k]), d(upright, names[i], names[k]), 1e-9,
            `${names[i]}..${names[k]} distance`)
    }
  }
  // The tilted body must lean forward: the torso top moves +X relative to upright.
  const topU = applyPoint(upright.links[model.root_link], [0, 0, 0.825])
  const topT = applyPoint(tilted.links[model.root_link], [0, 0, 0.825])
  assert.ok(topT[0] > topU[0] + 0.2, 'torso top should lean forward')
})

test('the pelvis strut tilts with the body', () => {
  // Regression: the pelvis centreline point is authored in base coordinates, so it has to go
  // through the root transform. If it does not, the pelvis stays level while the legs tilt.
  const R = baseRotationFromGravity([0.7071, 0, -0.7071])   // 45 deg forward
  const flat = buildDrawables(model, forwardKinematics(model, model.poses.default))
  const tilt = buildDrawables(model, forwardKinematics(model, model.poses.default, R))
  const pf = flat.bones.find((b) => b.role === 'pelvis')
  const pt = tilt.bones.find((b) => b.role === 'pelvis')
  assert.ok(Math.hypot(pf.a[0] - pt.a[0], pf.a[2] - pt.a[2]) > 1e-3,
    'pelvis root end must move with the base')
  // Both ends move by the same rigid transform, so the strut keeps its length.
  const len = (b) => Math.hypot(b.a[0] - b.b[0], b.a[1] - b.b[1], b.a[2] - b.b[2])
  close(len(pt), len(pf), 1e-9, 'pelvis strut length')
})

// ─── drawables stay sane across poses, attitudes and camera angles ───────────

test('every bone and box edge is finite and non-degenerate in any configuration', () => {
  const LIVE = [0.2300, -0.0421, 1.1160, 2.4376, -0.8227, 0.1503,
                0.1013, -0.1767, 1.0237, 2.4337, -0.7988, 0.0382]
  const attitudes = [[0, 0, -1], [0.325, 0.010, -0.946], [1, 0, 0], [0, 0, 1]]
  // Includes the zero pose, where consecutive leg segments are exactly collinear — the case
  // most likely to produce a zero-length bone.
  const poses = [LIVE, model.poses.zero.map((q, i) => q * model.joint_sign[i]), model.poses.default]

  for (const g of attitudes) {
    for (const pose of poses) {
      const fk = forwardKinematics(model, deviceToUrdf(pose, model.joint_sign),
                                   baseRotationFromGravity(g))
      const d = buildDrawables(model, fk)
      for (const b of d.bones) {
        ;[...b.a, ...b.b].forEach((v) => assert.ok(Number.isFinite(v), 'bone endpoint finite'))
        assert.ok(b.radius > 0, `bone ${b.joint} needs a positive width`)
        // The skeleton overlay is read for angles, so a bone must have real direction.
        const len = Math.hypot(b.a[0] - b.b[0], b.a[1] - b.b[1], b.a[2] - b.b[2])
        assert.ok(len > 1e-4, `bone ${b.joint} has ~zero length (${len})`)
      }
      for (const box of d.boxes) {
        for (const [p, q] of box.edges) {
          ;[...p, ...q].forEach((v) => assert.ok(Number.isFinite(v), 'box corner finite'))
        }
      }
      // Projection must stay finite at every camera angle too.
      for (const az of [0, Math.PI / 2, 1.2, -2.4]) {
        for (const b of d.bones) {
          project(b.a, az).forEach((v) => assert.ok(Number.isFinite(v), 'projected finite'))
        }
      }
    }
  }
})
