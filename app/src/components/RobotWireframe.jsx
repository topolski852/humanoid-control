import { useMemo } from 'react'
import {
  forwardKinematics, deviceToUrdf, project, depth, buildDrawables, arcPoints,
  buildTwistTicks, buildGripper, applyPoint, IDENTITY4,
} from '../viz/kinematics'

// 2D wireframe of the robot, posed from live encoder telemetry and oriented by the IMU.
//
// Presentational only: it takes a device-frame pose plus a base attitude and draws them. All
// frame conversion and geometry lives in ../viz/kinematics.js; the control wiring lives in
// RobotView.jsx.
//
// This is a MIRROR OF THE CODE'S MODEL, not of the robot. Angles are drawn exactly as the
// contract reports them — never clamped to limits, never sign-corrected to look plausible.
// When the drawing and the physical robot disagree, that gap is the finding.
//
// GRAVITY IS THE ANCHOR. The base origin is pinned and the body rotates about it, so the
// drawing is NOT re-grounded to stand the feet on a floor. If the robot is lying on the
// bench, its feet point at the sky — which is the honest picture.

// World is metres. The scale is FIXED (not fit-to-content) so the robot visibly rises, sinks
// and leans instead of being silently re-normalised into the same box every frame.
const VIEW_W = 320
const VIEW_H = 460
const PX_PER_M = 300
const ORIGIN_X = VIEW_W / 2
// Where the body anchor is pinned. Not centred: on the full robot it reaches ~0.28 m up to the
// torso top but ~0.48 m down to the soles, so sitting it above centre balances the frame.
const ORIGIN_Y = Math.round(VIEW_H * 0.45)
const MARGIN = 14                            // keep drawn content this far inside the frame

// The body pivots about where the limbs meet the torso, NOT the URDF base datum. The datum sits
// near ground level, so anchoring there swings the torso through a 0.8 m arc whenever the robot
// tilts. Derived from the model rather than hardcoded so an arm-only layout pivots about the
// shoulder instead of about a pelvis that isn't being drawn.
function bodyAnchor(model) {
  const roots = model.joints.filter((j) => j.parent === model.root_link)
  if (!roots.length) return [0, 0, 0]
  const sum = roots.reduce(
    (acc, j) => [acc[0] + j.xyz[0], acc[1], acc[2] + j.xyz[2]],
    [0, 0, 0],
  )
  return [sum[0] / roots.length, 0, sum[2] / roots.length]   // y=0: stay on the centreline
}

// Two layers, and the split is the point:
//   BODY     — thick soft strokes at the real link widths. Gives the shape of the robot, but
//              a fat stroke's centreline is hard to read an angle off.
//   SKELETON — a 2 px bright line straight between joint centres, drawn on top. THIS is what
//              you read joint angles from; the body is context around it.
const LIMB_SCALE = 0.75       // collision radii are physical hulls, generous as visual widths
const BODY_OPACITY = 0.5      // translucent so the skeleton inside stays legible
const SKELETON_W = 2

const COLORS = {
  left: '#e5e7eb',
  right: '#8b93a3',           // the two limbs differ in tone so they separate at a glance
  armLeft: '#d7c9a8',         // arms warmer than legs, so limb type reads at a glance too
  armRight: '#9c9079',
  torso: '#cbd2dd',
  dim: '#4b5563',
  skeleton: '#22ff88',        // bright green: never confusable with the body or the ghosts
  joint: '#3b82f6',
  jointWarn: '#f59e0b',
  jointBad: '#ef4444',
  jointOff: '#4b5563',
  arc: '#2d3148',
  arcHot: '#ef4444',
  twist: '#c084fc',           // inline-twist spurs: a distinct hue, they are not structure
  claw: '#e8b04b',            // the gripper — an indicator, so it reads as its own thing
  horizon: '#252836',
  gravity: '#3f4457',
}

const _ARM_ROLES = new Set(['clavicle', 'shoulder', 'upper_arm', 'forearm', 'wrist', 'hand'])

function limbColor(bone) {
  if (bone.role === 'pelvis') return COLORS.torso
  if (_ARM_ROLES.has(bone.role)) {
    return bone.side === 'left' ? COLORS.armLeft : COLORS.armRight
  }
  return bone.side === 'left' ? COLORS.left : COLORS.right
}

/** Health tone for a joint pivot. Mirrors the pill colours in LegDiagram. */
export function pivotColor(j) {
  if (!j || !j.online) return COLORS.jointOff
  if (j.error) return COLORS.jointBad
  if (!j.calibrated) return COLORS.jointWarn
  return COLORS.joint
}

/** Fraction of the way to the nearer limit: 0 at centre, 1 at a hard stop. */
function limitProximity(j) {
  if (!j || j.position == null || !j.limit) return 0
  const { min, max } = j.limit
  if (!(max > min)) return 0
  const margin = Math.min(j.position - min, max - j.position)
  const span = (max - min) / 2
  return Math.max(0, Math.min(1, 1 - margin / (span || 1)))
}

/** Thick soft limbs + the torso/foot boxes: the shape of the robot, drawn back-to-front. */
function Body({ drawables, toScreen, azimuth, faded }) {
  const bones = [...drawables.bones].sort(
    (a, b) => depth(a.a, azimuth) - depth(b.a, azimuth),
  )
  return (
    <g opacity={BODY_OPACITY} strokeLinecap="round" strokeLinejoin="round">
      {bones.map((b, i) => {
        const [x1, y1] = toScreen(b.a)
        const [x2, y2] = toScreen(b.b)
        return (
          <line key={`b${i}`} x1={x1} y1={y1} x2={x2} y2={y2}
                stroke={faded ? COLORS.dim : limbColor(b)}
                strokeWidth={Math.max(2, b.radius * 2 * PX_PER_M * LIMB_SCALE)} />
        )
      })}
      {drawables.boxes.map((box, i) => (
        <g key={`x${i}`}>
          {box.edges.map(([p, q], k) => {
            const [x1, y1] = toScreen(p)
            const [x2, y2] = toScreen(q)
            return <line key={k} x1={x1} y1={y1} x2={x2} y2={y2}
                         stroke={faded ? COLORS.dim : COLORS.torso} strokeWidth="1.4" />
          })}
        </g>
      ))}
    </g>
  )
}

/**
 * The angle-reading layer: a thin bright line from joint centre to joint centre, on top of
 * everything. A limb's thick stroke has no visible centreline, so you cannot judge its angle
 * from it; this can be read directly.
 */
function SkeletonOverlay({ drawables, toScreen }) {
  return (
    <g stroke={COLORS.skeleton} strokeWidth={SKELETON_W} strokeLinecap="round" fill="none">
      {drawables.bones.map((b, i) => {
        const [x1, y1] = toScreen(b.a)
        const [x2, y2] = toScreen(b.b)
        return <line key={i} x1={x1} y1={y1} x2={x2} y2={y2}
                     opacity={b.role === 'pelvis' ? 0.55 : 1} />
      })}
    </g>
  )
}

/** A ghost pose: thin dashed centreline only, so it never competes with the live skeleton. */
function Ghost({ drawables, toScreen, color }) {
  return (
    <g stroke={color} strokeWidth="1.2" strokeDasharray="4 4" strokeLinecap="round"
       fill="none" opacity="0.7">
      {drawables.bones.map((b, i) => {
        const [x1, y1] = toScreen(b.a)
        const [x2, y2] = toScreen(b.b)
        return <line key={i} x1={x1} y1={y1} x2={x2} y2={y2} />
      })}
    </g>
  )
}

export default function RobotWireframe({
  model,
  sign,
  devicePose,                 // (N,) device-frame radians, in model.joint_order; NaN/null where unknown
  baseRotation = IDENTITY4,   // base->world from the IMU; identity draws upright
  joints = [],                // telemetry joint objects, for pivot health + limits
  ghosts = [],                // [{pose (device frame), color, label}]
  azimuth = 0,
  showLimits = false,
  showPivots = true,
  showTwist = true,           // spurs on inline-twist joints (shoulder_yaw)
  gripperOpen = 0.35,         // claw splay, 0..1. Display only — there is no gripper actuator.
  compact = false,
  className = '',
}) {
  const live = useMemo(() => {
    const fk = forwardKinematics(model, deviceToUrdf(devicePose, sign), baseRotation)
    return {
      fk,
      drawables: buildDrawables(model, fk),
      twist: buildTwistTicks(model, fk),
      grippers: buildGripper(model, fk, gripperOpen),
    }
  }, [model, sign, devicePose, baseRotation, gripperOpen])

  const ghostFrames = useMemo(
    () => ghosts.map((g) => {
      const fk = forwardKinematics(model, deviceToUrdf(g.pose, sign), baseRotation)
      return { ...g, drawables: buildDrawables(model, fk) }
    }),
    [model, sign, ghosts, baseRotation],
  )

  // Pan (never zoom). A robot tilted past horizontal sticks out of the frame in both axes.
  // Shifting it back is information-free — gravity, not screen position, is the reference —
  // whereas rescaling would destroy the "rises, sinks and leans" signal the fixed scale
  // exists to give. Zero whenever the content already fits, so the common upright case stays
  // rock-steady.
  // Screen offset that puts the (rotated) pelvis on a fixed point. Recomputed from the live
  // base attitude, so the pelvis stays put and the body tips about it instead of sweeping.
  const anchor = useMemo(() => bodyAnchor(model), [model])

  const [baseX, baseY] = useMemo(() => {
    const [au, av] = project(applyPoint(baseRotation, anchor), azimuth)
    return [ORIGIN_X - au * PX_PER_M, ORIGIN_Y + av * PX_PER_M]
  }, [baseRotation, azimuth, anchor])

  const [panX, panY] = useMemo(() => {
    let left = Infinity, right = -Infinity, top = Infinity, bottom = -Infinity
    const see = (p) => {
      const [u, v] = project(p, azimuth)
      const x = baseX + u * PX_PER_M
      const y = baseY - v * PX_PER_M
      if (x < left) left = x
      if (x > right) right = x
      if (y < top) top = y
      if (y > bottom) bottom = y
    }
    for (const set of [live.drawables, ...ghostFrames.map((g) => g.drawables)]) {
      for (const b of set.bones) { see(b.a); see(b.b) }
      for (const bx of set.boxes) for (const [p, q] of bx.edges) { see(p); see(q) }
    }
    // The claw reaches past the last joint, so it has to be inside the fit or it gets clipped.
    for (const g of live.grippers) for (const jw of g.jaws) { see(jw.a); see(jw.b) }
    if (!Number.isFinite(top) || !Number.isFinite(left)) return [0, 0]
    // If the content is wider/taller than the band it cannot fit; centre it and accept the
    // overflow rather than jamming one edge in and letting the other run off unboundedly.
    const fit = (lo, hi, min, max) => {
      if (hi - lo > max - min) return (min + max - (lo + hi)) / 2
      if (lo < min) return min - lo
      if (hi > max) return max - hi
      return 0
    }
    // The body keeps clear of the level reference strip along the bottom.
    return [
      fit(left, right, MARGIN, VIEW_W - MARGIN),
      fit(top, bottom, MARGIN + 10, VIEW_H - 34),
    ]
  }, [live, ghostFrames, azimuth, baseX, baseY])

  const toScreen = useMemo(() => (p) => {
    const [u, v] = project(p, azimuth)
    return [baseX + u * PX_PER_M + panX, baseY - v * PX_PER_M + panY]
  }, [azimuth, baseX, baseY, panX, panY])

  const anyOnline = joints.some((j) => j.online && j.position != null)
  // TRUE LEVEL (perpendicular to gravity). Deliberately at a FIXED screen position rather
  // than through the base origin: it is a "which way is level" reference, not a floor and not
  // a body landmark. Pinning it to the base datum would drag it half a metre off-body in a
  // folded pose, or off-frame entirely once panning kicks in.
  const horizonY = VIEW_H - 26
  const pivotPt = toScreen(applyPoint(baseRotation, anchor))

  const arcs = useMemo(() => {
    if (!showLimits) return []
    return model.joints.map((j, i) => {
      const pts = arcPoints(model, live.fk, j, 0.055)
      const hot = limitProximity(joints[i]) > 0.97
      return {
        d: pts.map((p, k) => {
          const [x, y] = toScreen(p)
          return `${k ? 'L' : 'M'}${x.toFixed(1)} ${y.toFixed(1)}`
        }).join(' '),
        hot,
        key: j.name,
      }
    })
  }, [showLimits, model, live, joints, toScreen])

  return (
    <svg viewBox={`0 0 ${VIEW_W} ${VIEW_H}`} className={className} role="img"
         aria-label="Robot pose from live encoder values, oriented by the IMU">
      {/* level reference + which way is down, so a tilted body is never ambiguous */}
      <line x1="10" y1={horizonY} x2={VIEW_W - 10} y2={horizonY}
            stroke={COLORS.horizon} strokeWidth="1.5" strokeDasharray="6 5" />
      <text x={VIEW_W - 12} y={horizonY - 5} textAnchor="end" fontSize="8"
            fontFamily="monospace" className="fill-gray-700">LEVEL</text>
      <g stroke={COLORS.gravity} strokeWidth="1.2" fill="none">
        <line x1="20" y1={horizonY + 5} x2="20" y2={horizonY + 17} />
        <path d={`M16 ${horizonY + 12} L20 ${horizonY + 18} L24 ${horizonY + 12}`} />
      </g>
      <text x="32" y={horizonY + 16} fontSize="7"
            fontFamily="monospace" className="fill-gray-700">g</text>

      {arcs.map((a) => (
        <path key={a.key} d={a.d} fill="none" strokeWidth={a.hot ? 2 : 1}
              stroke={a.hot ? COLORS.arcHot : COLORS.arc} opacity={a.hot ? 0.9 : 0.7} />
      ))}

      {ghostFrames.map((g, i) => (
        <Ghost key={`g${i}`} drawables={g.drawables} toScreen={toScreen} color={g.color} />
      ))}

      <g opacity={anyOnline ? 1 : 0.4}>
        <Body drawables={live.drawables} toScreen={toScreen} azimuth={azimuth}
              faded={!anyOnline} />
        <SkeletonOverlay drawables={live.drawables} toScreen={toScreen} />

        {/* Inline-twist spurs. shoulder_yaw rotates the limb about its own centreline, which a
            centreline drawing cannot show — these sweep with it so the twist is readable. */}
        {showTwist && live.twist.map((tk, i) => {
          const [x1, y1] = toScreen(tk.a)
          const [x2, y2] = toScreen(tk.b)
          return (
            <line key={`tw${i}`} x1={x1} y1={y1} x2={x2} y2={y2}
                  stroke={COLORS.twist} strokeWidth="1.6" strokeLinecap="round" opacity="0.85" />
          )
        })}

        {/* The claw. Drawn, not derived — see buildGripper(). Its plane makes the wrist's
            inline rotation visible, and it is where a real gripper's state will show. */}
        {live.grippers.map((g) => {
          const seg = (p, q, w, key) => {
            const [x1, y1] = toScreen(p)
            const [x2, y2] = toScreen(q)
            return <line key={key} x1={x1} y1={y1} x2={x2} y2={y2}
                         stroke={COLORS.claw} strokeWidth={w} strokeLinecap="round" />
          }
          return (
            <g key={`cl${g.joint}`} opacity="0.95">
              {seg(g.knuckleBar[0], g.knuckleBar[1], 1.6, 'bar')}
              {g.jaws.map((jw, k) => seg(jw.a, jw.b, 2.2, `j${k}`))}
            </g>
          )
        })}
      </g>

      {showPivots && model.joints.map((j, i) => {
        const [x, y] = toScreen(live.fk.joints[j.name])
        return (
          <circle key={j.name} cx={x} cy={y} r={compact ? 2.6 : 3.6}
                  fill={pivotColor(joints[i])} stroke="#0f1117" strokeWidth="1" />
        )
      })}

      {/* where the limbs meet the torso — the point the body pivots about, held fixed on screen */}
      <circle cx={pivotPt[0]} cy={pivotPt[1]} r="2" fill="none"
              stroke={COLORS.gravity} strokeWidth="1" />

      {/* where the IMU actually sits: the tilt shown here is measured at THIS point, high on
          the frame, not at the base origin. Makes a mounting error easier to reason about. */}
      {model.imu && !compact && (() => {
        const [x, y] = toScreen(applyPoint(baseRotation, model.imu.xyz))
        return (
          <g>
            <rect x={x - 3} y={y - 3} width="6" height="6" rx="1"
                  fill="none" stroke={COLORS.gravity} strokeWidth="1.2" />
            <text x={x + 7} y={y + 3} fontSize="7" fontFamily="monospace"
                  className="fill-gray-700">imu</text>
          </g>
        )
      })()}

      {/* Orientation cue: which way the camera is looking. */}
      <text x={ORIGIN_X} y="14" textAnchor="middle" fontSize="9" fontFamily="monospace"
            className="fill-gray-600">
        {azimuthLabel(azimuth)}
      </text>
    </svg>
  )
}

export function azimuthLabel(azimuth) {
  const deg = Math.round((azimuth * 180) / Math.PI)
  const norm = ((deg % 360) + 360) % 360
  if (norm === 0) return 'FRONT'
  if (norm === 180) return 'BACK'
  if (norm === 90) return "ROBOT'S LEFT"
  if (norm === 270) return "ROBOT'S RIGHT"
  return `${deg}°`
}
