import { useMemo, useState } from 'react'
import { useTelemetry } from '../context/TelemetryContext'
import { useContract } from '../viz/useContract'
import { useDevicePose, mergeTarget } from '../viz/usePose'
import { baseRotationFromGravity, tiltAngle, IDENTITY4 } from '../viz/kinematics'
import { RAD2DEG } from '../format'
import RobotWireframe, { azimuthLabel } from './RobotWireframe'

// The Robot tab: the wireframe plus the controls and the numeric readout that turn "that
// looks wrong" into "joint N is off by M degrees".
//
// What this view is FOR: the angles drawn here are read from the same daemon telemetry cache
// the policy's observation reads (ControlService.telemetry_snapshot and LegInterface.read_states
// both call get_cached_joint_state). So if this picture and the physical robot disagree, the
// policy is being fed a robot that doesn't exist — a calibration offset or a sign is wrong.

const PRESETS = [
  ['Front', 0],
  ['3/4', 45],
  ['Left', 90],
  ['Right', -90],
]

function Toggle({ on, onClick, children, title }) {
  return (
    <button
      onClick={onClick}
      title={title}
      className={`px-2.5 py-1 rounded-md text-xs border transition ${
        on ? 'bg-accent/20 text-accent border-accent/40'
           : 'border-surface-3 text-gray-500 hover:text-gray-300'
      }`}
    >
      {children}
    </button>
  )
}

function JointRows({ model, joints, pose, target, contract }) {
  return (
    <div className="overflow-x-auto">
      <table className="w-full text-xs">
        <thead>
          <tr className="text-left border-b border-surface-3">
            <th className="py-1.5 pr-2 data-label font-medium">Joint</th>
            <th className="py-1.5 px-2 data-label font-medium text-right">Device°</th>
            <th className="py-1.5 px-2 data-label font-medium text-right">Sim°</th>
            <th className="py-1.5 px-2 data-label font-medium text-right">Target°</th>
            <th className="py-1.5 px-2 data-label font-medium text-right">Δ default°</th>
            <th className="py-1.5 pl-2 data-label font-medium">Range</th>
          </tr>
        </thead>
        <tbody className="font-mono tabular-nums">
          {model.joint_order.map((name, i) => {
            const j = joints[i]
            const sign = contract.policy_frame_sign[i]
            const device = j?.position
            const known = typeof device === 'number' && Number.isFinite(device)
            const dflt = contract.default_pose[i]
            const lo = contract.limits.lower[i]
            const hi = contract.limits.upper[i]
            const p = known ? device : pose[i]
            const out = known && (p < lo - 1e-4 || p > hi + 1e-4)
            const near = known && !out && Math.min(p - lo, hi - p) < 0.035   // within ~2°
            return (
              <tr key={name} className="border-b border-surface-3/40">
                <td className="py-1 pr-2 font-sans text-gray-400">
                  {name.replace(/_joint$/, '')}
                  {!j?.online && <span className="text-gray-600"> ·offline</span>}
                  {j?.online && !j?.calibrated && <span className="text-warn"> ⚠</span>}
                </td>
                <td className={`py-1 px-2 text-right ${
                  !known ? 'text-gray-600' : out ? 'text-danger' : near ? 'text-warn' : 'text-gray-200'
                }`}>
                  {known ? (device * RAD2DEG).toFixed(1) : '—'}
                </td>
                <td className={`py-1 px-2 text-right ${known ? 'text-gray-400' : 'text-gray-600'}`}>
                  {known ? (device * sign * RAD2DEG).toFixed(1) : '—'}
                  {sign < 0 && <span className="text-gray-600" title="sign-flipped vs device frame"> ⇄</span>}
                </td>
                <td className="py-1 px-2 text-right text-accent/80">
                  {target[i] != null ? (target[i] * RAD2DEG).toFixed(1) : '—'}
                </td>
                <td className={`py-1 px-2 text-right ${known ? 'text-gray-400' : 'text-gray-600'}`}>
                  {known ? ((device - dflt) * RAD2DEG).toFixed(1) : '—'}
                </td>
                <td className="py-1 pl-2 text-gray-600 text-[10px]">
                  {(lo * RAD2DEG).toFixed(0)}…{(hi * RAD2DEG).toFixed(0)}
                </td>
              </tr>
            )
          })}
        </tbody>
      </table>
    </div>
  )
}

export default function RobotView() {
  const t = useTelemetry()
  const { model, contract, error, loading } = useContract()

  const [azDeg, setAzDeg] = useState(45)
  const [showDefault, setShowDefault] = useState(true)
  const [showTarget, setShowTarget] = useState(true)
  const [showLimits, setShowLimits] = useState(false)
  const [useImu, setUseImu] = useState(true)
  const [paused, setPaused] = useState(false)

  const { pose, target, missing, hasTarget } = useDevicePose(t.joints, { paused })

  // Base attitude from gravity. `base` is null whenever the IMU is disabled, stale (>100 ms)
  // or the whole telemetry stream is stale, so upright is a normal fallback, not an error.
  const pg = t.base?.projected_gravity
  const imuLive = Array.isArray(pg) && pg.length >= 3 && pg.every(Number.isFinite)
  const baseRotation = useMemo(
    () => (useImu && imuLive ? baseRotationFromGravity(pg) : IDENTITY4),
    [useImu, imuLive, pg?.[0], pg?.[1], pg?.[2]],
  )
  const tiltDeg = imuLive ? tiltAngle(pg) * RAD2DEG : 0
  const leanDir = imuLive
    ? (Math.abs(pg[0]) >= Math.abs(pg[1])
        ? (pg[0] > 0 ? 'fwd' : 'back')
        : (pg[1] > 0 ? 'left' : 'right'))
    : ''

  // Joints reporting outside their own contract limits. This is the loudest evidence that the
  // code's model of the robot is self-inconsistent, so it gets a banner rather than only a
  // red cell in the table.
  const outOfLimit = useMemo(() => {
    if (!contract) return []
    return contract.joint_order
      .map((name, i) => {
        const p = t.joints[i]?.position
        if (typeof p !== 'number' || !Number.isFinite(p)) return null
        const lo = contract.limits.lower[i]
        const hi = contract.limits.upper[i]
        if (p < lo - 1e-4) return { name, by: lo - p }
        if (p > hi + 1e-4) return { name, by: p - hi }
        return null
      })
      .filter(Boolean)
  }, [contract, t.joints])

  const ghosts = useMemo(() => {
    if (!contract) return []
    const out = []
    if (showDefault) {
      out.push({ pose: contract.default_pose, color: '#4b5563', label: 'default_pose' })
    }
    if (showTarget && hasTarget) {
      out.push({ pose: mergeTarget(target, pose), color: '#3b82f6', label: 'commanded target' })
    }
    return out
  }, [contract, showDefault, showTarget, hasTarget, target, pose])

  if (loading) {
    return <div className="card p-8 text-center text-sm text-gray-500">Loading robot model…</div>
  }

  // A frame-mismatch is exactly the failure this tool exists to catch, so it refuses to draw
  // rather than showing a confident, wrong robot.
  if (error) {
    return (
      <div className="card p-5 border-danger/40">
        <div className="data-label text-danger mb-2">Visualizer disabled</div>
        <p className="text-sm text-gray-300 mb-2">
          Refusing to draw: the robot model and the server disagree about the joint frame, so any
          pose shown here would be untrustworthy.
        </p>
        <pre className="text-xs text-gray-500 whitespace-pre-wrap font-mono">{error}</pre>
      </div>
    )
  }

  const az = (azDeg * Math.PI) / 180

  return (
    <div className="max-w-6xl mx-auto grid grid-cols-1 lg:grid-cols-5 gap-5">
      <div className="lg:col-span-2 space-y-4">
        <div className="card p-4">
          <div className="flex items-center justify-between mb-2">
            <span className="data-label">Robot pose · live encoders</span>
            <span className={`text-[10px] font-medium ${
              paused ? 'text-warn' : t.wsConnected ? 'text-online' : 'text-offline'
            }`}>
              {paused ? '❚❚ paused' : t.wsConnected ? '● live' : '○ no link'}
            </span>
          </div>

          <div className="flex items-center justify-between mb-2 text-[10px] font-mono">
            <span className={imuLive && useImu ? 'text-online' : 'text-offline'}>
              {!imuLive
                ? '○ no imu — drawn upright'
                : useImu
                  ? `● imu ${tiltDeg.toFixed(1)}° ${leanDir}`
                  : `○ imu ignored (${tiltDeg.toFixed(1)}° ${leanDir})`}
            </span>
            <span className="text-gray-700">base pinned · gravity fixed</span>
          </div>

          <div className="flex justify-center">
            <RobotWireframe
              model={model}
              sign={contract.policy_frame_sign}
              devicePose={pose}
              baseRotation={baseRotation}
              joints={t.joints}
              ghosts={ghosts}
              azimuth={az}
              showLimits={showLimits}
              className="w-full max-w-[330px]"
            />
          </div>

          <div className="mt-3">
            <div className="flex items-center gap-2 mb-2">
              <span className="data-label w-14">View</span>
              <input
                type="range" min={-180} max={180} step={1} value={azDeg}
                onChange={(e) => setAzDeg(Number(e.target.value))}
                className="flex-1 accent-accent"
                aria-label="Camera azimuth"
              />
              <span className="font-mono text-xs text-gray-400 w-24 text-right">
                {azimuthLabel(az)}
              </span>
            </div>
            <div className="flex flex-wrap gap-1.5">
              {PRESETS.map(([label, deg]) => (
                <Toggle key={label} on={azDeg === deg} onClick={() => setAzDeg(deg)}>
                  {label}
                </Toggle>
              ))}
              <span className="flex-1" />
              <Toggle on={paused} onClick={() => setPaused((v) => !v)}
                      title="Freeze the drawing (telemetry keeps flowing)">
                {paused ? 'Resume' : 'Pause'}
              </Toggle>
            </div>
          </div>

          <div className="flex flex-wrap gap-1.5 mt-3 pt-3 border-t border-surface-3">
            <Toggle on={showDefault} onClick={() => setShowDefault((v) => !v)}
                    title="Ghost of the contract default_pose (the sim stand pose)">
              default ghost
            </Toggle>
            <Toggle on={showTarget} onClick={() => setShowTarget((v) => !v)}
                    title="Ghost of the last position commanded to each joint">
              target ghost{!hasTarget && <span className="text-gray-600"> ·idle</span>}
            </Toggle>
            <Toggle on={showLimits} onClick={() => setShowLimits((v) => !v)}
                    title="Sweep arc between each joint's position limits">
              limit arcs
            </Toggle>
            <Toggle on={useImu} onClick={() => setUseImu((v) => !v)}
                    title="Orient the body by the IMU (gravity). Off draws the base upright.">
              imu tilt
            </Toggle>
          </div>
        </div>

        {outOfLimit.length > 0 && (
          <div className="card p-4 border-danger/40">
            <div className="data-label text-danger mb-2">
              {outOfLimit.length} joint{outOfLimit.length > 1 ? 's' : ''} outside its own limits
            </div>
            <p className="text-xs text-gray-400 mb-2">
              The encoders report angles the contract says are unreachable, so the pose drawn
              above is impossible as well. Either the calibration offset or the frame convention
              for these joints is wrong.
            </p>
            <div className="space-y-0.5">
              {outOfLimit.map((o) => (
                <div key={o.name} className="flex justify-between text-xs font-mono">
                  <span className="text-gray-300">{o.name.replace(/_joint$/, '')}</span>
                  <span className="text-danger">
                    +{(o.by * RAD2DEG).toFixed(1)}° past the stop
                  </span>
                </div>
              ))}
            </div>
          </div>
        )}

        <div className="card p-4 text-xs text-gray-500 leading-relaxed">
          <div className="data-label mb-2">What this is</div>
          <p>
            This is <span className="text-gray-300">what the code believes the robot looks
            like</span> — the same encoder values the policy observation reads, through the same
            contract and the same URDF. Angles are drawn exactly as reported: never clamped to
            limits, never sign-corrected to look plausible.
          </p>
          <p className="mt-2">
            <span className="text-gray-300">Divergence from the real robot is the finding</span>,
            not a rendering bug. If a commanded pose draws correctly here but the physical robot
            sits differently, the fault is downstream — calibration, the URDF, or the hardware.
          </p>
          <p className="mt-2">
            Frame: <span className="text-gray-300">sim (URDF)</span> = device telemetry ×
            <span className="font-mono text-gray-300"> policy_frame_sign</span>;
            <span className="font-mono text-gray-400"> right_hip_roll</span>,
            <span className="font-mono text-gray-400"> right_hip_yaw</span> and
            <span className="font-mono text-gray-400"> right_ankle_roll</span> are sign-flipped
            (marked <span className="font-mono">⇄</span> in the table). Body attitude comes from
            the IMU's <span className="font-mono text-gray-400">projected_gravity</span> — tilt
            only; gravity cannot observe yaw. The body is not stood on a floor, so a robot on its
            back correctly shows its feet in the air.
          </p>
          {missing.length > 0 && (
            <p className="mt-2 text-warn">
              Holding last-known angle for {missing.length} offline joint
              {missing.length > 1 ? 's' : ''}: {missing.map((n) => n.replace(/_joint$/, '')).join(', ')}
            </p>
          )}
        </div>
      </div>

      <div className="lg:col-span-3">
        <div className="card p-4">
          <div className="flex items-center justify-between mb-3">
            <span className="data-label">Joint angles</span>
            <span className="text-[10px] text-gray-600 font-mono">
              device = raw encoder · sim = after frame flip
            </span>
          </div>
          <JointRows model={model} joints={t.joints} pose={pose} target={target} contract={contract} />
        </div>
      </div>
    </div>
  )
}
