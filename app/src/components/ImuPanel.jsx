import { useTelemetry } from '../context/TelemetryContext'
import { RAD2DEG } from '../format'

// Live IMU monitor. Shows ONLY the two quantities the control policy actually consumes from
// the daemon `base` block (docs/DAEMON_SPEC.md §9): projected_gravity (tilt/lean) and
// angular_velocity (body rotation rate). The raw quaternion carries no extra info the policy
// uses, so it isn't drawn. `base: null` from the daemon means no fresh IMU → shown as offline.

const R = 80                 // circle radius in the 200×200 viewBox
const AV_FULLSCALE = 120     // °/s that fills an angular-velocity bar

// Base frame is right-handed: +X forward, +Y left, +Z up. projected_gravity is the gravity
// unit vector in that frame ([0,0,-1] upright); its horizontal part (gx,gy) is the lean and
// points in the topple direction. Angles follow the right-hand rule about each body axis.
function attitude(pg) {
  const [gx, gy, gz] = pg
  const horiz = Math.hypot(gx, gy)
  const tilt = Math.atan2(horiz, -gz) * RAD2DEG          // angle from vertical, 0…180°
  const pitch = Math.atan2(gx, -gz) * RAD2DEG            // RH about +Y: positive = nose down
  const roll = Math.atan2(-gy, -gz) * RAD2DEG            // RH about +X: positive = lean right
  const s = Math.min(horiz, 1) / (horiz || 1)            // unit-scale, clamped to the rim
  // Screen axes: up = forward (+X), left = robot-left (+Y); SVG y grows downward.
  return { tilt, pitch, roll, dx: -gy * s * R, dy: -gx * s * R }
}

function AngVelRow({ label, axis, rad }) {
  const dps = (rad || 0) * RAD2DEG
  const frac = Math.max(-1, Math.min(1, dps / AV_FULLSCALE))
  const moving = Math.abs(dps) > 1
  return (
    <div className="flex items-center gap-2 text-xs">
      <span className="data-label w-14">{label}·<span className="text-gray-600">{axis}</span></span>
      <div className="relative flex-1 h-2 rounded-full bg-surface-2 overflow-hidden">
        <div className="absolute left-1/2 top-0 h-full w-px bg-surface-3" />
        <div
          className={`absolute top-0 h-full ${moving ? 'bg-accent' : 'bg-surface-3'}`}
          style={{
            left: frac >= 0 ? '50%' : `${50 + frac * 50}%`,
            width: `${Math.abs(frac) * 50}%`,
          }}
        />
      </div>
      <span className={`font-mono tabular-nums w-16 text-right ${moving ? 'text-gray-200' : 'text-gray-500'}`}>
        {dps.toFixed(1)}<span className="text-gray-600">°/s</span>
      </span>
    </div>
  )
}

export default function ImuPanel() {
  const t = useTelemetry()
  const base = t.base
  const live = !!base && Array.isArray(base.projected_gravity)

  const pg = live ? base.projected_gravity : [0, 0, -1]
  const av = live ? (base.angular_velocity || [0, 0, 0]) : [0, 0, 0]
  const { tilt, pitch, roll, dx, dy } = attitude(pg)

  // Reference rings at 30° and 60° of tilt (horizontal comp = sin θ).
  const ring30 = Math.sin((30 * Math.PI) / 180) * R
  const ring60 = Math.sin((60 * Math.PI) / 180) * R
  const stroke = live ? '#2d3148' : '#252836'
  const dotColor = !live ? '#4b5563' : tilt > 25 ? '#ef4444' : tilt > 10 ? '#f59e0b' : '#3b82f6'

  return (
    <div className="card p-4">
      <div className="flex items-center justify-between mb-3">
        <span className="data-label">IMU · Base</span>
        <span className={`text-[10px] font-medium ${live ? 'text-online' : 'text-offline'}`}>
          {live ? '● live' : '○ no imu'}
        </span>
      </div>

      <div className="flex items-center justify-center">
        <svg viewBox="0 0 200 200" className="w-40 h-40">
          {/* tilt reference rings + crosshair */}
          <circle cx="100" cy="100" r={R} fill="none" stroke={stroke} strokeWidth="1.5" />
          <circle cx="100" cy="100" r={ring60} fill="none" stroke={stroke} strokeWidth="1" strokeDasharray="2 3" />
          <circle cx="100" cy="100" r={ring30} fill="none" stroke={stroke} strokeWidth="1" strokeDasharray="2 3" />
          <line x1="100" y1={100 - R} x2="100" y2={100 + R} stroke={stroke} strokeWidth="1" />
          <line x1={100 - R} y1="100" x2={100 + R} y2="100" stroke={stroke} strokeWidth="1" />
          {/* live lean vector + bubble */}
          {live && (dx || dy) ? (
            <line x1="100" y1="100" x2={100 + dx} y2={100 + dy} stroke={dotColor} strokeWidth="1.5" opacity="0.5" />
          ) : null}
          <circle cx={100 + dx} cy={100 + dy} r="8" fill={dotColor} className={live ? '' : 'opacity-60'} />
          {/* orientation: top = robot forward (+X), left = robot left (+Y) */}
          <text x="100" y={100 - R - 4} textAnchor="middle" className="fill-gray-500" fontSize="10" fontFamily="monospace">FWD</text>
          <text x="100" y="184" textAnchor="middle" className="fill-gray-600" fontSize="9" fontFamily="monospace">
            30° · 60° tilt
          </text>
        </svg>
      </div>

      <div className="grid grid-cols-3 gap-2 mt-1 mb-3 text-center">
        {[['Tilt', tilt], ['Pitch', pitch], ['Roll', roll]].map(([label, v]) => (
          <div key={label}>
            <div className="data-label">{label}</div>
            <div className={`font-mono tabular-nums text-sm ${live ? 'text-gray-200' : 'text-gray-600'}`}>
              {live ? `${v.toFixed(1)}°` : '—'}
            </div>
          </div>
        ))}
      </div>

      <div className="data-label mb-1.5">Angular velocity</div>
      <div className="space-y-1.5">
        <AngVelRow label="roll" axis="X" rad={live ? av[0] : null} />
        <AngVelRow label="pitch" axis="Y" rad={live ? av[1] : null} />
        <AngVelRow label="yaw" axis="Z" rad={live ? av[2] : null} />
      </div>
    </div>
  )
}
