import { useTelemetry } from '../context/TelemetryContext'
import { deg, num } from '../format'

// One joint, rendered two ways depending on where it lives.
//
//   standalone  a square read-out — name small, position in large type. Meant to be read from
//               across the room, which is what a dashboard card is for.
//   row         a table row with the full columns, so a GroupCard full of these reproduces the
//               original Joints table.
//
// Both live here rather than in two components because they are the same data with different
// emphasis, and splitting them would mean two places to fix when telemetry changes shape.

export function jointStateTone(j) {
  if (!j?.online) return 'text-offline'
  if (j.error) return 'text-danger'
  if (j.state === 'ENABLED') return 'text-online'
  return 'text-gray-300'
}

/** Short label: left_shoulder_pitch_joint -> shoulder_pitch (left) */
export function shortJointName(name = '') {
  const side = name.startsWith('left_') ? 'L' : name.startsWith('right_') ? 'R' : ''
  const core = name.replace(/^(left|right)_/, '').replace(/_joint$/, '')
  return { core, side }
}

/** The columns a row shows. `columns` comes from the parent group's settings. */
export const ROW_COLUMNS = {
  full: ['state', 'cal', 'pos', 'vel', 'torque', 'err'],
  compact: ['state', 'pos', 'vel'],
  minimal: ['pos'],
}

export function JointRow({ joint, columns = 'full', onPopOut, editing }) {
  const t = useTelemetry()
  const j = t.joints.find((x) => x.name === joint)
  const cols = ROW_COLUMNS[columns] || ROW_COLUMNS.full
  const { core, side } = shortJointName(joint)

  if (!j) {
    return (
      <div className="flex items-center gap-2 px-3 py-1.5 text-xs text-gray-600">
        <span className="flex-1 truncate">{core || joint}</span>
        <span className="italic">not in telemetry</span>
        {editing && onPopOut && <PopOut onClick={onPopOut} />}
      </div>
    )
  }

  return (
    <div className="flex items-center gap-2 px-3 py-1.5 text-xs border-b border-surface-3/40
                    last:border-b-0 hover:bg-surface-2/40">
      <span className="flex-1 min-w-0 truncate text-gray-300">
        {core}{side && <span className="text-gray-600 ml-1">{side}</span>}
      </span>
      {cols.includes('state') && (
        <span className={`w-16 font-mono ${jointStateTone(j)}`}>
          {j.online ? (j.state || 'ON') : 'OFFLINE'}
        </span>
      )}
      {cols.includes('cal') && (
        <span className="w-5 text-center">
          {j.calibrated ? <span className="text-online" title="calibrated">✓</span>
                        : <span className="text-warn" title="uncalibrated">⚠</span>}
        </span>
      )}
      {cols.includes('pos') && <span className="w-16 text-right data-value">{deg(j.position)}°</span>}
      {cols.includes('vel') && <span className="w-16 text-right data-value">{deg(j.velocity)}</span>}
      {cols.includes('torque') && <span className="w-14 text-right data-value">{num(j.torque)}</span>}
      {cols.includes('err') && (
        <span className={`w-12 text-right font-mono ${j.error ? 'text-danger' : 'text-gray-600'}`}>
          {j.error ? `0x${Number(j.error).toString(16)}` : '—'}
        </span>
      )}
      {editing && onPopOut && <PopOut onClick={onPopOut} />}
    </div>
  )
}

function PopOut({ onClick }) {
  return (
    <button onClick={onClick} title="Move out of this group, onto the grid"
      className="text-[10px] px-1 rounded text-accent/70 hover:text-accent hover:bg-accent/10">
      ↗
    </button>
  )
}

/** Standalone: the big-number card. */
export default function JointCard({ props = {} }) {
  const t = useTelemetry()
  const name = props.joint
  const j = t.joints.find((x) => x.name === name)
  const { core, side } = shortJointName(name || '')

  if (!name) {
    return (
      <div className="card h-full p-3 flex flex-col items-center justify-center text-center gap-1">
        <span className="text-xs text-gray-500">No joint selected</span>
        <span className="text-[10px] text-gray-600">Pick one in this card's settings (⚙)</span>
      </div>
    )
  }

  // Out-of-limit is worth shouting about even on a card showing nothing else: an angle the
  // contract says is unreachable means the calibration or the frame is wrong.
  const p = j?.position
  const known = typeof p === 'number' && Number.isFinite(p)
  const lim = j?.limit
  const out = known && lim && (p < lim.min - 1e-4 || p > lim.max + 1e-4)

  return (
    <div className="card h-full p-3 flex flex-col">
      <div className="flex items-baseline justify-between gap-2 min-w-0">
        <span className="data-label truncate">{core}{side && ` ${side}`}</span>
        <span className={`text-[10px] font-mono ${jointStateTone(j)}`}>
          {!j ? '—' : j.online ? (j.state || 'ON') : 'OFFLINE'}
        </span>
      </div>
      <div className="flex-1 flex items-center justify-center min-h-0">
        <span className={`font-mono tabular-nums leading-none
                          text-[clamp(1.5rem,22cqw,5rem)] ${
          !known ? 'text-gray-600' : out ? 'text-danger' : 'text-gray-100'}`}>
          {known ? deg(p) : '—'}
          <span className="text-[0.4em] text-gray-500 ml-0.5">°</span>
        </span>
      </div>
      {out && <div className="text-[10px] text-danger text-center">outside its limits</div>}
      {j?.error ? (
        <div className="text-[10px] text-danger text-center">
          error 0x{Number(j.error).toString(16)}
        </div>
      ) : null}
    </div>
  )
}
