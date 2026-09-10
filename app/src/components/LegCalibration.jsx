import { useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'

// Whole-leg zeroing from one held stance.
//
// The per-joint card next to this needs every joint driven by hand to BOTH of its hardstops —
// twelve joints, twenty-four stops, repeated after every power cycle. Since a single ESC
// crashing forces a power cycle of the whole robot, that session gets repeated a lot. The
// folded stance below already parks four joints per leg against a mechanical stop, so one
// press zeroes everything that is on the bus.
//
// Accuracy is the operator's stance accuracy: hardstops are worth about a degree, the two
// declared joints however square the feet are. Use the per-joint flow when a joint needs to be
// better than that.

const LIMB_LABEL = { left_leg: 'Left Leg', right_leg: 'Right Leg' }

// How each joint's target was arrived at — worth showing so a declared angle is never mistaken
// for a measured one.
const SOURCE_LABEL = {
  measured: 'hardstop',
  declared: 'declared',
  mirrored: 'mirrored',
}

const shortJoint = (j) => j.replace(/^(left|right)_/, '').replace(/_joint$/, '')
const side = (j) => (j.startsWith('left_') ? 'L' : 'R')

export default function LegCalibration() {
  const t = useTelemetry()
  const [busy, setBusy] = useState(null)
  const [result, setResult] = useState(null)
  const [error, setError] = useState(null)

  const legs = (t.layout?.enabled || []).filter((l) => l.endsWith('_leg'))
  const connected = t.state === 'CONNECTED'
  const motion = ['ARMED', 'HOLDING', 'RUNNING'].includes(t.state)

  if (legs.length === 0) return null      // nothing to calibrate on an arms-only machine

  async function calibrate(limb) {
    setBusy(limb); setError(null); setResult(null)
    try {
      const d = await api.calibrateLegs(limb)
      setResult(d.cal)
    } catch (e) {
      setError(e.message)
    } finally {
      setBusy(null)
    }
  }

  const targets = legs.length > 1 ? ['both', ...legs] : legs

  return (
    <div className="card p-4 space-y-3">
      <div className="flex items-center justify-between">
        <span className="data-label">Leg zeroing (folded stance)</span>
        <span className="text-[10px] text-gray-600">one stance — every joint at once</span>
      </div>

      <div className="text-xs text-gray-400 leading-relaxed">
        Fold the robot into the reference stance, then press. Four joints per leg must be resting
        against their <b>mechanical hardstop</b> — hips folded fully back, knees straight, both
        ankle axes pushed fully onto their stops (pitch and roll load <i>opposite</i> ends, so
        push each until it stops rather than expecting them to match) — with the <b>feet touching</b> and pointing
        straight ahead. Push it into the stance so the stops are actually loaded; a joint hovering
        near its stop zeroes to wherever it is hovering.
      </div>

      <div className="flex items-center gap-2 flex-wrap">
        {targets.map((limb) => (
          <button key={limb}
            className={limb === 'both' ? 'btn-primary text-xs' : 'btn text-xs'}
            disabled={!!busy || !connected || motion}
            onClick={() => calibrate(limb)}
            title={!connected ? 'Connect first' : `Zero ${LIMB_LABEL[limb] || 'both legs'} from the held stance`}>
            {busy === limb ? 'Hold still…' : `Calibrate ${LIMB_LABEL[limb] || 'Both Legs'}`}
          </button>
        ))}
        {!connected && <span className="text-xs text-warn">connect first</span>}
        {motion && <span className="text-xs text-warn">stop the active session first</span>}
      </div>

      {error && (
        <div className="text-xs text-danger bg-danger/10 border border-danger/30 rounded-lg px-3 py-2">
          {error}
        </div>
      )}

      {result && (
        <div className={`rounded-lg border px-3 py-2 space-y-1.5 ${
          result.ok ? 'border-online/30 bg-online/5' : 'border-warn/30 bg-warn/5'}`}>
          <div className="flex items-center justify-between text-xs">
            <span className={result.ok ? 'text-online' : 'text-warn'}>
              {result.ok ? '✓ zeroed' : '⚠ partly zeroed'} — {result.zeroed}/{result.total} joints
            </span>
            <span className={`text-[10px] ${result.shaky ? 'text-warn' : 'text-gray-500'}`}>
              held steady to {result.steady_deg}°{result.shaky ? ' — shaky' : ''}
            </span>
          </div>

          <table className="w-full text-[11px]">
            <thead>
              <tr className="text-left text-gray-500">
                <th className="font-medium">joint</th>
                <th className="font-medium text-right">was</th>
                <th className="font-medium text-right">shift</th>
                <th className="font-medium text-right">now</th>
                <th className="font-medium pl-2">target from</th>
              </tr>
            </thead>
            <tbody className="font-mono tabular-nums">
              {result.joints.map((j) => (
                <tr key={j.joint} className={j.ok ? '' : 'text-danger'}>
                  <td className="font-sans text-gray-300">
                    <span className="text-gray-600">{side(j.joint)}</span> {shortJoint(j.joint)}
                  </td>
                  <td className="text-right text-gray-500">
                    {j.was_deg == null ? '—' : j.was_deg.toFixed(1)}
                  </td>
                  <td className="text-right text-gray-400">
                    {j.shift_deg == null ? '—' : `${j.shift_deg > 0 ? '+' : ''}${j.shift_deg.toFixed(1)}`}
                  </td>
                  <td className="text-right text-gray-200">
                    {j.now_deg == null ? (j.reason || '—') : j.now_deg.toFixed(1)}
                  </td>
                  <td className="pl-2 font-sans text-[10px] text-gray-600">
                    {j.source == null ? '—'
                      : j.source === 'mirrored' && j.mirrored_from
                        ? `mirrors ${side(j.mirrored_from)}`
                        : SOURCE_LABEL[j.source] || j.source}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>

          {result.shaky && (
            <div className="text-[10px] text-warn">
              The stance moved more than 2° while sampling — redo it if a joint looks off.
            </div>
          )}
          {/* Three different kinds of target, and they are not equally trustworthy. */}
          <div className="text-[10px] text-gray-600 leading-relaxed">
            <b>hardstop</b> is measured — the joint is resting on a physical stop.{' '}
            <b>declared</b> is hip_roll / hip_yaw, which have no stop here: feet touching and
            pointing forward <i>defines</i> their zero rather than measuring it.{' '}
            <b>mirrors L/R</b> means the target was copied from the already-calibrated twin,
            which is how a joint whose ESC is down gets re-zeroed once it is back on the bus.
          </div>
          {/* A joint zeroed against the WRONG end of its travel reads exactly like one zeroed
              correctly against the right end — no static reading can tell them apart. Only
              motion can, so the check has to be stated rather than computed. */}
          <div className="text-[10px] text-warn/80 leading-relaxed">
            Check it once by hand: lift a foot off its stop and watch the numbers. They should
            move the same way the joint does, and by the same amount. A number that moves the
            wrong way is a gear sign, not an offset — recalibrating will not fix it.
          </div>
        </div>
      )}
    </div>
  )
}
