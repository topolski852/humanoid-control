import { useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'

// T-pose zeroing for the arms.
//
// The arms have no hardstops, so the per-joint capture flow next to this does not apply to them
// — there is nothing to drive against. Instead the operator holds a T-pose and one press solves
// every joint's offset at once.
//
// This has to be redone after EVERY power cycle and flashing does not help: the encoders are
// single-turn, so behind 15:1 gearing the multi-turn reference is lost on power-down no matter
// what offset is stored. That is why a freshly powered arm reads nonsense while sitting
// physically relaxed.

const LIMB_LABEL = { left_arm: 'Left Arm', right_arm: 'Right Arm' }

export default function ArmCalibration() {
  const t = useTelemetry()
  const [busy, setBusy] = useState(null)
  const [result, setResult] = useState(null)
  const [error, setError] = useState(null)

  const arms = t.control?.arms || []
  const connected = t.state === 'CONNECTED'
  const motion = ['ARMED', 'HOLDING', 'RUNNING'].includes(t.state)

  if (arms.length === 0) return null      // nothing to calibrate on a legs-only machine

  async function calibrate(limb) {
    setBusy(limb); setError(null); setResult(null)
    try {
      const d = await api.calibrateArm(limb)
      setResult(d.cal)
    } catch (e) {
      setError(e.message)
    } finally {
      setBusy(null)
    }
  }

  return (
    <div className="card p-4 space-y-3">
      <div className="flex items-center justify-between">
        <span className="data-label">Arm zeroing (T-pose)</span>
        <span className="text-[10px] text-gray-600">no hardstops — held pose instead</span>
      </div>

      <div className="text-xs text-gray-400 leading-relaxed">
        Hold the arm in a <b>T-pose</b> — straight out to the side, horizontal, elbow straight,
        forearm untwisted — then press its button. Use a level edge to judge horizontal; the more
        square the hold, the better the zero.
      </div>

      <div className="flex items-center gap-2 flex-wrap">
        {arms.map((limb) => (
          <button key={limb} className="btn-primary text-xs"
            disabled={!!busy || !connected || motion}
            onClick={() => calibrate(limb)}
            title={!connected ? 'Connect first' : `Zero ${LIMB_LABEL[limb] || limb} from the held T-pose`}>
            {busy === limb ? 'Hold still…' : `Calibrate ${LIMB_LABEL[limb] || limb}`}
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
          result.ok ? 'border-online/30 bg-online/5' : 'border-danger/30 bg-danger/10'}`}>
          <div className="flex items-center justify-between text-xs">
            <span className={result.ok ? 'text-online' : 'text-danger'}>
              {result.ok ? '✓ zeroed' : '✗ did not complete'} — {LIMB_LABEL[result.limb] || result.limb}
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
                <th className="font-medium pl-2">source</th>
              </tr>
            </thead>
            <tbody className="font-mono tabular-nums">
              {result.joints.map((j) => (
                <tr key={j.joint} className={j.ok ? '' : 'text-danger'}>
                  <td className="font-sans text-gray-300">
                    {j.joint.replace(/^(left|right)_/, '').replace(/_joint$/, '')}
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
                  {/* A "declared" joint is an inline twist: a straight arm gives no geometric
                      constraint on rotation about the arm, so zero DEFINES untwisted rather
                      than measuring it. Worth showing so it is not mistaken for a measurement. */}
                  <td className="pl-2 font-sans text-[10px] text-gray-600">
                    {j.declared ? 'declared' : 'measured'}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>

          {result.shaky && (
            <div className="text-[10px] text-warn">
              The hold moved more than 2° while sampling — redo it if the arm looks off.
            </div>
          )}
          <div className="text-[10px] text-gray-600">
            Check it: let the arm hang relaxed. shoulder_pitch and elbow should read about 0°,
            shoulder_roll about −15…−8°. That pose was not used to calibrate, so agreement is
            real evidence.
          </div>
        </div>
      )}
    </div>
  )
}
