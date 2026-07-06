import { useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'
import { deg } from '../format'

// Per-joint position_offset calibration. Offsets are lost on power-down, so every joint
// must be calibrated after each power-up before the robot can be armed.
//
// Flow per joint (the joint is IDLE / zero-torque — you move it BY HAND, no commanded motion):
//   Start → move to LOWER hardstop → Capture lower → move to UPPER hardstop → Capture upper → Apply
export default function CalibrationPanel() {
  const t = useTelemetry()
  const [started, setStarted] = useState({})   // joint -> true once Start pressed
  const [busy, setBusy] = useState(null)        // "joint:action" in flight
  const [result, setResult] = useState({})      // joint -> apply result / message
  const [error, setError] = useState(null)

  const [connBusy, setConnBusy] = useState(null)
  const connected = t.state === 'CONNECTED'   // calibration requires an active connection
  const motion = t.state === 'HOLDING' || t.state === 'RUNNING'
  const calCount = t.joints.filter((j) => j.calibrated).length

  async function conn(action, fn) {
    setConnBusy(action); setError(null)
    try { await fn() } catch (e) { setError(e.message) } finally { setConnBusy(null) }
  }

  async function act(joint, action, fn) {
    setBusy(`${joint}:${action}`); setError(null)
    try { return await fn() }
    catch (e) { setError(`${joint.replace('_joint', '')}: ${e.message}`); throw e }
    finally { setBusy(null) }
  }

  const start = (j) => act(j, 'start', async () => {
    await api.calStart(j); setStarted((s) => ({ ...s, [j]: true }))
    setResult((r) => ({ ...r, [j]: null }))
  }).catch(() => {})

  const capture = (j, which) => act(j, which, () => api.calCapture(j, which)).catch(() => {})

  const apply = (j) => act(j, 'apply', async () => {
    const d = await api.calApply(j)
    setStarted((s) => ({ ...s, [j]: false }))
    setResult((r) => ({ ...r, [j]: d.cal }))
  }).catch(() => {})

  const cancel = (j) => act(j, 'reset', async () => {
    await api.calReset(j); setStarted((s) => ({ ...s, [j]: false }))
  }).catch(() => {})

  return (
    <div className="card p-4 space-y-4">
      <div className="flex items-center justify-between">
        <span className="data-label">Joint calibration (position offset)</span>
        <span className={`text-xs ${calCount === t.joints.length && t.joints.length ? 'text-online' : 'text-warn'}`}>
          {calCount}/{t.joints.length} calibrated
        </span>
      </div>

      {/* Connection (Connect: motors DISABLED→IDLE / clears E-STOP · Disconnect: → DISABLED) */}
      <div className="flex items-center gap-2">
        <button className="btn-primary text-xs"
          disabled={!!connBusy || connected || motion || !t.config_present}
          onClick={() => conn('connect', () => api.connect())}>
          {connBusy === 'connect' ? 'Connecting…' : (t.state === 'ESTOPPED' ? 'Connect (clear E-STOP)' : 'Connect')}
        </button>
        <button className="btn-ghost text-xs"
          disabled={!!connBusy || motion || t.state === 'DISCONNECTED'}
          onClick={() => conn('disconnect', () => api.disconnect())}
          title="Set motors to DISABLED">
          {connBusy === 'disconnect' ? 'Disconnecting…' : 'Disconnect'}
        </button>
        <span className="text-xs text-gray-500 ml-1">state: <span className="text-gray-300">{t.state}</span></span>
      </div>

      {!connected && (
        <div className="text-xs text-warn bg-warn/10 border border-warn/30 rounded-lg px-3 py-2">
          {t.state === 'ESTOPPED'
            ? 'E-STOP is latched — press Connect (Control tab) to clear it and bring joints to IDLE, then calibrate.'
            : 'Connect first (Control tab) to wake the joints to IDLE, then calibrate each one.'}
        </div>
      )}
      <div className="text-xs text-gray-400">
        Offsets reset on power-up. For each joint: <b>Start</b> → hand-move to the <b>lower</b> hardstop →
        <b> Capture lower</b> → move to the <b>upper</b> hardstop → <b>Capture upper</b> → <b>Apply</b>.
        The joint is zero-torque (movable by hand) during calibration.
      </div>

      <div className="space-y-1.5">
        {t.joints.map((j) => {
          const cap = j.cal_captured || {}
          const inProg = started[j.name]
          const res = result[j.name]
          const isBusy = (a) => busy === `${j.name}:${a}`
          const disabled = !connected || !j.online
          return (
            <div key={j.index} className="rounded-lg border border-surface-3 bg-surface-2/40 px-3 py-2">
              <div className="flex items-center justify-between gap-2">
                <div className="flex items-center gap-2 min-w-0">
                  <span className={j.calibrated ? 'text-online' : 'text-warn'}>{j.calibrated ? '✓' : '⚠'}</span>
                  <span className="text-sm text-gray-200 truncate">{j.name.replace(/_joint$/, '')}</span>
                  <span className="font-mono text-xs text-gray-500">{deg(j.position, 1)}°</span>
                </div>
                {!inProg ? (
                  <button className="btn-ghost text-xs" disabled={disabled || !!busy}
                    onClick={() => start(j.name)}>
                    {isBusy('start') ? '…' : (j.calibrated ? 'Recalibrate' : 'Start')}
                  </button>
                ) : (
                  <button className="btn-ghost text-xs" disabled={!!busy} onClick={() => cancel(j.name)}>Cancel</button>
                )}
              </div>

              {inProg && (
                <div className="mt-2 flex items-center gap-2 flex-wrap">
                  <CapBtn label="Capture lower" target={j.limit?.min} val={cap.lower}
                    busy={isBusy('lower')} disabled={!!busy} onClick={() => capture(j.name, 'lower')} />
                  <CapBtn label="Capture upper" target={j.limit?.max} val={cap.upper}
                    busy={isBusy('upper')} disabled={!!busy} onClick={() => capture(j.name, 'upper')} />
                  <button className="btn-success text-xs"
                    disabled={!!busy || cap.lower == null || cap.upper == null}
                    onClick={() => apply(j.name)}>{isBusy('apply') ? 'Applying…' : 'Apply'}</button>
                </div>
              )}

              {res && (
                <div className={`mt-1.5 text-[11px] ${res.range_ok ? 'text-online' : 'text-warn'}`}>
                  offset {deg(res.position_offset, 1)}° · range {deg(res.measured_range_rad, 1)}° vs {deg(res.expected_range_rad, 1)}°
                  {res.range_ok ? ' ✓' : ` ⚠ off by ${deg(res.range_error_rad, 1)}° — recheck hardstops`}
                </div>
              )}
            </div>
          )
        })}
        {t.joints.length === 0 && (
          <div className="text-center text-gray-600 text-xs py-6">No telemetry — is the daemon running?</div>
        )}
      </div>

      {error && (
        <div className="text-xs text-danger bg-danger/10 border border-danger/30 rounded-lg px-3 py-2">{error}</div>
      )}
    </div>
  )
}

function CapBtn({ label, target, val, busy, disabled, onClick }) {
  const done = val != null
  return (
    <button
      className={`text-xs px-2.5 py-1.5 rounded-lg border ${done
        ? 'border-online/40 bg-online/10 text-online'
        : 'border-surface-3 bg-surface-2 text-gray-300 hover:bg-surface-3'}`}
      disabled={disabled} onClick={onClick} title={target != null ? `stop ≈ ${deg(target, 0)}°` : ''}>
      {busy ? '…' : done ? `${label.split(' ')[1]} ✓ ${deg(val, 1)}°` : label}
      {target != null && !done && <span className="text-gray-500"> ({deg(target, 0)}°)</span>}
    </button>
  )
}
