import { useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'

// Shared connect/disconnect + arm/disarm bar (used by the Manual page).
// Connect: motors DISABLED→IDLE (clears a latched E-STOP). Disconnect: motors → DISABLED.
export default function SessionBar({ deadmanConnected }) {
  const t = useTelemetry()
  const [busy, setBusy] = useState(null)
  const [error, setError] = useState(null)

  const motion = t.state === 'HOLDING' || t.state === 'RUNNING'
  const isConnected = t.state === 'CONNECTED'
  const canConnect = !motion && t.config_present && t.state !== 'CONNECTED'
  const deadmanWarn = (t.armed || motion) && !(deadmanConnected && t.deadman_ok)

  async function run(name, fn) {
    setBusy(name); setError(null)
    try { await fn() } catch (e) { setError(e.message) } finally { setBusy(null) }
  }

  return (
    <div className="space-y-2">
      <div className="flex items-center gap-2 flex-wrap">
        <button className="btn-primary text-xs" disabled={!!busy || !canConnect}
          onClick={() => run('connect', () => api.connect())}>
          {busy === 'connect' ? 'Connecting…' : (t.state === 'ESTOPPED' ? 'Connect (clear E-STOP)' : 'Connect')}
        </button>
        <button className="btn-ghost text-xs" disabled={!!busy || motion || t.state === 'DISCONNECTED'}
          onClick={() => run('disconnect', () => api.disconnect())} title="Set motors to DISABLED">
          Disconnect
        </button>
        {!t.armed ? (
          <button className="btn-success text-xs"
            disabled={!!busy || t.state !== 'CONNECTED' || t.estop || !t.all_calibrated}
            onClick={() => run('arm', () => api.arm())}>I am present — Arm</button>
        ) : (
          <button className="btn-danger text-xs" disabled={!!busy || motion}
            onClick={() => run('disarm', () => api.disarm())}>Disarm</button>
        )}
        <span className={`text-xs ${t.armed ? 'text-online' : 'text-gray-500'}`}>{t.armed ? 'ARMED' : 'not armed'}</span>
        <span className="text-xs text-gray-500 ml-auto">state: <span className="text-gray-300">{t.state}</span></span>
      </div>

      {isConnected && !t.all_calibrated && (
        <div className="text-xs text-warn bg-warn/10 border border-warn/30 rounded-lg px-3 py-2">
          ⚠ Calibrate all joints (Calibration tab) before arming.
        </div>
      )}
      {deadmanWarn && (
        <div className="text-xs text-warn bg-warn/10 border border-warn/30 rounded-lg px-3 py-2">
          ⚠ No live deadman — keep this page open and focused; motion auto-E-STOPs if the heartbeat drops.
        </div>
      )}
      {(error || t.last_error) && (
        <div className="text-xs text-danger bg-danger/10 border border-danger/30 rounded-lg px-3 py-2">{error || t.last_error}</div>
      )}
    </div>
  )
}
