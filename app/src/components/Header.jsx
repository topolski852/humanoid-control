import { useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'
import StatusDot from './StatusDot'

// Persistent top bar: live status pills + the always-available big red E-STOP.
export default function Header({ deadmanConnected }) {
  const t = useTelemetry()
  const [busy, setBusy] = useState(false)

  const [clearing, setClearing] = useState(false)

  async function estop() {
    setBusy(true)
    try { await api.estop() } catch (e) { console.warn('estop', e) } finally { setBusy(false) }
  }

  async function clearFaults() {
    setClearing(true)
    try { await api.clearFaults() } catch (e) { console.warn('clearFaults', e) } finally { setClearing(false) }
  }

  const faulted = t.estop || (t.joints || []).some((j) => j.error)

  const daemonTone = t.daemon_alive ? 'online' : 'danger'
  const wsTone = t.wsConnected ? 'online' : 'danger'
  const deadmanTone = deadmanConnected && t.deadman_ok ? 'online' : (t.armed || t.state === 'HOLDING' || t.state === 'RUNNING' ? 'danger' : 'warn')

  return (
    <header className="flex items-center justify-between gap-4 px-5 py-3 border-b border-surface-3 bg-surface-1">
      <div className="flex items-center gap-3">
        <span className="text-lg">🤖</span>
        <div>
          <div className="font-semibold text-white leading-tight">Humanoid Control</div>
          <div className="data-label">Berkeley Humanoid Lite · legs</div>
        </div>
      </div>

      <div className="flex items-center gap-5">
        <StatusDot tone={wsTone} label="server" />
        <StatusDot tone={daemonTone} label="daemon" />
        <StatusDot tone={deadmanTone} label="deadman" />
        {t.gamepad?.enabled && (
          <StatusDot
            tone={t.gamepad.connected ? 'online' : 'danger'}
            label="🎮 gamepad"
            value={t.gamepad.connected ? (t.gamepad.run_gate ? 'RUN' : 'ready') : 'none'}
          />
        )}
        <StatusDot tone={t.estop ? 'danger' : 'offline'} label="state" value={t.state} />
      </div>

      <div className="flex items-center gap-2">
        {faulted && (
          <button
            onClick={clearFaults} disabled={clearing}
            className="px-4 py-3 rounded-xl font-semibold text-white bg-warn/90 hover:bg-warn
                       active:scale-95 transition disabled:opacity-60 text-sm"
            title="Clear firmware errors on all joints and release a latched E-STOP (recover without reconnect)"
          >
            {clearing ? 'Clearing…' : '⟳ Clear faults'}
          </button>
        )}
        <button
          onClick={estop} disabled={busy}
          className="px-6 py-3 rounded-xl font-bold text-white bg-danger hover:bg-red-600
                     shadow-lg shadow-danger/30 active:scale-95 transition disabled:opacity-60
                     text-base tracking-wide"
          title="Emergency stop — set all joints to IDLE (priority port 9002)"
        >
          ⛔ E-STOP
        </button>
      </div>
    </header>
  )
}
