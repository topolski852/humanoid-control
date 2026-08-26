import { useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'

// CONNECTION AND ARMING ONLY — the robot's lifecycle, not what is driving it.
//
// Choosing what drives the robot (Xbox / Quest / a policy) lives in the Control method card,
// and each method's own controls live with it. Splitting them keeps this card answerable at a
// glance: is the robot connected, is it armed, what state is it in.
//
// `Stop` deliberately stays HERE rather than moving with the motion controls. It is reachable
// from the card you are already looking at while the robot moves, and removing a way to stop a
// moving robot to satisfy a card boundary is not a trade worth making.
export default function ControlPanel({ deadmanConnected, size = 'full' }) {
  const t = useTelemetry()
  const [busy, setBusy] = useState(null)      // name of the in-flight action
  const [error, setError] = useState(null)

  const motion = t.state === 'HOLDING' || t.state === 'RUNNING'
  const isConnected = t.state === 'CONNECTED'
  // Connect is available from DISCONNECTED / ESTOPPED / ERROR (it clears a latched E-STOP and
  // wakes motors DISABLED→IDLE); only blocked while already connected or actively moving.
  // Allow (re)connect whenever not in a motion session — including from CONNECTED when joints
  // have dropped OFFLINE (an ESC brownout/reset), which is exactly when you must re-wake them.
  const jointsOffline = (t.joints || []).some((j) => j.online === false)
  const canConnect = !motion && t.config_present && (t.state !== 'CONNECTED' || jointsOffline)

  async function run(name, fn) {
    setBusy(name); setError(null)
    try { await fn() } catch (e) { setError(e.message) } finally { setBusy(null) }
  }

  const deadmanWarn = (t.armed || motion) && !(deadmanConnected && t.deadman_ok)
  const compact = size !== 'full'

  // MINI — a status read-out with no controls at all. For driving entirely from the gamepad,
  // where the screen is a glance rather than an input. Deliberately keeps E-STOP reachable:
  // the header's E-STOP is always on screen, so this card omitting buttons never removes the
  // ability to stop the robot.
  if (size === 'mini') {
    const tone = t.estop ? 'text-danger'
      : motion ? 'text-online'
      : t.armed ? 'text-warn'
      : isConnected ? 'text-gray-200' : 'text-gray-500'
    const label = t.estop ? 'E-STOP' : (t.state || 'DISCONNECTED')
    return (
      <div className="card p-3 h-full flex flex-col justify-center gap-2">
        <div className="flex items-baseline justify-between">
          <span className="data-label">State</span>
          <span className={`font-mono text-sm ${tone}`}>{label}</span>
        </div>
        <div className="grid grid-cols-3 gap-1.5 text-center">
          <Chip on={isConnected} label="conn" />
          <Chip on={t.armed} label="armed" tone="text-warn" />
          <Chip on={t.all_calibrated} label="cal" tone="text-online" />
        </div>
        {deadmanWarn && <div className="text-[10px] text-warn text-center">⚠ no deadman</div>}
        {(error || t.last_error) && (
          <div className="text-[10px] text-danger truncate" title={error || t.last_error}>
            {error || t.last_error}
          </div>
        )}
      </div>
    )
  }

  return (
    <div className={`card p-4 ${compact ? 'space-y-2' : 'space-y-4'}`}>
      <div className="flex items-center justify-between">
        <span className="data-label">Control</span>
        {!t.config_present && <span className="text-[10px] text-warn">no robot config loaded</span>}
      </div>

      {/* 1 · Connection — Connect: motors DISABLED→IDLE. Disconnect: motors → DISABLED. */}
      <Section step="1" title="Connection">
        <button className="btn-primary" disabled={busy || !canConnect}
          onClick={() => run('connect', () => api.connect())}
          title="Wake motors DISABLED→IDLE (clears a latched E-STOP)">
          {busy === 'connect' ? 'Connecting…' : (t.state === 'ESTOPPED' ? 'Connect (clear E-STOP)' : 'Connect')}
        </button>
        <button className="btn-ghost" disabled={busy || motion || t.state === 'DISCONNECTED'}
          onClick={() => run('disconnect', () => api.disconnect())}
          title="Set motors to DISABLED">Disconnect</button>
      </Section>

      {/* 2 · Arm — the "I am present / robot supported" gate */}
      <Section step="2" title="Arm">
        {!t.armed ? (
          <button className="btn-success" disabled={busy || t.state !== 'CONNECTED' || t.estop || !t.all_calibrated}
            onClick={() => run('arm', () => api.arm())}
            title="Confirm a human is present and the robot is supported/gantried">
            I am present — Arm
          </button>
        ) : (
          <button className="btn-danger" disabled={busy || motion}
            onClick={() => run('disarm', () => api.disarm())}>Disarm</button>
        )}
        <span className={`text-xs ${t.armed ? 'text-online' : 'text-gray-500'}`}>
          {t.armed ? 'ARMED' : 'not armed'}
        </span>
      </Section>
      {!compact && isConnected && !t.all_calibrated && (
        <div className="text-xs text-warn bg-warn/10 border border-warn/30 rounded-lg px-3 py-2">
          ⚠ Joints not calibrated — calibrate every joint (<b>Calibration</b> tab) to arm here and
          run policy / ramp-to-pose Hold. To just hold the current pose without calibrating, use the
          <b> Manual</b> tab (Capture &amp; hold).
        </div>
      )}

      {/* 3 · Stop — the one motion control that stays here. Choosing and starting a motion
          belongs to the Control method card; STOPPING one belongs wherever you are looking.
          Shown only while something is actually moving, so it is never a dead button. */}
      {motion && (
        <Section step="3" title="Stop">
          <button className="btn-danger" disabled={busy}
            onClick={() => run('stop', () => api.stop())}>
            {busy === 'stop' ? 'Stopping…' : 'Stop motion'}
          </button>
          <span className="text-[10px] text-gray-500">
            Graceful stop → IDLE. For an immediate cut, use E-STOP in the header.
          </span>
        </Section>
      )}

      {!compact && deadmanWarn && (
        <div className="text-xs text-warn bg-warn/10 border border-warn/30 rounded-lg px-3 py-2">
          ⚠ No live deadman connection. Keep this page open and focused — motion auto-E-STOPs if
          the heartbeat is lost.
        </div>
      )}
      {(error || t.last_error) && (
        <div className="text-xs text-danger bg-danger/10 border border-danger/30 rounded-lg px-3 py-2">
          {error || t.last_error}
        </div>
      )}
    </div>
  )
}

function Chip({ on, label, tone = 'text-gray-200' }) {
  return (
    <div className={`rounded-md border px-1 py-1 text-[10px] font-mono ${
      on ? `border-current/40 bg-current/10 ${tone}` : 'border-surface-3 text-gray-600'}`}>
      {label}
    </div>
  )
}

function Section({ step, title, children }) {
  return (
    <div className="flex items-center gap-3 flex-wrap">
      <span className="w-5 h-5 shrink-0 rounded-full bg-surface-2 text-[10px] flex items-center justify-center text-gray-400 font-mono">{step}</span>
      <span className="data-label w-20">{title}</span>
      {children}
    </div>
  )
}
