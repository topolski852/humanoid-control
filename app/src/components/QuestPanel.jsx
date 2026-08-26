import { useTelemetry } from '../context/TelemetryContext'
import StatusDot from './StatusDot'

// Quest link health — is the headset talking, is it tracking, and is the arm about to move.
//
// Separate from the Xbox controller card on purpose. That card is a raw evdev button/axis dump
// whose value IS being device-specific; this one shows a network link's health. They share no
// data shape, and a "generic controller card" adapting between them would be a switch statement
// that made both worse.
//
// The numbers here mirror the backend's liveness ladder exactly, so what you read is what the
// robot is deciding on:
//   < 200 ms   driving normally
//   > 200 ms   run gate dropped, arm IDLE (recoverable by itself)
//   > 1.0 s    treated as controller loss -> E-STOP

const STALL_MS = 200
const LOSS_MS = 1000

export default function QuestPanel({ size = 'full' }) {
  const t = useTelemetry()
  const q = t.quest || {}

  if (!q.enabled) {
    return (
      <div className="card p-4 space-y-2">
        <span className="data-label">Quest</span>
        <p className="text-xs text-gray-500">
          Bridge disabled. Start the server with <span className="font-mono">HUMANOID_QUEST_ENABLE=1</span>.
        </p>
      </div>
    )
  }

  const age = q.age_ms
  // Colour by the SAME thresholds the backend acts on, so the UI never implies the link is fine
  // while the arm has already been stopped.
  const linkTone = !q.connected ? 'offline'
    : age == null ? 'warn'
      : age > LOSS_MS ? 'danger'
        : age > STALL_MS ? 'warn' : 'online'
  const linkText = !q.connected ? 'disconnected'
    : age == null ? 'no frames yet'
      : age > LOSS_MS ? `lost (${(age / 1000).toFixed(1)}s)`
        : age > STALL_MS ? `stalled (${Math.round(age)} ms)` : `${Math.round(age)} ms`

  const driving = q.gate && q.owns_input
  const trigger = q.trigger ?? 0

  // MINI — the one line worth glancing at mid-session: is the link alive, is the arm moving.
  if (size === 'mini') {
    return (
      <div className="card p-3 h-full flex flex-col justify-center gap-2">
        <StatusDot tone={linkTone} label="Quest" value={q.connected ? `${q.hz ?? 0} Hz` : 'off'} />
        <div className={`text-center font-mono text-sm ${
          driving ? 'text-online' : q.connected ? 'text-gray-500' : 'text-gray-600'}`}>
          {driving ? 'DRIVING' : q.anchored ? 'anchored' : 'idle'}
        </div>
      </div>
    )
  }

  const compact = size !== 'full'

  return (
    <div className="card p-4 space-y-3">
      <div className="flex items-center justify-between">
        <span className="data-label">Quest</span>
        <span className={`text-[10px] ${q.owns_input ? 'text-accent' : 'text-gray-500'}`}>
          {q.owns_input ? 'active control method'
            : `not active — ${t.input_source || 'web'} is driving`}
        </span>
      </div>

      <StatusDot tone={linkTone} label="link" value={linkText} />

      <div className="grid grid-cols-4 gap-2 py-2 border-y border-surface-3 text-center">
        <Chip label="rate" value={q.connected ? `${q.hz ?? 0} Hz` : '—'} />
        <Chip label="frame" value={q.seq ?? '—'} />
        <Chip label="tracking" value={q.tracked ? 'ok' : 'lost'}
              tone={q.tracked ? 'text-online' : 'text-warn'} />
        <Chip label="clutch" value={q.anchored ? 'anchored' : 'open'}
              tone={q.anchored ? 'text-accent' : 'text-gray-500'} />
      </div>

      {/* The single most useful readout on a bench: is the arm about to move. */}
      <div>
        <div className="flex items-baseline justify-between mb-1">
          <span className="data-label">trigger</span>
          <span className={`font-mono text-xs ${driving ? 'text-online' : 'text-gray-400'}`}>
            {driving ? 'DRIVING' : trigger > 0 ? trigger.toFixed(2) : 'released'}
          </span>
        </div>
        <div className="h-2 rounded-full bg-surface-2 overflow-hidden relative">
          <div className={`h-full transition-all ${driving ? 'bg-online' : 'bg-gray-600'}`}
               style={{ width: `${Math.min(100, trigger * 100)}%` }} />
          {/* Threshold the deadman engages at. */}
          <div className="absolute top-0 bottom-0 w-px bg-warn/80" style={{ left: '50%' }} />
        </div>
      </div>

      {!compact && (
        <>
          {q.dropped > 0 && (
            <div className="text-[10px] text-warn">
              {q.dropped} frame(s) rejected as malformed or out-of-order
            </div>
          )}
          {q.reason && (
            <div className="text-[10px] text-gray-400">
              last gate release: <span className="text-gray-300">{q.reason}</span>
            </div>
          )}
          {!q.connected && (
            <p className="text-[10px] text-gray-500 leading-relaxed">
              No headset connected. Open <span className="font-mono">/xr/</span> on the Quest — it
              needs a secure context, so either HTTPS with a trusted certificate or
              <span className="font-mono"> adb reverse</span> plus
              <span className="font-mono"> http://localhost:8000/xr/</span>.
            </p>
          )}
          <p className="text-[10px] text-gray-600 leading-relaxed pt-1 border-t border-surface-3">
            Amber line = the trigger threshold the deadman engages at. Link over{' '}
            <span className="font-mono">{STALL_MS} ms</span> drops the run gate and the arm goes
            IDLE; over <span className="font-mono">{LOSS_MS / 1000}s</span> is treated as
            controller loss and E-STOPs. <b className="text-gray-400">B/Y</b> E-STOPs at any time,
            even when the Quest is not the active method.
          </p>
        </>
      )}
    </div>
  )
}

function Chip({ label, value, tone = 'text-gray-200' }) {
  return (
    <div>
      <div className="text-[9px] text-gray-500 uppercase tracking-wide">{label}</div>
      <div className={`font-mono text-xs ${tone}`}>{value}</div>
    </div>
  )
}
