import { useEffect, useRef, useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'

// WHAT IS DRIVING THE ROBOT — and, for whichever method is selected, its own controls.
//
// Exactly one source holds the input token at a time; the backend drops (and counts) commands
// from anything else, because two live sources both believing they are driving is the failure
// this exists to prevent. Switching is refused mid-session for the same reason the backend
// refuses it: handing authority over while the robot moves is the transition nobody can
// supervise.
//
// "Policy" is not an input DEVICE like the other two, but the card answers "what is driving the
// robot", and for that question a policy genuinely is one of the answers.

const METHODS = [
  {
    id: 'xbox',
    label: 'Xbox controller',
    blurb: 'Hold LT/RT to drive · A arms · START is E-STOP · Select toggles arm/leg',
  },
  {
    id: 'quest',
    label: 'Quest',
    blurb: 'Hold the trigger to drive · release re-anchors · B/Y is E-STOP',
  },
  {
    id: 'web',
    label: 'Policy',
    blurb: 'Run a trained checkpoint from this page. The browser is the deadman.',
  },
]

const UNAVAILABLE = {
  xbox: 'gamepad deadman not enabled (HUMANOID_GAMEPAD_ENABLE)',
  quest: 'Quest bridge not enabled (HUMANOID_QUEST_ENABLE)',
  web: 'unavailable',
}

export default function ControlMethodPanel() {
  const t = useTelemetry()
  const [busy, setBusy] = useState(null)
  const [error, setError] = useState(null)
  const [policies, setPolicies] = useState([])
  const [defaultPolicy, setDefaultPolicy] = useState(null)
  const [checkpoint, setCheckpoint] = useState('')

  const available = t.input_sources || []
  const active = t.input_source || 'web'
  const motion = t.state === 'HOLDING' || t.state === 'RUNNING'
  const live = t.armed || motion               // backend refuses a switch in these states
  const isConnected = t.state === 'CONNECTED'

  async function run(name, fn) {
    setBusy(name); setError(null)
    try { await fn() } catch (e) { setError(e.message) } finally { setBusy(null) }
  }

  useEffect(() => {
    api.getPolicies()
      .then((d) => { setPolicies(d.policies || []); setDefaultPolicy(d.default || null) })
      .catch(() => { /* ignore — policy dir may be absent */ })
  }, [])

  useEffect(() => {
    if (policies.length && !checkpoint) {
      const def = policies.find((p) => p.name === defaultPolicy) || policies[0]
      setCheckpoint(def.path)
    }
  }, [policies, defaultPolicy])

  // LOAD-BEARING, AND NOT OBVIOUS. This tells the backend which session a trigger-engage runs.
  // Without it, arm_deadman() falls back to a ZeroPolicy "hold" and the gamepad's A button
  // appears to arm while the sticks do nothing — a bug that reads as a broken controller.
  // Re-asserted on every entry to CONNECTED so a restarted service picks it up, and only from
  // CONNECTED because select_session is rejected while a session is live.
  const syncedRef = useRef(null)
  useEffect(() => {
    if (t.state !== 'CONNECTED') { syncedRef.current = null; return }
    if (!checkpoint || syncedRef.current === checkpoint) return
    syncedRef.current = checkpoint
    api.deadmanSelect('policy', checkpoint).catch(() => { syncedRef.current = null })
  }, [checkpoint, t.state])

  const xrUrl = `https://${window.location.hostname}:8443/xr/`

  return (
    <div className="card p-4 space-y-3">
      <div className="flex items-center justify-between">
        <span className="data-label">Control method</span>
        {live && (
          <span className="text-[10px] text-warn" title="the backend refuses a switch mid-session">
            locked while armed
          </span>
        )}
      </div>

      {/* Selector. Unavailable methods are shown DISABLED WITH THE REASON rather than hidden —
          "why is Quest not in the list" is otherwise unanswerable from the UI. */}
      <div className="grid grid-cols-3 gap-1.5">
        {METHODS.map((m) => {
          const enabled = available.includes(m.id)
          const on = active === m.id
          const blocked = !enabled || live
          return (
            <button
              key={m.id}
              disabled={blocked || busy === m.id}
              onClick={() => run(m.id, () => api.setInputSource(m.id))}
              title={!enabled ? UNAVAILABLE[m.id]
                : live ? 'Disarm to change control method' : `drive with ${m.label}`}
              className={`px-2 py-2 rounded-lg border text-xs transition ${
                on ? 'bg-accent/25 border-accent text-white'
                  : blocked ? 'border-surface-3/40 text-gray-600 cursor-not-allowed'
                    : 'border-surface-3 text-gray-300 hover:border-accent/60'}`}
            >
              {m.label}
            </button>
          )
        })}
      </div>

      <p className="text-[10px] text-gray-500 leading-relaxed">
        {METHODS.find((m) => m.id === active)?.blurb}
      </p>

      {/* ── per-method controls ─────────────────────────────────────────── */}

      {active === 'quest' && <QuestMethod t={t} xrUrl={xrUrl} />}

      {active === 'xbox' && (
        <div className="text-[11px] text-gray-400 bg-surface-2/50 rounded-lg px-3 py-2 space-y-1">
          <div>Arm from the controller: <b className="text-gray-200">A</b>. Disarm: <b className="text-gray-200">B</b>.</div>
          <div className="text-gray-500">Live button and axis state is on the Xbox controller card.</div>
        </div>
      )}

      {active === 'web' && (
        <div className="space-y-2">
          <div className="flex items-center gap-2">
            <select value={checkpoint} onChange={(e) => setCheckpoint(e.target.value)}
              className="bg-surface-2 border border-surface-3 rounded-lg px-2 py-2 text-xs text-gray-200 flex-1 min-w-0">
              {policies.length === 0 && <option value="">no checkpoints</option>}
              {policies.map((p) => <option key={p.path} value={p.path}>{p.name}</option>)}
            </select>
            <button className="btn-primary shrink-0"
              disabled={busy || !isConnected || !t.armed || !checkpoint || !t.all_calibrated}
              onClick={() => run('run', () => api.runPolicy(checkpoint))}>
              {busy === 'run' ? 'Starting…' : 'Run policy'}
            </button>
          </div>
          <button className="btn-primary w-full"
            disabled={busy || !isConnected || !t.armed || !t.all_calibrated}
            onClick={() => run('hold', () => api.hold())}>
            {busy === 'hold' ? 'Ramping…' : 'Ramp to pose / Hold'}
          </button>
          {motion && (
            <button className="btn-ghost w-full" disabled={busy}
              onClick={() => run('stop', () => api.stop())}>Stop</button>
          )}
          <p className="text-[10px] text-gray-500">
            Both command the calibrated <span className="font-mono">default_pose</span> frame, so
            both need every joint calibrated. To hold the current pose without calibrating, use
            the Manual tab.
          </p>
        </div>
      )}

      {error && (
        <div className="text-xs text-danger bg-danger/10 border border-danger/30 rounded-lg px-3 py-2">
          {error}
        </div>
      )}
    </div>
  )
}

/** Quest-specific setup: which arm, the mapping constants, and how to reach the headset page. */
function QuestMethod({ t, xrUrl }) {
  const q = t.quest || {}
  const arms = (t.layout?.enabled || []).filter((l) => l.endsWith('_arm'))
  return (
    <div className="space-y-2">
      <div className="grid grid-cols-3 gap-2 text-center">
        <Stat label="hand" value={q.hand || '—'} />
        <Stat label="scale" value={q.scale != null ? `${q.scale}×` : '—'} />
        <Stat label="yaw" value={q.yaw_deg != null ? `${q.yaw_deg}°` : '—'} />
      </div>
      {arms.length > 1 && (
        <p className="text-[10px] text-gray-500">
          Driving <b className="text-gray-300">{q.hand === 'right' ? 'right' : 'left'}</b> arm —
          set with the bumpers or <span className="font-mono">HUMANOID_QUEST_HAND</span>.
        </p>
      )}
      <div className="bg-surface-2/50 rounded-lg px-3 py-2 space-y-1">
        <div className="data-label">Open on the headset</div>
        <code className="text-[11px] text-accent break-all">{xrUrl}</code>
        <p className="text-[10px] text-gray-500 leading-relaxed">
          WebXR needs a <b>secure context</b>. A self-signed certificate is not enough on its
          own — if the headset reports “WebXR unavailable”, run
          <span className="font-mono"> adb reverse tcp:8000 tcp:8000</span> and open
          <span className="font-mono"> http://localhost:8000/xr/</span> instead, which Chromium
          trusts without any certificate.
        </p>
      </div>
      <p className="text-[10px] text-gray-500">
        Scale, yaw and hand are env vars (<span className="font-mono">HUMANOID_QUEST_*</span>) —
        they are read at startup, so changing one needs a restart.
      </p>
    </div>
  )
}

function Stat({ label, value }) {
  return (
    <div className="rounded-md border border-surface-3 px-1 py-1.5">
      <div className="text-[9px] text-gray-500 uppercase tracking-wide">{label}</div>
      <div className="font-mono text-xs text-gray-200">{value}</div>
    </div>
  )
}
