import { useEffect, useRef, useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'

// WHAT EACH LIMB DOES with the operator's input — per limb, because the two are not the same
// kind of thing and never were.
//
//   legs  which POLICY runs. The policy is always what moves the legs; the input device (Input
//         method card) only supplies its velocity command. With no device connected the policy
//         runs and stands still. That is why a policy is not an input method: it composes with
//         one rather than replacing it.
//   arms  HOW the operator's motion maps onto the arm. There is no policy for the arms, so on
//         an arms-only machine nothing here mentions one.
//
// A machine shows only the sections its layout supports, so a bench arm never sees a policy
// picker and a legs-only robot never sees an arm mapping.
export default function ControlMethodPanel() {
  const t = useTelemetry()
  const [busy, setBusy] = useState(null)
  const [error, setError] = useState(null)
  const [policies, setPolicies] = useState([])
  const [defaultPolicy, setDefaultPolicy] = useState(null)
  const [checkpoint, setCheckpoint] = useState('')

  const modes = t.control?.modes || []
  const hasLegs = modes.includes('leg')
  const hasArms = modes.includes('arm')
  const motion = t.state === 'HOLDING' || t.state === 'RUNNING'
  const live = t.armed || motion
  const isConnected = t.state === 'CONNECTED'

  async function run(name, fn) {
    setBusy(name); setError(null)
    try { await fn() } catch (e) { setError(e.message) } finally { setBusy(null) }
  }

  useEffect(() => {
    if (!hasLegs) return
    api.getPolicies()
      .then((d) => { setPolicies(d.policies || []); setDefaultPolicy(d.default || null) })
      .catch(() => { /* ignore — policy dir may be absent */ })
  }, [hasLegs])

  useEffect(() => {
    if (policies.length && !checkpoint) {
      // Never pre-select something the operator is not allowed to run.
      const usable = policies.filter((p) => p.compatible !== false)
      const def = usable.find((p) => p.name === defaultPolicy) || usable[0]
      if (def) setCheckpoint(def.path)
    }
  }, [policies, defaultPolicy])

  // LOAD-BEARING, AND NOT OBVIOUS. This tells the backend which session a trigger-engage runs.
  // Without it, arm_deadman() falls back to a ZeroPolicy "hold" and the controller's A button
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

  const armMethods = t.control?.arm_methods || []
  const activeArm = t.control?.arm_method

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

      {!hasLegs && !hasArms && (
        <p className="text-[10px] text-gray-500">
          Nothing to control — the layout has neither legs nor arms enabled.
        </p>
      )}

      {/* ── LEGS: which policy ──────────────────────────────────────────── */}
      {hasLegs && (
        <div className="space-y-2">
          <div className="text-[10px] text-gray-500 uppercase tracking-wide">Legs · policy</div>
          <div className="flex items-center gap-2">
            <select value={checkpoint} onChange={(e) => setCheckpoint(e.target.value)}
              className="bg-surface-2 border border-surface-3 rounded-lg px-2 py-2 text-xs text-gray-200 flex-1 min-w-0">
              {policies.length === 0 && <option value="">no checkpoints</option>}
              {/* INCOMPATIBLE BUNDLES ARE SHOWN BUT NOT SELECTABLE. Hiding them would raise
                  "where did my policy go?"; disabling them with the reason answers it. The
                  backend refuses them too — this is the convenience, not the safety. */}
              {policies.map((p) => (
                <option key={p.path} value={p.path} disabled={p.compatible === false}
                        title={p.issues?.join(' · ') || ''}>
                  {p.compatible === false ? `${p.name} — incompatible` : p.name}
                </option>
              ))}
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
          <p className="text-[10px] text-gray-500 leading-relaxed">
            The selected policy is what the legs run. It takes its velocity command from
            whichever device holds the input token — with none connected it stands still.
          </p>
          {policies.some((p) => p.compatible === false) && (
            // Switching policy switches the NETWORK only — gains, stand pose and timing keep
            // coming from the runtime contract. A bundle trained at other gains would run
            // against a robot it has never seen, so it is refused rather than offered.
            <div className="text-[10px] text-warn bg-surface-2/50 rounded-lg px-3 py-2 space-y-1">
              {policies.filter((p) => p.compatible === false).map((p) => (
                <div key={p.path}>
                  <span className="text-gray-300">{p.name}</span> is not selectable:{' '}
                  {p.issues?.join('; ')}
                </div>
              ))}
            </div>
          )}
          <p className="text-[10px] text-gray-500">
            Both command the calibrated <span className="font-mono">default_pose</span> frame, so
            both need every joint calibrated. To hold the current pose without calibrating, use
            the Manual tab.
          </p>
        </div>
      )}

      {hasLegs && hasArms && <div className="border-t border-surface-3/60" />}

      {/* ── ARMS: how the operator's motion maps ────────────────────────── */}
      {hasArms && (
        <div className="space-y-2">
          <div className="text-[10px] text-gray-500 uppercase tracking-wide">Arms · mapping</div>
          {armMethods.length === 0 && (
            <p className="text-[10px] text-gray-500">No arm mapping available for this layout.</p>
          )}
          {/* Each mapping is valid for exactly ONE input device, so the list changes when the
              token moves. Invalid ones stay visible with the reason — "why can I not pick
              mirror" is otherwise an invisible property of the calibration state. */}
          <div className="grid grid-cols-1 gap-1.5">
            {armMethods.map((m) => {
              const on = activeArm === m.id
              const blocked = !m.available || live
              return (
                <button
                  key={m.id}
                  disabled={blocked || busy === m.id}
                  onClick={() => run(m.id, () => api.setArmMethod(m.id))}
                  title={m.reason || (live ? 'Disarm to change arm mapping' : m.blurb)}
                  className={`px-2 py-2 rounded-lg border text-left text-xs transition ${
                    on ? 'bg-accent/25 border-accent text-white'
                      : blocked ? 'border-surface-3/40 text-gray-600 cursor-not-allowed'
                        : 'border-surface-3 text-gray-300 hover:border-accent/60'}`}
                >
                  <div className="flex items-center justify-between gap-2">
                    <span>{m.label}</span>
                    {!m.available && (
                      <span className="text-[9px] text-gray-600 shrink-0">{m.reason}</span>
                    )}
                  </div>
                </button>
              )
            })}
          </div>
          {activeArm && (
            <p className="text-[10px] text-gray-500 leading-relaxed">
              {armMethods.find((m) => m.id === activeArm)?.blurb}
            </p>
          )}
          <p className="text-[10px] text-gray-500">
            Resolved once when the session arms — the mapping cannot change under a moving arm.
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
