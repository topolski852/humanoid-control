import { useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'

// WHAT THE ARM DOES WHEN IT IS ARMED BUT NOT DRIVING.
//
// This is the state the arm spends most of its time in: every released trigger, every lost
// tracker, every aborted ramp, every torn-down session. It used to be IDLE — zero torque —
// which for a 5-DOF arm held out in front of a robot is not resting, it is falling.
//
// DAMPING is regenerative braking: the motor resists motion but holds no position target, so
// the arm sinks slowly instead of dropping. It is the default.
//
// It took a daemon change to become usable. Actuator::tick() used to send PDO2 only while a
// joint was ENABLED, so a damping joint was fed by nothing and the firmware watchdog expired
// after 1000 ms — every armed session E-STOPped within seconds on ERROR_WATCHDOG_TIMEOUT.
// The daemon now feeds DAMPING joints; verified on the arm for 60 s across all five.
//
// IDLE stays available because it is genuinely wanted — a damped joint is unpleasant to
// back-drive, so hand-positioning the arm, checking free play and re-zeroing all want the
// motors limp. The point of this card is that going limp is a deliberate choice made here,
// rather than the default a released trigger silently drops you into.
//
// THE CARD HAS TWO HALVES AND THEY ARE NOT THE SAME CONTROL.
//
//   Default   — what the NEXT release/loss/teardown does. Changes nothing that is already
//               resting. This is the safety policy.
//   Right now — re-commands the joints this instant. This is the one you want when the arm
//               is sitting in DAMPING, you need to move it by hand, and changing the default
//               appears to do nothing at all — because nothing has triggered a rest
//               transition since. That gap is a real trap and the reason this half exists.
//
// "Right now" is not sticky: the next rest transition re-applies the default. Going limp is
// for a task in front of you, not a state to leave the robot in and forget.

const MODES = [
  {
    id: 'damping',
    label: 'Damping',
    blurb: 'Powered braking — the arm resists gravity and sinks slowly instead of dropping. '
         + 'Also where an E-STOP now leaves it.',
  },
  {
    id: 'idle',
    label: 'Idle',
    blurb: 'Zero torque — limp and free to move by hand, and it WILL fall under its own '
         + 'weight on release. For repositioning or re-zeroing, not for driving.',
  },
]

export default function RestModePanel() {
  const t = useTelemetry()
  const [busy, setBusy] = useState(null)
  const [error, setError] = useState(null)

  const active = t.control?.rest || 'damping'   // matches the backend default
  const connected = t.state && t.state !== 'DISCONNECTED'
  const driving = t.state === 'HOLDING' || t.state === 'RUNNING'

  // What the joints are ACTUALLY in, from telemetry. Collapses to one label when they agree
  // and lists them when they do not — a split group is exactly the case worth seeing.
  const states = [...new Set((t.joints || []).map((j) => j.state).filter(Boolean))]
  const liveMode = states.length === 1 ? states[0] : states.join(' / ')

  async function pick(mode) {
    if (mode === active) return
    setBusy(mode); setError(null)
    try { await api.setRestMode(mode) } catch (e) { setError(e.message) } finally { setBusy(null) }
  }

  async function applyNow(mode) {
    setBusy(`now:${mode}`); setError(null)
    try { await api.setJointMode(mode) } catch (e) { setError(e.message) } finally { setBusy(null) }
  }

  return (
    <div className="card p-4 space-y-3">
      <div className="flex items-center justify-between">
        <span className="data-label">Rest state</span>
        {active === 'idle' && (
          <span className="text-[10px] text-warn" title="the arm is free to fall at rest">
            will fall
          </span>
        )}
      </div>

      <div className="grid grid-cols-2 gap-1.5">
        {MODES.map((m) => {
          const on = active === m.id
          const blocked = !connected || busy
          return (
            <button
              key={m.id}
              disabled={blocked}
              onClick={() => pick(m.id)}
              aria-pressed={on}
              title={connected ? m.blurb : 'Connect to the robot to change this'}
              className={`px-2 py-2 rounded-lg border text-xs transition ${
                on
                  ? (m.id === 'idle'
                      ? 'bg-warn/20 border-warn text-white'
                      : 'bg-accent/25 border-accent text-white')
                  : blocked ? 'border-surface-3/40 text-gray-600 cursor-not-allowed'
                    : 'border-surface-3 text-gray-300 hover:border-accent/60'}`}
            >
              {m.label}
            </button>
          )
        })}
      </div>

      <p className="text-[10px] text-gray-500 leading-relaxed">
        {MODES.find((m) => m.id === active)?.blurb}
      </p>

      {active === 'idle' && (
        // Not a nag. Idle is a real choice with a real consequence, and the moment it matters
        // is the moment the operator has stopped looking at this card.
        <div className="text-[11px] text-warn bg-surface-2/50 rounded-lg px-3 py-2">
          The arm will drop when you release the trigger, lose tracking, or end the session.
        </div>
      )}

      {driving && (
        <p className="text-[10px] text-gray-500">
          Applies at the next release — changing this never interrupts a driving joint.
        </p>
      )}

      {/* ── Right now ─────────────────────────────────────────────────────
          Re-commands the joints immediately. Shows the LIVE mode read back from
          telemetry, not the requested one, so "I pressed Idle and it is still stiff"
          is answerable from the card instead of from a log. */}
      <div className="border-t border-surface-3 pt-3 space-y-2">
        <div className="flex items-center justify-between">
          <span className="data-label">Right now</span>
          <span className="text-[10px] font-mono text-gray-400">
            {liveMode || '—'}
          </span>
        </div>

        <div className="grid grid-cols-2 gap-1.5">
          {MODES.map((m) => (
            <button
              key={m.id}
              disabled={!connected || !!busy || driving}
              onClick={() => applyNow(m.id)}
              title={driving
                ? 'Release the trigger first — the arm is driving'
                : `put the joints into ${m.label} immediately`}
              className={`px-2 py-2 rounded-lg border text-xs transition ${
                (!connected || driving)
                  ? 'border-surface-3/40 text-gray-600 cursor-not-allowed'
                  : 'border-surface-3 text-gray-300 hover:border-accent/60'}`}
            >
              {busy === `now:${m.id}` ? '…' : m.label} now
            </button>
          ))}
        </div>

        <p className="text-[10px] text-gray-500 leading-relaxed">
          {driving
            ? 'Disabled while driving. Release the trigger to change motor mode.'
            : 'Applies immediately and is not sticky — the next release goes back to '
              + `${MODES.find((m) => m.id === active)?.label ?? active}.`}
        </p>
      </div>

      {!connected && <p className="text-[10px] text-gray-500">Connect to the robot to change this.</p>}
      {error && <p className="text-[10px] text-danger">{error}</p>}
    </div>
  )
}
