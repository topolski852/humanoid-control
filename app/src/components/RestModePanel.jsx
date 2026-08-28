import { useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'

// WHAT THE ARM DOES WHEN IT IS ARMED BUT NOT DRIVING.
//
// This is the state the arm spends most of its time in: every released trigger, every lost
// tracker, every aborted ramp, every torn-down session. It used to be IDLE — zero torque —
// which for a 5-DOF arm held out in front of a robot is not resting, it is falling.
//
// DAMPING is regenerative braking. The motor resists motion but holds no position target, so
// the arm sinks slowly instead of dropping, and nothing fights you if you push it hard. It is
// also the firmware's OWN fail-safe: the watchdog drops a motor into DAMPING when its commands
// stop, which is why it needs no feed and holds indefinitely.
//
// IDLE stays available because it is genuinely wanted — a damped joint is unpleasant to
// back-drive, so hand-positioning the arm, checking free play and re-zeroing all want the
// motors limp. The point of this card is that going limp becomes a deliberate choice made
// here, rather than the default a released trigger silently drops you into.

const MODES = [
  {
    id: 'damping',
    label: 'Damping',
    blurb: 'Powered braking. The arm resists gravity and sinks slowly instead of dropping. '
         + 'This is also where the motor controller puts itself if it loses comms.',
  },
  {
    id: 'idle',
    label: 'Idle',
    blurb: 'Zero torque — limp and free to move by hand, and it will fall under its own '
         + 'weight. For repositioning or re-zeroing, not for driving.',
  },
]

export default function RestModePanel() {
  const t = useTelemetry()
  const [busy, setBusy] = useState(null)
  const [error, setError] = useState(null)

  const active = t.control?.rest || 'damping'
  const connected = t.state && t.state !== 'DISCONNECTED'
  const driving = t.state === 'HOLDING' || t.state === 'RUNNING'

  async function pick(mode) {
    if (mode === active) return
    setBusy(mode); setError(null)
    try { await api.setRestMode(mode) } catch (e) { setError(e.message) } finally { setBusy(null) }
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

      {!connected && <p className="text-[10px] text-gray-500">Connect to the robot to change this.</p>}
      {error && <p className="text-[10px] text-danger">{error}</p>}
    </div>
  )
}
