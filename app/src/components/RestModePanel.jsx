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
// the arm sinks slowly instead of dropping. It is the rest state this arm wants.
//
// It is NOT the default, and the card says so loudly. Measured on this hardware: with rest set
// to DAMPING, every armed session E-STOPped within seconds on ERROR_WATCHDOG_TIMEOUT (0x0040)
// across all five joints. The daemon's Actuator::tick() sends frames only to ENABLED joints, so
// a damping joint receives nothing and the firmware watchdog expires. docs/HANDOFF.md and two
// C++ comments claim the watchdog cannot fire in DAMPING; they are out of date, and the arm is
// the authority.
//
// So IDLE is the default by necessity rather than merit — a limp 5-DOF arm falls. The honest
// thing is to show both, name what each actually does to this robot, and let the operator pick
// with their eyes open. When the daemon feeds DAMPING, the default flips and this card keeps
// working unchanged.

const MODES = [
  {
    id: 'damping',
    label: 'Damping',
    blurb: 'Powered braking — the arm resists gravity instead of dropping. NOT USABLE YET on '
         + 'this firmware: the watchdog trips (0x0040) within a second because the daemon '
         + 'sends no frames to a damping joint, and the session E-STOPs.',
    warn: true,
  },
  {
    id: 'idle',
    label: 'Idle',
    blurb: 'Zero torque — limp and free to move by hand. The arm WILL fall under its own '
         + 'weight on release. The only rest state this firmware currently sustains.',
  },
]

export default function RestModePanel() {
  const t = useTelemetry()
  const [busy, setBusy] = useState(null)
  const [error, setError] = useState(null)

  const active = t.control?.rest || 'idle'   // matches the backend default
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
        {active === 'damping' && (
          <span className="text-[10px] text-danger" title="watchdog trips; sessions E-STOP">
            trips watchdog
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

      {active === 'damping' && (
        <div className="text-[11px] text-danger bg-surface-2/50 rounded-lg px-3 py-2">
          Measured on this arm: every armed session E-STOPped within seconds on
          ERROR_WATCHDOG_TIMEOUT, on all five joints. The daemon only sends frames to ENABLED
          joints, so nothing feeds a damping one. Needs a daemon fix before this is usable.
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
