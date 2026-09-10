import { useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'

// WHICH DEVICE DRIVES THE ROBOT. Xbox or Quest — nothing else.
//
// A policy is deliberately NOT one of these. The policy always runs on the legs; the device
// selected here supplies its velocity command. With neither device connected the policy simply
// stands still, which is why listing it here as a third "way of driving" was wrong: it implied
// picking one INSTEAD of a controller, when in fact the two compose.
//
// The browser is not listed either. It holds the input token by default so that a machine with
// no controller attached still has exactly one owner, and you do need this page to calibrate —
// but it is the console you supervise from, not a control method.
//
// Exactly one device holds the token at a time; the backend drops (and counts) writes from any
// other, because two live sources both believing they are driving is the failure that exists to
// prevent. Switching is refused mid-session for the same reason the backend refuses it.

const DEVICES = {
  xbox: {
    label: 'Xbox controller',
    blurb: 'Hold LT/RT to drive · A arms · START is E-STOP · Select toggles arm/leg',
    unavailable: 'gamepad deadman not enabled (HUMANOID_GAMEPAD_ENABLE)',
  },
  quest: {
    label: 'Quest',
    blurb: 'Hold the trigger to drive · release re-anchors · B/Y is E-STOP',
    unavailable: 'Quest bridge not enabled (HUMANOID_QUEST_ENABLE)',
  },
}
const ORDER = ['xbox', 'quest']

export default function InputMethodPanel() {
  const t = useTelemetry()
  const [busy, setBusy] = useState(null)
  const [error, setError] = useState(null)

  const available = t.input_devices || []
  const active = t.input_source
  const motion = t.state === 'HOLDING' || t.state === 'RUNNING'
  const live = t.armed || motion               // backend refuses a switch in these states
  const ignored = t.ignored_writes || {}

  async function run(name, fn) {
    setBusy(name); setError(null)
    try { await fn() } catch (e) { setError(e.message) } finally { setBusy(null) }
  }

  const xrUrl = `https://${window.location.hostname}:8443/xr/`

  return (
    <div className="card p-4 space-y-3">
      <div className="flex items-center justify-between">
        <span className="data-label">Input method</span>
        {live && (
          <span className="text-[10px] text-warn" title="the backend refuses a switch mid-session">
            locked while armed
          </span>
        )}
      </div>

      {/* Unavailable devices are shown DISABLED WITH THE REASON rather than hidden — "why is
          Quest not in the list" is otherwise unanswerable from the UI. */}
      <div className="grid grid-cols-2 gap-1.5">
        {ORDER.map((id) => {
          const d = DEVICES[id]
          const enabled = available.includes(id)
          const on = active === id
          const blocked = !enabled || live
          return (
            <button
              key={id}
              disabled={blocked || busy === id}
              onClick={() => run(id, () => api.setInputSource(id))}
              title={!enabled ? d.unavailable
                : live ? 'Disarm to change input method' : `drive with ${d.label}`}
              className={`px-2 py-2 rounded-lg border text-xs transition ${
                on ? 'bg-accent/25 border-accent text-white'
                  : blocked ? 'border-surface-3/40 text-gray-600 cursor-not-allowed'
                    : 'border-surface-3 text-gray-300 hover:border-accent/60'}`}
            >
              {d.label}
            </button>
          )
        })}
      </div>

      {DEVICES[active] && (
        <p className="text-[10px] text-gray-500 leading-relaxed">{DEVICES[active].blurb}</p>
      )}

      {/* NOT AN ERROR STATE, so it is phrased as a fact rather than a warning. The browser holds
          the token whenever no device does; the robot is supervisable, just not drivable. */}
      {!DEVICES[active] && (
        <p className="text-[10px] text-gray-500 leading-relaxed">
          No input device is driving. The legs' policy will run and stand still until an Xbox
          controller or Quest takes over.
        </p>
      )}

      {/* A device being deliberately ignored feels identical to a broken one. Say so. */}
      {Object.entries(ignored).filter(([, n]) => n > 0).length > 0 && (
        <div className="text-[10px] text-warn bg-surface-2/50 rounded-lg px-3 py-2 space-y-0.5">
          {Object.entries(ignored).filter(([, n]) => n > 0).map(([src, n]) => (
            <div key={src}>
              <span className="text-gray-300">{DEVICES[src]?.label || src}</span> sent {n} command
              {n === 1 ? '' : 's'} while <span className="text-gray-300">
                {DEVICES[active]?.label || active}
              </span> holds the token — ignored.
            </div>
          ))}
        </div>
      )}

      {active === 'quest' && <QuestSetup t={t} xrUrl={xrUrl} />}

      {active === 'xbox' && (
        <div className="text-[11px] text-gray-400 bg-surface-2/50 rounded-lg px-3 py-2 space-y-1">
          <div>Arm from the controller: <b className="text-gray-200">A</b>. Disarm: <b className="text-gray-200">B</b>.</div>
          <div className="text-gray-500">Live button and axis state is on the Xbox controller card.</div>
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
function QuestSetup({ t, xrUrl }) {
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
      <div className="bg-surface-2/50 rounded-lg px-3 py-2 space-y-1.5">
        <div className="data-label">Open on the headset</div>
        <code className="text-[11px] text-accent break-all">http://localhost:8000/xr/</code>
        <p className="text-[10px] text-gray-500 leading-relaxed">
          Requires <span className="font-mono">adb reverse tcp:8000 tcp:8000</span> with the
          headset in developer mode over USB. WebXR only runs in a <b>secure context</b>, and
          Chromium trusts <span className="font-mono">localhost</span> as one with no
          certificate at all.
        </p>
        <details className="text-[10px] text-gray-600">
          <summary className="cursor-pointer hover:text-gray-400">Why not the LAN address?</summary>
          <p className="pt-1 leading-relaxed">
            <code className="break-all">{xrUrl}</code> serves the same page over TLS, but the
            certificate is self-signed. Chromium keeps flagging an origin whose certificate you
            clicked through and withholds WebXR from it, so the page loads and then reports
            “WebXR unavailable”. A trusted certificate would fix it — the robot has no public
            DNS name to get one for.
          </p>
        </details>
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
