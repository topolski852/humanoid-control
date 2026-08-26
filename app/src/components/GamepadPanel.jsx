import { useTelemetry } from '../context/TelemetryContext'

// Live controller state — what the hardware is actually sending, and what the robot thinks it
// is being told to do.
//
// This exists to separate two failure modes that look identical from the outside: a button that
// is not mapped to the physical key you pressed, and a button that IS mapped but whose action is
// blocked downstream. Without a raw view you cannot tell them apart, and both present as
// "nothing happens".
//
// Every button the device advertises is shown, not just the bound ones, so a binding that points
// at the wrong physical key is visible rather than inferred.

function Pill({ on, label, action, phantom }) {
  // `phantom` = advertised by the kernel's generic gamepad profile but not physically present.
  // Shown rather than hidden, because "is there really a C button?" is exactly the sort of
  // question this panel exists to answer.
  return (
    <div
      className={`px-2 py-1.5 rounded-md border text-center transition ${
        on
          ? 'bg-accent/25 border-accent text-white'
          : phantom
            ? 'border-surface-3/30 text-gray-700 opacity-50'
            : action
              ? 'border-surface-3 text-gray-400'
              : 'border-surface-3/50 text-gray-600'
      }`}
      title={phantom
        ? 'advertised by the driver but not a real button on this controller'
        : action ? `bound to: ${action}` : 'no action bound'}
    >
      <div className="font-mono text-xs leading-tight">{label}</div>
      <div className={`text-[9px] leading-tight ${action ? 'text-accent/80' : 'text-gray-700'}`}>
        {action || (phantom ? 'n/a' : 'unbound')}
      </div>
    </div>
  )
}

/** Centred axis: origin in the middle, deadband shaded, live value as a bar from centre. */
function CenteredAxis({ name, value, deadband }) {
  const pct = (value + 1) / 2 * 100
  const dbHalf = (deadband / 2) * 100
  const live = Math.abs(value) > deadband
  return (
    <div className="flex items-center gap-2">
      <span className="font-mono text-[10px] text-gray-500 w-14">{name}</span>
      <div className="flex-1 h-4 bg-surface-2 rounded relative overflow-hidden">
        <div className="absolute inset-y-0 bg-surface-3/60"
             style={{ left: `${50 - dbHalf}%`, width: `${dbHalf * 2}%` }} />
        <div className="absolute inset-y-0 w-px bg-gray-600" style={{ left: '50%' }} />
        <div className={`absolute inset-y-0 ${live ? 'bg-accent' : 'bg-gray-600'}`}
             style={{
               left: `${Math.min(50, pct)}%`,
               width: `${Math.abs(pct - 50)}%`,
             }} />
      </div>
      <span className={`font-mono text-[10px] w-12 text-right tabular-nums ${
        live ? 'text-accent' : 'text-gray-600'}`}>
        {value >= 0 ? '+' : ''}{value.toFixed(2)}
      </span>
    </div>
  )
}

/** One-sided axis (a trigger): 0 at the left, threshold marked. */
function TriggerAxis({ name, value, threshold }) {
  const held = value >= threshold
  return (
    <div className="flex items-center gap-2">
      <span className="font-mono text-[10px] text-gray-500 w-14">{name}</span>
      <div className="flex-1 h-4 bg-surface-2 rounded relative overflow-hidden">
        <div className={`absolute inset-y-0 left-0 ${held ? 'bg-online' : 'bg-gray-600'}`}
             style={{ width: `${Math.max(0, Math.min(1, value)) * 100}%` }} />
        <div className="absolute inset-y-0 w-px bg-warn/70" style={{ left: `${threshold * 100}%` }} />
      </div>
      <span className={`font-mono text-[10px] w-12 text-right tabular-nums ${
        held ? 'text-online' : 'text-gray-600'}`}>
        {value.toFixed(2)}
      </span>
    </div>
  )
}

function Chip({ label, value, tone = 'text-gray-200' }) {
  return (
    <div className="flex flex-col">
      <span className="data-label text-[9px]">{label}</span>
      <span className={`font-mono text-xs ${tone}`}>{value}</span>
    </div>
  )
}

export default function GamepadPanel() {
  const t = useTelemetry()
  const gp = t.gamepad || {}
  const input = gp.input || {}
  const ctl = t.control || {}
  const buttons = input.buttons || []
  const axes = input.axes || []
  const deadband = input.deadband ?? 0.15
  const threshold = input.trigger_threshold ?? 0.5

  if (!gp.enabled) {
    return (
      <div className="card p-4">
        <div className="data-label mb-2">Controller</div>
        <p className="text-xs text-gray-500">
          Gamepad support is off. Start the server with
          <span className="font-mono text-gray-400"> HUMANOID_GAMEPAD_ENABLE=1</span>.
        </p>
      </div>
    )
  }

  const armMode = ctl.mode === 'arm'
  const limbLabel = armMode
    ? (ctl.limb ? ctl.limb.replace('_', ' ') : `${(ctl.arms || []).length > 1 ? 'none selected' : (ctl.arms || [])[0]?.replace('_', ' ') || 'no arm'}`)
    : 'legs'

  return (
    <div className="card p-4 space-y-3">
      <div className="flex items-center justify-between">
        <span className="data-label">Controller</span>
        <span className={`text-[10px] font-mono ${gp.connected ? 'text-online' : 'text-offline'}`}>
          {gp.connected ? `● ${gp.name}` : '○ not connected'}
          {input.layout && <span className="text-gray-600"> · {input.layout}</span>}
        </span>
      </div>

      {/* What the robot thinks it is being told to drive. */}
      <div className="grid grid-cols-4 gap-2 py-2 border-y border-surface-3">
        <Chip label="mode" value={armMode ? 'ARM' : 'LEG'}
              tone={armMode ? 'text-accent' : 'text-gray-200'} />
        <Chip label="driving" value={limbLabel} />
        <Chip label="speed" value={ctl.speed || '—'}
              tone={ctl.speed === 'creep' ? 'text-warn' : 'text-gray-200'} />
        <Chip label="run gate" value={gp.run_gate ? 'HELD' : 'released'}
              tone={gp.run_gate ? 'text-online' : 'text-gray-500'} />
      </div>

      {!gp.connected ? (
        <p className="text-xs text-gray-500">Connect the controller to see live input.</p>
      ) : (
        <>
          <div className="grid grid-cols-5 gap-1.5">
            {buttons.map((b) => (
              <Pill key={b.code} on={b.pressed} label={b.name} action={b.action}
                    phantom={b.phantom} />
            ))}
          </div>

          <div className="space-y-1 pt-1">
            {axes.filter((a) => a.centered).map((a) => (
              <CenteredAxis key={a.name} name={a.name} value={a.value} deadband={deadband} />
            ))}
            {axes.filter((a) => !a.centered).map((a) => (
              <TriggerAxis key={a.name} name={a.name} value={a.value} threshold={threshold} />
            ))}
          </div>

          <p className="text-[10px] text-gray-600 leading-relaxed pt-1 border-t border-surface-3">
            Shaded band = deadband (input inside it is ignored, so stick drift cannot creep the
            arm). Amber line on the triggers = the threshold the deadman engages at. A button
            with no action underneath is unbound. Faded buttons marked <span
            className="font-mono">n/a</span> are advertised by the driver but do not exist on
            this controller.
          </p>
        </>
      )}
    </div>
  )
}
