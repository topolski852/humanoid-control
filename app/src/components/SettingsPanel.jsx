import { useEffect, useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'

// Robot layout: which limbs are attached to THIS machine.
//
// The layout is the joint set for everything downstream — telemetry, the health check that
// gates connect, calibration, and what the wireframe draws. That is the point: with the legs
// unpowered on a bench, offline leg joints are the expected state, not a fault, and the app
// should say so instead of refusing to connect.
//
// Saved to a machine-local file, so this bench PC and the torso PC each keep their own answer.

const BUS_TONE = {
  UP: 'text-online',
  DOWN: 'text-warn',
  UNCONFIGURED: 'text-gray-600',
}

// Shape from DaemonClient.get_interface_stats(): {name, state, joints_online, joints_total, …}.
// It always emits all four canonical buses, so a missing entry means telemetry hasn't arrived.
function busInfo(buses, name) {
  const b = (buses || []).find((x) => x.name === name)
  if (!b) return { state: 'UNKNOWN', online: null, total: null }
  return { state: b.state ?? 'UNKNOWN', online: b.joints_online, total: b.joints_total }
}

function LimbRow({ limb, checked, disabled, reason, onToggle, buses }) {
  const bus = busInfo(buses, limb.bus)
  const tone = BUS_TONE[bus.state] ?? 'text-gray-500'
  return (
    <label
      className={`flex items-start gap-3 py-2.5 border-b border-surface-3/40 ${
        disabled ? 'opacity-50 cursor-not-allowed' : 'cursor-pointer'
      }`}
    >
      <input
        type="checkbox"
        checked={checked}
        disabled={disabled}
        onChange={() => onToggle(limb.id)}
        className="mt-0.5 accent-accent"
      />
      <div className="flex-1 min-w-0">
        <div className="flex items-center justify-between gap-2">
          <span className="text-sm text-gray-200">{limb.label}</span>
          <span className={`text-[10px] font-mono ${tone}`}>
            {limb.bus} · {bus.state}
            {bus.total != null && (
              <span className="text-gray-600"> · {bus.online ?? 0}/{bus.total} online</span>
            )}
          </span>
        </div>
        <div className="text-[10px] font-mono text-gray-600 mt-0.5 truncate">
          {limb.joints.map((j) => j.replace(/_joint$/, '')).join(' · ')}
        </div>
        {reason && <div className="text-[10px] text-danger mt-0.5">{reason}</div>}
      </div>
    </label>
  )
}

export default function SettingsPanel() {
  const t = useTelemetry()
  const [layout, setLayout] = useState(null)
  const [draft, setDraft] = useState(null)      // { enabled: Set, imu: bool } — null until loaded
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState(null)
  const [note, setNote] = useState(null)

  useEffect(() => {
    let live = true
    api.getLayout()
      .then((d) => {
        if (!live) return
        setLayout(d)
        setDraft({ enabled: new Set(d.enabled), imu: d.imu_expected })
      })
      .catch((e) => live && setError(e.message))
    return () => { live = false }
  }, [])

  if (error && !layout) {
    return <div className="card p-5 text-sm text-danger">Could not load layout: {error}</div>
  }
  if (!layout || !draft) {
    return <div className="card p-8 text-center text-sm text-gray-500">Loading layout…</div>
  }

  const enabledList = layout.limbs.filter((l) => draft.enabled.has(l.id)).map((l) => l.id)
  const dirty =
    enabledList.join(',') !== layout.enabled.join(',') || draft.imu !== layout.imu_expected
  const sessionActive = ['ARMED', 'HOLDING', 'RUNNING'].includes(t.state)
  const nothingChecked = enabledList.length === 0

  const toggle = (id) => {
    setNote(null)
    setError(null)
    setDraft((d) => {
      const next = new Set(d.enabled)
      next.has(id) ? next.delete(id) : next.add(id)
      return { ...d, enabled: next }
    })
  }

  async function save() {
    setBusy(true); setError(null); setNote(null)
    // Only a change to the LIMB set changes the joints being watched, and only that drops a
    // live connection. Flipping the IMU flag alone leaves the connection alone, so don't tell
    // the operator to reconnect when they don't have to.
    const limbsChanged = enabledList.join(',') !== layout.enabled.join(',')
    try {
      const d = await api.setLayout(enabledList, draft.imu)
      setLayout(d)
      setDraft({ enabled: new Set(d.enabled), imu: d.imu_expected })
      setNote(
        !d.saved
          ? `Applied, but NOT saved to disk (${d.save_error}) — it will revert on restart.`
          : limbsChanged
            ? `Saved to ${d.path}. Reconnect to pick up the new joint set.`
            : `Saved to ${d.path}.`,
      )
    } catch (e) {
      setError(e.message)
    } finally {
      setBusy(false)
    }
  }

  function revert() {
    setDraft({ enabled: new Set(layout.enabled), imu: layout.imu_expected })
    setError(null); setNote(null)
  }

  const imuLive = Array.isArray(t.base?.projected_gravity)

  return (
    <div className="max-w-3xl mx-auto space-y-5">
      <div className="card p-5">
        <div className="flex items-center justify-between mb-1">
          <span className="data-label">Robot layout</span>
          <span className="text-[10px] font-mono text-gray-600">{layout.describe}</span>
        </div>
        <p className="text-xs text-gray-500 mb-3 leading-relaxed">
          Tick what is physically attached and powered. This is the joint set the app watches:
          limbs left unticked are ignored entirely, so their joints being offline is not treated
          as a fault. Saved on this machine.
        </p>

        <div className="border-t border-surface-3">
          {layout.limbs.map((limb) => {
            const unknown = limb.unknown_joints?.length > 0
            return (
              <LimbRow
                key={limb.id}
                limb={limb}
                checked={draft.enabled.has(limb.id)}
                disabled={busy || sessionActive || unknown}
                reason={unknown
                  ? `not in the robot config: ${limb.unknown_joints.map((j) => j.replace(/_joint$/, '')).join(', ')}`
                  : null}
                onToggle={toggle}
                buses={t.buses}
              />
            )
          })}
        </div>

        <label className={`flex items-center gap-3 py-3 ${sessionActive ? 'opacity-50' : 'cursor-pointer'}`}>
          <input
            type="checkbox"
            checked={draft.imu}
            disabled={busy || sessionActive}
            onChange={() => setDraft((d) => ({ ...d, imu: !d.imu }))}
            className="accent-accent"
          />
          <div className="flex-1">
            <div className="flex items-center justify-between">
              <span className="text-sm text-gray-200">IMU fitted</span>
              <span className={`text-[10px] font-mono ${imuLive ? 'text-online' : 'text-gray-600'}`}>
                {imuLive ? '● reporting' : '○ no data'}
              </span>
            </div>
            <div className="text-[10px] text-gray-600 mt-0.5">
              Lets the app tell &ldquo;no IMU fitted&rdquo; apart from &ldquo;IMU fitted but silent&rdquo;.
              The wireframe orients the body from it when it is live.
            </div>
          </div>
        </label>

        {sessionActive && (
          <div className="text-xs text-warn mb-2">
            A session is active — disarm before changing the layout.
          </div>
        )}
        {nothingChecked && (
          <div className="text-xs text-warn mb-2">Enable at least one limb.</div>
        )}
        {error && <div className="text-xs text-danger mb-2">{error}</div>}
        {note && <div className="text-xs text-gray-400 mb-2">{note}</div>}

        <div className="flex gap-2 pt-3 border-t border-surface-3">
          <button
            onClick={save}
            disabled={busy || !dirty || sessionActive || nothingChecked}
            className="px-4 py-2 rounded-lg text-sm bg-accent/20 text-accent border border-accent/40
                       disabled:opacity-40 disabled:cursor-not-allowed"
          >
            {busy ? 'Saving…' : 'Save layout'}
          </button>
          <button
            onClick={revert}
            disabled={busy || !dirty}
            className="px-4 py-2 rounded-lg text-sm border border-surface-3 text-gray-400
                       disabled:opacity-40 disabled:cursor-not-allowed"
          >
            Revert
          </button>
        </div>
      </div>

      <div className="card p-4 text-xs text-gray-500 leading-relaxed">
        <div className="data-label mb-2">Motion</div>
        <p>
          <span className="text-gray-300">Hold</span> and <span className="text-gray-300">Run
          policy</span> command the twelve contract leg joints, so they need{' '}
          <span className="text-gray-300">both legs</span> configured — the walk policy is
          contract-bound and there is no single-leg version.
        </p>
        <p className="mt-2">
          <span className="text-gray-300">Arm teleop</span> needs an arm, and drives it from an
          Xbox pad or a Quest headset. <span className="text-gray-300">Manual</span> poses
          whatever the layout enables. An arm-only layout is a fully drivable configuration,
          not just look-and-calibrate.
        </p>
      </div>
    </div>
  )
}
