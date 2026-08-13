import { useEffect, useRef, useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'
import SessionBar from './SessionBar'

// Manual control (not policy): capture-and-hold the current pose, and move to saved poses.
// Gated by Arm + a live deadman, but NOT by calibration — capture-hold holds the live measured
// pose, which is valid in whatever frame the encoders currently report (works uncalibrated).
export default function ManualPanel({ deadmanConnected }) {
  const t = useTelemetry()
  const [poses, setPoses] = useState([])
  const [busy, setBusy] = useState(null)
  const [error, setError] = useState(null)
  const [editor, setEditor] = useState(null)   // {name, values:{joint:deg}, isNew}

  const motion = t.state === 'HOLDING' || t.state === 'RUNNING'
  const canMove = t.state === 'CONNECTED' && t.armed
  const jointNames = t.joints.map((j) => j.name)

  // Point the gamepad deadman at MANUAL capture-hold while this tab is open: the controller's
  // A-button then arms manual (no calibration), and holding a trigger locks the live pose.
  // Re-assert on each entry to CONNECTED; select_session is rejected mid-session, so guard on it.
  const gp = t.gamepad || {}
  const syncedRef = useRef(false)
  useEffect(() => {
    if (t.state !== 'CONNECTED') { syncedRef.current = false; return }
    if (syncedRef.current) return
    syncedRef.current = true
    api.deadmanSelect('manual').catch(() => { syncedRef.current = false })
  }, [t.state])

  async function refresh() {
    try { const d = await api.getPoses(); setPoses(d.poses || []) } catch (e) { setError(e.message) }
  }
  useEffect(() => { refresh() }, [])

  async function run(name, fn) {
    setBusy(name); setError(null)
    try { return await fn() } catch (e) { setError(e.message); throw e } finally { setBusy(null) }
  }

  const captureHold = () => run('capture', () => api.captureHold()).catch(() => {})
  const stop = () => run('stop', () => api.stop()).catch(() => {})
  const goTo = (name) => run(`goto:${name}`, () => api.gotoPose({ pose: name })).catch(() => {})
  const del = (name) => run(`del:${name}`, async () => { await api.deletePose(name); await refresh() }).catch(() => {})
  const openEdit = (p) => setEditor({ name: p.name, values: { ...p.resolved_deg }, isNew: false })
  const newBlank = () => setEditor({ name: '', values: Object.fromEntries(jointNames.map((n) => [n, 0])), isNew: true })
  const newFromCurrent = () => run('current', async () => {
    const d = await api.getCurrentPose()
    setEditor({ name: '', values: { ...d.joints_deg }, isNew: true })
  }).catch(() => {})
  const save = () => run('save', async () => {
    if (!editor.name.trim()) throw new Error('pose name required')
    await api.savePose(editor.name.trim(), editor.values); setEditor(null); await refresh()
  }).catch(() => {})

  return (
    <div className="card p-4 space-y-4">
      <span className="data-label">Manual control</span>
      <SessionBar deadmanConnected={deadmanConnected} />

      {/* Gamepad — drive manual hold from the controller (arms + holds without calibration) */}
      <div className="border-t border-surface-3 pt-3 space-y-1">
        <div className="flex items-center justify-between">
          <span className="data-label">Gamepad</span>
          <span className={`text-xs ${gp.connected ? 'text-online' : 'text-gray-500'}`}>
            {gp.connected ? (gp.name || 'connected') : 'not connected'}
            {t.selected === 'manual' && gp.run_gate && <span className="text-online"> · holding</span>}
          </span>
        </div>
        <div className="text-[11px] text-gray-500 leading-relaxed">
          <b>A</b> arms manual hold (no calibration) · hold <b>LT/RT</b> to lock the current pose ·
          release → limp · <b>B</b> = E-STOP.
          {t.selected && t.selected !== 'manual' && (
            <span className="text-warn"> Gamepad currently set to “{t.selected}” — reconnect/return to this tab to re-select manual.</span>
          )}
        </div>
        {gp.enabled === false && (
          <div className="text-[11px] text-warn">Gamepad deadman is disabled on the server (HUMANOID_GAMEPAD_ENABLE unset).</div>
        )}
      </div>

      {/* Capture & hold */}
      <div className="border-t border-surface-3 pt-3 space-y-2">
        <div className="data-label">Capture &amp; hold</div>
        <div className="flex gap-2 items-center flex-wrap">
          <button className="btn-primary" disabled={!!busy || !canMove} onClick={captureHold}>
            {busy === 'capture' ? 'Holding…' : 'Capture current pose & hold'}
          </button>
          <button className="btn-ghost" disabled={!!busy || !motion} onClick={stop}>Stop</button>
          <button className="btn-ghost text-xs" disabled={!!busy || t.state === 'DISCONNECTED'} onClick={newFromCurrent}>
            {busy === 'current' ? '…' : 'Save current as pose…'}
          </button>
        </div>
        <div className="text-[11px] text-gray-500">Holds the robot at its current live position (all 12 joints).</div>
      </div>

      {/* Saved poses */}
      <div className="border-t border-surface-3 pt-3 space-y-2">
        <div className="flex items-center justify-between">
          <span className="data-label">Saved poses</span>
          <button className="btn-ghost text-xs" disabled={!!busy} onClick={newBlank}>+ New</button>
        </div>
        {poses.map((p) => (
          <div key={p.name} className="rounded-lg border border-surface-3 bg-surface-2/40 px-3 py-2">
            <div className="flex items-center justify-between gap-2">
              <span className="text-sm text-gray-200">{p.name}</span>
              <div className="flex gap-1.5">
                <button className="btn-primary text-xs" disabled={!!busy || !canMove} onClick={() => goTo(p.name)}>
                  {busy === `goto:${p.name}` ? '…' : 'Go to'}
                </button>
                <button className="btn-ghost text-xs" disabled={!!busy} onClick={() => openEdit(p)}>Edit</button>
                <button className="btn-ghost text-xs" disabled={!!busy} onClick={() => del(p.name)}>Delete</button>
              </div>
            </div>
            <div className="mt-1 text-[11px] text-gray-500 font-mono truncate">
              {Object.entries(p.resolved_deg).map(([n, v]) =>
                `${n.replace(/_joint$/, '').replace(/^left_/, 'L·').replace(/^right_/, 'R·')} ${v.toFixed(0)}°`).join('   ')}
            </div>
          </div>
        ))}
        {poses.length === 0 && <div className="text-xs text-gray-600">No saved poses yet.</div>}
      </div>

      {editor && (
        <PoseEditor editor={editor} setEditor={setEditor} jointNames={jointNames} onSave={save} busy={busy} />
      )}
      {(error || t.last_error) && (
        <div className="text-xs text-danger bg-danger/10 border border-danger/30 rounded-lg px-3 py-2">{error || t.last_error}</div>
      )}
    </div>
  )
}

function PoseEditor({ editor, setEditor, jointNames, onSave, busy }) {
  const setVal = (n, v) => setEditor((e) => ({ ...e, values: { ...e.values, [n]: v } }))
  return (
    <div className="border-t border-surface-3 pt-3 space-y-2">
      <div className="flex items-center gap-2">
        <span className="data-label shrink-0">{editor.isNew ? 'New pose' : `Edit ${editor.name}`}</span>
        {editor.isNew && (
          <input className="bg-surface-2 border border-surface-3 rounded px-2 py-1 text-xs text-gray-200"
            placeholder="pose name" value={editor.name}
            onChange={(e) => setEditor((ed) => ({ ...ed, name: e.target.value }))} />
        )}
      </div>
      <div className="grid grid-cols-2 gap-x-4 gap-y-1">
        {jointNames.map((n) => (
          <label key={n} className="flex items-center justify-between gap-2 text-xs text-gray-400">
            <span className="truncate">{n.replace(/_joint$/, '')}</span>
            <span className="flex items-center gap-1">
              <input type="number" step="1"
                className="w-16 bg-surface-2 border border-surface-3 rounded px-1.5 py-1 text-right text-gray-200"
                value={editor.values[n] ?? 0}
                onChange={(e) => setVal(n, e.target.value === '' ? 0 : Number(e.target.value))} />
              <span className="text-gray-600">°</span>
            </span>
          </label>
        ))}
      </div>
      <div className="flex gap-2">
        <button className="btn-success text-xs" disabled={!!busy || !editor.name.trim()} onClick={onSave}>
          {busy === 'save' ? 'Saving…' : 'Save pose'}
        </button>
        <button className="btn-ghost text-xs" disabled={!!busy} onClick={() => setEditor(null)}>Cancel</button>
      </div>
    </div>
  )
}
