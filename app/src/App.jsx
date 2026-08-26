import { useEffect, useState } from 'react'
import AuthGate from './components/AuthGate'
import Header from './components/Header'
import Dashboard from './dashboard/Dashboard'
import AddCardDialog from './dashboard/AddCardDialog'
import { useLayouts } from './dashboard/useLayouts'
import { CHILD_TYPES } from './dashboard/GroupCard'
import { TelemetryProvider, useTelemetry } from './context/TelemetryContext'
import { useDeadman } from './hooks/useDeadman'

function TabButton({ tab, active, editing, onSelect, onRename, onDuplicate, onRemove, onMove, canRemove }) {
  const t = useTelemetry()
  const [renaming, setRenaming] = useState(false)
  const uncal = t.joints.length ? t.joints.length - t.joints.filter((j) => j.calibrated).length : 0

  if (renaming) {
    return (
      <input autoFocus defaultValue={tab.name}
        onBlur={(e) => { onRename(e.target.value); setRenaming(false) }}
        onKeyDown={(e) => {
          if (e.key === 'Enter') e.currentTarget.blur()
          if (e.key === 'Escape') { setRenaming(false) }
        }}
        className="px-2 py-1 w-28 bg-surface-2 border border-accent/50 rounded-lg text-sm
                   text-white" />
    )
  }

  return (
    <div className={`group flex items-center rounded-lg transition ${
      active ? 'bg-surface-2' : 'hover:bg-surface-2/50'}`}>
      <button onClick={onSelect}
        onDoubleClick={() => { if (editing) setRenaming(true) }}
        title={editing ? 'Double-click to rename' : undefined}
        className={`px-3 py-2 text-sm ${active ? 'text-white' : 'text-gray-400'}`}>
        {tab.name}
        {tab.badge === 'uncalibrated' && uncal > 0 && (
          <span className="ml-1.5 text-warn">⚠{uncal}</span>
        )}
      </button>
      {editing && (
        <span className="flex items-center pr-1 gap-0.5">
          <MiniBtn label="‹" title="Move left" onClick={() => onMove(-1)} />
          <MiniBtn label="›" title="Move right" onClick={() => onMove(1)} />
          <MiniBtn label="⧉" title="Duplicate tab" onClick={onDuplicate} />
          <MiniBtn label="✕" title={canRemove ? 'Delete tab' : 'The last tab cannot be deleted'}
                   onClick={onRemove} danger disabled={!canRemove} />
        </span>
      )}
    </div>
  )
}

function MiniBtn({ label, title, onClick, danger, disabled }) {
  return (
    <button title={title} disabled={disabled} onClick={onClick}
      className={`text-[10px] leading-none px-1 py-0.5 rounded disabled:opacity-30
                  disabled:cursor-not-allowed ${
        danger ? 'text-danger/70 hover:text-danger hover:bg-danger/10'
               : 'text-gray-500 hover:text-gray-200 hover:bg-surface-3'}`}>
      {label}
    </button>
  )
}

function Shell() {
  // One deadman socket for the whole page (heartbeat while visible; drop → server E-STOP).
  const deadmanConnected = useDeadman()
  const L = useLayouts()
  const [tabId, setTabId] = useState(() => L.layout.tabs[0]?.id)
  const [editing, setEditing] = useState(false)
  // null = closed; {} = adding to the tab; {groupKey} = adding into that group
  const [adding, setAdding] = useState(null)

  const tab = L.layout.tabs.find((t) => t.id === tabId) || L.layout.tabs[0]

  // A deleted tab must not leave the view pointing at nothing.
  useEffect(() => {
    if (!L.layout.tabs.some((t) => t.id === tabId)) setTabId(L.layout.tabs[0]?.id)
  }, [L.layout.tabs, tabId])

  const restore = () => {
    if (confirm('Restore the default layout? Every tab and card you have added will be lost.')) {
      L.restoreDefaults()
      setTabId('control')
    }
  }

  return (
    <div className="h-screen flex flex-col">
      {/* Fixed, outside the grid: E-STOP must be in the same place on every tab, and no layout
          edit should be able to move, shrink or bury it. */}
      <Header deadmanConnected={deadmanConnected} />

      <div className="flex items-center gap-1 px-5 pt-3 border-b border-surface-3 overflow-x-auto">
        {L.layout.tabs.map((tb) => (
          <TabButton key={tb.id} tab={tb} active={tb.id === tab?.id} editing={editing}
            canRemove={L.layout.tabs.length > 1}
            onSelect={() => setTabId(tb.id)}
            onRename={(name) => L.renameTab(tb.id, name)}
            onDuplicate={() => L.duplicateTab(tb.id)}
            onRemove={() => L.removeTab(tb.id)}
            onMove={(d) => L.moveTab(tb.id, d)} />
        ))}
        {editing && (
          <MiniBtn label="+ tab" title="Add a tab" onClick={() => L.addTab('New tab')} />
        )}
        <span className="flex-1" />
        <button onClick={() => setEditing((v) => !v)}
          title={editing ? 'Leave edit mode — cards lock in place'
                         : 'Move, resize, add and remove cards and tabs'}
          className={`px-2.5 py-1 text-xs rounded-md border transition shrink-0 ${
            editing ? 'bg-accent/20 text-accent border-accent/40'
                    : 'border-surface-3 text-gray-500 hover:text-gray-300'}`}>
          {editing ? '✓ Done' : '✎ Edit layout'}
        </button>
      </div>

      {editing && (
        <div className="px-5 py-1.5 bg-accent/10 border-b border-accent/20 text-[11px]
                        text-accent flex items-center gap-3 flex-wrap">
          <button onClick={() => setAdding({})}
            className="px-2 py-0.5 rounded border border-accent/40 hover:bg-accent/15">
            + Add card
          </button>
          <span className="flex-1">
            Drag a card by its blue header · resize from the bottom-right · ⚙ settings, ⧉ duplicate,
            ✕ remove · double-click a tab to rename. Controls are inert while editing.
          </span>
          <button onClick={restore} className="text-accent/70 hover:text-accent underline">
            Restore default layout
          </button>
        </div>
      )}

      <main className="flex-1 overflow-y-auto p-5">
        <div className="max-w-[1600px] mx-auto">
          <Dashboard
            tab={tab} editing={editing} renderProps={{ deadmanConnected }}
            onSaveGrid={L.saveGrid}
            onRemoveCard={L.removeCard}
            onDuplicateCard={L.duplicateCard}
            onUpdateCard={L.updateCard}
            onAddChild={(tid, groupKey) => setAdding({ groupKey })}
            onPopOutChild={L.popOutChild} />
        </div>
      </main>

      <AddCardDialog
        open={!!adding}
        title={adding?.groupKey ? 'Add card to group' : 'Add card'}
        // A group only accepts the child types it knows how to render as rows.
        filter={adding?.groupKey ? (w) => CHILD_TYPES.includes(w.id) : undefined}
        present={(tab?.cards || []).map((c) => c.type)}
        onPick={(type) => {
          if (adding?.groupKey) L.addChild(tab.id, adding.groupKey, type)
          else L.addCard(tab.id, type)
        }}
        onClose={() => setAdding(null)} />
    </div>
  )
}

export default function App() {
  return (
    <AuthGate>
      <TelemetryProvider>
        <Shell />
      </TelemetryProvider>
    </AuthGate>
  )
}
