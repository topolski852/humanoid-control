import { useEffect, useMemo, useState } from 'react'
import { WIDGETS } from './widgets'

// The card picker. Used both for adding to a tab and, filtered, for adding into a group.
//
// Shows the whole catalog grouped by category rather than hiding what is already placed:
// duplicates are a feature here (two joint cards, two Joints tables at different detail), so an
// already-present card is annotated, not disabled.

export default function AddCardDialog({ open, onClose, onPick, filter, title = 'Add card', present = [] }) {
  const [q, setQ] = useState('')

  useEffect(() => {
    if (!open) return
    setQ('')
    const onKey = (e) => { if (e.key === 'Escape') onClose() }
    document.addEventListener('keydown', onKey)
    return () => document.removeEventListener('keydown', onKey)
  }, [open, onClose])

  const groups = useMemo(() => {
    const needle = q.trim().toLowerCase()
    const list = WIDGETS
      .filter((w) => (filter ? filter(w) : true))
      .filter((w) => !needle
        || w.title.toLowerCase().includes(needle)
        || (w.description || '').toLowerCase().includes(needle))
    const out = new Map()
    for (const w of list) {
      const g = w.group || 'Other'
      if (!out.has(g)) out.set(g, [])
      out.get(g).push(w)
    }
    return [...out.entries()]
  }, [q, filter])

  if (!open) return null

  const count = present.reduce((m, t) => ({ ...m, [t]: (m[t] || 0) + 1 }), {})

  return (
    <div className="fixed inset-0 z-50 bg-black/60 flex items-start justify-center pt-24 px-4"
         onMouseDown={onClose}>
      <div className="card w-full max-w-lg max-h-[70vh] flex flex-col overflow-hidden"
           onMouseDown={(e) => e.stopPropagation()}>
        <div className="px-4 py-3 border-b border-surface-3 flex items-center justify-between">
          <span className="data-label">{title}</span>
          <button onClick={onClose} className="text-gray-500 hover:text-gray-300 text-sm">✕</button>
        </div>

        <div className="px-4 py-2 border-b border-surface-3">
          <input autoFocus value={q} onChange={(e) => setQ(e.target.value)}
            placeholder="Search cards…"
            className="w-full bg-surface-2 border border-surface-3 rounded px-2 py-1.5 text-sm
                       text-gray-200" />
        </div>

        <div className="flex-1 overflow-auto p-2">
          {groups.length === 0 && (
            <p className="text-xs text-gray-600 p-4 text-center">No cards match “{q}”.</p>
          )}
          {groups.map(([group, list]) => (
            <div key={group} className="mb-3">
              <div className="data-label px-2 py-1">{group}</div>
              {list.map((w) => (
                <button key={w.id} onClick={() => { onPick(w.id); onClose() }}
                  className="w-full text-left px-2 py-2 rounded-lg hover:bg-surface-2
                             transition flex items-start gap-2">
                  <div className="flex-1 min-w-0">
                    <div className="text-sm text-gray-200">
                      {w.title}
                      {count[w.id] > 0 && (
                        <span className="ml-2 text-[10px] text-gray-600">
                          {count[w.id]} already here
                        </span>
                      )}
                    </div>
                    {w.description && (
                      <div className="text-[11px] text-gray-500 leading-snug">{w.description}</div>
                    )}
                  </div>
                  <span className="text-accent text-sm">+</span>
                </button>
              ))}
            </div>
          ))}
        </div>
      </div>
    </div>
  )
}
