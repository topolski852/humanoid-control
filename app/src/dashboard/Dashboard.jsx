import { useEffect, useMemo, useRef, useState } from 'react'
// v2's default export is a rewrite WITHOUT isDraggable/isResizable — passing them there is
// silently ignored, which lets cards be resized outside edit mode. The `legacy` entry is the
// v1-compatible API and is the one that actually honours those props.
import { ReactGridLayout as GridLayout } from 'react-grid-layout/legacy'
import { GRID_COLS, ROW_HEIGHT, widgetById } from './widgets'
import GroupCard from './GroupCard'
import CardSettings from './CardSettings'

// Movable / resizable dashboard, rendered from card INSTANCES held in the saved layout.
//
// Cards are unchanged inside — the grid only positions them and tells them how much room they
// have. That boundary is why a widget can be added to the catalog without being rewritten.
//
// Edit mode is off by default and drag/resize are disabled with it. On a page with an E-STOP and
// live motion controls, a stray drag on a button would be a genuine hazard, so moving things is
// something you opt into.

/** Faint grid backdrop, shown only in edit mode so the placement targets are visible. */
function GridGuide({ cols, rowHeight, width, margin }) {
  const colW = (width - margin * (cols + 1)) / cols
  return (
    <div className="absolute inset-0 pointer-events-none" aria-hidden="true">
      {Array.from({ length: cols }, (_, i) => (
        <div key={i} className="absolute top-0 bottom-0 bg-accent/[0.04] border-x border-accent/10"
             style={{ left: margin + i * (colW + margin), width: colW }} />
      ))}
      <div className="absolute inset-0"
           style={{
             backgroundImage: `repeating-linear-gradient(to bottom, rgba(59,130,246,0.06) 0 1px, transparent 1px ${rowHeight + margin}px)`,
           }} />
    </div>
  )
}

/** A card whose type this build does not know — a stale layout must not blank the page. */
function UnknownCard({ card, editing, onRemove }) {
  return (
    <div className="card h-full p-3 flex flex-col items-center justify-center gap-2 text-center">
      <span className="text-xs text-warn">Unknown card type</span>
      <span className="text-[10px] font-mono text-gray-500">{card.type}</span>
      {editing && (
        <button onClick={onRemove} className="text-[10px] text-danger hover:underline">remove</button>
      )}
    </div>
  )
}

function CardFrame({ card, widget, editing, size, onRemove, onDuplicate, onChange, children }) {
  const [settingsOpen, setSettingsOpen] = useState(false)
  useEffect(() => { if (!editing) setSettingsOpen(false) }, [editing])

  const body = widget?.bare
    ? children
    : <div className="card-fill h-full overflow-auto">{children}</div>

  return (
    <div className={`h-full relative ${editing ? 'ring-1 ring-accent/40 rounded-xl' : ''}`}>
      {editing && (
        <>
          {/* Only the header starts a drag, so controls inside a card stay reachable. */}
          <div className="widget-drag-handle absolute -top-px left-0 right-0 h-7 z-20 rounded-t-xl
                          bg-accent/20 border-b border-accent/30 cursor-move flex items-center
                          justify-between px-2 gap-1">
            <span className="text-[10px] font-mono text-accent truncate">
              {card.title || widget?.title || card.type}
            </span>
            <span className="flex items-center gap-1 shrink-0">
              {size && size !== 'full' && (
                <span className="text-[9px] text-accent/60">{size}</span>
              )}
              <HeaderBtn label="⚙" title="Card settings"
                         onClick={() => setSettingsOpen((v) => !v)} />
              <HeaderBtn label="⧉" title="Duplicate card" onClick={onDuplicate} />
              <HeaderBtn label="✕" title="Remove card" onClick={onRemove} danger />
            </span>
          </div>
          {/* Controls are inert while editing — a mis-click here could move a robot. */}
          <div className="absolute inset-0 z-10 rounded-xl bg-surface-1/10" />
          {settingsOpen && (
            <div className="absolute inset-0 z-30 pointer-events-none">
              <div className="pointer-events-auto">
                <CardSettings card={card} onChange={onChange}
                              onClose={() => setSettingsOpen(false)} />
              </div>
            </div>
          )}
        </>
      )}
      <div className={editing ? 'pt-7 h-full' : 'h-full'}>{body}</div>
    </div>
  )
}

function HeaderBtn({ label, title, onClick, danger }) {
  return (
    <button title={title}
      // Stop the drag handler seeing this press, or clicking a button starts a drag instead.
      onMouseDown={(e) => e.stopPropagation()}
      onClick={(e) => { e.stopPropagation(); onClick?.() }}
      className={`text-[10px] leading-none px-1 py-0.5 rounded hover:bg-accent/20 ${
        danger ? 'text-danger/80 hover:text-danger' : 'text-accent/80 hover:text-accent'}`}>
      {label}
    </button>
  )
}

export default function Dashboard({
  tab, editing, renderProps,
  onSaveGrid, onRemoveCard, onDuplicateCard, onUpdateCard, onAddChild, onPopOutChild,
}) {
  const wrapRef = useRef(null)
  const [width, setWidth] = useState(1200)

  // react-grid-layout needs a pixel width. Track the container rather than the window so the
  // grid stays correct inside whatever the shell does around it.
  useEffect(() => {
    const el = wrapRef.current
    if (!el) return
    const ro = new ResizeObserver(([e]) => setWidth(Math.max(320, e.contentRect.width)))
    ro.observe(el)
    setWidth(Math.max(320, el.getBoundingClientRect().width))
    return () => ro.disconnect()
  }, [])

  const cards = tab?.cards || []
  const layout = useMemo(() => cards.map((c) => {
    const w = widgetById(c.type)
    return {
      i: c.key, x: c.x ?? 0, y: c.y ?? 0, w: c.w ?? 4, h: c.h ?? 6,
      minW: w?.minW ?? 2, minH: w?.minH ?? 2,
    }
  }), [cards])

  if (cards.length === 0) {
    return (
      <div className="h-64 flex flex-col items-center justify-center gap-2 text-center">
        <span className="text-sm text-gray-500">This tab is empty</span>
        <span className="text-xs text-gray-600">
          {editing ? 'Use “+ Add card” above.' : 'Turn on “Edit layout” to add cards.'}
        </span>
      </div>
    )
  }

  return (
    <div ref={wrapRef} className="relative">
      {editing && <GridGuide cols={GRID_COLS} rowHeight={ROW_HEIGHT} width={width} margin={10} />}
      <GridLayout
        className={`layout ${editing ? 'is-editing' : ''}`}
        layout={layout}
        cols={GRID_COLS}
        rowHeight={ROW_HEIGHT}
        width={width}
        margin={[10, 10]}
        containerPadding={[0, 0]}
        isDraggable={editing}
        isResizable={editing}
        draggableHandle=".widget-drag-handle"
        compactType="vertical"
        onLayoutChange={(l) => { if (editing) onSaveGrid(tab.id, l) }}
        resizeHandles={['se']}
      >
        {cards.map((card) => {
          const widget = widgetById(card.type)
          const l = layout.find((x) => x.i === card.key)

          // Explicit tier wins; 'auto' (the default) falls back to the height heuristic, which
          // is what made shrinking a card degrade it gracefully in the first place.
          const chosen = card.props?.size || 'auto'
          const h = l?.h ?? 99
          const size = !widget?.tiers ? 'full'
            : chosen !== 'auto' ? chosen
            : h < (widget.tiers.mini ?? 0) ? 'mini'
            : h < (widget.tiers.compact ?? 0) ? 'compact'
            : 'full'

          const common = {
            card, widget, editing, size,
            onRemove: () => onRemoveCard(tab.id, card.key),
            onDuplicate: () => onDuplicateCard(tab.id, card.key),
            onChange: (patch) => onUpdateCard(tab.id, card.key, patch),
          }

          let body
          if (!widget) {
            body = <UnknownCard card={card} editing={editing}
                                onRemove={() => onRemoveCard(tab.id, card.key)} />
          } else if (widget.container) {
            // Containers need the layout mutators, so the Dashboard renders them rather than
            // the catalog's `render`.
            body = (
              <GroupCard
                card={card} editing={editing}
                onAddChild={(k) => onAddChild(tab.id, k)}
                onPopOutChild={(gk, ck, discard) => onPopOutChild(tab.id, gk, ck, discard)}
              />
            )
          } else {
            body = widget.render({ ...renderProps, size, props: card.props || {} })
          }

          return (
            <div key={card.key}>
              <CardFrame {...common}>{body}</CardFrame>
            </div>
          )
        })}
      </GridLayout>
    </div>
  )
}
