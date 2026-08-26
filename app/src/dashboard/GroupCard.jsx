import { JointRow } from '../components/JointCard'

// A container card: holds child cards and stacks them as rows.
//
// Children have no x/y/w/h — the group owns their arrangement. That is what makes a group of
// joint cards reproduce the original table while each child remains an independent, removable
// thing with its own settings.
//
// Children are added from the group's own "+" rather than by dragging a card in.
// react-grid-layout is a flat grid with no nesting; detecting a drop into a container means
// intercepting drag positions and fighting the library's placement. The "+" and the per-child
// pop-out cover the same intent without that fragility.

const CHILD_RENDERERS = {
  joint: (child, group, ctx) => (
    <JointRow
      key={child.key}
      joint={child.props?.joint}
      columns={group.props?.columns || 'full'}
      editing={ctx.editing}
      onPopOut={ctx.editing ? () => ctx.onPopOutChild(group.key, child.key) : undefined}
    />
  ),
}

/** Widget types that make sense inside a group. The picker filters on this. */
export const CHILD_TYPES = Object.keys(CHILD_RENDERERS)

export default function GroupCard({ card, editing, onAddChild, onPopOutChild }) {
  const children = card.children || []
  const ctx = { editing, onPopOutChild }

  return (
    <div className="card h-full flex flex-col overflow-hidden">
      <div className="px-3 py-2 border-b border-surface-3 flex items-center justify-between gap-2">
        <span className="data-label truncate">{card.title || 'Group'}</span>
        <div className="flex items-center gap-2">
          <span className="text-[10px] text-gray-600">{children.length}</span>
          {editing && (
            <button onClick={() => onAddChild(card.key)}
              title="Add a card to this group"
              className="text-xs px-1.5 rounded border border-accent/40 text-accent
                         hover:bg-accent/15">
              +
            </button>
          )}
        </div>
      </div>

      <div className="flex-1 overflow-auto">
        {children.length === 0 ? (
          <div className="h-full flex flex-col items-center justify-center gap-1 text-center px-3">
            <span className="text-xs text-gray-500">Empty group</span>
            <span className="text-[10px] text-gray-600">
              {editing ? 'Use + above to add cards' : 'Turn on Edit layout to add cards'}
            </span>
          </div>
        ) : (
          children.map((child) => {
            const render = CHILD_RENDERERS[child.type]
            if (!render) {
              // A child of a type this build does not know. Say so rather than rendering
              // nothing, so a stale layout is visible instead of silently short.
              return (
                <div key={child.key}
                     className="px-3 py-1.5 text-xs text-warn flex items-center justify-between">
                  <span>unknown card type: {child.type}</span>
                  {editing && (
                    <button onClick={() => onPopOutChild(card.key, child.key, true)}
                      className="text-[10px] text-danger hover:underline">remove</button>
                  )}
                </div>
              )
            }
            return render(child, card, ctx)
          })
        )}
      </div>
    </div>
  )
}
