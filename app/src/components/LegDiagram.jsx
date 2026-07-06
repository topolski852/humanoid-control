import { useTelemetry } from '../context/TelemetryContext'

// Lightweight two-column leg view: each joint a pill colored by health, with its live angle.
function jointTone(j) {
  if (!j || !j.online) return 'bg-surface-2 text-gray-600 border-surface-3'
  if (j.error) return 'bg-danger/20 text-danger border-danger/40'
  if (j.state === 'ENABLED') return 'bg-online/15 text-online border-online/40'
  return 'bg-surface-2 text-gray-300 border-surface-3'
}

function Column({ title, joints }) {
  return (
    <div className="flex-1">
      <div className="data-label mb-2">{title}</div>
      <div className="space-y-1.5">
        {joints.map((j) => (
          <div key={j.index} className={`flex items-center justify-between px-3 py-1.5 rounded-lg border text-xs ${jointTone(j)}`}>
            <span>{j.name.replace(/^(left|right)_/, '').replace(/_joint$/, '')}</span>
            <span className="font-mono tabular-nums">
              {j.position == null ? '—' : Number(j.position).toFixed(2)}
            </span>
          </div>
        ))}
      </div>
    </div>
  )
}

export default function LegDiagram() {
  const t = useTelemetry()
  const left = t.joints.filter((j) => j.name.startsWith('left_'))
  const right = t.joints.filter((j) => j.name.startsWith('right_'))
  return (
    <div className="card p-4">
      <div className="data-label mb-3">Legs</div>
      <div className="flex gap-4">
        <Column title="Left" joints={left} />
        <Column title="Right" joints={right} />
      </div>
    </div>
  )
}
