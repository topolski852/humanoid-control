// Small labelled status pill. `tone` picks the dot color.
const TONE = {
  online: 'bg-online',
  offline: 'bg-offline',
  danger: 'bg-danger',
  warn: 'bg-warn',
  accent: 'bg-accent',
}

export default function StatusDot({ tone = 'offline', label, value }) {
  return (
    <div className="flex items-center gap-2">
      <span className={`w-2.5 h-2.5 rounded-full ${TONE[tone] || TONE.offline}`} />
      <span className="data-label">{label}</span>
      {value != null && <span className="text-xs text-gray-300 font-mono">{value}</span>}
    </div>
  )
}
