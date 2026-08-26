import { useEffect, useRef } from 'react'
import { useTelemetry } from '../context/TelemetryContext'
import { settingsFor, widgetById } from './widgets'
import { shortJointName } from '../components/JointCard'

// Settings for one card, built from the widget's own schema.
//
// The dashboard never knows what a widget's options mean — it renders whatever `settings`
// declares. That is what keeps per-widget configuration from becoming N bespoke features: a new
// option is a catalog entry plus a prop read in the component.
//
// `optionsFrom` fills a select from live data instead of a fixed list, so the joint picker
// offers whatever the configured layout actually has attached rather than a hardcoded set.

function useDynamicOptions(source) {
  const t = useTelemetry()
  if (source !== 'joints') return null
  return (t.joints || []).map((j) => {
    const { core, side } = shortJointName(j.name)
    return { value: j.name, label: side ? `${core} (${side})` : core }
  })
}

export default function CardSettings({ card, onChange, onClose }) {
  const widget = widgetById(card.type)
  const schema = settingsFor(widget)
  const ref = useRef(null)
  const jointOptions = useDynamicOptions('joints')

  // Click-away and Escape both close. A settings popover that traps you is worse than one that
  // closes a moment early — nothing here is destructive.
  useEffect(() => {
    const onDown = (e) => { if (ref.current && !ref.current.contains(e.target)) onClose() }
    const onKey = (e) => { if (e.key === 'Escape') onClose() }
    document.addEventListener('mousedown', onDown)
    document.addEventListener('keydown', onKey)
    return () => {
      document.removeEventListener('mousedown', onDown)
      document.removeEventListener('keydown', onKey)
    }
  }, [onClose])

  const setProp = (key, value) => onChange({ props: { ...(card.props || {}), [key]: value } })

  return (
    <div ref={ref}
         className="absolute z-40 top-7 right-0 w-64 card p-3 space-y-2 shadow-2xl
                    border border-accent/30">
      <div className="flex items-center justify-between">
        <span className="data-label">{widget?.title || card.type}</span>
        <button onClick={onClose} className="text-xs text-gray-500 hover:text-gray-300">✕</button>
      </div>

      <label className="block">
        <span className="text-[10px] text-gray-500">Title</span>
        <input
          value={card.title || ''}
          onChange={(e) => onChange({ title: e.target.value })}
          placeholder={widget?.title || ''}
          className="w-full bg-surface-2 border border-surface-3 rounded px-2 py-1 text-xs
                     text-gray-200"
        />
      </label>

      {schema.map((s) => {
        const options = s.optionsFrom === 'joints' ? (jointOptions || []) : (s.options || [])
        const value = card.props?.[s.key] ?? s.default ?? ''
        if (s.type === 'select') {
          return (
            <label key={s.key} className="block">
              <span className="text-[10px] text-gray-500">{s.label}</span>
              <select value={value} onChange={(e) => setProp(s.key, e.target.value)}
                className="w-full bg-surface-2 border border-surface-3 rounded px-2 py-1 text-xs
                           text-gray-200">
                {s.optionsFrom === 'joints' && <option value="">— pick a joint —</option>}
                {options.length === 0 && s.optionsFrom === 'joints' && (
                  <option disabled>no joints in telemetry</option>
                )}
                {options.map((o) => (
                  <option key={o.value ?? o} value={o.value ?? o}>{o.label ?? o}</option>
                ))}
              </select>
            </label>
          )
        }
        if (s.type === 'toggle') {
          return (
            <label key={s.key} className="flex items-center gap-2 cursor-pointer">
              <input type="checkbox" checked={!!value} className="accent-accent"
                     onChange={(e) => setProp(s.key, e.target.checked)} />
              <span className="text-xs text-gray-300">{s.label}</span>
            </label>
          )
        }
        return null
      })}

      {schema.length === 0 && (
        <p className="text-[10px] text-gray-600">This card has no options yet.</p>
      )}
    </div>
  )
}
