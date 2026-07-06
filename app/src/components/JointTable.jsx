import { useTelemetry } from '../context/TelemetryContext'
import { deg } from '../format'

function stateTone(j) {
  if (!j.online) return 'text-offline'
  if (j.error) return 'text-danger'
  if (j.state === 'ENABLED') return 'text-online'
  return 'text-gray-300'
}

// Live 12-joint telemetry table in canonical order.
export default function JointTable() {
  const t = useTelemetry()
  return (
    <div className="card overflow-hidden">
      <div className="px-4 py-2.5 border-b border-surface-3 flex items-center justify-between">
        <span className="data-label">Joints (canonical order)</span>
        <span className="data-label">{t.joints.filter((j) => j.online).length}/{t.joints.length} online</span>
      </div>
      <div className="overflow-x-auto">
        <table className="w-full text-sm">
          <thead>
            <tr className="text-left data-label border-b border-surface-3">
              <th className="px-4 py-2 font-medium">#</th>
              <th className="px-2 py-2 font-medium">joint</th>
              <th className="px-2 py-2 font-medium">state</th>
              <th className="px-2 py-2 font-medium">cal</th>
              <th className="px-2 py-2 font-medium text-right">pos (°)</th>
              <th className="px-2 py-2 font-medium text-right">vel (°/s)</th>
              <th className="px-2 py-2 font-medium text-right">τ (Nm)</th>
              <th className="px-4 py-2 font-medium text-right">err</th>
            </tr>
          </thead>
          <tbody>
            {t.joints.map((j) => (
              <tr key={j.index} className="border-b border-surface-2/60 hover:bg-surface-2/40">
                <td className="px-4 py-1.5 font-mono text-gray-500">{j.index}</td>
                <td className="px-2 py-1.5 text-gray-300">{j.name.replace(/_joint$/, '')}</td>
                <td className={`px-2 py-1.5 font-mono ${stateTone(j)}`}>{j.online ? (j.state || 'ON') : 'OFFLINE'}</td>
                <td className="px-2 py-1.5">
                  {j.calibrated
                    ? <span className="text-online" title="calibrated">✓</span>
                    : <span className="text-warn" title="uncalibrated">⚠</span>}
                </td>
                <td className="px-2 py-1.5 text-right data-value">{deg(j.position)}</td>
                <td className="px-2 py-1.5 text-right data-value">{deg(j.velocity)}</td>
                <td className="px-2 py-1.5 text-right data-value">{j.torque == null ? '—' : Number(j.torque).toFixed(2)}</td>
                <td className={`px-4 py-1.5 text-right font-mono ${j.error ? 'text-danger' : 'text-gray-600'}`}>
                  {j.error ? `0x${j.error.toString(16)}` : '—'}
                </td>
              </tr>
            ))}
            {t.joints.length === 0 && (
              <tr><td colSpan={8} className="px-4 py-8 text-center text-gray-600 text-xs">
                No telemetry — is the daemon running?
              </td></tr>
            )}
          </tbody>
        </table>
      </div>
    </div>
  )
}
