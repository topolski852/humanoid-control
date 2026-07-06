// Display helpers. The backend/contract work in radians; the UI shows degrees.
export const RAD2DEG = 180 / Math.PI

export function deg(rad, d = 1) {
  if (rad == null || Number.isNaN(rad)) return '—'
  return (Number(rad) * RAD2DEG).toFixed(d)
}

export function num(v, d = 2) {
  if (v == null || Number.isNaN(v)) return '—'
  return Number(v).toFixed(d)
}
