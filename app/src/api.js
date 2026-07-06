// Same-origin API client. The web build is served by the FastAPI backend at '/', so BASE=''
// works from any PC on the LAN. In dev (`npm run dev`), vite proxies /api /auth /ws to :8000.
const BASE = ''
const TOKEN_KEY = 'humanoid_token'

export function getToken() {
  try { return localStorage.getItem(TOKEN_KEY) } catch { return null }
}
export function setToken(t) {
  try { t ? localStorage.setItem(TOKEN_KEY, t) : localStorage.removeItem(TOKEN_KEY) } catch { /* ignore */ }
}

export function wsUrl(path) {
  const token = getToken()
  const q = token ? `?token=${encodeURIComponent(token)}` : ''
  const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  return `${proto}//${window.location.host}${path}${q}`
}

async function request(path, options = {}) {
  const token = getToken()
  const headers = { 'Content-Type': 'application/json', ...(options.headers || {}) }
  if (token) headers['Authorization'] = `Bearer ${token}`
  const res = await fetch(`${BASE}${path}`, { ...options, headers })
  if (res.status === 401 && !path.startsWith('/auth/')) {
    setToken(null)
    window.dispatchEvent(new Event('humanoid-auth-expired'))
    throw new Error('session expired — please log in again')
  }
  const ct = res.headers.get('content-type') ?? ''
  if (!ct.includes('application/json')) {
    throw new Error(`HTTP ${res.status}: unexpected response "${ct}"`)
  }
  const json = await res.json()
  if (!json.success) throw new Error(json.error || `HTTP ${res.status}`)
  return json.data
}

export const api = {
  // auth
  getAuthStatus: () => request('/auth/status'),
  login: async (password) => {
    const data = await request('/auth/login', { method: 'POST', body: JSON.stringify({ password }) })
    setToken(data?.token ?? null)
    return data
  },
  logout: () => setToken(null),

  // read-only
  getStatus: () => request('/api/status'),
  getPolicies: () => request('/api/policies'),

  // connection lifecycle
  connect: () => request('/api/connect', { method: 'POST' }),
  disconnect: () => request('/api/disconnect', { method: 'POST' }),

  // arming ("I am present / robot supported")
  arm: () => request('/api/arm', { method: 'POST' }),
  disarm: () => request('/api/disarm', { method: 'POST' }),

  // motion
  hold: (ramp = 5.0, seconds = null) =>
    request('/api/hold', { method: 'POST', body: JSON.stringify({ ramp, seconds }) }),
  runPolicy: (checkpoint, command = null, ramp = 5.0, seconds = null) =>
    request('/api/run_policy', { method: 'POST', body: JSON.stringify({ checkpoint, command, ramp, seconds }) }),
  stop: () => request('/api/stop', { method: 'POST' }),
  estop: () => request('/api/estop', { method: 'POST' }),

  // position_offset calibration (per joint)
  calStart: (joint) => request(`/api/calibrate/${joint}/start`, { method: 'POST' }),
  calCapture: (joint, which) =>
    request(`/api/calibrate/${joint}/capture`, { method: 'POST', body: JSON.stringify({ which }) }),
  calApply: (joint) => request(`/api/calibrate/${joint}/apply`, { method: 'POST' }),
  calReset: (joint) => request(`/api/calibrate/${joint}/reset`, { method: 'POST' }),
  calComplete: () => request('/api/calibrate/complete', { method: 'POST' }),

  // manual control + saved poses (values in degrees)
  getPoses: () => request('/api/poses'),
  savePose: (name, joints) =>
    request(`/api/poses/${encodeURIComponent(name)}`, { method: 'PUT', body: JSON.stringify({ joints }) }),
  deletePose: (name) => request(`/api/poses/${encodeURIComponent(name)}`, { method: 'DELETE' }),
  getCurrentPose: () => request('/api/pose/current'),
  captureHold: (ramp = 4.0) =>
    request('/api/manual/capture_hold', { method: 'POST', body: JSON.stringify({ ramp }) }),
  gotoPose: ({ pose = null, target = null, ramp = 4.0 }) =>
    request('/api/manual/goto', { method: 'POST', body: JSON.stringify({ pose, target, ramp }) }),
}
