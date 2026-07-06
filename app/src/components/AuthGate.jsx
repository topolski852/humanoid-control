import { useEffect, useState } from 'react'
import { api, getToken } from '../api'

// Gates the app behind a shared password when the backend requires it (HUMANOID_WEB_PASSWORD).
// When auth is disabled (trusted LAN, default) it renders instantly.
export default function AuthGate({ children }) {
  const [status, setStatus] = useState('checking')  // checking | login | ok
  const [password, setPassword] = useState('')
  const [error, setError] = useState(null)
  const [busy, setBusy] = useState(false)

  async function check() {
    try {
      const s = await api.getAuthStatus()
      if (!s.auth_required) { setStatus('ok'); return }
      if (getToken()) {
        try { await api.getStatus(); setStatus('ok'); return }
        catch { setStatus('login'); return }
      }
      setStatus('login')
    } catch {
      setStatus('ok')  // backend unreachable — render; protected calls bounce via 401
    }
  }

  useEffect(() => { check() }, [])
  useEffect(() => {
    const onExpired = () => setStatus('login')
    window.addEventListener('humanoid-auth-expired', onExpired)
    return () => window.removeEventListener('humanoid-auth-expired', onExpired)
  }, [])

  async function submit(e) {
    e.preventDefault()
    setBusy(true); setError(null)
    try { await api.login(password); setPassword(''); setStatus('ok') }
    catch (err) { setError(err.message) }
    finally { setBusy(false) }
  }

  if (status === 'checking') {
    return <div className="h-screen flex items-center justify-center text-sm text-gray-500">Loading…</div>
  }

  if (status === 'login') {
    return (
      <div className="h-screen flex items-center justify-center bg-surface">
        <form onSubmit={submit} className="card p-6 w-80 space-y-4">
          <div className="flex items-center gap-2">
            <span className="text-xl">🤖</span>
            <h1 className="font-semibold text-white">Humanoid Control</h1>
          </div>
          <p className="text-xs text-gray-500">This robot is password protected. Enter the access password.</p>
          <input
            type="password" autoFocus value={password}
            onChange={(e) => setPassword(e.target.value)} placeholder="Password"
            className="w-full bg-surface-2 border border-surface-3 rounded-lg px-3 py-2 text-sm text-gray-200"
          />
          {error && <div className="text-xs text-danger">{error}</div>}
          <button type="submit" disabled={busy || !password} className="btn-primary w-full">
            {busy ? 'Signing in…' : 'Sign in'}
          </button>
        </form>
      </div>
    )
  }

  return children
}
