import { createContext, useContext, useEffect, useRef, useState } from 'react'
import { wsUrl } from '../api'

// The full robot snapshot pushed by /ws/telemetry (~20 Hz). `wsConnected` reflects whether the
// backend socket itself is reachable (distinct from `daemon_alive`, the C++ CAN daemon).
const EMPTY = {
  wsConnected: false,
  daemon_alive: false,
  config_present: false,
  state: 'DISCONNECTED',
  armed: false,
  estop: false,
  deadman_ok: false,
  control_clients: 0,
  last_error: null,
  gamepad: { enabled: false, connected: false, name: null, run_gate: false },
  joints: [],
  buses: [],
  base: null,
}

const TelemetryContext = createContext(EMPTY)

export function TelemetryProvider({ children }) {
  const [snap, setSnap] = useState(EMPTY)
  const wsRef = useRef(null)
  const reconnectTimer = useRef(null)

  useEffect(() => {
    let cancelled = false

    function connect() {
      if (cancelled) return
      const ws = new WebSocket(wsUrl('/ws/telemetry'))
      wsRef.current = ws
      ws.onopen = () => setSnap((s) => ({ ...s, wsConnected: true }))
      ws.onmessage = (ev) => {
        try {
          const data = JSON.parse(ev.data)
          setSnap({ ...EMPTY, ...data, wsConnected: true })
        } catch (e) { console.warn('telemetry parse error', e) }
      }
      ws.onclose = () => {
        setSnap(EMPTY)
        if (!cancelled) reconnectTimer.current = setTimeout(connect, 2000)
      }
      ws.onerror = () => ws.close()
    }

    connect()
    return () => {
      cancelled = true
      clearTimeout(reconnectTimer.current)
      const ws = wsRef.current
      if (!ws) return
      ws.onclose = null
      ws.onerror = null
      if (ws.readyState === WebSocket.CONNECTING) ws.onopen = () => ws.close()
      else ws.close()
    }
  }, [])

  return <TelemetryContext.Provider value={snap}>{children}</TelemetryContext.Provider>
}

export const useTelemetry = () => useContext(TelemetryContext)
