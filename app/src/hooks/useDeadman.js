import { useEffect, useRef, useState } from 'react'
import { wsUrl } from '../api'

// The browser side of the deadman. Opens /ws/control and sends a heartbeat every 250 ms while
// the tab is visible. If this socket drops, or the tab is closed/backgrounded during a motion
// session, the server stops seeing heartbeats and E-STOPs the robot. This is the wireless
// replacement for the CLI keyboard-kill — keep the control page open and focused while moving.
const HEARTBEAT_MS = 250

export function useDeadman() {
  const [connected, setConnected] = useState(false)
  const wsRef = useRef(null)
  const beatTimer = useRef(null)
  const reconnectTimer = useRef(null)

  useEffect(() => {
    let cancelled = false

    function startBeat(ws) {
      clearInterval(beatTimer.current)
      beatTimer.current = setInterval(() => {
        if (ws.readyState === WebSocket.OPEN && document.visibilityState === 'visible') {
          try { ws.send('ping') } catch { /* ignore */ }
        }
      }, HEARTBEAT_MS)
    }

    function connect() {
      if (cancelled) return
      const ws = new WebSocket(wsUrl('/ws/control'))
      wsRef.current = ws
      ws.onopen = () => { setConnected(true); ws.send('ping'); startBeat(ws) }
      ws.onclose = () => {
        setConnected(false)
        clearInterval(beatTimer.current)
        if (!cancelled) reconnectTimer.current = setTimeout(connect, 1000)
      }
      ws.onerror = () => ws.close()
    }

    // Closing the tab/navigating away drops the socket → server trips deadman if mid-motion.
    const onUnload = () => { try { wsRef.current?.close() } catch { /* ignore */ } }
    window.addEventListener('beforeunload', onUnload)

    connect()
    return () => {
      cancelled = true
      window.removeEventListener('beforeunload', onUnload)
      clearInterval(beatTimer.current)
      clearTimeout(reconnectTimer.current)
      const ws = wsRef.current
      if (ws) { ws.onclose = null; ws.onerror = null; ws.close() }
    }
  }, [])

  return connected
}
