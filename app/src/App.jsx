import AuthGate from './components/AuthGate'
import Header from './components/Header'
import ControlPanel from './components/ControlPanel'
import JointTable from './components/JointTable'
import LegDiagram from './components/LegDiagram'
import { TelemetryProvider } from './context/TelemetryContext'
import { useDeadman } from './hooks/useDeadman'

function Dashboard() {
  // One deadman socket for the whole page (heartbeat while visible; drop → server E-STOPs).
  const deadmanConnected = useDeadman()
  return (
    <div className="h-screen flex flex-col">
      <Header deadmanConnected={deadmanConnected} />
      <main className="flex-1 overflow-y-auto p-5">
        <div className="max-w-6xl mx-auto grid grid-cols-1 lg:grid-cols-3 gap-5">
          <div className="lg:col-span-1 space-y-5">
            <ControlPanel deadmanConnected={deadmanConnected} />
            <LegDiagram />
          </div>
          <div className="lg:col-span-2">
            <JointTable />
          </div>
        </div>
      </main>
    </div>
  )
}

export default function App() {
  return (
    <AuthGate>
      <TelemetryProvider>
        <Dashboard />
      </TelemetryProvider>
    </AuthGate>
  )
}
