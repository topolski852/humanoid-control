import { useState } from 'react'
import AuthGate from './components/AuthGate'
import Header from './components/Header'
import ControlPanel from './components/ControlPanel'
import JointTable from './components/JointTable'
import LegDiagram from './components/LegDiagram'
import ImuPanel from './components/ImuPanel'
import CalibrationPanel from './components/CalibrationPanel'
import ManualPanel from './components/ManualPanel'
import { TelemetryProvider } from './context/TelemetryContext'
import { useTelemetry } from './context/TelemetryContext'
import { useDeadman } from './hooks/useDeadman'

function ControlView({ deadmanConnected }) {
  return (
    <div className="max-w-6xl mx-auto grid grid-cols-1 lg:grid-cols-3 gap-5">
      <div className="lg:col-span-1 space-y-5">
        <ControlPanel deadmanConnected={deadmanConnected} />
        <ImuPanel />
        <LegDiagram />
      </div>
      <div className="lg:col-span-2">
        <JointTable />
      </div>
    </div>
  )
}

function CalibrationView() {
  return (
    <div className="max-w-5xl mx-auto grid grid-cols-1 lg:grid-cols-3 gap-5">
      <div className="lg:col-span-1"><LegDiagram /></div>
      <div className="lg:col-span-2"><CalibrationPanel /></div>
    </div>
  )
}

function ManualView({ deadmanConnected }) {
  return (
    <div className="max-w-5xl mx-auto grid grid-cols-1 lg:grid-cols-3 gap-5">
      <div className="lg:col-span-1"><LegDiagram /></div>
      <div className="lg:col-span-2"><ManualPanel deadmanConnected={deadmanConnected} /></div>
    </div>
  )
}

function Tabs({ view, setView }) {
  const t = useTelemetry()
  const uncal = t.joints.length ? t.joints.length - t.joints.filter((j) => j.calibrated).length : 0
  const tab = (id, label, badge) => (
    <button onClick={() => setView(id)}
      className={`px-4 py-2 text-sm rounded-lg transition ${view === id ? 'bg-surface-2 text-white' : 'text-gray-400 hover:text-gray-200'}`}>
      {label}{badge != null && badge > 0 && <span className="ml-1.5 text-warn">⚠{badge}</span>}
    </button>
  )
  return (
    <div className="flex gap-1 px-5 pt-3 border-b border-surface-3">
      {tab('control', 'Control')}
      {tab('manual', 'Manual')}
      {tab('calibration', 'Calibration', uncal)}
    </div>
  )
}

function Shell() {
  // One deadman socket for the whole page (heartbeat while visible; drop → server E-STOP).
  const deadmanConnected = useDeadman()
  const [view, setView] = useState('control')
  return (
    <div className="h-screen flex flex-col">
      <Header deadmanConnected={deadmanConnected} />
      <Tabs view={view} setView={setView} />
      <main className="flex-1 overflow-y-auto p-5">
        {view === 'control' && <ControlView deadmanConnected={deadmanConnected} />}
        {view === 'manual' && <ManualView deadmanConnected={deadmanConnected} />}
        {view === 'calibration' && <CalibrationView />}
      </main>
    </div>
  )
}

export default function App() {
  return (
    <AuthGate>
      <TelemetryProvider>
        <Shell />
      </TelemetryProvider>
    </AuthGate>
  )
}
