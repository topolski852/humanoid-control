import { useMemo, useState } from 'react'
import { useTelemetry } from '../context/TelemetryContext'
import { useContract } from '../viz/useContract'
import { useDevicePose } from '../viz/usePose'
import { baseRotationFromGravity, tiltAngle } from '../viz/kinematics'
import { RAD2DEG } from '../format'
import RobotWireframe from './RobotWireframe'
import LegDiagram, { LegColumns } from './LegDiagram'

// Compact wireframe for the side column of the Control / Manual / Calibration tabs — the
// same drawing as the Robot tab, minus the controls and the readout. Useful precisely while
// calibrating, when you want to see a joint's effect on the whole body as you capture it.
//
// Falls back to the LegDiagram pill list if the model can't be trusted, so the column always
// shows something usable.

const AZIMUTH = (35 * Math.PI) / 180

export default function RobotMini() {
  const t = useTelemetry()
  const { model, contract, error } = useContract()
  const [showPills, setShowPills] = useState(false)
  const { pose } = useDevicePose(t.joints)

  const pg = t.base?.projected_gravity
  const imuLive = Array.isArray(pg) && pg.length >= 3 && pg.every(Number.isFinite)
  const baseRotation = useMemo(
    () => baseRotationFromGravity(imuLive ? pg : null),
    [imuLive, pg?.[0], pg?.[1], pg?.[2]],
  )

  if (error) return <LegDiagram />

  return (
    <div className="card p-4">
      <div className="flex items-center justify-between mb-2">
        <span className="data-label">Robot pose</span>
        <div className="flex items-center gap-2">
          {imuLive && (
            <span className="text-[10px] font-mono text-gray-600"
                  title="Body tilt from the IMU (gravity)">
              {(tiltAngle(pg) * RAD2DEG).toFixed(0)}°
            </span>
          )}
          <button
            onClick={() => setShowPills((v) => !v)}
            className="text-[10px] text-gray-600 hover:text-gray-300 transition"
          >
            {showPills ? 'diagram' : 'angles'}
          </button>
        </div>
      </div>
      {showPills || !contract ? (
        <LegColumns />
      ) : (
        <div className="flex justify-center">
          <RobotWireframe
            model={model}
            sign={contract.policy_frame_sign}
            devicePose={pose}
            baseRotation={baseRotation}
            joints={t.joints}
            azimuth={AZIMUTH}
            compact
            className="w-full max-w-[240px]"
          />
        </div>
      )}
    </div>
  )
}
