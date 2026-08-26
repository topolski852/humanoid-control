import { useEffect, useRef, useState } from 'react'

// Turn a telemetry snapshot into the (N,) device-frame vectors the wireframe draws, where N is
// however many joints the layout has configured — 12 legs, 5 for a bench arm, 22 for the lot.
//
// Two things the raw snapshot does not give us:
//
//  - Offline joints omit `position` ENTIRELY (see ControlService.telemetry_snapshot). Feeding
//    that straight to FK yields NaN and a blank canvas, so we hold the last good value and
//    report which joints are stale. Holding beats zeroing: a dropped CAN frame shouldn't snap
//    a limb to an anatomically impossible pose.
//  - Pausing. The browser may be running on the onboard PC, so freezing the render has to be
//    possible without tearing down the telemetry socket everything else depends on.

/**
 * @param joints  telemetry `joints[]`, in the configured layout order
 * @param opts.paused  freeze the returned pose at its current value
 * @returns {{pose: number[], target: (number|null)[], missing: string[], hasTarget: boolean}}
 */
export function useDevicePose(joints, { paused = false } = {}) {
  const N = joints?.length ?? 0
  // Keyed by joint NAME, not index: a layout change reorders the array, and holding a stale
  // angle from whatever used to sit at index i would draw a limb that never existed.
  const lastGood = useRef(new Map())
  const frozen = useRef(null)
  const [, bump] = useState(0)

  // Re-render once on unpause so the frozen frame is replaced by the live one.
  useEffect(() => {
    if (!paused) {
      frozen.current = null
      bump((n) => n + 1)
    }
  }, [paused])

  if (paused && frozen.current) return frozen.current

  const pose = new Array(N)
  const target = new Array(N)
  const missing = []
  let hasTarget = false

  for (let i = 0; i < N; i++) {
    const j = joints[i]
    const p = j?.position
    if (typeof p === 'number' && Number.isFinite(p)) {
      pose[i] = p
      if (j?.name) lastGood.current.set(j.name, p)
    } else {
      pose[i] = (j?.name ? lastGood.current.get(j.name) : undefined) ?? 0
      if (j) missing.push(j.name)
    }
    const t = j?.target
    if (typeof t === 'number' && Number.isFinite(t)) {
      target[i] = t
      hasTarget = true
    } else {
      target[i] = null
    }
  }

  const result = { pose, target, missing, hasTarget }
  if (paused) frozen.current = result
  return result
}

/**
 * Fill the gaps in a sparse target vector with the live pose, so a partial command (manual
 * mode can drive a subset of joints) still produces a drawable whole-body ghost.
 */
export function mergeTarget(target, pose) {
  return target.map((t, i) => (t == null ? pose[i] : t))
}
