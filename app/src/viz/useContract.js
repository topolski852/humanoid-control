import { useEffect, useMemo, useState } from 'react'
import { api } from '../api'
import { useTelemetry } from '../context/TelemetryContext'
import bundled from '../data/viz_kinematics.json'
import { selectModel } from './kinematics'

// The wireframe's geometry is bundled (viz_kinematics.json, inlined by Vite) and describes every
// limb the robot COULD have. Which limbs are actually attached is a runtime fact (the layout),
// and the values that decide WHICH robot gets drawn — the device->URDF sign map, default_pose
// and the device-frame limits — come from GET /api/contract for exactly the configured joints.
//
// A visualizer whose job is to catch frame and offset mistakes must not itself be capable of a
// silent frame mistake, so the fetched sign map is checked, by joint NAME, against the copy
// recorded in the bundled model. On a mismatch we return an error instead of a contract, and the
// view refuses to draw rather than showing a confident, wrong robot.

function verify(data) {
  const order = data.joint_order ?? []
  const live = data.policy_frame_sign
  if (!Array.isArray(live) || live.length !== order.length) {
    throw new Error(
      `contract sign map has ${live?.length ?? 0} entries for ${order.length} joints`,
    )
  }

  const bundledSign = new Map(bundled.joint_order.map((n, i) => [n, bundled.joint_sign[i]]))

  const unknown = order.filter((n) => !bundledSign.has(n))
  if (unknown.length) {
    throw new Error(
      `the server reports joints the bundled robot model does not contain: ${unknown.join(', ')}. ` +
      `Regenerate app/src/data/viz_kinematics.json (python scripts/gen_viz_kinematics.py).`,
    )
  }

  const bad = order
    .map((n, i) => (bundledSign.get(n) === live[i] ? null : `${n}: model ${bundledSign.get(n)} vs server ${live[i]}`))
    .filter(Boolean)
  if (bad.length) {
    throw new Error(
      `frame sign map disagrees with the bundled robot model — ${bad.join('; ')}. ` +
      `Regenerate app/src/data/viz_kinematics.json (python scripts/gen_viz_kinematics.py).`,
    )
  }
  return data
}

// Cached per joint-set: changing the layout changes the contract, so the key has to include it.
const cache = new Map()   // key -> resolved contract
const inflight = new Map()

function load(key) {
  if (cache.has(key)) return Promise.resolve(cache.get(key))
  if (!inflight.has(key)) {
    inflight.set(
      key,
      api
        .getContract()
        .then((data) => {
          const ok = verify(data)
          cache.set(key, ok)
          return ok
        })
        .finally(() => inflight.delete(key)),
    )
  }
  return inflight.get(key)
}

/**
 * @returns {{model: object|null, contract: object|null, error: string|null, loading: boolean}}
 *   `model` is the bundled geometry narrowed to the configured joints, index-aligned with
 *   `contract` and with telemetry `joints[]`. Both are null while loading or on a mismatch.
 */
export function useContract() {
  const t = useTelemetry()
  // Re-fetch whenever the configured limb set changes — the contract is per-layout.
  const key = (t.layout?.enabled ?? []).join(',') || 'default'

  const [state, setState] = useState(() => ({
    contract: cache.get(key) ?? null, error: null, loading: !cache.has(key),
  }))

  useEffect(() => {
    let live = true
    const cached = cache.get(key)
    if (cached) {
      setState({ contract: cached, error: null, loading: false })
      return
    }
    setState((s) => ({ ...s, loading: true }))
    load(key)
      .then((data) => live && setState({ contract: data, error: null, loading: false }))
      .catch((err) => live && setState({ contract: null, error: String(err.message || err), loading: false }))
    return () => { live = false }
  }, [key])

  const model = useMemo(
    () => (state.contract ? selectModel(bundled, state.contract.joint_order) : null),
    [state.contract],
  )

  // selectModel only returns null when a joint is missing from the bundle, which verify()
  // already rejects — but a null model would render as a blank robot, so fail loudly instead.
  const error = state.error || (state.contract && !model ? 'bundled robot model is missing joints the server reports' : null)

  return { model, contract: state.contract, error, loading: state.loading }
}
