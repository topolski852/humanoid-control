import { useEffect, useState } from 'react'
import { api } from '../api'
import model from '../data/viz_kinematics.json'

// The wireframe's geometry is bundled (viz_kinematics.json, inlined by Vite), but the values
// that decide WHICH robot gets drawn — the device->URDF sign map, default_pose and the
// device-frame limits — are fetched from GET /api/contract so they can never go stale
// against humanoid_control/config.py.
//
// A visualizer whose job is to catch frame and offset mistakes must not itself be capable of
// a silent frame mistake, so the fetched sign map is checked against the copy recorded in the
// bundled model. On a mismatch we return an error instead of a contract, and the view refuses
// to draw rather than showing a confident, wrong robot.

let cache = null      // resolved contract, shared across every mounted component
let inflight = null

function verify(data) {
  const bundled = model.joint_sign
  const live = data.policy_frame_sign
  if (!Array.isArray(live) || live.length !== bundled.length) {
    throw new Error(`contract sign map has ${live?.length ?? 0} entries, model has ${bundled.length}`)
  }
  const bad = bundled
    .map((s, i) => (s === live[i] ? null : `${data.joint_order?.[i] ?? i}: model ${s} vs server ${live[i]}`))
    .filter(Boolean)
  if (bad.length) {
    throw new Error(
      `frame sign map disagrees with the bundled robot model — ${bad.join('; ')}. ` +
      `Regenerate app/src/data/viz_kinematics.json (python scripts/gen_viz_kinematics.py).`,
    )
  }
  const order = data.joint_order ?? []
  const mismatched = model.joint_order.filter((n, i) => order[i] !== n)
  if (mismatched.length) {
    throw new Error(`joint order disagrees with the bundled model at: ${mismatched.join(', ')}`)
  }
  return data
}

function load() {
  if (cache) return Promise.resolve(cache)
  if (!inflight) {
    inflight = api
      .getContract()
      .then((data) => {
        cache = verify(data)
        return cache
      })
      .finally(() => { inflight = null })
  }
  return inflight
}

/**
 * @returns {{model: object, contract: object|null, error: string|null, loading: boolean}}
 *   `model` is always available (bundled); `contract` resolves once the fetch succeeds.
 */
export function useContract() {
  const [state, setState] = useState(() => ({
    contract: cache, error: null, loading: !cache,
  }))

  useEffect(() => {
    if (cache) return
    let live = true
    load()
      .then((data) => live && setState({ contract: data, error: null, loading: false }))
      .catch((err) => live && setState({ contract: null, error: String(err.message || err), loading: false }))
    return () => { live = false }
  }, [])

  return { model, ...state }
}
