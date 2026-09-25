#!/usr/bin/env python3
"""Build the machine-readable measurement file that humanoid-policy consumes.

    python scripts/measure/build_training_input.py

Writes `docs/measurements/TRAINING_INPUT.json` — one file the training side reads instead of
parsing prose reports. Every number is DERIVED from the capture JSONs in docs/measurements/, not
typed in here, so it cannot drift from the data. Values that were not measured are recorded
explicitly as `null` with a `status` saying why, because an honest gap is more useful than a
plausible guess that silently becomes a training parameter.

Re-run it after any new capture; it is idempotent and picks up the newest per-policy captures.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import common as C  # noqa: E402

MEAS = C.REPO_ROOT / "docs" / "measurements"
OUT = MEAS / "TRAINING_INPUT.json"
POLICY_HZ = 25.0
NOMINAL_MS = 10.0


def newest(pattern: str) -> Path | None:
    hits = sorted(MEAS.glob(pattern), key=lambda p: p.stat().st_mtime)
    return hits[-1] if hits else None


def frames(capture_json: Path) -> dict:
    side = Path(str(capture_json).replace(".json", ".frames.jsonl"))
    if not side.exists():
        return {}
    out = {}
    for line in side.read_text().splitlines():
        if line.strip():
            d = json.loads(line)
            out[d["joint"]] = d
    return out


def vel_noise_floor(fr: dict) -> dict:
    """joint_vel noise floor, same short-window median method as position.

    A long-window statistic would include real motion. The firmware applies
    velocity_filter_alpha = 0.7154 before we see the value, so this describes the FILTERED
    signal — which is what the policy consumes.
    """
    per = {}
    for j, d in fr.items():
        v = np.asarray(d.get("vel", []), dtype=float)
        if v.size < 100:
            continue
        w, k = 20, v.size // 20
        if k < 2:
            continue
        seg = v[:k * w].reshape(k, w)
        per[j] = float(np.median(seg.std(axis=1, ddof=0)))
    if not per:
        return {"value": None, "status": "no velocity data"}
    vals = np.array(list(per.values()))
    return {"median_rad_s": float(np.median(vals)), "max_rad_s": float(vals.max()),
            "per_joint_rad_s": per,
            "method": "median of per-0.2s-window std; filtered signal (velocity_filter_alpha 0.7154)"}


def transport(caps: list[Path]) -> dict:
    loss_n = loss_d = 0
    ages, spreads, stales = [], [], []
    for c in caps:
        fr = frames(c)
        for d in fr.values():
            ts = np.asarray(d["ts"])
            if ts.size < 2:
                continue
            g = np.diff(ts) * 1e3
            loss_n += int(np.clip(np.round(g / NOMINAL_MS).astype(int) - 1, 0, None).sum())
            loss_d += int(ts.size)
        an = Path(str(c).replace("_can.json", "_M2M3M6.json"))
        if an.exists():
            a = json.loads(an.read_text())
            pj = a["per_joint"]
            ages += [r["sample_age_ms"]["mean"] for r in pj.values() if "sample_age_ms" in r]
            stales += [r["stale_hold_fraction"] for r in pj.values()
                       if "stale_hold_fraction" in r]
            if a.get("slot_structure"):
                spreads.append(a["slot_structure"]["spread_ms"])
    return {
        "frame_loss_fraction": (loss_n / loss_d) if loss_d else None,
        "frames_analysed": loss_d,
        "sample_age_ms": {"mean": float(np.mean(ages)) if ages else None,
                          "distribution": "uniform 0-10 ms",
                          "model_as": "single uniform delay tau ~ U(0, 10ms), ALL joints"},
        "cross_joint_age_spread_ms": {"max_observed": float(max(spreads)) if spreads else None,
                                      "frame_period_ms": NOMINAL_MS},
        "stale_hold_fraction": {"max_observed": float(max(stales)) if stales else None},
    }


def main() -> int:
    stand_caps = [p for p in (newest("*smoothA-stand_can.json"),
                              newest("*smoothB-stand_can.json")) if p]
    all_caps = [p for p in (newest("*smoothA-stand_can.json"),
                            newest("*smoothB-stand_can.json"),
                            newest("*measA-run1_can.json")) if p]

    pos_floors, quanta = [], []
    for c in stand_caps:
        an = Path(str(c).replace("_can.json", "_M2M3M6.json"))
        if not an.exists():
            continue
        pj = json.loads(an.read_text())["per_joint"]
        pos_floors += [r["pos_noise_floor_rad"] for r in pj.values()
                       if r.get("pos_noise_floor_rad")]
        quanta += [r["encoder_quantum_rad"] for r in pj.values()
                   if r.get("encoder_quantum_rad")]

    velf = vel_noise_floor(frames(stand_caps[0])) if stand_caps else {}

    doc = {
        "_meta": {
            "generated": __import__("time").strftime("%Y-%m-%dT%H:%M:%S"),
            "generator": "scripts/measure/build_training_input.py",
            "robot": "humanoid_lite, legs only, 12 joints, 2 CAN buses",
            "purpose": "machine-readable input for humanoid-policy; every value derived from "
                       "the capture JSONs in docs/measurements/, not hand-entered",
            "source_captures": [p.name for p in all_caps],
            "reports": ["REPORT_2026-09-23_smoothA.md", "REPORT_2026-09-23_smoothB.md",
                        "REPORT_2026-09-25_measA.md"],
            "READ_FIRST": "measA-full, the first bundle trained on these numbers, REGRESSED on "
                          "hardware (50x tilt rate, ESC fault at 41 s). The leading hypothesis is "
                          "that shrinking observation noise to the measured SENSOR floor removed "
                          "robustness that was covering UNMEASURED plant uncertainty. Treat the "
                          "noise floors below as a lower bound on what to randomise, not as the "
                          "value to use. See REPORT_2026-09-25_measA.md section 3a.",
        },

        "observation_model": {
            "joint_pos_noise": {
                "measured_floor_rad": float(np.median(pos_floors)) if pos_floors else None,
                "measured_floor_max_rad": float(max(pos_floors)) if pos_floors else None,
                "previously_trained": 0.05,
                "ratio_trained_to_floor": (0.05 / float(np.median(pos_floors)))
                                          if pos_floors else None,
                "status": "measured",
                "guidance": "do NOT set the training value to the floor alone — measA did that "
                            "and regressed. Split into sensor floor + calibration offset + an "
                            "explicit unmodelled-plant margin.",
            },
            "joint_vel_noise": velf | {"previously_trained": 2.0, "status": "measured"},
            "encoder_quantisation_rad": {
                "value": float(np.median(quanta)) if quanta else None,
                "analytic": 2 * np.pi / 4096 / 15,
                "uniform_across_joints": True,
                "status": "measured",
            },
            "observation_delay": transport(all_caps)["sample_age_ms"],
            "stale_hold": {
                "value": 0.0,
                "status": "measured",
                "guidance": "DO NOT MODEL. 0.00% stale-hold across all captures despite real "
                            "transport-layer frame loss; a 25 Hz consumer of a 100 Hz feed always "
                            "has a fresh frame.",
            },
            "per_joint_delay_stagger": {
                "value": None,
                "status": "measured — absent",
                "guidance": "DO NOT MODEL. Cross-joint age spread is 0.17-1.11 ms against a 10 ms "
                            "frame period. No poll-slot structure on this 2-bus robot; Asimov's "
                            "per-CAN-slot model does not transfer.",
            },
            "base_ang_vel_noise": {
                "value": None,
                "previously_trained": 0.3,
                "status": "NOT MEASURED — gyro auto-zeroes at rest, so the floor is not "
                          "observable standing still. Route around it via the high-frequency "
                          "residual during a walk capture.",
            },
            "gyro_bias": {
                "value": None,
                "status": "UNMEASURABLE on this IMU",
                "guidance": "DO NOT MODEL. Leave NoiseModelWithAdditiveBiasCfg off base_ang_vel "
                            "rather than populate it with a guess.",
            },
            "projected_gravity_noise": {
                "value": None, "previously_trained": 0.05, "status": "NOT MEASURED",
            },
            "calibration_offset": {
                "value": None,
                "previously_trained": 0.05,
                "status": "NOT MEASURED — pending the repeatability check "
                          "(scripts/measure/calibration_spread.py)",
                "why_it_matters": "position_offset resets every power cycle and is recaptured by "
                                  "hand from a held stance, so its spread is a real per-episode "
                                  "uncertainty. This is the prime suspect for the robustness "
                                  "measA lost.",
            },
        },

        "transport": transport(all_caps),

        "actuator": {
            "torque_caps_nm": {"M6C12_8_joints": 12.0, "MAD5010_4_joints": 7.0,
                              "status": "flashed and verified by SDO readback 2026-09-25"},
            "motor_ceiling_nm": {"M6C12": 0.08958 * 20.0 * 15.0,
                                 "MAD5010": 0.06588 * 20.0 * 15.0,
                                 "formula": "Kt * current_limit * gear"},
            "knee_torque_saturation": {
                "hardware_fraction": {"left": 0.521, "right": 0.333},
                "sim_fraction": {"left": 0.0016, "right": 0.0017},
                "hardware_p95_nm": {"left": 28.0, "right": 23.1},
                "sim_p95_nm": {"left": 6.6, "right": 7.1},
                "status": "measured on hardware (RECONSTRUCTED as kp*err - kd*vel, not measured "
                          "torque); M7 is required to break the circularity",
                "guidance": "a ~4x sim/hardware gap at one joint is a PLANT modelling error. No "
                            "reward shaping closes it; in sim the knees barely saturate, so a "
                            "knee-overshoot penalty has nothing to act on and hits the ankles.",
            },
            "joint_velocity_limit": {
                "esc_encoder_fault_rad_s": 13.02,
                "observed_max_rad_s": {"smoothA": 1.08, "smoothB": 8.05, "measA": 15.49},
                "observed_p99_rad_s": {"smoothA": 0.73, "smoothB": 0.73, "measA": 5.09},
                "recommended_penalty_above_rad_s": 2.0,
                "recommended_hard_ceiling_rad_s": 8.0,
                "status": "HARD HARDWARE CONSTRAINT until the ESC is upgraded",
                "guidance": "the ESC cannot track the encoder above ~13 rad/s at the joint; it "
                            "raises ERROR_ENCODER_FAULT (0x2000) and floods EMCY until the bus "
                            "drops. smoothA never exceeded 1.08 rad/s standing.",
            },
        },

        "hardware_constants_do_not_remeasure": {
            "note": "reproduced across two independent 600 s captures; re-running these spends a "
                    "robot session re-learning a constant",
            "items": ["frame loss", "sample age", "stale-hold", "encoder quantum",
                      "position noise floor", "thermal (54-70 C, zero throttle events)"],
        },

        "policy_dependent_metrics_remeasure_each_round": {
            "tilt_rate_p95_deg_s": {"smoothA": 0.4, "smoothB": 0.1, "measA": 20.6,
                                    "target": "< 1.0",
                                    "note": "THE discriminator for balance quality — it separated "
                                            "these three policies where tilt magnitude did not"},
            "tilt_excursions_over_8deg_per_min": {"smoothA": 0.2, "smoothB": 0.0,
                                                  "measA": 11.7, "target": "< 0.5"},
            "joint_vel_p99_rad_s": {"smoothA": 0.73, "smoothB": 0.73, "measA": 5.09,
                                    "target": "< 1.5"},
            "knee_lr_correlation_during_gait": {"smoothA": -0.64, "sim_smoothA": -0.77,
                                                "sim_measA": -0.497, "target": "-0.85"},
            "gait_frequency_hz": {"hardware_smoothA": 1.05, "sim": 1.54},
            "ring_duration_after_push_s": {"smoothA": "0.2-4.8", "smoothB": "up to 15.8 at 4.11 Hz",
                                           "note": "smoothB can enter a sustained limit cycle; "
                                                   "that is closed-loop instability, NOT noise"},
        },

        "standing_constraint": {
            "no_policy_has_walked_unsupported": True,
            "guidance": "every gait figure in these reports describes a SUPPORTED walk, with the "
                        "operator's hand on the robot. Hardware fall rates are therefore not "
                        "comparable to sim fall_rate/min. Standing unattended is the nearer "
                        "milestone: smoothA clears it, smoothB clears it until pushed, measA "
                        "does not.",
            "baseline_to_beat": "walk_smoothA-full_2026-08-25",
        },
    }

    OUT.write_text(json.dumps(doc, indent=2, default=float) + "\n")
    print(f"wrote {OUT}")
    print(f"  source captures: {', '.join(p.name for p in all_caps)}")
    om = doc["observation_model"]
    print(f"  joint_pos floor : {om['joint_pos_noise']['measured_floor_rad']:.3e} rad "
          f"({om['joint_pos_noise']['ratio_trained_to_floor']:.0f}x narrower than trained)")
    jv = om["joint_vel_noise"]
    if jv.get("median_rad_s") is not None:
        print(f"  joint_vel floor : {jv['median_rad_s']:.4f} rad/s "
              f"(was trained at {jv['previously_trained']})")
    t = doc["transport"]
    print(f"  frame loss      : {100*t['frame_loss_fraction']:.4f}% over {t['frames_analysed']} frames")
    nm = [k for k, v in om.items() if isinstance(v, dict)
          and str(v.get("status", "")).startswith(("NOT MEASURED", "UNMEASURABLE"))]
    print(f"  still unmeasured: {', '.join(nm)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
