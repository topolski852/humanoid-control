"""
Arm forward kinematics and differential IK, for Cartesian teleop.

Geometry comes from ``app/src/data/viz_kinematics.json`` — the vendored URDF kinematics that
``scripts/gen_viz_kinematics.py`` generates and ``app/src/viz/kinematics.test.mjs`` checks
against independently-derived goldens. Reading the same artifact here rather than re-parsing the
URDF means the drawing and the controller cannot disagree about the robot's shape, which is a
failure mode worth designing out: an IK that solves against different geometry than the picture
shows is very hard to debug.

Frames are URDF frames: +X forward, +Y robot-left, +Z up, metres and radians. The arm's device
frame equals the URDF frame once calibrated (see the wiki page "Arm Joint Frames and
Calibration"), so joint values here are the same numbers telemetry reports.

**The controlled point is the wrist pivot**, not the fingertip. The last joint is an inline twist
that barely moves the wrist (~0.1 cm/rad), so including the hand would add a near-singular column
to the Jacobian for no useful reach. Grip position is the claw's job, not the arm's.
"""
from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path

import numpy as np

from .config import REPO_ROOT

VIZ_MODEL_PATH = REPO_ROOT / "app" / "src" / "data" / "viz_kinematics.json"


@lru_cache(maxsize=1)
def _load_model(path: str = str(VIZ_MODEL_PATH)) -> dict:
    return json.loads(Path(path).read_text())


def _axis_rotation(axis: np.ndarray, q: float) -> np.ndarray:
    """Rodrigues rotation about a unit axis."""
    x, y, z = axis
    c, s, t = np.cos(q), np.sin(q), 1.0 - np.cos(q)
    return np.array([
        [t * x * x + c,     t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c,     t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])


class ArmChain:
    """One arm's kinematic chain, in URDF frame.

    ``joint_names`` are DEVICE names (``left_shoulder_pitch_joint`` …) in proximal-to-distal
    order — the same order and the same values the telemetry reports.
    """

    def __init__(self, joint_names: list[str] | tuple[str, ...], model: dict | None = None):
        model = model or _load_model()
        by_name = {j["name"]: j for j in model["joints"]}
        missing = [n for n in joint_names if n not in by_name]
        if missing:
            raise KeyError(f"not in the vendored kinematics model: {', '.join(missing)}")
        self.joint_names = tuple(joint_names)
        self._joints = [by_name[n] for n in self.joint_names]
        self._xyz = [np.array(j["xyz"], dtype=float) for j in self._joints]
        self._R = [np.array(j["R"], dtype=float).reshape(3, 3) for j in self._joints]
        self._axis = [np.array(j["axis"], dtype=float) for j in self._joints]
        self.limits_lower = np.array([j["limit"]["lower"] for j in self._joints])
        self.limits_upper = np.array([j["limit"]["upper"] for j in self._joints])

    @property
    def n(self) -> int:
        return len(self._joints)

    # --- forward kinematics ----------------------------------------------
    def frames(self, q: np.ndarray) -> list[np.ndarray]:
        """World 4x4 of each joint frame AFTER its own rotation, base-relative."""
        out = []
        T = np.eye(4)
        for i in range(self.n):
            step = np.eye(4)
            step[:3, :3] = self._R[i]
            step[:3, 3] = self._xyz[i]
            T = T @ step
            rot = np.eye(4)
            rot[:3, :3] = _axis_rotation(self._axis[i], float(q[i]))
            T = T @ rot
            out.append(T.copy())
        return out

    def joint_origins(self, q: np.ndarray) -> np.ndarray:
        """(n,3) world position of each joint's pivot."""
        pts = []
        T = np.eye(4)
        for i in range(self.n):
            step = np.eye(4)
            step[:3, :3] = self._R[i]
            step[:3, 3] = self._xyz[i]
            T = T @ step
            pts.append(T[:3, 3].copy())
            rot = np.eye(4)
            rot[:3, :3] = _axis_rotation(self._axis[i], float(q[i]))
            T = T @ rot
        return np.array(pts)

    def tool(self, q: np.ndarray) -> np.ndarray:
        """World position of the controlled point (the wrist pivot)."""
        return self.joint_origins(q)[-1]

    def shoulder(self, q: np.ndarray | None = None) -> np.ndarray:
        return self.joint_origins(np.zeros(self.n) if q is None else q)[0]

    # --- differential kinematics -----------------------------------------
    def jacobian(self, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        """(3,n) position Jacobian of the tool point, by central differences.

        Numerical rather than analytic on purpose: the analytic form would have to re-derive the
        chain and could drift out of step with `frames()`. At n=5 the cost is 10 FK evaluations
        per tick, which is nothing at 50 Hz.
        """
        J = np.zeros((3, self.n))
        for i in range(self.n):
            dq = np.zeros(self.n)
            dq[i] = eps
            J[:, i] = (self.tool(q + dq) - self.tool(q - dq)) / (2.0 * eps)
        return J

    def ik_step(self, q: np.ndarray, dx: np.ndarray, *,
                damping: float = 0.01,
                posture: np.ndarray | None = None,
                posture_gain: float = 0.05,
                max_step: float = 0.02) -> np.ndarray:
        """Joint delta that moves the tool by ``dx`` (metres, world frame).

        Damped least squares: ``dq = Jᵀ(JJᵀ + λ²I)⁻¹ dx``. The damping is what keeps this stable
        near singularities — a straight arm at full stretch has a direction it cannot move in,
        and an undamped pseudo-inverse responds with an enormous joint velocity. Damping trades
        a little tracking accuracy for never doing that.

        The arm has 4 joints positioning 3 axes, so one degree of redundancy. ``posture`` uses it
        to drift gently toward a preferred pose (projected into the null space, so it never
        fights the commanded motion).

        ``max_step`` caps the per-tick joint delta as a final backstop — at the 50 Hz teleop
        tick the default 0.02 rad works out to ~1 rad/s (57 deg/s) per joint. Near the edge of
        the workspace the joints need ever more travel per millimetre of hand motion, so this
        cap engages and the hand visibly slows instead of the arm lunging. That is the intended
        behaviour, not a tracking failure.

        Defaults were tuned against the real chain: damping 0.01 tracks to ~100% of the
        requested step in open space while holding joint travel to ~2 deg per tick at full
        extension (undamped, the same command produces ~18 deg).
        """
        J0 = self.jacobian(q)
        lam2 = damping * damping
        free = np.ones(self.n, dtype=bool)
        dq = np.zeros(self.n)

        # CLAMPING LOOP. Solve, find any joint the solution would push past its limit, LOCK it
        # at the limit, and re-solve for what is left of the motion using only the joints that
        # can still help. Clamping after a single solve instead would silently throw away that
        # joint's share of the command, so the hand stalls when it could have slid along the
        # constraint using the others. A few passes is plenty for a 5-joint chain.
        for _ in range(self.n):
            Jf = J0[:, free]
            if Jf.shape[1] == 0:
                break
            residual = dx - J0 @ dq
            JJt = Jf @ Jf.T + lam2 * np.eye(3)
            step = Jf.T @ np.linalg.solve(JJt, residual)

            if posture is not None and posture_gain:
                # Null-space drift toward the preferred posture, over the free joints only.
                Jpinv = Jf.T @ np.linalg.inv(JJt)
                null = np.eye(Jf.shape[1]) - Jpinv @ Jf
                step = step + null @ (posture_gain * (posture[free] - (q[free] + dq[free])))

            trial = dq.copy()
            trial[free] += step
            over = free & ((q + trial < self.limits_lower - 1e-9)
                           | (q + trial > self.limits_upper + 1e-9))
            if not over.any():
                dq = trial
                break
            # Pin the offenders exactly at their stops and take them out of the next solve.
            dq[over] = np.where(q[over] + trial[over] < self.limits_lower[over],
                                self.limits_lower[over] - q[over],
                                self.limits_upper[over] - q[over])
            free = free & ~over

        m = float(np.max(np.abs(dq))) if self.n else 0.0
        if m > max_step:
            dq *= max_step / m
        return dq

    def clamp(self, q: np.ndarray) -> np.ndarray:
        return np.clip(q, self.limits_lower, self.limits_upper)

    def reach(self) -> float:
        """Straight-arm distance from the shoulder to the tool at the ZERO pose.

        NOT the maximum — the chain's canted stubs mean some other configuration reaches
        further. Use :meth:`reach_bounds` for a workspace envelope.
        """
        z = np.zeros(self.n)
        return float(np.linalg.norm(self.tool(z) - self.shoulder(z)))

    @lru_cache(maxsize=8)
    def _reach_bounds_cached(self, samples: int) -> tuple[float, float]:
        # Grid over the joints that actually move the tool (the last is an inline twist that
        # contributes nothing), taking min and max radius from the shoulder. Coarse is fine:
        # this is a safety envelope, and the margin the caller applies dwarfs the grid error.
        import itertools
        sh = self.shoulder()
        axes = [np.linspace(self.limits_lower[i], self.limits_upper[i], samples)
                for i in range(min(4, self.n))]
        lo, hi = np.inf, 0.0
        for combo in itertools.product(*axes):
            q = np.zeros(self.n)
            q[:len(combo)] = combo
            d = float(np.linalg.norm(self.tool(q) - sh))
            lo = min(lo, d)
            hi = max(hi, d)
        return lo, hi

    def reach_bounds(self, samples: int = 9) -> tuple[float, float]:
        """(min, max) distance from the shoulder the tool can actually reach, over the joint
        limits. The zero pose is NOT the extreme, so this is measured rather than assumed."""
        return self._reach_bounds_cached(samples)


def chain_for(limb: str, joint_names) -> ArmChain:
    """Build the chain for a limb from its configured joint names."""
    if not limb.endswith("_arm"):
        raise ValueError(f"{limb} is not an arm")
    return ArmChain(joint_names)
