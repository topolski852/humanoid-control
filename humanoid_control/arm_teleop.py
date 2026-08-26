"""
Cartesian arm teleop: stick deflection in, joint targets out.

Deliberately free of I/O so it can be tested without a robot — the caller feeds it the measured
joint angles and a stick command, and gets back joint targets to send.

**Sticks command VELOCITY, not position.** Deflection sets how fast the hand moves; releasing
stops it where it is. Position-mapped sticks would teleport the arm whenever the stick re-centres
or a frame is dropped, and would make the reachable set depend on where the stick happened to be.

The target point is integrated in Cartesian space and the IK chases it. That ordering matters:
integrating in joint space instead would make a straight-line hand motion impossible, and letting
the target run away from the arm (past its reach, or into a limit) would wind up an error the arm
can never work off. `_leash` below keeps the target within reach of where the hand actually is.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .arm_kinematics import ArmChain


@dataclass
class TeleopTuning:
    """Speeds and safety envelope. Defaults are deliberately slow — first-run values."""

    # Which coordinates the sticks drive. "spherical" is the default because it matches the
    # mechanism: the shoulder is a 2-DOF gimbal that AIMS the arm and the elbow sets reach, so
    # elevation/azimuth/reach are the arm's own degrees of freedom. Cartesian XYZ asks the
    # gimbal for motions it can only approximate — from the hanging pose, world "up" costs
    # ~306 rad of joint motion per metre against ~44 for the identical-looking "raise", so the
    # arm stalls against its own geometry and feels blocked.
    # "pose" is the 6-DOF-tracker frame (Quest controller): the caller supplies an absolute
    # hand displacement rather than a stick deflection, and ArmTeleop.step_pose is used
    # instead of step. Position only — see step_pose on why orientation is not tracked.
    frame: str = "joint"              # "joint" | "cartesian" | "spherical" | "pose"

    # JOINT mode: which joint each stick axis drives, indexed by stick in the order
    # (left_x, left_y, right_y, right_x). None leaves that axis unbound.
    #   left_x  -> 1 shoulder_roll       left_y  -> 0 shoulder_pitch
    #   right_y -> 3 elbow_pitch         right_x -> 2 shoulder_yaw
    # No IK, no target, no coupling: one stick moves exactly one joint. That predictability is
    # the whole point — Cartesian and spherical both trade it for convenience, and for a demo
    # the trade is not worth it.
    joint_map: tuple = (1, 0, 3, 2)
    # Direction per stick axis, same indexing as joint_map. Which way a joint turns for a given
    # stick push depends on how the motor is mounted, so this is a per-robot fact to be observed
    # on the hardware, not derived. shoulder_pitch reads inverted on this arm.
    joint_signs: tuple = (+1.0, -1.0, +1.0, +1.0)
    joint_rate_normal_deg: float = 30.0
    joint_rate_creep_deg: float = 10.0

    # How far the commanded joint position may lead the measured one, radians. The command is
    # integrated from ITS OWN last value, not from the encoder — integrating from measured is
    # self-defeating, because the command can then never get more than one tick ahead of where
    # the joint already is, the position error pins at that one tick, and the joint creeps at
    # whatever speed that small error happens to produce. This leash is what stops the opposite
    # failure: if a joint physically cannot keep up, the command waits for it instead of winding
    # up an error that discharges violently when the joint comes free.
    joint_leash_deg: float = 8.0

    # Hand speed at full stick deflection, metres/second (reach, and all Cartesian axes).
    speed_normal: float = 0.06
    speed_creep: float = 0.02

    # Aiming rate is DERIVED from the hand speed, not set independently: w = v / reach. A fixed
    # angular rate means the hand speed changes with how far the arm is extended, and it is easy
    # to pick a number that quietly asks for far more than the arm can deliver — 45 deg/s at
    # 29 cm reach is 23 cm/s, four times what this arm tracks. `rate_cap_deg` only stops the
    # division blowing up at small reach.
    rate_cap_deg: float = 60.0

    # The wrist does not move the hand, so it is a plain joint rate.
    wrist_rate_normal_deg: float = 60.0
    wrist_rate_creep_deg: float = 20.0

    # Stick deadband, as a fraction of full deflection. This controller drifts (see gamepad.py's
    # own note), and teleop INTEGRATES its input — a drift a walk command would shrug off makes
    # a held arm creep away on its own. Applied before integration, never after.
    deadband: float = 0.15

    # How far the integrated target may run ahead of the actual hand, metres. Without this the
    # target keeps travelling while the arm is blocked (limit, singularity, something in the
    # way), and the arm lunges when it comes free.
    leash: float = 0.03

    # Workspace: keep the target inside a shell around the shoulder, measured from the chain's
    # actual reachable radii (ArmChain.reach_bounds) rather than assumed. `reach_margin` is how
    # far inside those bounds the target must stay.
    reach_margin: float = 0.02

    posture_gain: float = 0.05
    damping: float = 0.01

    # Joint speed limit in rad/SECOND, multiplied by the real dt each tick. Expressing it
    # per-tick meant it silently depended on the loop rate: sized for 50 Hz, it halved when the
    # session turned out to run at the policy's 25 Hz. This is a safety backstop and should sit
    # well above what the commanded hand speed needs, or it becomes the controller.
    max_joint_rate: float = 1.2       # rad/s ~= 69 deg/s


@dataclass
class ArmTeleop:
    """Integrates a stick command into a hand target and solves joint angles for it."""

    chain: ArmChain
    tuning: TeleopTuning = field(default_factory=TeleopTuning)
    posture: np.ndarray | None = None       # preferred pose for the redundant DOF

    target: np.ndarray | None = field(default=None, init=False)
    q_cmd: np.ndarray | None = field(default=None, init=False)
    # Where the hand was when the clutch last engaged. The 'pose' frame maps operator
    # displacement RELATIVE to this, so every engage re-anchors and the arm never jumps.
    pose_anchor: np.ndarray | None = field(default=None, init=False)
    _clipped: bool = field(default=False, init=False)

    def reset(self, q: np.ndarray) -> None:
        """Seed the target at the hand's current position. Call on every engage — otherwise the
        arm jumps to wherever the target was left last time."""
        self.target = self.chain.tool(np.asarray(q, dtype=float)).copy()
        self.pose_anchor = self.target.copy()
        # Commanded joint vector, integrated independently of the encoder. Seeded at the
        # measured pose on every engage so nothing jumps.
        self.q_cmd = np.asarray(q, dtype=float).copy()
        self._clipped = False
        if self.posture is None:
            # Neutral = the MIDPOINT of each joint's range, not "wherever we started".
            # Anchoring the redundancy on the start pose means the further you drive, the
            # harder it pulls back, and it eventually shoves the redundant joint into a stop —
            # which showed up as shoulder_yaw pinning and elevation stalling ~40 deg short of
            # the reachable limit. Mid-range is what null-space resolution is FOR: keep joints
            # away from their stops so there is room to manoeuvre.
            self.posture = (self.chain.limits_lower + self.chain.limits_upper) / 2.0

    # --- spherical coordinates about the shoulder -------------------------
    # elevation: 0 = hanging straight down, 90 = horizontal, >90 = above the shoulder
    # azimuth:   0 = straight out to the side, +90 = forward
    def _to_spherical(self, p: np.ndarray) -> tuple[float, float, float]:
        v = p - self.chain.shoulder()
        r = float(np.linalg.norm(v))
        elev = float(np.arctan2(float(np.hypot(v[0], v[1])), float(-v[2])))
        azim = float(np.arctan2(float(v[0]), float(v[1])))
        return r, elev, azim

    def _from_spherical(self, r: float, elev: float, azim: float) -> np.ndarray:
        horiz = r * np.sin(elev)
        return self.chain.shoulder() + np.array([
            horiz * np.sin(azim),
            horiz * np.cos(azim),
            -r * np.cos(elev),
        ])

    @staticmethod
    def apply_deadband(v: float, deadband: float) -> float:
        """Deadband with rescaling, so the output is continuous at the edge rather than
        jumping from 0 to `deadband` the instant the stick crosses it."""
        a = abs(v)
        if a <= deadband:
            return 0.0
        return float(np.sign(v) * (a - deadband) / (1.0 - deadband))

    def _clamp_to_shell(self, p: np.ndarray) -> tuple[np.ndarray, bool]:
        """Pull a point inside the reachable shell about the shoulder.

        Radii come from ``ArmChain.reach_bounds()`` — measured over the joint limits, because
        the zero pose is NOT the extreme on this chain. Returns (point, was_clipped).
        """
        t = self.tuning
        shoulder = self.chain.shoulder()
        rmin, rmax = self.chain.reach_bounds()
        rmin, rmax = rmin + t.reach_margin, rmax - t.reach_margin
        r = p - shoulder
        rn = float(np.linalg.norm(r))
        if rn > rmax:
            return shoulder + r * (rmax / rn), True
        if rn < rmin:
            return shoulder + (r / rn if rn > 1e-9 else np.array([1.0, 0, 0])) * rmin, True
        return p, False

    def _leash(self, q: np.ndarray) -> None:
        """Hold the target within `leash` of the hand, and inside the workspace shell.

        Currently unused: the velocity frames solve for joint motion directly, so the
        commanded point is achievable by construction and there is no free-running target to
        leash (see :meth:`step`). Kept because it is the correct behaviour for any frame that
        DOES chase an independently-moving target.
        """
        t = self.tuning
        hand = self.chain.tool(q)
        d = self.target - hand
        n = float(np.linalg.norm(d))
        if n > t.leash:
            self.target = hand + d * (t.leash / n)
            self._clipped = True
        self.target, clipped = self._clamp_to_shell(self.target)
        self._clipped = self._clipped or clipped

    def step_pose(self, q_measured, delta_m, dt: float, *, creep: bool = False):
        """One teleop tick driven by a 6-DOF tracker (the Quest controller) instead of sticks.

        ``delta_m`` is the operator's hand DISPLACEMENT since the clutch engaged, already in
        robot frame (+X forward, +Y left, +Z up), already scaled. The absolute target is
        ``pose_anchor + delta_m`` — so the two anchors meet on a displacement vector: the XR
        side owns "where the controller was at the press", this side owns "where the arm's
        hand was at the press". Neither needs to know the other's coordinate system, and a
        missing or zero delta means HOLD, which is the safe default.

        RELATIVE, NOT ABSOLUTE. A 29 cm arm bolted to a bench and a human arm do not share a
        workspace, so an absolute map would put most of both out of reach. Anchoring on the
        trigger press also gives a mouse-lift ratchet for free: release, reposition your hand,
        press again, keep going — no extra button, and it reuses the engage-time
        :meth:`reset` the deadman worker already performs.

        NOT A VELOCITY INTEGRATOR, so there is nothing to wind up: ``delta_m`` is recomputed
        from the live controller pose every tick rather than accumulated. Out-running the arm
        therefore costs tracking error, not a stored-up lunge. That error is reported as
        ``tracking_error_m`` and shown in the UI, because the failure it warns about is silent
        otherwise: once the operator's hand is far past the workspace shell, small hand
        motions produce no arm motion at all and the mapping feels dead. The fix is to release
        and re-anchor, which is why the number needs to be visible.

        Orientation is NOT tracked. The Jacobian is position-only and the tool is the wrist
        pivot (see arm_kinematics), so a 5-joint arm whose last joint is an inline twist
        cannot hold hand orientation. Reporting otherwise would be a lie.

        Returns (joint_targets, info) — the same contract as :meth:`step`.
        """
        q = np.asarray(q_measured, dtype=float)
        if self.target is None or self.pose_anchor is None:
            self.reset(q)
        t = self.tuning
        hand = self.chain.tool(q)

        delta = np.asarray(delta_m, dtype=float).reshape(3)
        if not np.all(np.isfinite(delta)):
            delta = np.zeros(3)          # a bad sample must HOLD, never fling the arm
        p_desired, shell_clipped = self._clamp_to_shell(self.pose_anchor + delta)

        # Per-tick motion is capped at the same hand speed the stick frames use, so "how fast
        # may the hand move" stays one number regardless of what is driving. A fast operator
        # gets a smooth lag, not a lunge.
        speed = t.speed_creep if creep else t.speed_normal
        to_go = p_desired - hand
        dist = float(np.linalg.norm(to_go))
        step_m = speed * dt
        dx_des = to_go * (step_m / dist) if dist > step_m else to_go

        dq = self.chain.ik_step(
            q, dx_des,
            damping=t.damping,
            posture=self.posture,
            posture_gain=t.posture_gain,
            max_step=t.max_joint_rate * dt,
        )
        q_target = self.chain.clamp(q + dq)
        self.target = self.chain.tool(q_target)

        achieved = self.target - hand
        want = float(np.linalg.norm(dx_des))
        got = float(np.linalg.norm(achieved))
        along = (float(np.dot(achieved, dx_des)) / (want * want)) if want > 1e-9 else 1.0

        at_limit = [
            self.chain.joint_names[i]
            for i in range(self.chain.n)
            if q_target[i] <= self.chain.limits_lower[i] + 1e-6
            or q_target[i] >= self.chain.limits_upper[i] - 1e-6
        ]
        r2, e2, a2 = self._to_spherical(self.target)
        return q_target, {
            "target": self.target.copy(),
            "hand": hand,
            "desired": p_desired.copy(),
            # How far the hand still is from where the operator is pointing. Grows while the
            # arm is outrun or pinned at the shell; the cue to release and re-anchor.
            "tracking_error_m": round(dist, 4),
            "error_m": max(0.0, want - got),
            "follow": round(along, 3),
            "sliding": bool(want > 1e-9 and along < 0.8),
            "clipped": bool(shell_clipped),
            "frame": "pose",
            "spherical": {"reach_m": round(r2, 4),
                          "elevation_deg": round(float(np.degrees(e2)), 1),
                          "azimuth_deg": round(float(np.degrees(a2)), 1)},
            "at_limit": at_limit,
            "commanding": bool(dist > 1e-4),
        }

    def step(self, q_measured, command, dt: float, *, creep: bool = False):
        """One teleop tick.

        ``command`` is the RAW stick quad ``(left_x, left_y, right_y, right_x)`` in [-1, 1],
        already sign-corrected so "up"/"right" are positive. What the axes mean depends on
        ``tuning.frame``:

          cartesian (default)  left_y  -> +X forward
                               left_x  -> +Y robot-left
                               right_y -> +Z up
          spherical            left_x  -> azimuth, left_y -> elevation, right_y -> reach

        ``right_x`` drives the WRIST in both frames, as a direct joint rate — it is an inline
        twist that does not move the hand, so it has no place in a position solve.

        VELOCITY CONTROL, NOT TARGET CHASING. Earlier this integrated a Cartesian target and
        had the IK chase it, which fails in two ways: the target runs away whenever the arm
        cannot keep up (winding up an error the arm can never work off), and clamping a runaway
        target back into the workspace DEFLECTS it, so commanding one axis produced motion in
        another. Solving for the joint motion directly and letting the hand follow means the
        commanded point is achievable by construction — and a joint pinned at its limit simply
        drops out of the solution while the others keep moving, so the hand SLIDES along the
        constraint instead of stopping dead.

        Returns (joint_targets, info).
        """
        q = np.asarray(q_measured, dtype=float)
        if self.target is None:
            self.reset(q)
        t = self.tuning

        cmd = np.array([self.apply_deadband(float(c), t.deadband) for c in command])
        lx, ly, ry = float(cmd[0]), float(cmd[1]), float(cmd[2])
        rx = float(cmd[3]) if len(cmd) > 3 else 0.0
        speed = t.speed_creep if creep else t.speed_normal

        hand = self.chain.tool(q)

        if t.frame == "joint":
            # Direct joint rates. Deliberately bypasses the IK entirely.
            if self.q_cmd is None:
                self.q_cmd = q.copy()
            rate = np.radians(t.joint_rate_creep_deg if creep else t.joint_rate_normal_deg)
            dq = np.zeros(self.chain.n)
            for stick_i, joint_i in enumerate(t.joint_map):
                if joint_i is None or joint_i >= self.chain.n or stick_i >= len(cmd):
                    continue
                sign = (t.joint_signs[stick_i]
                        if stick_i < len(t.joint_signs) else 1.0)
                dq[joint_i] += float(cmd[stick_i]) * sign * rate * dt
            # Integrate the COMMAND, not the encoder — see joint_leash_deg. An uncommanded
            # joint therefore holds its last commanded angle instead of following its own droop.
            leash = np.radians(t.joint_leash_deg)
            self.q_cmd = self.chain.clamp(
                np.clip(self.q_cmd + dq, q - leash, q + leash))
            q_target = self.q_cmd.copy()
            self.target = self.chain.tool(q_target)
            at_limit = [
                self.chain.joint_names[i] for i in range(self.chain.n)
                if q_target[i] <= self.chain.limits_lower[i] + 1e-6
                or q_target[i] >= self.chain.limits_upper[i] - 1e-6
            ]
            r2, e2, a2 = self._to_spherical(self.target)
            return q_target, {
                "target": self.target.copy(), "hand": hand,
                "error_m": 0.0, "follow": 1.0, "sliding": False, "clipped": False,
                "frame": "joint",
                "spherical": {"reach_m": round(r2, 4),
                              "elevation_deg": round(float(np.degrees(e2)), 1),
                              "azimuth_deg": round(float(np.degrees(a2)), 1)},
                "at_limit": at_limit,
                "commanding": bool(np.any(cmd != 0.0)),
            }

        if t.frame == "spherical":
            # Convert the aiming command into a world velocity at the hand's current position.
            r, elev, azim = self._to_spherical(hand)
            rate = min(speed / max(r, 1e-3), np.radians(t.rate_cap_deg))
            eps = 1e-4
            d_elev = (self._from_spherical(r, elev + eps, azim) - hand) / eps
            d_azim = (self._from_spherical(r, elev, azim + eps) - hand) / eps
            d_reach = (self._from_spherical(r + eps, elev, azim) - hand) / eps
            v_des = ly * rate * d_elev + lx * rate * d_azim + ry * speed * d_reach
        else:
            v_des = np.array([ly, lx, ry]) * speed

        dx_des = v_des * dt
        dq = self.chain.ik_step(
            q, dx_des,
            damping=t.damping,
            posture=self.posture,
            posture_gain=t.posture_gain,
            max_step=t.max_joint_rate * dt,
        )
        q_target = self.chain.clamp(q + dq)

        # Wrist: direct joint rate, layered on top (see the docstring).
        if rx and self.chain.n:
            wrist_rate = np.radians(t.wrist_rate_creep_deg if creep else t.wrist_rate_normal_deg)
            i = self.chain.n - 1
            q_target[i] = float(np.clip(q_target[i] + rx * wrist_rate * dt,
                                        self.chain.limits_lower[i], self.chain.limits_upper[i]))

        # The commanded point is wherever those joints put the hand — achievable by definition,
        # so there is no target to leash and nothing to wind up.
        self.target = self.chain.tool(q_target)
        achieved = self.target - hand
        want = float(np.linalg.norm(dx_des))
        got = float(np.linalg.norm(achieved))
        # How much of the COMMANDED DIRECTION survived. Below ~1 means the arm is sliding along
        # a constraint rather than following the stick, which is the honest thing to report.
        along = (float(np.dot(achieved, dx_des)) / (want * want)) if want > 1e-9 else 1.0

        at_limit = [
            self.chain.joint_names[i]
            for i in range(self.chain.n)
            if q_target[i] <= self.chain.limits_lower[i] + 1e-6
            or q_target[i] >= self.chain.limits_upper[i] - 1e-6
        ]
        r2, e2, a2 = self._to_spherical(self.target)
        info = {
            "target": self.target.copy(),
            "hand": hand,
            "error_m": max(0.0, want - got),
            "follow": round(along, 3),
            "sliding": bool(want > 1e-9 and along < 0.8),
            "clipped": bool(want > 1e-9 and along < 0.8),
            "frame": t.frame,
            "spherical": {"reach_m": round(r2, 4),
                          "elevation_deg": round(float(np.degrees(e2)), 1),
                          "azimuth_deg": round(float(np.degrees(a2)), 1)},
            "at_limit": at_limit,
            "commanding": bool(np.any(cmd != 0.0)),
        }
        return q_target, info
