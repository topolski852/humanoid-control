"""Gravitational torque at each leg joint, from the URDF, for a free-hanging leg.

The robot's own limb IS a calibrated load. The URDF's link masses sum to **12.61191 kg** against
a **12.61 kg** measured robot (0.015% error), so the masses are validated, not assumed — which is
what makes this usable as the reference load for M7 instead of hanging weights.

Better than an external mass in one way: gravitational torque varies as cos(angle), so **sweeping
a joint sweeps the applied torque** over a continuous range with no hardware and no re-rigging.
Three hand-placed weights give three points; an angle sweep gives as many as you like.

REQUIRES A FREE-HANGING LEG — robot lifted, foot off the ground. Standing is a closed kinematic
chain (the floor carries the load) and none of this applies.

    tau_j = sum over links DISTAL to joint j of  ((p_i - o_j) x m_i g) . axis_j
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

G = np.array([0.0, 0.0, -9.80665])
URDF = Path("/home/nse/humanoid-policy/source/humanoid_policy_assets/data/robots/humanoid/"
            "urdf/humanoid_biped.urdf")


def _rpy(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp,     cp * sr,               cp * cr]])


def _vec(s):
    return np.array([float(x) for x in (s or "0 0 0").split()])


class LegModel:
    """Parsed URDF tree with forward kinematics and gravity torques."""

    def __init__(self, urdf: Path = URDF):
        root = ET.parse(urdf).getroot()
        self.mass, self.com = {}, {}
        for l in root.findall("link"):
            i = l.find("inertial")
            if i is None:
                continue
            self.mass[l.get("name")] = float(i.find("mass").get("value"))
            o = i.find("origin")
            self.com[l.get("name")] = _vec(o.get("xyz") if o is not None else None)
        self.joints = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            ax = j.find("axis")
            self.joints[j.get("name")] = {
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": _vec(o.get("xyz") if o is not None else None),
                "rpy": _vec(o.get("rpy") if o is not None else None),
                "axis": _vec(ax.get("xyz") if ax is not None else "0 0 1"),
                "type": j.get("type"),
            }
        self.child_of = {v["child"]: k for k, v in self.joints.items()}
        self.kids = {}
        for k, v in self.joints.items():
            self.kids.setdefault(v["parent"], []).append(k)

    def fk(self, q: dict[str, float]):
        """World transform per link, and (origin, axis) per joint, for configuration q."""
        roots = [l for l in self.mass if l not in self.child_of]
        T = {r: (np.eye(3), np.zeros(3)) for r in roots}
        jinfo = {}
        stack = list(roots)
        while stack:
            link = stack.pop()
            R, p = T[link]
            for jn in self.kids.get(link, []):
                J = self.joints[jn]
                Rj = R @ _rpy(J["rpy"])
                pj = p + R @ J["xyz"]
                ax = Rj @ J["axis"]
                ax = ax / (np.linalg.norm(ax) or 1.0)
                th = float(q.get(jn, 0.0)) if J["type"] in ("revolute", "continuous") else 0.0
                # Rodrigues about the joint axis
                K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                Rr = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)
                jinfo[jn] = {"origin": pj, "axis": ax}
                T[J["child"]] = (Rr @ Rj, pj)
                stack.append(J["child"])
        return T, jinfo

    def distal_links(self, joint: str) -> list[str]:
        out, stack = [], [self.joints[joint]["child"]]
        while stack:
            l = stack.pop()
            out.append(l)
            for jn in self.kids.get(l, []):
                stack.append(self.joints[jn]["child"])
        return out

    def gravity_torque(self, q: dict[str, float], joints: list[str] | None = None) -> dict:
        """Gravitational torque about each joint axis, in N·m, for a FREE-HANGING leg."""
        T, jinfo = self.fk(q)
        out = {}
        for jn in (joints or self.joints):
            if jn not in jinfo or self.joints[jn]["type"] not in ("revolute", "continuous"):
                continue
            o, ax = jinfo[jn]["origin"], jinfo[jn]["axis"]
            tau = 0.0
            for l in self.distal_links(jn):
                m = self.mass.get(l, 0.0)
                if m <= 0:
                    continue
                R, p = T[l]
                r = (p + R @ self.com[l]) - o
                tau += float(np.dot(np.cross(r, m * G), ax))
            out[jn] = tau
        return out

    def distal_mass(self, joint: str) -> float:
        return sum(self.mass.get(l, 0.0) for l in self.distal_links(joint))


# device joint name -> URDF joint name
def urdf_name(device_joint: str) -> str:
    return "leg_" + device_joint.replace("_joint", "") + "_joint"


def total_mass(m: LegModel | None = None) -> float:
    m = m or LegModel()
    return sum(m.mass.values())
