"""Feasibility checks for the 3-DOF arm. Run: .venv/bin/python check_arm.py"""
from math import acos, atan2, cos, hypot, pi, sin
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).parent
L1, L2 = 0.12, 0.12
SHOULDER_Z = 0.10
PINCH_DROP = 0.09
RATED_TORQUE = 2.94
OPEN, CLOSED = 0.035, 0.0


def ik(theta, r, wrist_z):
    """Return (base, shoulder, elbow), elbow-up, or None if out of reach."""
    dz = wrist_z - SHOULDER_Z
    c = (r**2 + dz**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(c) > 1:
        return None
    q3 = acos(c)
    q2 = atan2(r, dz) - atan2(L2 * sin(q3), L1 + L2 * cos(q3))
    return np.array([theta, q2, q3])


def load(payload=0.2, friction=0.8):
    m = mujoco.MjModel.from_xml_path(str(HERE / "arm.xml"))
    cube = m.body("cube").id
    scale = payload / m.body_mass[cube]
    m.body_mass[cube] *= scale
    m.body_inertia[cube] *= scale
    for name in ("cube", "pad_l", "pad_r"):
        m.geom(name).friction[0] = friction
    return m


def set_pose(m, d, q):
    for name, value in zip(("base_yaw", "shoulder", "elbow"), q):
        d.qpos[m.joint(name).qposadr[0]] = value
    d.qpos[m.joint("wrist").qposadr[0]] = -(q[1] + q[2])
    d.ctrl[:3] = q
    d.ctrl[3] = OPEN
    d.qpos[m.joint("finger_l").qposadr[0]] = OPEN
    d.qpos[m.joint("finger_r").qposadr[0]] = OPEN


def geom_names(m, contact):
    return {m.geom(int(contact.geom1)).name, m.geom(int(contact.geom2)).name}


def reach_sweep():
    """Check table-level picks of a 40 mm cube for limits and collisions."""
    m = load()
    # Park the cube out of the way so only the arm's own clearance is tested.
    d = mujoco.MjData(m)
    cube_adr = m.joint("cube").qposadr[0]
    limits = {n: m.joint(n).range for n in ("shoulder", "elbow")}
    wrist_z = 0.02 + PINCH_DROP
    ok = []
    for r_mm in range(40, 251, 5):
        q = ik(0.0, r_mm / 1000, wrist_z)
        if q is None:
            continue
        in_limits = all(
            limits[n][0] <= v <= limits[n][1] for n, v in zip(("shoulder", "elbow"), q[1:])
        )
        mujoco.mj_resetData(m, d)
        set_pose(m, d, q)
        d.qpos[cube_adr:cube_adr + 3] = [-0.5, -0.5, 0.02]
        mujoco.mj_forward(m, d)
        clear = all(geom_names(m, d.contact[i]) == {"cube", "table"} for i in range(d.ncon))
        if in_limits and clear:
            ok.append(r_mm)
    return ok


def min_jerk(s):
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def script(pick_r=0.20, slow=1.0):
    """Waypoints: (theta, r, wrist_z, gripper, seconds)."""
    grasp_z, carry_z = 0.02 + PINCH_DROP, 0.18
    place_theta = pi / 2
    return [
        (0.0, 0.14, 0.20, OPEN, 0.5),
        (0.0, pick_r, carry_z, OPEN, 2.0 * slow),
        (0.0, pick_r, grasp_z, OPEN, 1.5 * slow),
        (0.0, pick_r, grasp_z, CLOSED, 1.0),
        (0.0, pick_r, carry_z, CLOSED, 1.5 * slow),
        (place_theta, pick_r, carry_z, CLOSED, 2.0 * slow),
        (place_theta, pick_r, grasp_z, CLOSED, 1.5 * slow),
        (place_theta, pick_r, grasp_z, OPEN, 1.0),
        (place_theta, pick_r, carry_z, OPEN, 1.5 * slow),
        (0.0, 0.14, 0.20, OPEN, 2.0 * slow),
    ]


def pick_and_place(payload=0.2, friction=0.8, slow=1.0, frame_cb=None, cube_offset=(0, 0), arm_mass_scale=1.0):
    m = load(payload, friction)
    for body in ("upper_arm", "forearm", "gripper", "finger_l", "finger_r"):
        m.body_mass[m.body(body).id] *= arm_mass_scale
        m.body_inertia[m.body(body).id] *= arm_mass_scale
    d = mujoco.MjData(m)
    plan = script(slow=slow)
    start = (0.0, 0.14, 0.20, OPEN)
    set_pose(m, d, ik(*start[:3]))
    d.qpos[m.joint("cube").qposadr[0]:m.joint("cube").qposadr[0] + 2] += np.array(cube_offset)
    mujoco.mj_forward(m, d)

    pinch, cube = m.site("pinch").id, m.body("cube").id
    cube_adr = m.joint("cube").qposadr[0]
    arm_act = [m.actuator(n).id for n in ("base_yaw", "shoulder", "elbow")]
    peak, peak_hold = np.zeros(3), 0.0
    peak_segment, peak_speed = np.zeros(3, dtype=int), 0.0
    arm_dof = [m.joint(n).dofadr[0] for n in ("base_yaw", "shoulder", "elbow")]
    track_err, tilt, bad = 0.0, 0.0, set()
    allowed = {"table", "cube", "pad_l", "pad_r"}
    carried_min_z = None
    place = np.array([0.0, 0.2, 0.02])

    prev = np.array(start)
    for index, (*target, seconds) in enumerate(plan):
        target = np.array(target)
        steps = int(seconds / m.opt.timestep)
        for step in range(steps):
            s = min_jerk((step + 1) / steps)
            theta, r, z, grip = prev + (target - prev) * s
            q = ik(theta, r, z)
            d.ctrl[:3] = q
            d.ctrl[3] = grip
            mujoco.mj_step(m, d)
            frame_cb and frame_cb(m, d)

            if d.time > 0.5:  # ignore the settling segment
                force = np.abs(d.actuator_force[arm_act])
                peak_segment[force > peak] = index
                peak = np.maximum(peak, force)
                peak_speed = max(peak_speed, np.abs(d.qvel[arm_dof]).max())
                wanted = np.array([r * cos(theta), r * sin(theta), z - PINCH_DROP])
                track_err = max(track_err, np.linalg.norm(d.site_xpos[pinch] - wanted))
                tilt = max(tilt, abs(d.ten_length[m.tendon("level").id]))
                for i in range(d.ncon):
                    names = geom_names(m, d.contact[i])
                    if not names <= allowed or names == {"table", "pad_l"} or names == {"table", "pad_r"}:
                        bad.add(tuple(sorted(names)))
            if index == 4 and step == steps - 1:
                carried_min_z = d.xpos[cube][2]
        if index == 3:
            peak_hold = abs(d.actuator_force[arm_act[1]])
        prev = target

    for _ in range(1000):
        mujoco.mj_step(m, d)
    final = d.qpos[cube_adr:cube_adr + 3].copy()
    return {
        "lifted": carried_min_z > 0.07,
        "miss_mm": hypot(*(final - place)[:2]) * 1000,
        "final_z_mm": final[2] * 1000,
        "peak_nm": peak,
        "peak_segment": peak_segment,
        "peak_speed": peak_speed,
        "shoulder_at_grasp_nm": peak_hold,
        "track_mm": track_err * 1000,
        "tilt_deg": np.degrees(tilt),
        "bad_contacts": bad,
    }


def row(label, result):
    success = result["lifted"] and result["miss_mm"] < 10
    p = result["peak_nm"]
    print(
        f"{label:<30} {'PASS' if success else 'FAIL':<5} miss {result['miss_mm']:6.1f} mm  "
        f"peak yaw/sh/el {p[0]:.2f}/{p[1]:.2f}/{p[2]:.2f} N·m  "
        f"track {result['track_mm']:5.1f} mm  tilt {result['tilt_deg']:.2f}°  "
        f"contacts {sorted(result['bad_contacts']) or 'clean'}"
    )


if __name__ == "__main__":
    ok = reach_sweep()
    gaps = [b for a, b in zip(ok, ok[1:]) if b - a != 5]
    print(f"1. Table-level pick range: {ok[0]}–{ok[-1]} mm" + (f" (gaps near {gaps})" if gaps else ""))
    print(f"   200 mm reachable: {200 in ok}")
    print(f"\n2/3. Pick and place, limit 50% of rating = {RATED_TORQUE / 2:.2f} N·m")
    row("baseline 200 g", pick_and_place())
    for payload in (0.02, 0.1, 0.3, 0.4, 0.5):
        row(f"payload {payload * 1000:.0f} g", pick_and_place(payload=payload))
    for friction in (0.3, 0.5, 1.0):
        row(f"friction {friction}", pick_and_place(friction=friction))
    row("twice as fast", pick_and_place(slow=0.5))
    row("twice as slow", pick_and_place(slow=2.0))
