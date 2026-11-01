"""Stress cases that try to make the arm fail."""
import mujoco, numpy as np
from check_arm import pick_and_place as run, row, load, ik, set_pose, OPEN

base = run()
print("peak segment yaw/sh/el:", base["peak_segment"], "peak joint speed rad/s:", round(base["peak_speed"], 2))
slow = run(slow=2.0)
print("slow peak segment:", slow["peak_segment"])
for mu in (0.1, 0.07, 0.05, 0.03):
    row(f"friction {mu}", run(friction=mu))
for off in ((0, .01), (0, .02), (0, .03), (0, .04), (.01, 0), (.02, 0), (.03, 0), (.04, 0)):
    row(f"offset x{off[0]*1000:.0f} y{off[1]*1000:.0f} mm", run(cube_offset=off))
for k in (1.5, 2.0, 3.0):
    row(f"gripper+links mass x{k}", run(arm_mass_scale=k))
row("fast x2, speed check", (f := run(slow=0.5)))
print("fast peak speed rad/s", round(f["peak_speed"], 2))

# Static check against a hand calculation at the 200 mm grasp pose, holding 200 g.
m = load(); d = mujoco.MjData(m)
q = ik(0, .2, .11); set_pose(m, d, q); mujoco.mj_forward(m, d)
hand = 9.81 * (.3 * .2 + .075 * .12 * np.sin(q[1]) + .075 * (.12 * np.sin(q[1]) + .06 * np.sin(q[1] + q[2])))
print("hand static shoulder torque N·m", round(hand, 3))
