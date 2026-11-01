"""Watch the run live on macOS: .venv/bin/mjpython view.py"""
import time
import mujoco.viewer
from check_arm import pick_and_place

viewer, steps = [None], [0]

def show(m, d):
    if viewer[0] is None:
        viewer[0] = mujoco.viewer.launch_passive(m, d)
    steps[0] += 1
    if steps[0] % 16 == 0:
        viewer[0].sync()
        time.sleep(0.016)

pick_and_place(frame_cb=show)
time.sleep(2)
viewer[0].close()
