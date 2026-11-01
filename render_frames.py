"""Save a 4x2 contact sheet of the baseline pick-and-place run."""
import mujoco, numpy as np
from PIL import Image
from check_arm import pick_and_place, script

ends = np.cumsum([w[-1] for w in script()])
times = [ends[1], ends[2], ends[3], ends[4], ends[5], ends[6], ends[7], ends[8]]
frames, renderer = [], [None]

def grab(m, d):
    if renderer[0] is None:
        renderer[0] = mujoco.Renderer(m, 360, 480)
    if len(frames) < len(times) and d.time >= times[len(frames)]:
        renderer[0].update_scene(d, camera="side")
        frames.append(renderer[0].render().copy())

pick_and_place(frame_cb=grab)
rows = [np.hstack(frames[i:i + 4]) for i in (0, 4)]
Image.fromarray(np.vstack(rows)).save("pick_and_place.png")
print("saved", len(frames))
