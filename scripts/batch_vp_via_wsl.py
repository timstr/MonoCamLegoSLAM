from math import isnan
import subprocess
import sys

print("Building...")
subprocess.run(
    ["wsl", "make"],
    cwd="../build_wsl"
)

print("Running VP estimation...")
proc = subprocess.run(
    ["wsl", "python3.7", "../scripts/batch_vp.py", "../dataset/lines/\\*.txt"],
    capture_output=True,
    cwd="../build_wsl/"
)
errs = proc.stderr.decode()
if errs:
    sys.stderr.write(errs)
    exit()

import numpy as np
import matplotlib.pyplot as plt
import pickle

with open("odometry_angles.pkl", "rb") as f:
    odo_angles = pickle.load(f)
with open("odometry_timestamps.pkl", "rb") as f:
    odo_timestamps = pickle.load(f)

odo_angles /= (0.5 * np.pi)
odo_angles += 0.5
odo_angles = (odo_angles - np.floor(odo_angles))
odo_angles *= (0.5 * np.pi)

arr = np.array([float(s) for s in proc.stdout.decode().split()]).reshape(-1, 2)

vp_timestamps = arr[:,0]
vp_angles = arr[:,1]

vp_angles /= (0.5 * np.pi)
vp_angles += 0.5
vp_angles = (vp_angles - np.floor(vp_angles))
vp_angles *= (0.5 * np.pi)

w = vp_timestamps[-1]
# plt.plot(arr[12:35])

labeled = False
for vpt, vpa in zip(vp_timestamps, vp_angles):
    if not isnan(vpa):
        continue
    lbl_args = {"label":"Vanishing Point (Missing)"} if not labeled else {}
    labeled = True
    plt.plot([vpt, vpt], [0.1 * np.pi, 0.4 * np.pi], c="red", linestyle="--", **lbl_args)

plt.gca().set_ylim(-0.1, 0.5 * np.pi + 0.1)

plt.plot(odo_timestamps, odo_angles, c="tab:orange")
plt.plot(vp_timestamps, vp_angles, c="tab:blue")

plt.scatter(odo_timestamps, odo_angles, label="Dead Reckoning", c="tab:orange")

plt.scatter(vp_timestamps, vp_angles, label="Vanishing Point", c="tab:blue")

plt.gca().set_xlim(0.0, w)

# plt.gca().axis("off")
plt.xlabel("Time (seconds)")
plt.ylabel("Estimated Angle (radians modulo π/2)")

# plt.legend(loc="upper right")
plt.gcf().set_dpi(200)


handles,labels = plt.gca().get_legend_handles_labels()

handles = [handles[1], handles[2], handles[0]]
labels =  [ labels[1],  labels[2],  labels[0]]

plt.gca().legend(handles,labels,loc="upper right")

plt.show()


err = 0
nnan = 0
for vpa, vpt in zip(vp_angles, vp_timestamps):
    if (isnan(vpa)):
        nnan += 1
        continue
    dtmin = None
    closest_oa = None
    for oa, ot in zip(odo_angles, odo_timestamps):
        if dtmin is None or abs(ot - vpt) < dtmin:
            dtmin = abs(ot - vpt)
            closest_oa = oa
    assert closest_oa is not None
    err += abs(closest_oa - vpa)

print("Total error:", err)
print(nnan, " values were NaN")