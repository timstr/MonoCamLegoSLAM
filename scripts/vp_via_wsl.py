from math import isnan
import subprocess
import sys

print("Building...")
subprocess.run(
    ["wsl", "make"],
    cwd="../build_wsl"
)


import glob
import os
import numpy as np
import matplotlib.pyplot as plt

# files = [
#     "lines_img_096_151099.txt",
#     "lines_img_000_2006.txt",
#     "lines_img_001_3538.txt",
#     "lines_img_002_5173.txt",
#     "lines_img_003_6709.txt",
#     "lines_img_004_8244.txt",
#     "lines_img_005_9781.txt",
#     "lines_img_006_11348.txt",
#     "lines_img_007_12884.txt",
#     "lines_img_008_14419.txt",
#     "lines_img_009_15989.txt"
# ]

files = [os.path.basename(f) for f in glob.glob("../dataset/lines/*.txt")]

for file in files:
    print(f"Running VP estimation for file \"{file}\"")
    proc = subprocess.run(
        ["wsl", "./vp", f"../dataset/lines/{file}"],
        capture_output=True,
        cwd="../build_wsl/"
    )
    errs = proc.stderr.decode()
    if errs:
        sys.stderr.write(errs)
        exit()

    arr = np.array([float(s) for s in proc.stdout.decode().split()[:-1]])
    
    plt.plot(arr)
    plt.show()

    # fileid = str.join("_", file.split(".")[0].split("_")[2:])
    # print("fileid = ", fileid)
    # subprocess.run(
    #     ["mogrify", "-format", "jpeg", "*.ppm"],
    #     capture_output=True,
    #     cwd=".."
    # )

    # for f in ["temp_all.jpeg", "temp_outliers.jpeg", "temp_vertical.jpeg", "temp_vp1.jpeg", "temp_vp2.jpeg"]:
    #     newname = f"../temp/line_segments_{fileid}_{f.split('.')[0].split('_')[1]}.jpg"
    #     os.rename(f"../{f}", newname)
