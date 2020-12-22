import glob
import PIL.Image as Image
import matplotlib.pyplot as plt

# files = list(glob.glob("../dataset/img_jpeg/*.jpeg"))[0:56:2]

# fig, ax = plt.subplots(1, len(files))

# for i, f in enumerate(files):
#     ax[i].imshow(Image.open(f), cmap="gray")
#     ax[i].axis("off")

# plt.show()

files = [f"../../pictures/coloured grid directions {id}.jpg" for id in [
    "01",
    "02",
    "03",
    "04",
    "05",
    "06",
    "07",
    "08",
    "09",
    "10"
]]

fig, ax = plt.subplots(2, 5)

for i, f in enumerate(files):
    a = ax[i // 5][i % 5]
    a.imshow(Image.open(f))
    a.axis("off")

plt.show()