import glob
import os
import sys
import numpy as np
import torch
import PIL.Image as Image
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # Not unused, needed for 3D plot
import csv
import math

from odometry import parse_odometry, deadReckoning

def progress_bar(current, total):
    i = current + 1
    bar_fill = '=' * (i * 50 // total)
    end_of_line = "\n" if (i == total) else ""
    sys.stdout.write("\r[%-50s] %d/%d%s" % (bar_fill, i, total, end_of_line))
    sys.stdout.flush()

def load_image(name):
    img = Image.open(f"../dataset/img_jpeg/{name}")
    return np.array(img)

def parse_lines(name):
    data = []
    filepath = f"../dataset/lines/{name}"
    with open(filepath, "r") as infile:
        rdr = csv.reader(infile, delimiter=" ")
        for row in rdr:
            datarow = []
            for i in range(4):
                datarow.append(float(row[i]))
            data.append(datarow)
    return np.array(data, dtype="float64")

image_patch_rad = 5
image_patch_size = 2 * image_patch_rad + 1

def extract_line_tag(line_segment, image):
    x0, y0, x1, y1 = line_segment
    xm = int(round((x0 + x1) * 0.5))
    ym = int(round((y0 + y1) * 0.5))
    h, w = image.shape
    assert (h, w) == (144, 176)
    t = ym - image_patch_rad
    b = ym + image_patch_rad + 1
    l = xm - image_patch_rad
    r = xm + image_patch_rad + 1
    if (t < 0) or (b >= h) or (l < 0) or (r >= w):
        return None
    out = image[t:b, l:r]
    assert out.shape == (image_patch_size, image_patch_size)
    return out

sse_threshold=500

def find_matching_tags(tags_a, tags_b):
    matches = []
    for ia, ta in enumerate(tags_a):
        if ta is None:
            continue
        sse_best = sse_threshold
        ib_best = None
        for ib, tb in enumerate(tags_b):
            if tb is None:
                continue
            sse = np.sum((ta - tb)**2)
            if sse < sse_best:
                sse_best = sse
                ib_best = ib
            if ib_best is not None:
                matches.append((ia, ib_best))
    return matches

def plot_landmark_motion(lines_a, lines_b, matches):
    for ia, ib in matches:
        x0a, y0a, x1a, y1a = lines_a[ia]
        x0b, y0b, x1b, y1b = lines_b[ib]
        
        plt.plot([x0a, x1a], [y0a, y1a], c="black")
        plt.plot([x0b, x1b], [y0b, y1b], c="black")
        xma = (x0a + x1a) * 0.5
        yma = (y0a + y1a) * 0.5
        xmb = (x0b + x1b) * 0.5
        ymb = (y0b + y1b) * 0.5
        plt.plot([xma, xmb], [yma, ymb], c="blue")

def extract_tags(img, lines):
    return [extract_line_tag(l, img) for l in lines]

def get_data(fid):
    img = load_image(f"{fid}.jpeg")
    lines = parse_lines(f"lines_{fid}.txt")
    tags = extract_tags(img, lines)
    return img, lines, tags

# Tag SSE statistics for first pair of images:
# min:  1128
# max:  17238
# mean:  12499.870546810273


# Painstakingly measured
camera_displacement_x = 107
camera_displacement_y = 16
camera_fov_vertical = np.pi * 0.2
camera_fov_horizontal = np.pi * 0.25
camera_pixels_vertical = 144
camera_pixels_horizontal = 176

def projectFromWorldToCameraPlane(robotX, robotY, robotTheta, worldPointXYZ):
    # Assumption: Z is up
    # translate to place robot X,Y at origin
    wx = worldPointXYZ[0] - robotX
    wy = worldPointXYZ[1] - robotY
    wz = worldPointXYZ[2]
    # rotate to make robot look down +X direction, with +Y to the right and +Z upwards
    st = torch.sin(-robotTheta)
    ct = torch.cos(-robotTheta)
    wx, wy = (
        wx * ct - wy * st,
        wx * st + wy * ct
    )
    # translate to place camera at origin
    # camera is located forward 107 mm and right 16 mm from robot center
    wx = wx - camera_displacement_x
    wy = wy - camera_displacement_y

    # project to camera plane by dividing by X (distance along line of sight)
    cx = wy / wx # +Y points to the right (horizontal in image plane)
    cy = wz / wx # +Z points upwards (vertical in image plane)

    # scale according to FOV
    cx = cx / math.tan(0.5 * camera_fov_horizontal)
    cy = cy / math.tan(0.5 * camera_fov_vertical)

    # map from [-1, 1]x[-1, 1] to [0, image_w-1]x[0, image_h-1] (and flip Y)
    cx = ( 0.5 * cx + 0.5) * (camera_pixels_horizontal - 1)
    cy = (-0.5 * cy + 0.5) * (camera_pixels_vertical - 1)

    return cx, cy

# TODO: tagged landmark database
# what:
# - list of tuples containing
#    - image patch tag
#    - xyz position and orientation

# List of tuple of (image_patch, vec3, vec3)

# given an image patch (from an observed line segment), returns the 3D endpoint locations
# of the best-matching landmark in the database, or None if no match is good enough
def findLandmarkLocation(image_patch, landmark_database):
    assert image_patch is not None
    sse_best = sse_threshold
    loc_best = None
    for lmdb_patch, lmdb_p0, lmdb_p1 in landmark_database:
        assert lmdb_patch is not None
        assert torch.is_tensor(lmdb_p0)
        assert torch.is_tensor(lmdb_p1)
        sse = np.sum((image_patch - lmdb_patch)**2)
        if sse < sse_best:
            sse_best = sse
            loc_best = (lmdb_p0, lmdb_p1)
    return loc_best

def createLandmarkDatabase(taggedLines, robotPoses):
    print("Creating Landmark Database")

    def extractTags(tls):
        tags = []
        for x0, y0, x1, y1, tag in tls:
            tags.append(tag)
        return tags

    lmdb = []

    assert len(taggedLines) > 1
    assert len(taggedLines) == len(robotPoses)

    prevTaggedLines = taggedLines[0]
    prevTags = extractTags(prevTaggedLines)
    for i, (nextTaggedLines, robotPose) in enumerate(zip(taggedLines[1:], robotPoses[1:])):
        nextTags = extractTags(nextTaggedLines)
        matchingTags = find_matching_tags(prevTags, nextTags)
        for prevId, nextId in matchingTags:
            if findLandmarkLocation(prevTags[prevId], lmdb) is not None:
                continue
            if findLandmarkLocation(nextTags[nextId], lmdb) is not None:
                continue
            rx, ry, rtheta = robotPose
            d = 100.0
            dx = d * math.cos(rtheta)
            dy = d * math.sin(rtheta)
            lmdb.append((
                prevTags[prevId],
                torch.tensor([rx + dx, ry + dy, 0.0], requires_grad=True),
                torch.tensor([rx + dx, ry + dy, 500.0], requires_grad=True)
            ))

        prevTaggedLines = nextTaggedLines
        prevTags = nextTags
        progress_bar(i, len(file_ids) - 1)
    return lmdb

def distanceFromLine2D(p0x, p0y, p1x, p1y, qx, qy):
    # dot product of (Q - P0) and (P1 - P0)
    dx = p1x - p0x
    dy = p1y - p0y
    dp = (qx - p0x) * dx + (qy - p0y) * dy
    lsqr = dx**2 + dy**2
    h = dp / lsqr
    distSqr = (qx - (p0x + h * dx))**2 + (qy - (p0y + h * dy))**2
    # return distSqr
    return torch.sqrt(distSqr)

def distanceFromPoint2D(px, py, qx, qy):
    return torch.sqrt((px - qx)**2 + (py - qy)**2)

# TODO: landmark alignment
# inputs:
# - list of tuples, each containing:
#   - estimated pose (e.g. from dead reckoning or previous pose correction)
#   - list of visible tagged landmarks and apparent line segment positions in image space
# - tagged landmark database (see above)
# algorithm:
# TLDR: maximize colinearity of projected 3D landmark segments with detected landark segments at each pose
# - err <- 0
# - for each estimated pose X_t, Y_t, Theta_t and visible landmark tuple:
#     - for each visible landmark:
#         - let Q_0^c, Q_1^c be the 2D endpoints of the visible landmark in camera image space
#         - let P_0^w, P_1^w be the 3D endpoints of the matching landmark in world space
#         - let F : R^3 -> R^2 be a perspective transformation from 3D world space into 
#           2D camera image space (using estimated pose and estimated 3D landmark pose)
#         - Let P_0^c be F(P_0^w)
#         - Let P_1^c be F(P_1^c)
#         - Let err_p0 be the distance P_0^c to the line passing through Q_0^c and Q_1^c
#         - Let err_p1 be the distance P_1^c to the line passing through Q_0^c and Q_1^c
#         - add err_p0 and err_p1 to err
# - minimize err over 3D landmark poses and estimated robot poses

def landmarkAlignmentParams(listOfPosesAndSegments, landmarkDatabase):
    # Not every parameter returned here will end up being used,
    # in which case its derivative will always be zero and it
    # will not be changed
    params = []
    for robotPose, _ in listOfPosesAndSegments:
        robotX, robotY, robotTheta = robotPose
        assert torch.is_tensor(robotX)
        assert torch.is_tensor(robotY)
        assert torch.is_tensor(robotTheta)
        assert robotX.requires_grad
        assert robotY.requires_grad
        assert robotTheta.requires_grad
        params.append({"params": [robotX, robotY, robotTheta], "lr": 1e-3})
    for _, p0, p1 in landmarkDatabase:
        assert torch.is_tensor(p0)
        assert torch.is_tensor(p1)
        assert p0.requires_grad
        assert p1.requires_grad
        params.append({"params": [p0, p1]})
    return params

def landmarkAlignmentErrorFn(listOfPosesAndSegments, landmarkDatabase):
    # assumptions:
    # - robot poses are differentiable
    # - landmark endpoints are differentiable
    # nothing else is necessarily differentiable
    print("Constructing alignment criteria")
    terms = []
    for i, (robotPose, visibleSegments) in enumerate(listOfPosesAndSegments):
        robotX, robotY, robotTheta = robotPose
        assert torch.is_tensor(robotX)
        assert torch.is_tensor(robotY)
        assert torch.is_tensor(robotTheta)
        for p0x, p0y, p1x, p1y, tag in visibleSegments:
            landmarkPose = findLandmarkLocation(tag, landmarkDatabase)
            if landmarkPose is None:
                continue
            lm_p0, lm_p1 = landmarkPose
            assert torch.is_tensor(lm_p0)
            assert torch.is_tensor(lm_p1)
            def errTerm():
                lm_p0_x, lm_p0_y = projectFromWorldToCameraPlane(robotX, robotY, robotTheta, lm_p0)
                lm_p1_x, lm_p1_y = projectFromWorldToCameraPlane(robotX, robotY, robotTheta, lm_p1)
                dist_p0 = distanceFromLine2D(p0x, p0y, p1x, p1y, lm_p0_x, lm_p0_y)
                dist_p1 = distanceFromLine2D(p0x, p0y, p1x, p1y, lm_p1_x, lm_p1_y)
                # dist_p0 = distanceFromPoint2D(p0x, p0y, lm_p0_x, lm_p0_y)
                # dist_p1 = distanceFromPoint2D(p1x, p1y, lm_p1_x, lm_p1_y)
                # TODO: constraint to encourage non-zero distance between landmark endpoints
                return dist_p0 + dist_p1
            terms.append(errTerm)
        progress_bar(i, len(listOfPosesAndSegments))
    print(f"{len(terms)} landmark observations were recorded")
    def sumTerms():
        err = 0.0
        for t in terms:
            err = err + t()
        return err
    return sumTerms

def alignLandmarks(listOfPosesAndSegments, landmarkDatabase):
    print("Aligning landmarks")
    errFn = landmarkAlignmentErrorFn(listOfPosesAndSegments, landmarkDatabase)
    print("Gathering differentiable parameters")
    params = landmarkAlignmentParams(listOfPosesAndSegments, landmarkDatabase)

    losses = []

    plt.ion()
    fig = plt.figure()
    axl = fig.add_subplot(121)
    axr = fig.add_subplot(122, projection='3d')

    def plotStuff():
        axl.clear()
        axr.cla()
        axr.set_xlim(-2000, 2000)
        axr.set_ylim(-2000, 2000)
        axr.set_zlim(    0, 4000)

        axl.plot(losses)

        pose_pts_x = []
        pose_pts_y = []
        pose_pts_z = []
        for pose, _ in listOfPosesAndSegments:
            x, y, theta = pose
            pose_pts_x.append(x.item())
            pose_pts_y.append(y.item())
            pose_pts_z.append(0.0)
            st = math.sin(theta)
            ct = math.cos(theta)
            rad = 150.0
            axr.plot(
                xs=[x.item(), x.item() + rad * ct],
                ys=[y.item(), y.item() + rad * st],
                zs=[0.0, 0.0],
                c="red"
            )
        axr.scatter(pose_pts_x, pose_pts_y, pose_pts_z, c="red")
        for _, p0, p1 in landmarkDatabase:
            x0 = p0[0].item()
            y0 = p0[1].item()
            z0 = p0[2].item()
            x1 = p1[0].item()
            y1 = p1[1].item()
            z1 = p1[2].item()
            axr.plot(xs=[x0, x1], ys=[y0, y1], zs=[z0, z1], c="blue")

    # plotStuff()
    # plt.show()
    # assert False

    opt = torch.optim.Adam(params, lr=1)
    N = 1000
    for i in range(N):
        opt.zero_grad()
        err = errFn()
        err.backward()
        opt.step()
        losses.append(err.item())
        progress_bar(i, N)

        if i % 10 == 0:
            plotStuff()
        fig.canvas.flush_events()
        fig.canvas.draw()
        # plt.show()

def findNearestPose(timestamp, poses):
    d_min = 1e6
    p_min = None
    for p in poses:
        t = p[0]
        d = abs(t - timestamp)
        if d < d_min:
            d_min = d
            p_min = p
    assert p_min is not None
    t, x, y, theta = p_min
    return (
        torch.tensor(x, requires_grad=True),
        torch.tensor(y, requires_grad=True),
        torch.tensor(theta, requires_grad=True)
    )

# if __name__ == "__main__":
#     tag = np.random.uniform(11, 11)

#     # (tag, vec3, vec3)[]
#     lmdb = [(
#         tag,
#         torch.tensor([0.0, 0.0, 0.0], requires_grad=True),
#         torch.tensor([0.0, 0.0, 1000.0], requires_grad=True)
#     )]

#     # (x, y, theta)[]
#     robotPoses = [
#         (-1000.0,    0.0, 0.0),
#         (    0.0, 1000.0, -np.pi * 0.5)
#     ]

#     robotPoses = [[torch.tensor(v, requires_grad=True) for v in a] for a in robotPoses]

#     cl = camera_fov_horizontal * 0.25
#     cr = camera_fov_horizontal * 0.75
#     ct = camera_fov_vertical * 0.25
#     cb = camera_fov_vertical * 0.75
#     taggedSegments = [
#         [(cl, ct, cr, cb, tag)],
#         [(cr, ct, cl, cb, tag)]
#     ]

#     posesAndSegments = list(zip(robotPoses, taggedSegments))

#     alignLandmarks(posesAndSegments, lmdb)

#     exit()

if __name__ == "__main__":
    filepaths = list(sorted(glob.glob("../dataset/img_jpeg/img_*.jpeg")))[:80]
    file_ids = [os.path.basename(fp).split(".")[0] for fp in filepaths]

    # HACK BEGIN vvv
    # 
    # begin, end = (0,6)
    # begin, end = (12,35)
    # data = [get_data(fid) for fid in file_ids[begin:end]]
    # images, lines, tags = zip(*data)
    # indices = range(len(data))
    # for i, j in zip(indices, indices[1:]):
    #     plot_landmark_motion(lines[i], lines[j], find_matching_tags(tags[i], tags[j]))
    # plt.gca().set_ylim(0, 144)
    # plt.gca().set_xlim(0, 175)
    # plt.show()
    # exit()
    # # HACK END ^^^

    fileTimeStamp = lambda fid: 0.001 * int(fid.split("_")[-1])

    loadImageAndLines = lambda fid: (load_image(f"{fid}.jpeg"), parse_lines(f"lines_{fid}.txt"))

    imageLineTimestamps = [loadImageAndLines(fid) + (fileTimeStamp(fid),) for fid in file_ids]

    bothSomething = lambda p: p[0] is not None and p[1] is not None

    imageLineTimestamps = list(filter(bothSomething, imageLineTimestamps))

    images, lines, timestamps = zip(*imageLineTimestamps)

    assert len(images) == len(imageLineTimestamps)
    assert len(lines) == len(imageLineTimestamps)
    assert len(timestamps) == len(imageLineTimestamps)
    
    taggedTimeStampedLines = []
    for img, ts, ls in zip(images, timestamps, lines):
        linesOut = []
        for l in ls:
            x0, y0, x1, y1 = l
            tag = extract_line_tag(l, img)
            if tag is None:
                continue
            linesOut.append((x0, y0, x1, y1, tag))
        taggedTimeStampedLines.append((linesOut, ts))

    odo_data = parse_odometry("../dataset/motorLog.txt")

    dead_reckoning_poses = deadReckoning(odo_data, t0=0, x0=0, y0=0, theta0=0)

    poses_and_segments = [(findNearestPose(t, dead_reckoning_poses), segments) for segments, t in taggedTimeStampedLines]

    lmdb = createLandmarkDatabase(
        [taggedLine for taggedLine, timeStamp in taggedTimeStampedLines],
        [pose for pose, segments in poses_and_segments]
    )

    print(f"{len(lmdb)} landmarks were identified")

    alignLandmarks(poses_and_segments, lmdb)
