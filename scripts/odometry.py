import numpy as np
import csv
import math

def parse_odometry(filepath):
    data = []
    with open(filepath, "r") as infile:
        rdr = csv.reader(infile, delimiter=" ")
        for row in rdr:
            datarow = []
            for val in row:
                datarow.append(int(val))
            data.append(datarow)
    data = np.array(data, dtype="float64")

    # conversion from cumulative measurements (time and ticks) to differences
    data = data[1:] - data[:-1]

    # conversion from milliseconds to seconds
    data[:,0] *= 0.001

    # conversion from ticks to radians
    data[:,1] *= (np.pi / 180.0)
    data[:,2] *= (np.pi / 180.0)

    return data

wheelRadius = 27.3449 # mm
wheelDisplacement = 153.4351 # mm

def stepOdometry(t, x, y, theta, dt, deltaThetaL, deltaThetaR):
    dLeft = deltaThetaL * 2.0 * wheelRadius
    dRight = deltaThetaR * 2.0 * wheelRadius
    dCenter = (dLeft + dRight) * 0.5
    phi = (dRight - dLeft) / (wheelDisplacement * 2)

    if abs(phi) < (np.pi * 0.01):
        dx = dCenter * math.cos(theta)
        dy = dCenter * math.sin(theta)
    else:
        rCenter = dCenter / phi
        dx = rCenter * (-math.sin(theta) + math.sin(phi + theta))
        dy = rCenter * (math.cos(theta) - math.cos(phi + theta))
    dTheta = phi

    return t + dt, x + dx, y + dy, theta + dTheta

def deadReckoning(odometry_data, t0, x0, y0, theta0):
    t = t0
    x = x0
    y = y0
    theta = theta0
    poses = [[t, x, y, theta]]

    for dt, dl, dr in odometry_data:
        t, x, y, theta = stepOdometry(t, x, y, theta, dt, dl, dr)
        poses.append([t, x, y, theta])

    poses = np.array(poses)
    return poses

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    odo_data = parse_odometry("../dataset/motorLog.txt")
    poses = deadReckoning(odo_data, 0, 0, 0, 0)

    last_pose = poses[-1]
    dist_err = math.hypot(last_pose[1], last_pose[2])
    print("Distance error: ", dist_err)
    ang_err = abs(last_pose[3]) - (2.0 * math.pi)
    print("Angular error: ", ang_err)

    plt.plot(poses[:,1], poses[:,2], c="black")
    # plt.scatter(poses[:,0], poses[:,1], s=2, c="black")
    plt.axis('equal')
    plt.show()
