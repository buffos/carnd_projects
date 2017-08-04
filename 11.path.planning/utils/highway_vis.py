"""Draw the path of the provided waypoints"""
#%%
import matplotlib.pyplot as plt

#%%
WAYPOINTS_FILE = "../data/highway_map.csv"
SPLINES_FILE = "./splines.csv"
SCALE_NORMALS = 50


def read_waypoints(filename: str):
    """Read the data from the csv file"""
    with open(filename) as f:
        x_waypoints = []
        y_waypoints = []
        x_normal = []
        y_normal = []
        for line in f:
            data = line.split(' ')
            x_waypoints.append(float(data[0]))
            y_waypoints.append(float(data[1]))
            x_normal.append(float(data[0]) + SCALE_NORMALS * float(data[3]))
            y_normal.append(float(data[1]) + SCALE_NORMALS * float(data[4]))
    return x_waypoints, y_waypoints, x_normal, y_normal


def read_spline(filename: str):
    """read points for the track from a file"""
    with open(filename) as f:
        x_0 = []  # lane d = 0
        y_0 = []
        x_4 = []  # lane d = 0
        y_4 = []
        x_8 = []  # lane d = 0
        y_8 = []
        x_12 = []  # lane d = 0
        y_12 = []
        for line in f:
            data = line.split(',')
            x_0.append(float(data[0]))
            y_0.append(float(data[1]))
            x_4.append(float(data[2]))
            y_4.append(float(data[3]))
            x_8.append(float(data[4]))
            y_8.append(float(data[5]))
            x_12.append(float(data[6]))
            y_12.append(float(data[7]))
    return x_0, y_0, x_4, y_4, x_8, y_8, x_12, y_12


#%%
plt.figure(figsize=(15, 15))
plt.title('HighWay Map')
plt.xlabel("X", fontsize=10)
plt.ylabel("Y", fontsize=10)

X_POINTS, Y_POINTS, X_NORMAL, Y_NORMAL = read_waypoints(WAYPOINTS_FILE)
N_POINTS = len(X_POINTS)

START = plt.plot([X_POINTS[0]], [Y_POINTS[0]], 'go', ms=10)
END = plt.plot([X_POINTS[N_POINTS - 1]], [Y_POINTS[N_POINTS - 1]], 'ro', ms=10)
POINTS = plt.plot(X_POINTS, Y_POINTS, 'bo', ms=5)
# for line in zip(X_POINTS, X_NORMAL, Y_POINTS, Y_NORMAL):
#     plt.plot(line[:2], line[2:], color='red', linewidth=3)
x_0, y_0, x_4, y_4, x_8, y_8, x_12, y_12 = read_spline(SPLINES_FILE)
D1 = plt.plot(x_0, y_0, 'b')
D4 = plt.plot(x_4, y_4, 'm')
D8 = plt.plot(x_8, y_8, 'g')
D12 = plt.plot(x_12, y_12, 'r')


plt.savefig('highway track.jpg')
plt.show()
