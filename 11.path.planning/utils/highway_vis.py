"""Draw the path of the provided waypoints"""
#%%
import matplotlib.pyplot as plt

#%%
WAYPOINTS_FILE = "../data/highway_map.csv"
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
for line in zip(X_POINTS, X_NORMAL, Y_POINTS, Y_NORMAL):
    plt.plot(line[:2], line[2:], color='red', linewidth=3)

plt.savefig('highway track.jpg')
plt.show()
