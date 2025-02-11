import os
from math import pi, atan2, sin, dist
# import pyperclip

def get_latest_file(download_path):
    """Gets the most recently downloaded file in the specified directory."""

    files = [f for f in os.listdir(download_path) if os.path.isfile(os.path.join(download_path, f))]
    if not files:
        raise FileNotFoundError

    latest_file = max(files, key=lambda f: os.path.getmtime(os.path.join(download_path, f)))
    print(f"Parsing {latest_file}")
    return os.path.join(download_path, latest_file)

with open(get_latest_file(r"C:\Users\ethan\Downloads"), "r") as f:
    data = f.readlines()

#* READ SECTIONS THAT CORRESPOND TO EACH PATH
all_paths_segments = []
# Cuts off the first and last lines
path_segments = [1]
for i, line in enumerate(data[1:]):
    if "#PATH-" in line:
        # we have found the start of the next path
        # finish the previous path and start the new one
        # finish the previous path
        path_segments.append(i)
        all_paths_segments.append(path_segments)
        path_segments = [i+2]
    if "PATH." in line:
        # last marker, tie off last path
        path_segments.append(i)
        all_paths_segments.append(path_segments)
print(all_paths_segments)

# # get data from line with #PATH-POINTS-START - #PATH-POINTS-START and #PATH-POINTS-START - #PATH.JERRYIO-DATA
#* SPLIT BIG DATA INTO EACH PATH
ap = []
for path_number, path in enumerate(all_paths_segments):
    # print(path)
    this_path_data = data[path[0]:path[1]]
    nd = []
    for line in this_path_data:
        line = line.strip().split(",")[:2] # only keep x, y
        nl = []
        for i, num in enumerate(line):
            num = round(float(num)*10, 2)
            nl.append(num)

        nd.append(tuple(nl))

    path = tuple(nd)
    ap.append(path)
    # print("#"*20)
    # print(f"Path {path_number+1}")
    # print(path, end=",\n")

# calculate curvature of each point

def angle_diff(a, b):
    diff = a - b
    while diff > pi:
        diff -= 2 * pi
    while diff < -pi:
        diff += 2 * pi
    return diff

def lerp(a, b, t):
    return a * t + (b - a)

def calculate_curvature(path) -> list:
    np = []
    for i in range(len(path)):
        point = path[i][:2]
        if i + 1 < len(path):
            point2 = path[i + 1][:2]
        else:
            continue

        bx, by = point2
        ax, ay = point

        # point curvature
        prev_point = path[max(0, i - 1)]
        # atan2(by​−ay​, bx​−xi​)−atan2(yi​−yi−1​,xi​−xi−1​)
        curvature_heading = angle_diff(atan2(by-ay, bx-ax), atan2(ay-prev_point[1], ax-prev_point[0]))
        curvature = 2*sin(curvature_heading) / dist(point, point2)
        if i == 0: curvature = 0
        output = abs(round(curvature * 100, 2))

        np.append((*path[i], output))
        # print(np[i])

    return np

ap2 = []
for path in ap:
    ap2.append(calculate_curvature(path))

print(ap2)
# print(path)
# pyperclip.copy(str(path))
# print("Copied to clipboard!")
