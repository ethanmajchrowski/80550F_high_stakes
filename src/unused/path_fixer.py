import os
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
    print("#"*20)
    print(f"Path {path_number+1}")
    print(path, end=",\n")

# print(path)
# pyperclip.copy(str(path))
# print("Copied to clipboard!")
