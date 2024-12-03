import os
import pyperclip #type:ignore

def get_latest_file(download_path):
    """Gets the most recently downloaded file in the specified directory."""

    files = [f for f in os.listdir(download_path) if os.path.isfile(os.path.join(download_path, f))]
    if not files:
        raise FileNotFoundError

    latest_file = max(files, key=lambda f: os.path.getmtime(os.path.join(download_path, f)))
    return os.path.join(download_path, latest_file)

with open(get_latest_file(r"C:\Users\ethan\Downloads"), "r") as f:
    data = f.readlines()

# Cuts off the first and last lines
data = data[1:-2] 
nd = []
for line in data:
    line = line.strip().split(",")[:2]

    nl = []
    for i, num in enumerate(line):
        num = round(float(num)*10, 2)
        nl.append(num)

    nd.append(tuple(nl))

path = tuple(nd)

print(path)
pyperclip.copy(str(path))
print("Copied to clipboard!")
