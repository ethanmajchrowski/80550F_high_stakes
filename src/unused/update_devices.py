import os, datetime

print("")

with open(r"C:\Users\ethan\Documents\vex-vscode-projects\80550F_high_stakes\src\unused\devices.py", "r") as f:
    edited_devices = []
    for line in f.readlines():
        line = line.strip("\n")
        edited_devices.append(line)

# steps are as follows:
# 1. Delete all code between lines #start and #end
# 2. Paste in code at the start of file

for file in os.scandir(r"C:\Users\ethan\Documents\vex-vscode-projects\80550F_high_stakes\src"):
    if file.name[-3:] == ".py":
        # gets the filename without the ".py" at the end
        # file.name gives the full name
        filename = file.name[:-3]

        with open(file, "r") as f:
            lines = f.readlines()
            edited_lines = []
            for line in lines:
                line = line.strip("\n")
                edited_lines.append(line)
            lines = edited_lines
            try: 
                end_index = lines.index("#end_1301825#")
                print(f"END TAG FOUND AT INDEX {end_index} FOR FILE {file.name}")
                found_tag = True
            except:
                print(f"NO END TAG FOUND FOR FILE {file.name}")
                found_tag = False

            if found_tag:
                with open(file, "w") as f:
                    # start_writing = False
                    # here we write the persistent code
                    # start with headers (filename, date updated, etc.)
                    f.write(f"# Filename: {file.name}\n")
                    f.write(f"# Devices & variables last updated:\n\t# {datetime.datetime.now()}\n")
                    f.write("#"*20 + "\n")
                    f.write("#region Devices\n")

                    for line in edited_devices:
                        f.write(line + "\n")

                    f.write("#endregion Devices")
                    f.write("#"*20)
                    f.write("\n#DO NOT CHANGE THE FOLLOWING LINE:#\n")
                    f.write("#end_1301825#")

                    # here we put the code back
                    for line in lines[end_index+1::]:
                        f.write("\n")
                        f.write(line)
                        # print(line)