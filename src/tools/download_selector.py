from json import load, dump
import pick

files = {
    "driver": {
        "slot": 2,
        "path": "src/driver.py"
    },
    "comp": {
        "slot": 1,
        "path": "src/comp.py"
    },
    "config": {
        "slot": 3,
        "path": "src/config.py"
    }
}

options = list(files.keys())
option, index = pick.pick(options, "Title", indicator='=>', default_index=0)

def main():
    # Makes it easy to modify the vex_project_settings.json file for easier downloading to the brain
    with open(r".vscode\vex_project_settings.json") as f:
        data = load(f)

    try:
        selected = str(option)
    except:
        print("Could not cast selected to str")
        return

    if selected not in files.keys():
        print("Invalid setting")
        return

    data["project"]["name"] = selected
    data["project"]["slot"] = files[selected]["slot"]
    data["project"]["python"]["main"] = files[selected]["path"]

    with open(r".vscode\vex_project_settings.json", "w") as f:
        dump(data, f, indent=4)

if __name__ == "__main__":
    main()
