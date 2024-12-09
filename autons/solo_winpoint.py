def gen_data():
    data = {
        "start_pos": [-1360, -790],
        "start_heading": 90,
    }
    return data

def gen_paths(main):
        paths = {
             # Grab ring at (-600, -600)
            "ring_1": {
                "points": ((-1360.0, -790.0), (-1232.92, -817.39), (-1106.37, -847.11), (-981.12, -881.85), (-859.02, -926.23), (-744.67, -987.57), (-651.69, -1077.29), (-600.49, -1195.13)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False,)            
            },
            "mogo_1": {
                "points": ((-600.31, -1218.89), (-602.98, -1088.92), (-605.2, -958.94), (-606.65, -828.94), (-607.26, -698.95), (-607.38, -568.95), (-607.38, -518.95)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)            
            },
            "ring_2": {
                "points": ((-600.49, -615.39), (-644.28, -737.32), (-729.0, -832.25), (-853.86, -838.18), (-958.89, -763.16), (-1035.85, -658.77), (-1094.52, -542.95), (-1135.59, -419.69), (-1163.77, -292.89), (-1180.34, -164.05), (-1187.13, -34.26), (-1187.47, 95.74), (-1188.25, 225.73), (-1190.01, 355.72), (-1193.11, 485.68), (-1197.21, 615.62), (-1201.23, 745.56), (-1204.67, 875.51), (-1207.57, 1005.48), (-1208.52, 1053.14)),
                "events": [[["drop goal"], (-1000,600), main["mogo_pneu"].set, (False)],[["stop chain"], (-1000, 600), main["misc"]["intake_chain"].stop, ()],[["raise intake"], (-1000, 600), main["intake_pneu"].set, (True)],],
                "checkpoints": [],
                "custom_args": (False,)            
            },
        }
        return paths

def path_run(controller, path):
    controller.path(path["points"], path["events"], path["checkpoints"], *path["custom_args"])

def run(main):
    paths = gen_paths(main)

    # Refereneces for easier access
    # Global Types
    BrakeType = main["BrakeType"]
    DirectionType = main["DirectionType"]
    VelocityUnits = main["VelocityUnits"]
    TimeUnits = main["TimeUnits"]
    RotationUnits = main["RotationUnits"]
    TurnType = main["TurnType"]
    # Custom objects
    motors = main["motors"]
    brain = main["brain"]
    controller = main["auton"]
    drivetrain = main["drivetrain"]

    controller.dynamic_vars["fwd_speed"] = 4
    main["mogo_pneu"].set(False)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    path_run(controller, paths["ring_1"])
    path_run(controller, paths["mogo_1"])
    main["sleep"](100)
    main["mogo_pneu"].set(True)
    main["sleep"](200)
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    
    path_run(controller, paths["ring_2"])

    
    