def gen_data():
    data = {
        "start_pos": [1350, -1530],
        "start_heading": 90,
    }
    return data

def gen_paths(main):
        paths = {
             # Grab ring at (-600, -600)
            "mogo_rush": {
                "points": ((1350.0, -1530.0), (1220.06, -1525.99), (1090.15, -1521.22), (960.3, -1515.03), (830.62, -1505.94), (701.46, -1491.43), (573.8, -1467.35), (449.97, -1428.28), (332.39, -1373.08), (220.2, -1307.47), (110.76, -1237.31)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True, 200, 150)            
            },
            "corner": {
                "points": ((141.86, -1357.75), (256.29, -1419.43), (373.9, -1474.64), (499.43, -1506.94), (629.19, -1512.71), (759.13, -1509.01), (889.01, -1503.44), (1018.9, -1498.12), (1148.82, -1493.64), (1278.65, -1491.9), (1396.9, -1442.61), (1473.9, -1339.24), (1516.77, -1216.71), (1520.52, -1202.2)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False,)            
            },
            "ring_2": {
                "points": ((1513.45, -1180.99), (1491.93, -1052.8), (1466.71, -925.27), (1439.0, -798.26), (1410.99, -671.31), (1385.26, -543.89), (1364.1, -415.64), (1349.21, -286.52), (1342.46, -156.71), (1343.77, -0.3)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False,)            
            },
            "mogo_2": {
                "points": ((1223.58, 35.05), (1222.83, -94.92), (1213.83, -224.55), (1194.3, -353.0), (1157.45, -477.42), (1096.49, -591.61), (997.71, -674.32), (870.74, -695.95), (745.08, -665.56), (628.36, -609.04), (519.79, -537.86), (417.59, -459.85)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)            
            },
            "ring_3": {
                "points": ((615.56, -551.76), (687.31, -660.11), (767.71, -762.23), (841.13, -868.86), (828.92, -993.25), (755.23, -1099.99), (669.12, -1197.21), (556.42, -1257.35), (455.7, -1197.49), (420.62, -1073.46), (412.52, -943.85), (416.43, -813.98), (428.41, -684.58), (446.64, -555.89), (470.35, -428.07), (500.3, -301.56), (536.09, -176.61), (579.03, -53.96), (631.96, 64.69), (707.47, 176.46)),
                "events": [[["reverse intake"], (300, -1100), main["motors"]["intake"].spin, (main["DirectionType"].REVERSE, 50, main["PercentUnits"].PERCENT)],],
                "checkpoints": [7,],
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
    main["mogo_pneu"].set(True)
    path_run(controller, paths["mogo_rush"])
    main["mogo_pneu"].set(False)
    main["sleep"](500)
    motors["intake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.dynamic_vars["fwd_speed"] = 5.5
    path_run(controller, paths["corner"])
    main["mogo_pneu"].set(True)
    controller.dynamic_vars["fwd_speed"] = 6.5
    main["intake_pneu"].set(True)
    controller.dynamic_vars["intake_auto_halt"] = True 
    motors["intake"].spin(DirectionType.FORWARD, 50, VelocityUnits.PERCENT)
    path_run(controller, paths["ring_2"])
    main["intake_pneu"].set(False)
    main["sleep"](200)
    main["mogo_pneu"].set(True)
    controller.dynamic_vars["fwd_speed"] = 4.5
    path_run(controller, paths["mogo_2"])
    controller.dynamic_vars["intake_auto_halt"] = False 
    main["mogo_pneu"].set(False)
    motors["intake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    path_run(controller, paths["ring_3"])

    
    
