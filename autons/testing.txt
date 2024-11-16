def gen_data():
    data = {
        "start_pos": [-1590, 0],
        "start_heading": 90,
    }
    return data

def gen_paths(main):
        paths = {
             # Grab ring at (-600, -600)
            "ring_1": {
                "points": ((-1590.0, 0.0), (-1460.52, -11.26), (-1332.05, -30.99), (-1205.2, -59.33), (-1080.69, -96.5), (-959.89, -144.36), (-846.48, -207.58), (-746.48, -289.94), (-673.36, -396.28), (-622.42, -515.34), (-599.41, -642.05), (-620.32, -770.09), (-555.34, -876.76), (-443.26, -942.34), (-317.15, -997.86)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False, 350, 150, 75, None, 1.5)            
            },
            # Positive corners side
            "mogo_1": {
                # "points": ((-317.0, -997.0), (-413.31, -925.72), (-524.79, -882.81), (-643.48, -865.67), (-761.36, -843.94), (-874.23, -803.79), (-980.94, -749.09), (-1081.83, -684.2), (-1178.16, -612.65), (-1270.92, -536.52), (-1358.14, -459.91)),
                "points": ((-317.0, -997.0), (-412.75, -924.9), (-521.62, -875.26), (-638.06, -846.5), (-754.6, -818.03), (-867.12, -776.52), (-974.02, -722.2), (-1075.84, -658.8), (-1173.35, -588.94), (-1267.3, -514.33), (-1400.0, -399.97)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)    
            },
            "bottom_red_rings": {
                "points": ((-1358.0, -460.0), (-1259.42, -528.42), (-1155.52, -588.36), (-1047.01, -639.49), (-934.89, -682.15), (-820.01, -716.77), (-703.81, -746.74), (-587.84, -777.45), (-480.09, -827.79), (-447.78, -935.22), (-495.23, -1044.95), (-560.15, -1145.8), (-632.06, -1241.84), (-706.95, -1335.59), (-785.71, -1426.1), (-869.88, -1511.57), (-962.02, -1588.2), (-1069.73, -1638.87), (-1175.73, -1600.71), (-1220.98, -1490.8), (-1181.23, -1397.87), (-1082.76, -1329.34), (-990.64, -1252.68), (-1048.17, -1198.51), (-1167.95, -1205.15), (-1287.77, -1211.44), (-1407.67, -1208.94), (-1524.18, -1182.57), (-1612.32, -1106.56), (-1617.94, -991.17), (-1588.85, -874.81), (-1543.76, -763.94), (-1462.21, -677.72), (-1368.13, -603.67), (-1295.7, -508.32), (-1213.66, -351.07)),
                "events": [],
                "checkpoints": [12,22,27,],
                "custom_args": (False, 250,)
            }
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

    motors["intake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    main["sleep"](500)
    motors["intake"].stop(BrakeType.COAST)
    main["sleep"](100)

    controller.dynamic_vars["intake_auto_halt"] = True
    controller.dynamic_vars["fwd_speed"] = 5.5
    motors["intake"].spin(DirectionType.FORWARD, 80, VelocityUnits.PERCENT)
    path_run(controller, paths["ring_1"])
    main["mogo_pneu"].set(True)
    motors["intake"].stop(BrakeType.HOLD)
    main["sleep"](100)

    controller.dynamic_vars["mogo_listen"] = True
    path_run(controller, paths["mogo_1"])
    main["mogo_pneu"].set(False)
    main["sleep"](150)

    controller.dynamic_vars["intake_auto_halt"] = False
    motors["intake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.dynamic_vars["fwd_speed"] = 3.5
    main["sleep"](100)
    path_run(controller, paths["bottom_red_rings"])
    main["sleep"](100)
    motors["intake"].stop(BrakeType.COAST)