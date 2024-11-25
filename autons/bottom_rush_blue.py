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
                "points": ((1350.0, -1530.0), (1220.06, -1525.99), (1090.15, -1521.22), (960.3, -1515.03), (830.62, -1505.94), (701.46, -1491.43), (573.8, -1467.35), (449.97, -1428.28), (332.39, -1373.08), (220.2, -1307.47), (110.76, -1237.31), (57.02, -1202.2)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)            
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

    main["mogo_pneu"].set(False)
    path_run(controller, paths["mogo_rush"])
    main["mogo_pneu"].set(True)
    main["sleep"](100)
    motors["misc"]["intake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    
    
