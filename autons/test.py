def gen_data():
    data = {
        "start_pos": [1500, 200],
        "start_heading": 360,
    }
    return data

def gen_paths(main):
        paths = {
                "test_curve": {
                     "points": ((1500.0, 440.0), (1476.92, 342.78), (1470.47, 243.15), (1488.77, 145.2), (1535.11, 57.18), (1607.84, -10.83), (1696.46, -56.49), (1792.84, -82.51), (1892.14, -93.34), (1966.28, -93.82)),
                     "events": [],
                     "checkpoints": [],
                     "custom_args": (True, 350, 100, 75, None, 1)
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
    VoltageUnits = main["VoltageUnits"]
    # Custom objects
    motors = main["motors"]
    controller = main["auton"]
    drivetrain = main["drivetrain"]

    controller.dynamic_vars["fwd_speed"] = 3

    # path_run(controller, paths["test_curve"])
    # back up into alliance stake
    motors["left"]["A"].spin(DirectionType.REVERSE, 3, VoltageUnits.VOLT)
    motors["left"]["B"].spin(DirectionType.REVERSE, 3, VoltageUnits.VOLT)
    motors["left"]["C"].spin(DirectionType.REVERSE, 3, VoltageUnits.VOLT)

    motors["right"]["A"].spin(DirectionType.REVERSE, 3, VoltageUnits.VOLT)
    motors["right"]["B"].spin(DirectionType.REVERSE, 3, VoltageUnits.VOLT)
    motors["right"]["C"].spin(DirectionType.REVERSE, 3, VoltageUnits.VOLT)

    main["sleep"](750)

    controller.kill_motors()
