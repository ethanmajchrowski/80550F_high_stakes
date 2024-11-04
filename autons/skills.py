def gen_data():
    data = {
        "start_pos": [-1590, 0],
        "start_heading": 90,
    }
    return data

def gen_paths(main):
        paths = {
            "ring_1": {
                "points": ((-1590.0, 0.0), (-1460.52, -11.26), (-1332.05, -30.99), (-1205.2, -59.33), (-1080.69, -96.5), (-959.89, -144.36), (-846.48, -207.58), (-746.48, -289.94), (-673.36, -396.28), (-622.42, -515.34), (-599.41, -642.05), (-620.32, -770.09), (-555.34, -876.76), (-443.26, -942.34), (-317.15, -997.86)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False, 350, 150, 75, None, 1.5)            
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
    motors["intake"].stop(BrakeType.HOLD)
    main["sleep"](100)