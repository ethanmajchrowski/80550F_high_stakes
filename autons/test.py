def gen_data():
    data = {
        "start_pos": [-1500, 300],
        "start_heading": 0,
    }
    return data

def gen_paths(main):
        paths = {
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

    motors['intake'].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    main["sleep"](1000000)