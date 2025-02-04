def gen_data():
    data = {
        "start_pos": [-1590, 0],
        "start_heading": 90,
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
    DsitanceUnits = main["DistanceUnits"]
    TurnType = main["TurnType"]
    # Custom objects
    motors = main["motors"]
    brain = main["brain"]
    controller = main["auton"]
    drivetrain = main["drivetrain"]
    imu = main["imu"]

    drivetrain.drive_for(DirectionType.FORWARD, 500, DsitanceUnits.MM, 40, VelocityUnits.PERCENT, True)
