def gen_data():
    data = {
        "start_pos": [-1590, 0],
        "start_heading": 90,
    }
    return data

def run(main):
    drivetrain = main["drivetrain"]
    DirectionType = main["DirectionType"]
    DsitanceUnits = main["DsitanceUnits"]
    VelocityUnits = main["VelocityUnits"]

    drivetrain.drive_for(DirectionType.FORWARD, 500, DsitanceUnits.MM, 40, VelocityUnits.PERCENT, True)
