def gen_data():
    # The code to parse this is in autonomous_setup() in comp 2.0
    data = {
        "start_pos": [0, 0],
        "start_heading": 0
    }
    return data

def run(main):
    motor = main["motor"]
    pneumatic = main["pneumatic"]
    drivetrain = main["drivetrain"]
    
    BrakeType = main["BrakeType"]
    DirectionType = main["DirectionType"]
    VelocityUnits = main["VelocityUnits"]
    TimeUnits = main["TimeUnits"]
    RotationUnits = main["RotationUnits"]
    TurnType = main["TurnType"]
    VoltageUnits = main["VoltageUnits"]
    # Custom objects
    robot = main["robot"]
    controller = robot.autonomous_controller
    color_sorter = robot.color_sort_controller
    drivetrain = main["drivetrain"]
    sensor = main["sensor"]
    imu = sensor.imu
    brain = main["brain"]

    paths = [((0.81, 1.89), (-2.87, 71.75), (-17.16, 140.18), (-42.16, 205.41), (-76.93, 266.0), (-119.85, 321.17), (-169.07, 370.86), (-223.07, 415.37), (-280.41, 455.49), (-339.94, 492.31), (-400.99, 526.54), (-463.08, 558.85), (-525.84, 589.86), (-588.96, 620.12), (-652.17, 650.18), (-715.21, 680.61), (-777.78, 711.98), (-839.52, 744.95), (-899.96, 780.24), (-958.41, 818.71), (-1013.94, 861.27), (-1064.84, 909.25), (-1108.4, 963.91), (-1142.67, 1024.75), (-1165.42, 1090.78), (-1174.64, 1184.0))]

    controller.fwd_speed = 8
    controller.path(paths[0])
