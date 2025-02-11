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

    paths = ((-848.31, 1117.4), (-850.76, 1047.44), (-849.51, 977.5), (-843.3, 907.86), (-830.61, 839.11), (-809.92, 772.37), (-780.02, 709.24), (-740.4, 651.68), (-690.65, 602.58), (-634.45, 561.0), (-573.51, 526.66), (-509.9, 497.49), (-445.09, 471.04), (-379.99, 445.3), (-315.54, 418.02), (-252.78, 387.07), (-193.61, 349.79), (-139.53, 305.49), (-93.92, 252.57), (-57.88, 192.73), (-31.37, 128.09), (-13.72, 60.46), (-3.55, -8.72), (0.69, -78.53), (0.0, -174.6))

    controller.fwd_speed = 8
    controller.path(paths[0])
