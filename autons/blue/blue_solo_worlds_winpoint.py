def gen_data():
    data = {
        "start_pos": [1560.0, 330.0],
        "start_heading": 150,
    }
    return data

def run(main):
    #region Bindings
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
    log = main["log"]
    flags = main["flags"]
    ColorSort = main["ColorSort"]
    sleep = main["sleep"]
    #endregion Bindings

    paths = [[(1547.38, 312.87, 0), (1521.81, 399.14, 0.17), (1489.72, 483.18, 0.19), (1450.73, 564.24, 0.25), (1403.02, 640.48, 0.31), (1345.13, 709.18, 0.39), (1276.14, 766.61, 0.53), (1195.49, 805.97, 0.6), (1107.42, 822.31, 0.52), (1017.82, 817.44, 0.49), (931.39, 792.98, 0.34), (849.59, 755.75, 0.25), (772.43, 709.61, 0.19), (699.42, 657.11, 0.14), (629.82, 600.06, 0.14), (564.03, 538.65, 0.09), (500.91, 474.55, 0.05)], [(576.13, 492.17, 0), (519.41, 562.01, 0.17), (457.62, 627.42, 0.16), (400.96, 697.07, 0.68), (368.38, 780.51, 0.5), (355.3, 869.47, 0.22), (351.03, 959.34, 0.1), (350.89, 1049.34, 0.06), (353.18, 1139.31, 0.03), (356.82, 1229.23, 0.45), (377.48, 1313.89, 0.83), (429.9, 1386.95, 0.5), (496.87, 1446.09, 2.12), (574.27, 1419.97, 1.55), (615.64, 1340.3, 0.14), (651.98, 1257.99, 0.61), (708.21, 1189.65, 1.74), (796.68, 1189.56, 0.24), (886.17, 1199.11, 0.01), (975.7, 1208.29, 0.03), (1065.35, 1216.29, 0.05), (1155.15, 1222.19, 0.1), (1245.11, 1223.86, 0.3), (1334.32, 1213.4, 0.77), (1413.5, 1173.89, 1.44), (1449.62, 1093.29, 0.84), (1452.67, 1003.49, 0.26), (1445.1, 913.84, 0.1), (1433.62, 824.58, 0.03), (1421.04, 735.46, 0.0), (1408.38, 646.36, 0.02), (1396.37, 557.16, 0.02), (1385.3, 467.85, 0.02), (1375.04, 378.44, 0.03), (1365.98, 288.89, 0.03), (1358.25, 199.23, 0.02), (1351.15, 109.51, 0.05), (1346.0, 19.66, 0.01), (1341.06, -70.2, 0.08), (1339.38, -160.16, 0.05), (1339.75, -250.16, 0.06), (1337.67, -340.12, 0.04), (1334.14, -430.05, 0.1), (1326.44, -519.71, 0.06), (1316.36, -609.11, 0.08), (1303.07, -698.12, 0.14), (1284.0, -786.08, 0.13), (1259.74, -872.7, 0.17), (1229.09, -957.23, 0.21), (1190.67, -1038.51, 0.25), (1143.27, -1114.89, 0.32), (1085.43, -1183.67, 0.37), (1017.0, -1241.9, 0.34), (940.57, -1289.16, 0.36), (857.45, -1323.38, 0.26), (770.83, -1347.56, 0.22), (682.23, -1363.17, 0.14), (592.81, -1373.25, 0.07), (503.09, -1380.32, 0.01), (413.33, -1386.88, 0.05)], [(377.5, -1376.02, 0), (432.04, -1304.46, 0.22), (493.34, -1238.62, 0.14), (558.77, -1176.83, 0.03), (623.32, -1114.13, 0.28), (679.53, -1044.02, 0.4), (722.07, -964.88, 0.39), (750.12, -879.44, 0.27), (767.59, -791.19, 0.19), (777.58, -701.77, 0.13), (782.52, -611.92, 0.12)], [(640.55, -640.55, 0), (631.85, -551.0, 0.22), (632.24, -461.08, 0.31), (645.01, -372.1, 0.34), (671.18, -286.07, 0.24), (706.44, -203.28, 0.03), (740.54, -120.01, 0.28), (763.83, -33.18, 0.37), (772.2, 56.32, 0.31)]]
    sensor.imu.set_heading(150)
    controller.robot.pos = [1560.0, 330.0]

    flags.color_setting = ColorSort.EJECT_RED
    motor.ladyBrown.spin(DirectionType.FORWARD, 80, VelocityUnits.PERCENT)
    sleep(400, TimeUnits.MSEC)

    controller.fwd_speed = 7
    brain.timer.event(motor.ladyBrown.spin, 800, (DirectionType.REVERSE, 80, VelocityUnits.PERCENT))
    controller.path(paths[0], backwards=True, speed_ramp_time=400, slowdown_distance=600)
    motor.ladyBrown.stop(BrakeType.COAST)
    pneumatic.mogo.set(True)
    drivetrain.turn_for(TurnType.LEFT, 50, RotationUnits.DEG, 100, VelocityUnits.PERCENT)

    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 5
    controller.path(paths[1], speed_ramp_time=400, heading_authority=1.7, event_look_ahead_dist=200, 
    events=[
        ["intake", (1200.0, 825.0), pneumatic.intake.set, (True,)],
        ["drop", (1190.0, -840.0), pneumatic.mogo.set, (False,)],
        ["turn off color sort", (1190.0, -840.0), "flags", "color_setting", ColorSort.NONE],
        ["lower", (1200.0, -500.0), pneumatic.intake.set, (False,)],
        ["autostop for last ring", (1000.0, -1100.0), "AutonomousFlags", "intake_auto_halt", True],
        ["speed to launch", (1240.0, 1146.0), "fwd_speed", 7],
        ["slow for stack", (1400.0, 644.9), "fwd_speed", 4.5],
    ],
    checkpoints=[12,],
    )

    controller.fwd_speed = 7
    controller.path(paths[2], backwards=True, speed_ramp_time=400)
    pneumatic.mogo.set(True)

    sleep(150, TimeUnits.MSEC)
    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    drivetrain.turn_for(TurnType.LEFT, 160, RotationUnits.DEG, 80, VelocityUnits.PERCENT)
    controller.fwd_speed = 5.5
    controller.path(paths[3], speed_ramp_time=400, timeout=800)
