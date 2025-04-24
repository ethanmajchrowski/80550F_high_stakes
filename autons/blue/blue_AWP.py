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

    paths = [[(1547.38, 312.87, 0), (1509.84, 405.3, 0.51), (1449.82, 485.02, 0.46), (1373.22, 549.05, 0.38), (1285.75, 597.23, 0.33), (1191.58, 630.47, 0.29), (1093.57, 649.61, 0.28), (993.85, 654.94, 0.28), (894.34, 646.14, 0.31), (797.41, 621.95, 0.35), (706.28, 581.18, 0.46)], [(576.13, 492.17, 0), (523.6, 577.15, 0.28), (483.29, 668.58, 0.22), (453.25, 763.91, 0.17), (431.37, 861.45, 0.13), (415.67, 960.19, 0.09), (404.4, 1059.55, 0.05), (395.83, 1159.18, 0.02), (388.09, 1258.88, 0.04), (378.5, 1358.41, 1.96), (461.27, 1402.91, 0.66), (559.22, 1418.49, 1.22), (645.27, 1374.28, 1.24), (687.09, 1284.25, 0.32), (714.08, 1188.01, 1.84), (795.5, 1156.71, 0.81), (895.37, 1161.49, 0.17), (995.24, 1157.71, 0.23), (1093.89, 1142.3, 0.41), (1187.11, 1107.14, 0.62), (1264.9, 1045.35, 0.69), (1316.95, 960.57, 0.54), (1344.49, 864.61, 0.31), (1356.97, 765.45, 0.19), (1360.0, 665.52, 0.1), (1358.19, 565.54, 0.08), (1352.15, 465.74, 0.09), (1341.44, 366.33, 0.07), (1327.26, 267.34, 0.04), (1311.34, 168.61, 0.01), (1295.76, 69.84, 0.08), (1283.91, -29.44, 0.22), (1282.79, -129.31, 0.47), (1305.12, -226.18, 0.83), (1365.06, -304.81, 1.14)], [(1446.26, -344.01, 0), (1378.21, -271.14, 0.83), (1346.34, -177.62, 0.65), (1346.82, -77.7, 0.09), (1352.03, 22.16, 0.05)], [(1349.18, 94.91, 0), (1303.19, 6.14, 0.15), (1264.05, -85.86, 0.14), (1231.49, -180.39, 0.13), (1205.05, -276.81, 0.11), (1184.06, -374.57, 0.1), (1167.77, -473.23, 0.08), (1155.52, -572.47, 0.07), (1146.6, -672.06, 0.05), (1140.29, -771.86, 0.04), (1136.03, -871.77, 0.37), (1114.25, -967.15, 0.63), (1062.54, -1052.65, 0.31), (998.28, -1129.13, 0.31), (922.81, -1194.55, 0.3), (838.33, -1247.85, 0.28), (747.2, -1288.82, 0.23), (651.96, -1319.13, 0.18), (554.36, -1340.78, 0.12)], [(377.5, -1376.02, 0), (429.95, -1290.91, 0.2), (473.68, -1201.02, 0.21), (507.89, -1107.1, 0.19), (532.93, -1010.33, 0.17), (549.8, -911.8, 0.13), (560.04, -812.34, 0.1), (565.45, -712.5, 0.06), (567.84, -612.53, 0.02)], [(564.73, -511.29, 0), (554.9, -411.81, 0.25), (557.32, -311.98, 0.46), (582.61, -215.64, 0.48), (630.22, -127.79, 0.26), (665.73, -34.72, 0.54), (674.82, 64.63, 0.39)]]
    sensor.imu.set_heading(150)
    controller.robot.pos = [1560.0, 330.0]

    flags.color_setting = ColorSort.EJECT_RED
    motor.ladyBrown.spin(DirectionType.FORWARD, 80, VelocityUnits.PERCENT)
    sleep(400, TimeUnits.MSEC)

    controller.fwd_speed = 7
    controller.path(paths[0], backwards=True, speed_ramp_time=400, slowdown_distance=600)
    
    motor.ladyBrown.spin(DirectionType.REVERSE, 80, VelocityUnits.PERCENT)
    pneumatic.mogo.set(True)
    drivetrain.turn_for(TurnType.LEFT, 50, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    motor.ladyBrown.stop(BrakeType.COAST)

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
