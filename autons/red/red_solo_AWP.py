def gen_data():
    data = {
        "start_pos": [-1560, 330],
        "start_heading": 210,
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
    paths = [[(-1547.38, 312.87, 0), (-1521.81, 399.14, 0.17), (-1489.72, 483.18, 0.19), (-1450.73, 564.24, 0.25), (-1403.02, 640.48, 0.31), (-1345.13, 709.18, 0.39), (-1276.14, 766.61, 0.53), (-1195.49, 805.97, 0.6), (-1107.42, 822.31, 0.52), (-1017.82, 817.44, 0.49), (-931.39, 792.98, 0.34), (-849.59, 755.75, 0.25), (-772.43, 709.61, 0.19), (-699.42, 657.11, 0.14), (-629.82, 600.06, 0.14), (-564.03, 538.65, 0.09), (-500.91, 474.55, 0.05)], [(-576.13, 492.17, 0), (-520.44, 562.85, 0.09), (-462.02, 631.31, 0.14), (-408.01, 703.2, 0.45), (-369.7, 784.41, 0.42), (-347.51, 871.55, 0.24), (-334.65, 960.59, 0.13), (-326.96, 1050.25, 0.08), (-322.37, 1140.13, 0.05), (-319.65, 1230.08, 0.03), (-318.28, 1320.07, 1.03), (-355.14, 1397.73, 0.68), (-416.54, 1463.36, 0.8), (-495.52, 1502.5, 2.01), (-567.26, 1455.31, 1.12), (-607.39, 1374.93, 0.19), (-640.67, 1291.31, 0.26), (-683.34, 1212.43, 1.63), (-763.18, 1185.94, 0.92), (-852.66, 1195.6, 0.01), (-942.19, 1204.86, 0.02), (-1031.78, 1213.34, 0.04), (-1121.51, 1220.19, 0.09), (-1211.42, 1223.58, 0.18), (-1301.27, 1219.82, 0.54), (-1387.13, 1194.64, 1.32), (-1441.34, 1125.86, 1.13), (-1453.38, 1037.18, 0.42), (-1448.44, 947.36, 0.13), (-1438.11, 857.97, 0.05), (-1425.77, 768.82, 0.01), (-1413.1, 679.72, 0.01), (-1400.86, 590.55, 0.02), (-1389.32, 501.3, 0.02), (-1378.58, 411.94, 0.04), (-1369.37, 322.42, 0.02), (-1360.9, 232.82, 0.03), (-1353.81, 143.1, 0.03), (-1347.85, 53.31, 0.03), (-1342.91, -36.56, 0.03), (-1339.28, -126.47, 0.1), (-1339.89, -216.47, 0.02), (-1339.63, -306.46, 0.06), (-1336.85, -396.41, 0.06), (-1331.79, -486.25, 0.05), (-1324.74, -575.96, 0.11), (-1313.32, -665.24, 0.1), (-1297.99, -753.89, 0.12), (-1277.83, -841.54, 0.16), (-1251.33, -927.46, 0.21), (-1216.71, -1010.42, 0.27), (-1172.23, -1088.5, 0.37), (-1115.43, -1158.03, 0.44), (-1046.05, -1215.04, 0.45), (-966.65, -1256.88, 0.38), (-881.13, -1284.58, 0.31), (-792.52, -1300.04, 0.17), (-702.96, -1308.74, 0.1), (-613.09, -1313.55, 0.01), (-523.21, -1318.1, 0.07)], [(-369.79, -1307.3, 0), (-425.31, -1236.53, 0.3), (-489.87, -1173.93, 0.23), (-560.63, -1118.36, 0.0), (-631.3, -1062.69, 0.37), (-691.5, -996.14, 0.55), (-733.52, -916.81, 0.44), (-759.04, -830.6, 0.29), (-773.23, -741.76, 0.16), (-780.96, -652.12, 0.12)], [(-640.55, -640.55, 0), (-631.85, -551.0, 0.22), (-632.24, -461.08, 0.31), (-645.01, -372.1, 0.34), (-671.18, -286.07, 0.24), (-706.44, -203.28, 0.03), (-740.54, -120.01, 0.28), (-763.83, -33.18, 0.37), (-772.2, 56.32, 0.31)]]
    sensor.imu.set_heading(210)
    controller.robot.pos = [-1560.0, 330.0]
    
    # ladder touch
    # doinker

    flags.color_setting = ColorSort.EJECT_BLUE
    main["AutonomousFlags"].intake_auto_stop_type = BrakeType.COAST
    motor.ladyBrown.spin(DirectionType.FORWARD, 80, VelocityUnits.PERCENT)
    sleep(400, TimeUnits.MSEC)

    controller.fwd_speed = 7
    brain.timer.event(motor.ladyBrown.spin, 800, (DirectionType.REVERSE, 80, VelocityUnits.PERCENT))
    controller.path(paths[0], backwards=True, speed_ramp_time=400, slowdown_distance=600)
    motor.ladyBrown.stop(BrakeType.COAST)
    pneumatic.mogo.set(True)
    drivetrain.turn_for(TurnType.RIGHT, 50, RotationUnits.DEG, 100, VelocityUnits.PERCENT)

    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 5
    controller.path(paths[1], speed_ramp_time=400, heading_authority=1.7, event_look_ahead_dist=200, 
    events=[
        ["intake", (-1200.0, 825.0), pneumatic.intake.set, (True,)],
        ["drop", (-1190.0, -840.0), pneumatic.mogo.set, (False,)],
        ["turn off color sort", (-1190.0, -840.0), "flags", "color_setting", ColorSort.NONE],
        ["lower", (-1200.0, -500.0), pneumatic.intake.set, (False,)],
        ["autostop for last ring", (-1000.0, -1100.0), "AutonomousFlags", "intake_auto_halt", True],
        ["speed to launch", (-1240.0, 1146.0), "fwd_speed", 7],
        ["slow for stack", (-1400.0, 644.9), "fwd_speed", 4.5],
    ],
    checkpoints=[12,],
    )

    controller.fwd_speed = 7
    controller.path(paths[2], backwards=True, speed_ramp_time=400)
    pneumatic.mogo.set(True)

    sleep(150, TimeUnits.MSEC)
    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    drivetrain.turn_for(TurnType.RIGHT, 160, RotationUnits.DEG, 80, VelocityUnits.PERCENT)
    controller.fwd_speed = 5.5
    controller.path(paths[3], speed_ramp_time=400, timeout=800)



