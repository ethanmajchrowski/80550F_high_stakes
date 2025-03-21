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
    paths = [[(-1547.38, 312.87, 0), (-1521.81, 399.14, 0.17), (-1489.72, 483.18, 0.19), (-1450.73, 564.24, 0.25), (-1403.02, 640.48, 0.31), (-1345.13, 709.18, 0.39), (-1276.14, 766.61, 0.53), (-1195.49, 805.97, 0.6), (-1107.42, 822.31, 0.52), (-1017.82, 817.44, 0.49), (-931.39, 792.98, 0.34), (-849.59, 755.75, 0.25), (-772.43, 709.61, 0.19), (-699.42, 657.11, 0.14), (-629.82, 600.06, 0.14), (-564.03, 538.65, 0.09), (-500.91, 474.55, 0.05)], [(-576.13, 492.17, 0), (-520.44, 562.85, 0.09), (-462.02, 631.31, 0.14), (-408.01, 703.2, 0.45), (-369.7, 784.41, 0.42), (-347.51, 871.55, 0.24), (-334.65, 960.59, 0.13), (-326.96, 1050.25, 0.08), (-322.37, 1140.13, 0.05), (-319.65, 1230.08, 0.03), (-318.28, 1320.07, 1.03), (-355.14, 1397.73, 0.68), (-416.54, 1463.36, 0.8), (-495.52, 1502.5, 2.01), (-567.26, 1455.31, 1.12), (-607.39, 1374.93, 0.19), (-640.67, 1291.31, 0.26), (-683.34, 1212.43, 1.63), (-763.19, 1185.92, 0.92), (-852.67, 1195.54, 0.01), (-942.21, 1204.66, 0.02), (-1031.83, 1212.89, 0.05), (-1121.59, 1219.26, 0.1), (-1211.52, 1221.7, 0.22), (-1301.12, 1215.1, 0.72), (-1382.79, 1180.71, 1.5), (-1421.53, 1102.09, 0.96), (-1422.66, 1012.4, 0.28), (-1412.35, 923.03, 0.1), (-1398.21, 834.15, 0.03), (-1383.06, 745.43, 0.01), (-1368.23, 656.66, 0.02), (-1354.3, 567.75, 0.03), (-1341.38, 478.68, 0.02), (-1329.4, 389.49, 0.05), (-1319.42, 300.04, 0.02), (-1310.32, 210.51, 0.04), (-1302.69, 120.83, 0.04), (-1296.5, 31.05, 0.03), (-1291.35, -58.8, 0.04), (-1287.79, -148.71, 0.08), (-1287.61, -238.71, 0.04), (-1285.86, -328.69, 0.04), (-1282.56, -418.62, 0.06), (-1276.81, -508.43, 0.08), (-1267.99, -597.99, 0.08), (-1255.92, -687.16, 0.11), (-1239.65, -775.64, 0.14), (-1218.01, -862.96, 0.17), (-1189.6, -948.31, 0.27), (-1151.13, -1029.57, 0.31), (-1101.85, -1104.73, 0.43), (-1038.92, -1168.78, 0.45), (-964.46, -1218.87, 0.42), (-881.87, -1254.23, 0.31), (-795.01, -1277.58, 0.22), (-706.23, -1292.25, 0.11), (-616.84, -1302.56, 0.07)], [(-369.79, -1307.3, 0), (-423.87, -1235.39, 0.21), (-484.56, -1168.99, 0.13), (-549.14, -1106.31, 0.32), (-603.9, -1035.31, 0.65), (-635.43, -951.32, 0.4), (-651.47, -862.83, 0.19), (-659.89, -773.24, 0.09), (-664.49, -683.37, 0.06)], [(-640.55, -640.55, 0), (-631.85, -551.0, 0.22), (-632.24, -461.08, 0.31), (-645.01, -372.1, 0.34), (-671.18, -286.07, 0.24), (-706.44, -203.28, 0.03), (-740.54, -120.01, 0.28), (-763.83, -33.18, 0.37), (-772.2, 56.32, 0.31)]]
    sensor.imu.set_heading(210)
    controller.robot.pos = [-1560.0, 330.0]
    
    # ladder touch
    # doinker

    # flags.color_setting = ColorSort.EJECT_BLUE
    main["AutonomousFlags"].intake_anti_jam = False
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



