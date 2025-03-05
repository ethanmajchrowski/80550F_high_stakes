def gen_data():
    data = {
        "start_pos": [1500, 1500],
        "start_heading": 45,
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
    #endregion Bindings

    paths = [[(1508.16, 1494.38, 0), (1571.54, 1542.91, 0.42), (1625.95, 1601.5, 0.21), (1675.33, 1664.42, 0.09), (1722.48, 1729.05, 0.03)], [(1699.44, 1660.51, 0), (1626.21, 1628.29, 0.07), (1553.94, 1594.03, 0.09), (1482.92, 1557.25, 0.11), (1413.57, 1517.41, 0.13), (1346.41, 1474.0, 0.15), (1282.05, 1426.54, 0.17), (1221.15, 1374.73, 0.19), (1164.29, 1318.5, 0.22), (1112.69, 1257.4, 0.18), (1065.62, 1192.74, 0.15), (1022.65, 1125.29, 0.11), (982.56, 1056.06, 0.04), (943.6, 986.19, 0.01), (904.22, 916.55, 0.06), (863.28, 847.83, 0.09), (820.01, 780.54, 0.12), (773.7, 715.33, 0.12), (724.39, 652.35, 0.11), (672.28, 591.66, 0.11), (617.61, 533.27, 0.1), (560.68, 477.07, 0.09)], [(510.12, 419.89, 0), (550.58, 488.9, 0.06), (589.3, 558.89, 0.08), (625.64, 630.13, 0.11), (658.69, 702.96, 0.15), (687.16, 777.68, 0.21), (709.21, 854.52, 0.29), (722.32, 933.33, 0.47), (720.32, 1013.03, 0.68), (696.96, 1088.98, 0.87), (648.83, 1152.14, 0.79), (583.14, 1197.22, 0.54), (509.09, 1227.16, 0.33), (431.7, 1247.21, 0.19), (353.0, 1261.47, 0.09), (273.81, 1272.82, 0.0)], [(142.86, 1346.06, 0), (222.81, 1348.81, 0.15), (302.75, 1346.66, 0.28), (381.88, 1335.5, 0.49), (457.13, 1309.15, 0.72), (521.56, 1262.2, 0.15), (583.17, 1211.4, 0.81), (657.61, 1183.32, 0.63), (737.05, 1174.95, 0.32)], [(824.4, 1177.86, 0), (746.22, 1160.92, 0.18), (669.46, 1138.49, 0.19), (594.59, 1110.32, 0.0), (519.76, 1082.11, 0.37), (441.71, 1065.17, 0.43), (361.85, 1062.02, 0.27), (282.08, 1067.61, 0.12)], [(168.07, 1103.44, 0), (244.72, 1080.53, 0.04), (321.03, 1056.51, 0.02), (397.1, 1031.74, 0.01), (473.05, 1006.61, 0.0), (549.03, 981.59, 0.02), (625.21, 957.16, 0.05), (701.82, 934.12, 0.07), (779.05, 913.29, 0.13), (857.24, 896.45, 0.21), (936.51, 886.15, 0.39), (1016.27, 888.35, 0.79), (1090.77, 915.16, 1.32), (1140.17, 976.58, 0.87)], [(1227.37, 897.57, 0), (1225.59, 817.59, 0.03), (1222.81, 737.64, 0.05), (1218.51, 657.76, 0.06), (1212.22, 578.01, 0.08), (1203.25, 498.52, 0.09), (1191.46, 419.4, 0.03), (1178.58, 340.44, 0.05), (1167.43, 261.23, 0.09), (1159.06, 181.67, 0.08), (1153.13, 101.89, 0.05)], [(1189.64, 16.42, 0), (1189.1, 96.42, 0.02), (1188.04, 176.41, 0.0), (1186.94, 256.41, 0.01), (1186.11, 336.4, 0.02), (1185.89, 416.4, 0.03), (1186.73, 496.39, 0.06)]]

    drivetrain.set_turn_constant(0.5)
    drivetrain.set_turn_threshold(5)
    drivetrain.set_timeout(1500) # MSEC

    # 6 ring elims
    motor.ladyBrown.stop(BrakeType.COAST)
    flags.color_setting = ColorSort.EJECT_RED

    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 5
    log("starting path: corner")
    controller.path(paths[0], [], [], False, 350, 100, 75, 1000)

    # motors["misc"]["intake_flex"].stop()
    controller.fwd_speed = 5.5
    log("starting path: mogo")
    controller.path(paths[1], [], [], True)

    pneumatic.mogo.set(True)
    main["sleep"](200, TimeUnits.MSEC)

    #motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    brain.timer.event(motor.intakeChain.spin, 300, (DirectionType.FORWARD, 65, VelocityUnits.PERCENT))
    # motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 65, VelocityUnits.PERCENT)
    controller.fwd_speed = 4.5
    log("starting path: top_border_ring")
    controller.path(paths[2], backwards=False)
    main["sleep"](200, TimeUnits.MSEC)

    controller.fwd_speed = 7
    log("starting path: backup_for_bottom_border_ring")
    controller.path(paths[3], [], [], True)

    controller.fwd_speed = 4.5
    log("starting path: bottom_border_ring")
    controller.path(paths[4], [], [], False)
    main["sleep"](200, TimeUnits.MSEC)

    controller.fwd_speed = 7
    log("starting path: last_ring_reorient")
    controller.path(paths[5], [], [], True, 350, 200)

    log("intake up")
    pneumatic.intake.set(True)

    controller.fwd_speed = 6
    log("starting path: last_ring")
    controller.path(paths[6], [], [], False, 350, 100, 75, 5000)

    motor.intakeChain.stop()
    main["sleep"](500, TimeUnits.MSEC)
    log("intake down")
    pneumatic.intake.set(False)
    main["sleep"](250, TimeUnits.MSEC)

    motor.intakeChain.spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    controller.fwd_speed = 4
    log("starting path: end_backup")
    controller.path(paths[7], [], [], True)

    motor.intakeFlex.stop()
    motor.intakeChain.stop()
    controller.kill_motors()
    main["mogo_pneu"].set(True)
