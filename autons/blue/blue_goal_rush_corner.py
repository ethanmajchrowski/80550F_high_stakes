def gen_data():
    data = {
        "start_pos": [1244.6, -870.0],
        "start_heading": 250,
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

    paths = [[(1300.0, -900.0, 0), (1227.28, -933.31, 0.13), (1156.32, -970.23, 0.05), (1086.14, -1008.64, 0.05), (1015.16, -1045.53, 0.15), (942.07, -1078.01, 0.21), (866.55, -1104.28, 0.22), (788.97, -1123.7, 0.2), (710.1, -1136.98, 0.17), (630.51, -1144.97, 0.14), (550.6, -1148.42, 0.1), (470.61, -1148.58, 0.1)], [(477.38, -1257.87, 0), (557.31, -1260.74, 0.09), (637.09, -1266.64, 0.14), (717.02, -1267.91, 0.61), (794.47, -1249.99, 1.1), (856.24, -1200.41, 0.9), (896.43, -1131.56, 0.47), (923.02, -1056.18, 0.25)], [(977.85, -1050.17, 0), (928.56, -987.15, 0.02), (878.88, -924.45, 0.03), (828.49, -862.31, 0.03), (777.36, -800.78, 0.03), (725.61, -739.78, 0.02), (673.41, -679.16, 0.01), (620.87, -618.83, 0.01), (568.08, -558.72, 0.01), (515.12, -498.76, 0.01), (462.02, -438.92, 0.0)], [(482.52, -455.19, 0), (539.85, -510.51, 0.58), (582.91, -577.81, 0.31), (617.41, -649.96, 0.16), (647.1, -724.24, 0.09), (674.22, -799.5, 0.04)], [(670.99, -866.78, 0), (681.07, -946.06, 0.05), (692.75, -1025.06, 1.46), (659.27, -1094.22, 1.53), (588.06, -1129.79, 0.48), (511.0, -1151.11, 0.19), (432.53, -1166.62, 0.1), (353.51, -1179.08, 0.05)], [(436.18, -1519.6, 0), (510.99, -1491.26, 0.03), (586.11, -1463.75, 0.05), (661.75, -1437.7, 0.08), (738.15, -1414.0, 0.12), (815.6, -1394.05, 0.21), (894.43, -1380.66, 0.29), (974.23, -1376.49, 0.37), (1053.73, -1384.1, 0.39), (1131.13, -1403.95, 0.35), (1205.09, -1434.33, 0.24), (1275.78, -1471.69, 0.17), (1343.87, -1513.66, 0.1), (1410.17, -1558.42, 0.07), (1475.15, -1605.07, 0.05), (1539.25, -1652.94, 0.03), (1602.78, -1701.56, 0.02), (1665.98, -1750.61, 0.01), (1729.0, -1799.89, 0.0)], [(1732.4, -1842.7, 0), (1674.78, -1787.3, 0.28), (1623.58, -1725.86, 0.17), (1576.64, -1661.1, 0.11), (1532.64, -1594.29, 0.06), (1490.33, -1526.39, 0.03), (1448.79, -1458.02, 0.01), (1407.0, -1389.81, 0.06), (1363.7, -1322.54, 0.13), (1316.87, -1257.72, 0.38)], [(1313.24, -1191.3, 0), (1337.34, -1263.75, 2.11), (1414.35, -1283.82, 1.11), (1473.89, -1334.21, 1.29), (1499.61, -1409.55, 0.51), (1509.41, -1488.87, 0.28)], [(1716.07, -1734.34, 0), (1676.29, -1664.94, 0.09), (1634.08, -1596.98, 0.08), (1589.77, -1530.37, 0.07), (1543.62, -1465.02, 0.06), (1496.02, -1400.73, 0.04), (1447.37, -1337.22, 0.02), (1398.11, -1274.19, 0.0), (1348.74, -1211.24, 0.02), (1299.92, -1147.86, 0.07), (1252.88, -1083.16, 0.13), (1209.21, -1016.16, 0.27), (1172.92, -944.97, 0.51), (1151.78, -868.21, 0.93), (1160.33, -789.53, 1.16), (1204.28, -723.35, 0.81), (1267.51, -674.62, 0.28)], [(1385.51, -599.17, 0), (1305.53, -600.51, 0.03), (1225.56, -602.8, 0.01), (1145.61, -605.52, 0.01), (1065.64, -607.89, 0.04), (985.65, -608.84, 0.08), (905.67, -607.33, 0.11), (825.83, -602.46, 0.12), (746.32, -593.69, 0.13), (667.35, -580.9, 0.12), (589.1, -564.32, 0.11), (511.61, -544.45, 0.07)]]
    controller.robot.pos = [1244.6, -870.0]
    sensor.imu.set_heading(250)

    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    flags.color_setting = ColorSort.EJECT_RED
    controller.fwd_speed = 11
    controller.path(paths[0], finish_margin=85)
    pneumatic.doinker.set(True)

    motor.ladyBrown.set_stopping(BrakeType.HOLD)
    motor.ladyBrown.set_velocity(35, VelocityUnits.PERCENT)

    controller.fwd_speed = 7.5
    # drag goal back
    controller.path(paths[1], backwards=True, finish_margin=200, look_ahead_dist=200)

    pneumatic.doinker.set(False)
    drivetrain.turn_for(TurnType.LEFT, 90, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 8
    controller.path(paths[2], backwards=True, heading_authority=1.5)
    pneumatic.mogo.set(True)
    sleep(150, TimeUnits.MSEC)

    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    brain.timer.event(motor.intakeChain.spin, 250, (DirectionType.FORWARD, 100, VelocityUnits.PERCENT))

    controller.path(paths[3], backwards=False, finish_margin=170)
    # sleep(200, TimeUnits.MSEC)
    motor.intakeChain.stop()

    pneumatic.mogo.set(False)

    drivetrain.turn_for(TurnType.LEFT, 100, RotationUnits.DEG, 90, VelocityUnits.PERCENT)

    controller.path(paths[4], backwards=True, heading_authority=1.5, timeout=1050)
    pneumatic.mogo.set(True)
    sleep(150)

    controller.fwd_speed = 9
    brain.timer.event(motor.intakeChain.spin, 400, (DirectionType.FORWARD, 100, VelocityUnits.PERCENT))
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    
    controller.path(paths[5], look_ahead_dist=350, timeout = 2000, heading_authority=3)

    controller.fwd_speed = 4
    controller.path(paths[6], backwards=True)

    # doink
    # pneumatic.doinker.set(True)
    controller.path(paths[7], timeout=700)
    drivetrain.turn_for(TurnType.LEFT, 180, RotationUnits.DEG, 100, VelocityUnits.PERCENT)

    pneumatic.doinker.set(False)
    drivetrain.turn_for(TurnType.LEFT, 80, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    pneumatic.mogo.set(False)
    drivetrain.turn_for(TurnType.RIGHT, 80, RotationUnits.DEG, 100, VelocityUnits.PERCENT)

    controller.fwd_speed = 8.5
    controller.path(paths[8], backwards=False, heading_authority = 2)

    controller.path(paths[9], backwards=True, heading_authority = 2)
