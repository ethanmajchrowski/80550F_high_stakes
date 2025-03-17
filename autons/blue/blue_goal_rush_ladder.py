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
    
    # ladder touch
    paths = [[(1300.0, -900.38, 0), (1244.27, -922.6, 0.01), (1188.6, -944.99, 0.01), (1132.86, -967.21, 0.03), (1076.92, -988.89, 0.05), (1020.66, -1009.75, 0.07), (964.0, -1029.48, 0.08), (906.89, -1047.87, 0.08), (849.34, -1064.83, 0.09), (791.38, -1080.31, 0.08), (733.04, -1094.32, 0.08), (674.38, -1106.92, 0.07), (615.45, -1118.2, 0.07), (556.29, -1128.18, 0.06), (496.94, -1137.04, 0.06), (437.46, -1144.88, 0.06), (377.85, -1151.73, 0.05)], [(338.0, -1153.76, 0), (396.26, -1163.37, 1.13), (448.56, -1192.64, 0.3), (498.02, -1226.6, 0.04), (547.12, -1261.09, 0.13), (597.5, -1293.65, 0.29), (650.5, -1321.68, 0.55), (707.29, -1340.59, 0.91), (766.89, -1343.37, 1.28), (822.96, -1323.26, 1.15), (868.9, -1285.04, 0.81), (904.33, -1236.72, 0.49), (932.25, -1183.67, 0.31), (955.12, -1128.23, 0.22), (974.25, -1071.38, 0.25)], [(977.85, -1050.17, 0), (941.02, -1002.8, 0.0), (904.26, -955.38, 0.01), (867.36, -908.07, 0.02), (830.15, -861.0, 0.03), (792.55, -814.24, 0.03), (754.57, -767.8, 0.03), (716.23, -721.64, 0.02), (677.58, -675.75, 0.02), (638.66, -630.08, 0.02), (599.52, -584.61, 0.01), (560.2, -539.29, 0.01), (520.7, -494.12, 0.01), (481.08, -449.07, 0.01), (441.32, -404.13, 0.01), (401.46, -359.28, 0.01)], [(482.52, -455.19, 0), (522.61, -499.09, 1.29), (542.87, -555.32, 0.73), (550.33, -614.8, 0.36), (551.31, -674.77, 0.17), (549.19, -734.73, 0.07), (545.88, -794.64, 0.0), (542.63, -854.55, 0.09)], [(772.85, -852.73, 0), (777.16, -912.57, 0.08), (782.84, -972.3, 0.07), (787.31, -1032.13, 0.15), (789.11, -1092.09, 0.24), (786.5, -1152.02, 0.32), (778.22, -1211.4, 0.42), (762.52, -1269.25, 0.53), (737.8, -1323.82, 0.59), (703.81, -1373.13, 0.63), (661.16, -1415.21, 0.6), (611.64, -1449.0, 0.51), (557.52, -1474.8, 0.41), (500.64, -1493.76, 0.31), (442.24, -1507.39, 0.27)], [(436.18, -1519.6, 0), (492.27, -1498.29, 0.02), (548.5, -1477.36, 0.04), (604.96, -1457.05, 0.05), (661.73, -1437.64, 0.08), (718.93, -1419.53, 0.11), (776.69, -1403.31, 0.17), (835.18, -1390.02, 0.22), (894.4, -1380.53, 0.28), (954.21, -1376.1, 0.37), (1014.14, -1378.39, 0.4), (1073.35, -1387.82, 0.38), (1131.07, -1404.01, 0.33), (1186.93, -1425.82, 0.28), (1240.81, -1452.18, 0.2), (1293.05, -1481.67, 0.15), (1343.88, -1513.54, 0.1), (1393.71, -1546.96, 0.08), (1442.76, -1581.52, 0.06), (1491.2, -1616.91, 0.04), (1539.21, -1652.91, 0.03), (1586.9, -1689.32, 0.02), (1634.37, -1726.01, 0.01), (1681.71, -1762.87, 0.01), (1728.97, -1799.84, 0.0), (1776.18, -1836.87, 0.0), (1823.37, -1873.93, 0.0)], [(1732.4, -1842.7, 0), (1688.46, -1801.88, 0.31), (1648.51, -1757.14, 0.21), (1611.47, -1709.96, 0.16), (1576.66, -1661.09, 0.11), (1543.42, -1611.14, 0.08), (1511.33, -1560.45, 0.04), (1479.91, -1509.33, 0.02), (1448.79, -1458.03, 0.01), (1417.54, -1406.81, 0.04), (1385.66, -1355.98, 0.09), (1352.49, -1305.99, 0.16), (1316.93, -1257.67, 0.34)], [(1305.35, -1241.46, 0), (1270.0, -1192.98, 0.05), (1235.32, -1144.02, 0.05), (1201.32, -1094.58, 0.05), (1168.03, -1044.66, 0.05), (1135.46, -994.28, 0.05), (1103.61, -943.43, 0.05), (1072.49, -892.13, 0.05), (1042.2, -840.34, 0.05), (1012.63, -788.13, 0.04), (983.73, -735.55, 0.05), (955.56, -682.58, 0.04), (927.95, -629.3, 0.03), (900.87, -575.77, 0.02), (874.18, -522.03, 0.02), (847.74, -468.17, 0.01), (821.39, -414.27, 0.01), (794.93, -360.41, 0.02), (768.21, -306.69, 0.03), (740.97, -253.23, 0.04), (713.05, -200.13, 0.05), (684.29, -147.47, 0.07), (654.45, -95.41, 0.07), (623.56, -43.98, 0.09)]]

    sensor.imu.set_heading(250)
    controller.robot.pos = [1244.6, -870.0]

    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    flags.color_setting = ColorSort.EJECT_RED
    controller.fwd_speed = 11
    controller.path(paths[0], backwards=False)
    pneumatic.doinker.set(True)

    controller.fwd_speed = 7.5
    controller.path(paths[1], backwards=True, finish_margin=200, look_ahead_dist=200)

    pneumatic.doinker.set(False)

    controller.fwd_speed = 8
    controller.path(paths[2], backwards=True, heading_authority=2)
    pneumatic.mogo.set(True)
    sleep(150, TimeUnits.MSEC)

    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    brain.timer.event(motor.intakeFlex.spin, 200, (DirectionType.FORWARD, 100, VelocityUnits.PERCENT))

    controller.path(paths[3], backwards=False, finish_margin=170)
    sleep(1000, TimeUnits.MSEC)
    motor.intakeChain.stop()

    pneumatic.mogo.set(False)

    drivetrain.turn_for(TurnType.LEFT, 160, RotationUnits.DEG, 90, VelocityUnits.PERCENT)

    controller.path(paths[4], backwards=True, heading_authority=2, timeout=2000)
    pneumatic.mogo.set(True)

    controller.fwd_speed = 9
    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    
    controller.path(paths[5], look_ahead_dist=350, timeout = 2000, heading_authority=3)

    controller.fwd_speed = 4
    controller.path(paths[6], backwards=True)

    # ladder touch
    drivetrain.turn_for(TurnType.LEFT, 180, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 8
    controller.path(paths[7], backwards=False, finish_margin=150, timeout=1300)
