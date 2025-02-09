def gen_data():
    data = {
        "start_pos": [1244.6, -870.0],
        "start_heading": 250,
    }
    return data

def run(main):
    # Refereneces for easier access
    # Global Types
    BrakeType = main["BrakeType"]
    DirectionType = main["DirectionType"]
    VelocityUnits = main["VelocityUnits"]
    TimeUnits = main["TimeUnits"]
    RotationUnits = main["RotationUnits"]
    TurnType = main["TurnType"]
    VoltageUnits = main["VoltageUnits"]
    # Custom objects
    motors = main["motors"]
    controller = main["auton"]
    drivetrain = main["drivetrain"]
    imu = main["imu"]
    brain = main["brain"]
    sleep = main["sleep"]

    paths = [((1300.0, -900.38), (1244.27, -922.6), (1188.6, -944.99), (1132.86, -967.21), (1076.92, -988.89), (1020.66, -1009.75), (964.0, -1029.48), (906.89, -1047.87), (849.34, -1064.83), (791.38, -1080.31), (733.04, -1094.32), (674.38, -1106.92), (615.45, -1118.2), (556.29, -1128.18), (496.94, -1137.04), (437.46, -1144.88), (377.85, -1151.73), (332.77, -1156.38)), ((338.0, -1153.76), (396.26, -1163.37), (448.56, -1192.64), (498.02, -1226.6), (547.12, -1261.09), (597.5, -1293.65), (650.5, -1321.68), (707.29, -1340.59), (766.89, -1343.37), (822.96, -1323.26), (868.9, -1285.04), (904.33, -1236.72), (932.25, -1183.67), (955.12, -1128.23), (974.25, -1071.38), (982.17, -1044.86)), ((977.85, -1050.17), (941.02, -1002.8), (904.26, -955.38), (867.36, -908.07), (830.15, -861.0), (792.55, -814.24), (754.57, -767.8), (716.23, -721.64), (677.58, -675.75), (638.66, -630.08), (599.52, -584.61), (560.2, -539.29), (520.7, -494.12), (481.08, -449.07), (441.32, -404.13), (401.46, -359.28), (372.04, -326.29)), ((482.52, -455.19), (522.61, -499.09), (542.87, -555.32), (550.33, -614.8), (551.31, -674.77), (549.19, -734.73), (545.88, -794.64), (542.63, -854.55), (541.93, -869.3)), ((772.85, -852.73), (774.61, -912.7), (774.65, -972.7), (771.84, -1032.63), (765.38, -1092.26), (754.63, -1151.27), (739.04, -1209.19), (717.9, -1265.31), (691.17, -1318.98), (658.77, -1369.43), (620.95, -1415.96), (578.26, -1458.06), (531.42, -1495.49), (481.27, -1528.37), (428.4, -1556.69), (372.58, -1581.42)), ((436.18, -1519.6), (492.27, -1498.29), (548.5, -1477.36), (604.96, -1457.05), (661.73, -1437.64), (718.93, -1419.53), (776.69, -1403.31), (835.18, -1390.02), (894.4, -1380.53), (954.21, -1376.1), (1014.14, -1378.39), (1073.35, -1387.82), (1131.07, -1404.01), (1186.93, -1425.82), (1240.81, -1452.18), (1293.05, -1481.67), (1343.88, -1513.54), (1393.71, -1546.96), (1442.76, -1581.52), (1491.2, -1616.91), (1539.21, -1652.91), (1586.9, -1689.32), (1634.37, -1726.01), (1681.71, -1762.87), (1728.97, -1799.84), (1776.18, -1836.87), (1823.37, -1873.93), (1847.6, -1892.95)), ((1732.4, -1842.7), (1688.46, -1801.88), (1648.51, -1757.14), (1611.47, -1709.96), (1576.66, -1661.09), (1543.42, -1611.14), (1511.33, -1560.45), (1479.91, -1509.33), (1448.79, -1458.03), (1417.54, -1406.81), (1385.66, -1355.98), (1352.49, -1305.99), (1316.93, -1257.67), (1291.3, -1227.41)), ((1313.24, -1191.3), (1323.39, -1249.64), (1375.22, -1275.45), (1432.62, -1291.72), (1473.92, -1334.1), (1495.35, -1389.92), (1505.95, -1448.93), (1510.63, -1508.73), (1511.69, -1551.28))]

    imu.set_heading(250)
    controller.dynamic_vars["position"] = [1244.6, -870.0]


    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    controller.dynamic_vars["intake_color_sort"] = "eject_red"
    controller.fwd_speed = 12
    controller.path(paths[0])
    main["doinker_pneu"].set(True)

    motors["misc"]["wall_stake"].set_stopping(BrakeType.HOLD)
    motors["misc"]["wall_stake"].set_velocity(35, VelocityUnits.PERCENT)

    controller.fwd_speed = 7.5
    controller.path(paths[1], backwards=True, finish_margin=200, look_ahead_dist=200)

    main["doinker_pneu"].set(False)
    drivetrain.turn_for(TurnType.LEFT, 90, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 8
    controller.path(paths[2], backwards=True, heading_authority=2)
    main["mogo_pneu"].set(True)
    sleep(150, TimeUnits.MSEC)

    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    brain.timer.event(motors["misc"]["intake_flex"].spin, 200, (DirectionType.FORWARD, 100, VelocityUnits.PERCENT))

    controller.path(paths[3], backwards=False, finish_margin=170)
    sleep(1000, TimeUnits.MSEC)
    motors["misc"]["intake_chain"].stop()

    main["mogo_pneu"].set(False)

    drivetrain.turn_for(TurnType.LEFT, 160, RotationUnits.DEG, 90, VelocityUnits.PERCENT)

    controller.path(paths[4], backwards=True, heading_authority=2, timeout=2000)
    main["mogo_pneu"].set(True)

    controller.fwd_speed = 9
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    
    controller.path(paths[5], look_ahead_dist=350, timeout = 2000, heading_authority=3)

    controller.fwd_speed = 4
    controller.path(paths[6], backwards=True)

    # doink
    main["doinker_pneu"].set(True)
    controller.path(paths[7], timeout=700)
    brain.timer.event(motors["misc"]["intake_flex"].stop, 200)
    drivetrain.turn_for(TurnType.LEFT, 180, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
