def gen_data():
    data = {
        "start_pos": [-1244.6, -1533.906],
        "start_heading": 70,
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

    paths = [((-1300.0, -1500.0), (-1227.21, -1466.88), (-1157.59, -1427.52), (-1090.11, -1384.56), (-1021.76, -1343.02), (-949.86, -1308.04), (-874.04, -1282.7), (-795.81, -1266.24), (-716.35, -1257.2), (-636.45, -1253.68), (-556.46, -1254.35), (-480.25, -1257.87)), ((-477.38, -1257.87), (-556.08, -1243.58), (-633.37, -1223.09), (-707.12, -1192.42), (-772.43, -1146.55), (-833.61, -1095.15), (-906.27, -1062.49), (-977.85, -1047.9)), ((-977.85, -1050.17), (-925.1, -990.08), (-866.22, -935.97), (-803.96, -885.74), (-741.15, -836.19), (-679.3, -785.46), (-618.95, -732.95), (-560.11, -678.75), (-502.76, -622.98), (-446.67, -565.94), (-391.75, -507.77), (-326.85, -436.62)), ((-482.52, -455.19), (-531.32, -517.1), (-548.76, -594.88), (-551.3, -674.79), (-548.14, -754.72), (-543.65, -834.6), (-541.93, -869.3)), ((-772.85, -852.73), (-777.89, -932.57), (-782.49, -1012.43), (-782.83, -1092.41), (-776.04, -1172.08), (-759.85, -1250.34), (-732.0, -1325.22), (-690.95, -1393.74), (-637.28, -1452.92), (-573.49, -1501.05), (-502.73, -1538.14), (-372.58, -1581.42)), ((-436.18, -1519.6), (-510.99, -1491.26), (-586.11, -1463.75), (-661.75, -1437.7), (-738.15, -1414.0), (-815.6, -1394.05), (-894.43, -1380.66), (-974.23, -1376.49), (-1053.73, -1384.1), (-1131.13, -1403.95), (-1205.09, -1434.33), (-1275.78, -1471.69), (-1343.87, -1513.66), (-1410.17, -1558.42), (-1475.15, -1605.07), (-1539.25, -1652.94), (-1602.78, -1701.56), (-1665.98, -1750.61), (-1729.0, -1799.89), (-1847.6, -1892.95)), ((-1732.4, -1842.7), (-1674.78, -1787.3), (-1623.58, -1725.86), (-1576.64, -1661.1), (-1532.64, -1594.29), (-1490.33, -1526.39), (-1448.79, -1458.02), (-1407.0, -1389.81), (-1363.7, -1322.54), (-1316.87, -1257.72), (-1291.3, -1227.41)), ((-1313.24, -1191.3), (-1337.34, -1263.75), (-1414.35, -1283.82), (-1473.89, -1334.21), (-1499.61, -1409.55), (-1509.41, -1488.87), (-1511.69, -1551.28)), ((-1716.07, -1734.34), (-1676.29, -1664.94), (-1634.08, -1596.98), (-1589.77, -1530.37), (-1543.62, -1465.02), (-1496.02, -1400.73), (-1447.37, -1337.22), (-1398.11, -1274.19), (-1348.74, -1211.24), (-1299.92, -1147.86), (-1252.88, -1083.16), (-1209.21, -1016.16), (-1172.92, -944.97), (-1151.78, -868.21), (-1160.33, -789.53), (-1204.28, -723.35), (-1267.51, -674.62), (-1397.59, -611.24)), ((-1385.51, -599.17), (-1305.53, -600.51), (-1225.56, -602.8), (-1145.61, -605.52), (-1065.64, -607.89), (-985.65, -608.84), (-905.67, -607.33), (-825.83, -602.46), (-746.32, -593.69), (-667.35, -580.9), (-589.1, -564.32), (-511.61, -544.45), (-399.29, -510.61))]

    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    flags.color_setting = ColorSort.EJECT_BLUE
    controller.fwd_speed = 11
    controller.path(paths[0])
    pneumatic.doinker.set(True)

    motor.ladyBrown.set_stopping(BrakeType.HOLD)
    motor.ladyBrown.set_velocity(35, VelocityUnits.PERCENT)

    controller.fwd_speed = 7.5
    # drag goal back
    controller.path(paths[1], backwards=True, finish_margin=200, look_ahead_dist=200)

    pneumatic.doinker.set(False)
    drivetrain.turn_for(TurnType.RIGHT, 90, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 8
    controller.path(paths[2], backwards=True, heading_authority=1.5)
    pneumatic.mogo.set(True)
    sleep(150, TimeUnits.MSEC)

    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    brain.timer.event(motor.intakeChain.spin, 200, (DirectionType.FORWARD, 100, VelocityUnits.PERCENT))

    controller.path(paths[3], backwards=False, finish_margin=170)
    sleep(1000, TimeUnits.MSEC)
    motor.intakeChain.stop()

    pneumatic.mogo.set(False)

    drivetrain.turn_for(TurnType.RIGHT, 160, RotationUnits.DEG, 90, VelocityUnits.PERCENT)

    controller.path(paths[4], backwards=True, heading_authority=1.5, timeout=900)
    pneumatic.mogo.set(True)
    sleep(150)

    controller.fwd_speed = 9
    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    
    controller.path(paths[5], look_ahead_dist=350, timeout = 2000, heading_authority=3)

    controller.fwd_speed = 4
    controller.path(paths[6], backwards=True)

    # doink
    pneumatic.doinker.set(True)
    controller.path(paths[7], timeout=700)
    drivetrain.turn_for(TurnType.LEFT, 180, RotationUnits.DEG, 100, VelocityUnits.PERCENT)

    pneumatic.doinker.set(False)
    drivetrain.turn_for(TurnType.RIGHT, 80, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    pneumatic.mogo.set(False)
    drivetrain.turn_for(TurnType.LEFT, 80, RotationUnits.DEG, 100, VelocityUnits.PERCENT)

    controller.fwd_speed = 8.5
    controller.path(paths[8], backwards=False, heading_authority = 2)

    controller.path(paths[9], backwards=True, heading_authority = 2)
    pneumatic.mogo.set(True)

