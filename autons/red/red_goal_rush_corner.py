def gen_data():
    data = {
        "start_pos": [-1244.6, -1533.906],
        "start_heading": 70,
    }
    return data

def run(main):
    paths = [((-1300.0, -1500.0), (-1227.21, -1466.88), (-1157.59, -1427.52), (-1090.11, -1384.56), (-1021.76, -1343.02), (-949.86, -1308.04), (-874.04, -1282.7), (-795.81, -1266.24), (-716.35, -1257.2), (-636.45, -1253.68), (-556.46, -1254.35), (-480.25, -1257.87)), ((-477.38, -1257.87), (-556.08, -1243.58), (-633.37, -1223.09), (-707.12, -1192.42), (-772.43, -1146.55), (-833.61, -1095.15), (-906.27, -1062.49), (-977.85, -1047.9)), ((-977.85, -1050.17), (-925.1, -990.08), (-866.22, -935.97), (-803.96, -885.74), (-741.15, -836.19), (-679.3, -785.46), (-618.95, -732.95), (-560.11, -678.75), (-502.76, -622.98), (-446.67, -565.94), (-391.75, -507.77), (-326.85, -436.62)), ((-482.52, -455.19), (-531.32, -517.1), (-548.76, -594.88), (-551.3, -674.79), (-548.14, -754.72), (-543.65, -834.6), (-541.93, -869.3)), ((-772.85, -852.73), (-763.69, -931.99), (-740.25, -1008.42), (-709.1, -1082.08), (-673.09, -1153.5), (-633.62, -1223.07), (-591.44, -1291.04), (-546.98, -1357.55), (-500.49, -1422.65), (-451.99, -1486.26), (-401.37, -1548.21), (-372.58, -1581.42)), ((-436.18, -1519.6), (-510.99, -1491.26), (-586.11, -1463.75), (-661.75, -1437.7), (-738.15, -1414.0), (-815.6, -1394.05), (-894.43, -1380.66), (-974.23, -1376.49), (-1053.73, -1384.1), (-1131.13, -1403.95), (-1205.09, -1434.33), (-1275.78, -1471.69), (-1343.87, -1513.66), (-1410.17, -1558.42), (-1475.15, -1605.07), (-1539.25, -1652.94), (-1602.78, -1701.56), (-1665.98, -1750.61), (-1729.0, -1799.89), (-1847.6, -1892.95)), ((-1732.4, -1842.7), (-1674.78, -1787.3), (-1623.58, -1725.86), (-1576.64, -1661.1), (-1532.64, -1594.29), (-1490.33, -1526.39), (-1448.79, -1458.02), (-1407.0, -1389.81), (-1363.7, -1322.54), (-1316.87, -1257.72), (-1291.3, -1227.41)), ((-1313.24, -1191.3), (-1337.34, -1263.75), (-1414.35, -1283.82), (-1473.89, -1334.21), (-1499.61, -1409.55), (-1509.41, -1488.87), (-1511.69, -1551.28))]

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

    imu.set_heading(70)  
    controller.dynamic_vars["position"] = [-1244.6, -1533.906]

    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    controller.dynamic_vars["intake_color_sort"] = "eject_blue"
    controller.fwd_speed = 11
    controller.path(paths[0])
    main["doinker_pneu"].set(True)

    motors["misc"]["wall_stake"].set_stopping(BrakeType.HOLD)
    motors["misc"]["wall_stake"].set_velocity(35, VelocityUnits.PERCENT)

    controller.fwd_speed = 7.5
    controller.path(paths[1], backwards=True, finish_margin=200, look_ahead_dist=200)

    main["doinker_pneu"].set(False)
    drivetrain.turn_for(TurnType.RIGHT, 90, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 8
    controller.path(paths[2], backwards=True, heading_authority=1.5)
    main["mogo_pneu"].set(True)
    main["sleep"](150, TimeUnits.MSEC)

    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    brain.timer.event(motors["misc"]["intake_chain"].spin, 200, (DirectionType.FORWARD, 100, VelocityUnits.PERCENT))

    controller.path(paths[3], backwards=False, finish_margin=170)
    main["sleep"](1000, TimeUnits.MSEC)
    motors["misc"]["intake_chain"].stop()

    main["mogo_pneu"].set(False)

    drivetrain.turn_for(TurnType.RIGHT, 160, RotationUnits.DEG, 90, VelocityUnits.PERCENT)

    controller.path(paths[4], backwards=True, heading_authority=1.5)
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
    drivetrain.turn_for(TurnType.LEFT, 180, RotationUnits.DEG, 100, VelocityUnits.PERCENT)

    main["doinker_pneu"].set(False)
    drivetrain.turn_for(TurnType.RIGHT, 80, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    main["mogo_pneu"].set(False)
