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

    # ladder touch
    paths = [[(-1300.0, -1500.0, 0), (-1226.87, -1467.62, 0.15), (-1155.84, -1430.83, 0.08), (-1085.95, -1391.9, 0.03), (-1015.59, -1353.84, 0.14), (-943.26, -1319.7, 0.19), (-868.51, -1291.28, 0.22), (-791.54, -1269.57, 0.22), (-712.98, -1254.59, 0.19), (-633.51, -1245.56, 0.15), (-553.64, -1241.31, 0.14), (-473.65, -1241.39, 0.09)], [(-477.38, -1257.87, 0), (-556.08, -1243.58, 0.2), (-633.37, -1223.09, 0.34), (-707.12, -1192.42, 0.54), (-772.43, -1146.55, 0.22), (-833.61, -1095.15, 0.68), (-906.27, -1062.49, 0.6)], [(-977.85, -1050.17, 0), (-925.1, -990.08, 0.27), (-866.22, -935.97, 0.16), (-803.96, -885.74, 0.03), (-741.15, -836.19, 0.05), (-679.3, -785.46, 0.07), (-618.95, -732.95, 0.07), (-560.11, -678.75, 0.07), (-502.76, -622.98, 0.06), (-446.67, -565.94, 0.05), (-391.75, -507.77, 0.04)], [(-403.81, -475.06, 0), (-461.38, -530.09, 0.7), (-501.4, -599.13, 0.52), (-526.19, -675.08, 0.38), (-539.15, -753.96, 0.29), (-542.75, -833.84, 0.38)], [(-772.85, -852.73, 0), (-763.69, -931.99, 0.45), (-740.25, -1008.42, 0.26), (-709.1, -1082.08, 0.17), (-673.09, -1153.5, 0.12), (-633.62, -1223.07, 0.1), (-591.44, -1291.04, 0.08), (-546.98, -1357.55, 0.08), (-500.49, -1422.65, 0.08), (-451.99, -1486.26, 0.08), (-401.37, -1548.21, 0.13)], [(-436.18, -1519.6, 0), (-510.99, -1491.26, 0.03), (-586.11, -1463.75, 0.05), (-661.75, -1437.7, 0.08), (-738.15, -1414.0, 0.12), (-815.6, -1394.05, 0.21), (-894.43, -1380.66, 0.29), (-974.23, -1376.49, 0.37), (-1053.73, -1384.1, 0.39), (-1131.13, -1403.95, 0.35), (-1205.09, -1434.33, 0.24), (-1275.78, -1471.69, 0.17), (-1343.87, -1513.66, 0.1), (-1410.17, -1558.42, 0.07), (-1475.15, -1605.07, 0.05), (-1539.25, -1652.94, 0.03), (-1602.78, -1701.56, 0.02), (-1665.98, -1750.61, 0.01), (-1729.0, -1799.89, 0.0)], [(-1732.4, -1842.7, 0), (-1674.78, -1787.3, 0.28), (-1623.58, -1725.86, 0.17), (-1576.64, -1661.1, 0.11), (-1532.64, -1594.29, 0.06), (-1490.33, -1526.39, 0.03), (-1448.79, -1458.02, 0.01), (-1407.0, -1389.81, 0.06), (-1363.7, -1322.54, 0.13), (-1316.87, -1257.72, 0.38)], [(-1305.35, -1241.46, 0), (-1258.37, -1176.71, 0.05), (-1212.59, -1111.1, 0.05), (-1168.06, -1044.64, 0.05), (-1124.81, -977.35, 0.05), (-1082.85, -909.23, 0.05), (-1042.19, -840.34, 0.05), (-1002.94, -770.63, 0.04), (-964.91, -700.25, 0.04), (-927.94, -629.31, 0.03), (-891.95, -557.86, 0.02), (-856.53, -486.13, 0.01), (-821.38, -414.27, 0.01), (-786.05, -342.49, 0.02), (-750.11, -271.02, 0.04), (-713.01, -200.14, 0.05), (-674.47, -130.04, 0.07), (-633.92, -61.08, 0.08)]]

    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    flags.color_setting = ColorSort.EJECT_BLUE
    controller.fwd_speed = 11
    controller.path(paths[0])
    pneumatic.doinker.set(True)

    motor.ladyBrown.set_stopping(BrakeType.HOLD)
    motor.ladyBrown.set_velocity(35, VelocityUnits.PERCENT)
    # motor.ladyBrown.spin_for(DirectionType.FORWARD, 160, RotationUnits.DEG, False)

    controller.fwd_speed = 7.5
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

    controller.path(paths[4], backwards=True, heading_authority=1.5)
    pneumatic.mogo.set(True)
    sleep(150)

    controller.fwd_speed = 9
    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    
    controller.path(paths[5], look_ahead_dist=350, timeout = 2000, heading_authority=3)

    controller.fwd_speed = 4
    controller.path(paths[6], backwards=True)

    # ladder touch
    drivetrain.turn_for(TurnType.RIGHT, 180, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 8
    controller.path(paths[7], backwards=False, finish_margin=150, timeout=1300, slowdown_distance=800)
