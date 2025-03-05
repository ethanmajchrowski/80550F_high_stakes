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
    paths = [((-1547.38, 312.87), (-1509.84, 405.3), (-1449.82, 485.02), (-1373.22, 549.05), (-1285.75, 597.23), (-1191.58, 630.47), (-1093.57, 649.61), (-993.85, 654.94), (-894.34, 646.14), (-797.41, 621.95), (-706.28, 581.18), (-651.29, 545.86)),((-576.13, 492.17), (-523.6, 577.15), (-483.29, 668.58), (-453.25, 763.91), (-431.37, 861.45), (-415.67, 960.19), (-404.4, 1059.55), (-395.83, 1159.18), (-388.09, 1258.88), (-378.5, 1358.41), (-461.27, 1402.91), (-559.22, 1418.49), (-645.27, 1374.28), (-687.09, 1284.25), (-714.08, 1188.01), (-795.5, 1156.71), (-895.37, 1161.49), (-995.24, 1157.71), (-1093.89, 1142.3), (-1187.11, 1107.14), (-1264.9, 1045.35), (-1316.95, 960.57), (-1344.49, 864.61), (-1356.97, 765.45), (-1360.0, 665.52), (-1358.19, 565.54), (-1352.15, 465.74), (-1341.44, 366.33), (-1327.26, 267.34), (-1311.34, 168.61), (-1295.76, 69.84), (-1283.91, -29.44), (-1282.79, -129.31), (-1305.12, -226.18), (-1365.06, -304.81), (-1413.57, -336.74)),((-1446.26, -344.01), (-1378.21, -271.14), (-1346.34, -177.62), (-1346.82, -77.7), (-1352.03, 22.16), (-1354.55, 100.28)),((-1349.18, 94.91), (-1303.19, 6.14), (-1264.05, -85.86), (-1231.49, -180.39), (-1205.05, -276.81), (-1184.06, -374.57), (-1167.77, -473.23), (-1155.52, -572.47), (-1146.6, -672.06), (-1140.29, -771.86), (-1136.03, -871.77), (-1114.25, -967.15), (-1062.54, -1052.65), (-998.28, -1129.13), (-922.81, -1194.55), (-838.33, -1247.85), (-747.2, -1288.82), (-651.96, -1319.13), (-554.36, -1340.78)),((-377.5, -1376.02), (-433.93, -1293.46), (-489.34, -1210.22), (-542.1, -1125.28), (-590.03, -1037.54), (-630.38, -946.1), (-659.69, -850.59), (-674.18, -751.77), (-672.92, -651.91), (-656.66, -554.66)),((-640.55, -640.55), (-631.43, -541.01), (-633.9, -441.14), (-652.46, -343.03), (-686.34, -249.04), (-726.18, -157.33), (-757.44, -62.47), (-771.64, 36.36), (-769.39, 137.86)),]

    flags.color_setting = ColorSort.EJECT_BLUE
    
    end_time = 7500 + brain.timer.system()
    log("Spinning lady brown from {} to {} or until threshold.".format(brain.timer.system(), end_time))

    sensor.wallEncoder.set_position(20)
    while (sensor.wallEncoder.position() < 380):
        if (brain.timer.system() > end_time):
            break
        else:
            motor.ladyBrown.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        sleep(20)

    log("Wall encoder at {}".format(sensor.wallEncoder.position()))

    # stop LB mech
    motor.ladyBrown.stop(BrakeType.COAST)
    # path to grab mogo @ 8V
    controller.fwd_speed = 7
    brain.timer.event(motor.ladyBrown.spin, 200, (DirectionType.REVERSE, 100, VelocityUnits.PERCENT))
    controller.path(paths[0], [], [], True, 200, 100, 75, None, 1.25, 8, 0.1, 0.01, 0, 0, None, 0)
    motor.ladyBrown.stop(BrakeType.COAST)
    # after we are at the mogo start retracting the LB bar
    # grab mobile goal
    pneumatic.mogo.set(True)
    # start intaking rings
    motor.intakeChain.spin(DirectionType.FORWARD, 65, VelocityUnits.PERCENT)
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    # wait for mogo grabber to engage
    sleep(200, TimeUnits.MSEC)
    # turn to face the rings on the auton line (w/timeout & threshold)
    drivetrain.set_timeout(1.5, TimeUnits.SECONDS)
    drivetrain.set_turn_threshold(5)
    drivetrain.set_turn_constant(0.6)
    drivetrain.turn_for(TurnType.RIGHT, 90, RotationUnits.DEG, 100, VelocityUnits.PERCENT)

    controller.fwd_speed = 6
    log("path: grab_border_rings")
    controller.path(paths[1], [
        ["raise intake", (-1294, 741), pneumatic.intake.set, (True,)], 
        ["speed up", (-1294, 741), "fwd_speed", 7]
    ], [22], False, 200, 130, 120, None, 1.25, 8, 0.1, 0.01, 0, 0, None, 0)

    pneumatic.intake.set(False)
    sleep(250, TimeUnits.MSEC)
    controller.fwd_speed = 3
    log("path: backup_for_stack_ring")
    controller.path(paths[2], [], [], True)
    # sleep(200, MSEC)

    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    controller.fwd_speed = 9
    log("path: bottom_blue_ring")
    controller.path(paths[3], [["drop goal", (-951, -1162), pneumatic.mogo.set, [False]]], [], False, 300, 100, 200)

    motor.intakeChain.stop()
    pneumatic.mogo.set(False)

    log("path: last_mogo")
    controller.fwd_speed = 6
    controller.path(paths[4], [], [], True, 300, 175, 75)
    pneumatic.mogo.set(True)
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    flags.color_setting = ColorSort.NONE
    
    drivetrain.turn_for(TurnType.RIGHT, 150, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    motor.intakeChain.spin(DirectionType.FORWARD, 65, VelocityUnits.PERCENT)
    controller.fwd_speed = 8
    log("path: ladder_hit")
    controller.path(paths[5], [], [], False, 300, 70, 75, 2000)



