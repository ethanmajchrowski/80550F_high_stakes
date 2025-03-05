def gen_data():
    data = {
        "start_pos": [1560, 330],
        "start_heading": 150,
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

    paths = [[(1547.38, 312.87, 0), (1509.84, 405.3, 0.51), (1449.82, 485.02, 0.46), (1373.22, 549.05, 0.38), (1285.75, 597.23, 0.33), (1191.58, 630.47, 0.29), (1093.57, 649.61, 0.28), (993.85, 654.94, 0.28), (894.34, 646.14, 0.31), (797.41, 621.95, 0.35), (706.28, 581.18, 0.46)], [(576.13, 492.17, 0), (523.6, 577.15, 0.28), (483.29, 668.58, 0.22), (453.25, 763.91, 0.17), (431.37, 861.45, 0.13), (415.67, 960.19, 0.09), (404.4, 1059.55, 0.05), (395.83, 1159.18, 0.02), (388.09, 1258.88, 0.04), (378.5, 1358.41, 1.96), (461.27, 1402.91, 0.66), (559.22, 1418.49, 1.22), (645.27, 1374.28, 1.24), (687.09, 1284.25, 0.32), (714.08, 1188.01, 1.84), (795.5, 1156.71, 0.81), (895.37, 1161.49, 0.17), (995.24, 1157.71, 0.23), (1093.89, 1142.3, 0.41), (1187.11, 1107.14, 0.62), (1264.9, 1045.35, 0.69), (1316.95, 960.57, 0.54), (1344.49, 864.61, 0.31), (1356.97, 765.45, 0.19), (1360.0, 665.52, 0.1), (1358.19, 565.54, 0.08), (1352.15, 465.74, 0.09), (1341.44, 366.33, 0.07), (1327.26, 267.34, 0.04), (1311.34, 168.61, 0.01), (1295.76, 69.84, 0.08), (1283.91, -29.44, 0.22), (1282.79, -129.31, 0.47), (1305.12, -226.18, 0.83), (1365.06, -304.81, 1.14)], [(1446.26, -344.01, 0), (1378.21, -271.14, 0.83), (1346.34, -177.62, 0.65), (1346.82, -77.7, 0.09), (1352.03, 22.16, 0.05)], [(1349.18, 94.91, 0), (1303.19, 6.14, 0.15), (1264.05, -85.86, 0.14), (1231.49, -180.39, 0.13), (1205.05, -276.81, 0.11), (1184.06, -374.57, 0.1), (1167.77, -473.23, 0.08), (1155.52, -572.47, 0.07), (1146.6, -672.06, 0.05), (1140.29, -771.86, 0.04), (1136.03, -871.77, 0.37), (1114.25, -967.15, 0.63), (1062.54, -1052.65, 0.31), (998.28, -1129.13, 0.31), (922.81, -1194.55, 0.3), (838.33, -1247.85, 0.28), (747.2, -1288.82, 0.23), (651.96, -1319.13, 0.18), (554.36, -1340.78, 0.12)], [(377.5, -1376.02, 0), (433.93, -1293.46, 0.02), (489.34, -1210.22, 0.06), (542.1, -1125.28, 0.11), (590.03, -1037.54, 0.17), (630.38, -946.1, 0.24), (659.69, -850.59, 0.3), (674.18, -751.77, 0.32), (672.92, -651.91, 0.31)], [(640.55, -640.55, 0), (631.43, -541.01, 0.23), (633.9, -441.14, 0.32), (652.46, -343.03, 0.32), (686.34, -249.04, 0.13), (726.18, -157.33, 0.18), (757.44, -62.47, 0.35), (771.64, 36.36, 0.32)]]

    flags.color_setting = ColorSort.EJECT_RED

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
    controller.fwd_speed = 7 * 1.15
    brain.timer.event(motor.ladyBrown.spin, 200, (DirectionType.REVERSE, 100, VelocityUnits.PERCENT))
    log("Path 'grab_mogo_1'")
    controller.path(paths[0], backwards=True, look_ahead_dist=200, heading_authority=1.25)
    motor.ladyBrown.stop(BrakeType.COAST)
    # after we are at the mogo start retracting the LB bar
    # grab mobile goal
    pneumatic.mogo.set(True)
    # start intaking rings
    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    # wait for mogo grabber to engage
    sleep(200, TimeUnits.MSEC)
    # turn to face the rings on the auton line (w/timeout & threshold)
    drivetrain.set_timeout(1.5, TimeUnits.SECONDS)
    drivetrain.set_turn_threshold(5)
    drivetrain.set_turn_constant(0.6)
    drivetrain.turn_for(TurnType.LEFT, 90, RotationUnits.DEG, 100, VelocityUnits.PERCENT)

    controller.fwd_speed = 6
    log("path: grab_border_rings")
    controller.path(paths[1], [
        ["raise intake", (1294, 741), pneumatic.intake.set, (True,)], 
        ["speed up", (1294, 741), "fwd_speed", 7]
    ], [22], False, 200, 130, event_look_ahead_dist=120, heading_authority=1.25)

    pneumatic.intake.set(False)
    sleep(250, TimeUnits.MSEC)
    controller.fwd_speed = 3.5
    log("path: backup_for_stack_ring")
    controller.path(paths[2], backwards=True)
    # sleep(200, MSEC)

    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    controller.fwd_speed = 9
    log("path: bottom_blue_ring")
    controller.path(paths[3], [["drop goal", (951, -1162), pneumatic.mogo.set, [False]]], [], False, 250, 150, 200, None, 2.0)

    motor.intakeChain.stop()
    pneumatic.mogo.set(False)

    log("path: last_mogo")
    controller.fwd_speed = 6 * 1.15
    controller.path(paths[4], [], [], True, 300, 175, 75)
    pneumatic.mogo.set(True)
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    flags.color_setting = ColorSort.NONE
    
    drivetrain.turn_for(TurnType.LEFT, 150, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 9.5
    log("path: ladder_hit")
    controller.path(paths[5], [], [], False, 300, 70, 75, 2000)
