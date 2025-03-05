def gen_data():
    data = {
        "start_pos": [-1500, 1500],
        "start_heading": 315,
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

    paths = [((-1508.16, 1494.38, 0.0), (-1571.54, 1542.91, 0.0), (-1625.95, 1601.5, 0.0), (-1675.33, 1664.42, 0.0), (-1722.48, 1729.05, 0.0), (-1767.23, 1791.67), 0.0),((-1699.44, 1660.51, 0.0), (-1626.21, 1628.29, 0.0), (-1553.94, 1594.03, 0.0), (-1482.92, 1557.25, 0.0), (-1413.57, 1517.41, 0.0), (-1346.41, 1474.0, 0.0), (-1282.05, 1426.54, 0.0), (-1221.15, 1374.73, 0.0), (-1164.29, 1318.5, 0.0), (-1112.69, 1257.4, 0.0), (-1065.62, 1192.74, 0.0), (-1022.65, 1125.29, 0.0), (-982.56, 1056.06, 0.0), (-943.6, 986.19, 0.0), (-904.22, 916.55, 0.0), (-863.28, 847.83, 0.0), (-820.01, 780.54, 0.0), (-773.7, 715.33, 0.0), (-724.39, 652.35, 0.0), (-672.28, 591.66, 0.0), (-617.61, 533.27, 0.0), (-560.68, 477.07, 0.0), (-493.13, 415.64), 0.0),((-510.12, 419.89, 0.0), (-550.58, 488.9, 0.0), (-589.3, 558.89, 0.0), (-625.64, 630.13, 0.0), (-658.69, 702.96, 0.0), (-687.16, 777.68, 0.0), (-709.21, 854.52, 0.0), (-722.32, 933.33, 0.0), (-720.32, 1013.03, 0.0), (-696.96, 1088.98, 0.0), (-648.83, 1152.14, 0.0), (-583.14, 1197.22, 0.0), (-509.09, 1227.16, 0.0), (-431.7, 1247.21, 0.0), (-353.0, 1261.47, 0.0), (-273.81, 1272.82, 0.0), (-158.62, 1289.34), 0.0),((-142.86, 1346.06, 0.0), (-222.81, 1348.81, 0.0), (-302.75, 1346.66, 0.0), (-381.88, 1335.5, 0.0), (-457.13, 1309.15, 0.0), (-521.56, 1262.2, 0.0), (-583.17, 1211.4, 0.0), (-657.61, 1183.32, 0.0), (-737.05, 1174.95, 0.0), (-824.4, 1177.86), 0.0),((-824.4, 1177.86, 0.0), (-746.22, 1160.92, 0.0), (-669.46, 1138.49, 0.0), (-594.59, 1110.32, 0.0), (-519.76, 1082.11, 0.0), (-441.71, 1065.17, 0.0), (-361.85, 1062.02, 0.0), (-282.08, 1067.61, 0.0), (-161.33, 1084.96), 0.0),((-139.71, 1103.44, 0.0), (-216.38, 1080.6, 0.0), (-292.75, 1056.75, 0.0), (-368.91, 1032.27, 0.0), (-444.98, 1007.51, 0.0), (-521.11, 982.95, 0.0), (-597.45, 959.02, 0.0), (-674.17, 936.35, 0.0), (-751.51, 915.91, 0.0), (-829.64, 898.78, 0.0), (-908.77, 887.19, 0.0), (-988.68, 885.7, 0.0), (-1066.04, 904.17, 0.0), (-1126.75, 954.32, 0.0), (-1152.94, 1029.13, 0.0), (-1156.39, 1093.4), 0.0),((-1227.37, 897.57, 0.0), (-1225.59, 817.59, 0.0), (-1222.81, 737.64, 0.0), (-1218.51, 657.76, 0.0), (-1212.22, 578.01, 0.0), (-1203.25, 498.52, 0.0), (-1191.46, 419.4, 0.0), (-1178.58, 340.44, 0.0), (-1167.43, 261.23, 0.0), (-1159.06, 181.67, 0.0), (-1153.13, 101.89, 0.0), (-1147.99, -2.51), 0.0), ((-1189.64, 16.42, 0.0), (-1189.1, 96.42, 0.0), (-1188.04, 176.41, 0.0), (-1186.94, 256.41, 0.0), (-1186.11, 336.4, 0.0), (-1185.89, 416.4, 0.0), (-1186.73, 496.39, 0.0), (-1189.64, 577.03), 0.0),]

    drivetrain.set_turn_constant(0.5)
    drivetrain.set_turn_threshold(5)
    drivetrain.set_timeout(1500) # MSEC

    log("auton testing")
    # 6 ring elims
    flags.color_setting = ColorSort.EJECT_BLUE

    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 5
    log("starting path: corner")
    controller.path(paths[0], timeout=1000)

    # motor.intakeFlex.stop()
    controller.fwd_speed = 5.5
    log("starting path: mogo")
    controller.path(paths[1], backwards=True)

    pneumatic.mogo.set(True)
    sleep(200, TimeUnits.MSEC)

    #motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    brain.timer.event(motor.intakeChain.spin, 300, (DirectionType.FORWARD, 65, VelocityUnits.PERCENT))
    # motor.intakeChain.spin(DirectionType.FORWARD, 65, VelocityUnits.PERCENT)
    controller.fwd_speed = 4.5
    log("starting path: top_border_ring")
    controller.path(paths[2])
    sleep(200, TimeUnits.MSEC)

    controller.fwd_speed = 7
    log("starting path: backup_for_bottom_border_ring")
    controller.path(paths[3], backwards=True)

    controller.fwd_speed = 4.5
    log("starting path: bottom_border_ring")
    controller.path(paths[4])
    sleep(200, TimeUnits.MSEC)

    controller.fwd_speed = 7
    log("starting path: last_ring_reorient")
    controller.path(paths[5], backwards=True, look_ahead_dist=50, finish_margin=200)

    log("intake up")
    pneumatic.intake.set(True)

    controller.fwd_speed = 6
    log("starting path: last_ring")
    controller.path(paths[6], [], [], False, 350, 100, 75, timeout=5000)

    motor.intakeChain.stop()
    sleep(500, TimeUnits.MSEC)
    log("intake down")
    pneumatic.intake.set(False)
    sleep(250, TimeUnits.MSEC)

    motor.intakeChain.spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    controller.fwd_speed = 4
    log("starting path: end_backup")
    controller.path(paths[7], backwards=True)

    motor.intakeFlex.stop()
    motor.intakeChain.stop()
    controller.kill_motors()
    pneumatic.mogo.set(True)
