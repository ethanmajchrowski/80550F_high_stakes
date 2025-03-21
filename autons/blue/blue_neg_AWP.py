def gen_data():
    data = {
        "start_pos": [1510.0, 330.0],
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
    AutonomousFlags = main["AutonomousFlags"]

    #endregion Bindings

    paths = [[(1510.0, 330.0, 0), (1474.09, 364.72, 0.54), (1433.79, 394.26, 0.37), (1390.89, 419.92, 0.12), (1347.25, 444.33, 0.25), (1305.25, 471.39, 0.84), (1269.92, 506.51, 1.45), (1249.67, 551.84, 1.39), (1246.36, 601.54, 0.88), (1254.08, 650.88, 0.44)], [(1259.67, 710.7, 0), (1243.86, 663.27, 0.16), (1229.95, 615.24, 0.16), (1218.0, 566.69, 0.16), (1208.04, 517.7, 0.16), (1200.05, 468.35, 0.16), (1194.02, 418.72, 0.15), (1189.83, 368.9, 0.13), (1187.29, 318.96, 0.12), (1186.2, 268.98, 0.09), (1186.27, 218.98, 0.06), (1187.15, 168.99, 0.03), (1188.45, 119.0, 0.0), (1189.71, 69.02, 0.04), (1190.45, 19.02, 0.08), (1190.13, -30.97, 0.13)], [(1184.28, -76.7, 0), (1182.92, -26.73, 0.4), (1176.63, 22.85, 0.38), (1165.61, 71.6, 0.37), (1150.17, 119.14, 0.35), (1130.68, 165.16, 0.33), (1107.51, 209.45, 0.3), (1081.05, 251.85, 0.29), (1051.61, 292.25, 0.27), (1019.51, 330.57, 0.26), (985.01, 366.74, 0.25), (948.32, 400.69, 0.24), (909.64, 432.36, 0.24), (869.13, 461.65, 0.24), (826.93, 488.45, 0.25), (783.17, 512.6, 0.25), (737.95, 533.92, 0.27), (691.4, 552.13, 0.29), (643.65, 566.92, 0.32), (594.89, 577.88, 0.33)], [(543.48, 582.95, 0), (543.72, 632.95, 0.05), (544.54, 682.94, 0.06), (546.13, 732.92, 0.07), (548.62, 782.86, 0.07), (552.03, 832.74, 0.05), (556.11, 882.57, 0.03), (560.54, 932.38, 0.0), (565.02, 982.17, 0.01), (569.41, 1031.98, 0.01), (573.63, 1081.8, 0.07), (577.02, 1131.67, 0.05), (579.75, 1181.5, 0.87), (593.18, 1229.38, 1.37), (621.94, 1269.69, 1.55), (664.09, 1295.86, 1.14), (712.17, 1309.03, 0.68), (761.89, 1313.91, 0.4), (811.87, 1313.82, 0.21), (861.79, 1311.1, 0.12), (911.61, 1306.87, 0.07), (961.36, 1301.82, 0.01), (1011.09, 1296.67, 0.03), (1060.86, 1291.85, 0.06), (1110.69, 1287.75, 0.09), (1160.59, 1284.72, 0.12), (1210.56, 1283.13, 0.14), (1260.56, 1283.33, 0.18), (1310.49, 1285.84, 0.21), (1360.22, 1290.99, 0.24), (1409.53, 1299.15, 0.27), (1458.16, 1310.66, 0.31), (1505.76, 1325.86, 0.34), (1551.91, 1345.0, 0.36), (1596.15, 1368.19, 0.37), (1638.05, 1395.38, 0.38), (1677.22, 1426.38, 0.37), (1713.38, 1460.83, 0.35), (1746.44, 1498.28, 0.32), (1776.42, 1538.23, 0.28), (1803.51, 1580.22, 0.25), (1827.89, 1623.87, 0.3), (1848.93, 1669.23, 0.19), (1867.82, 1715.49, 0.15), (1884.92, 1762.46, 0.15)], [(1785.64, 1764.06, 0), (1752.28, 1726.82, 0.02), (1718.72, 1689.75, 0.02), (1684.94, 1652.89, 0.03), (1650.92, 1616.25, 0.03), (1616.61, 1579.88, 0.03), (1582.0, 1543.79, 0.04), (1547.05, 1508.04, 0.04), (1511.72, 1472.66, 0.05), (1475.96, 1437.71, 0.05), (1439.77, 1403.21, 0.05), (1403.12, 1369.21, 0.05), (1366.0, 1335.71, 0.05), (1328.42, 1302.72, 0.05), (1290.44, 1270.21, 0.04)], [(1260.43, 1242.9, 0), (1213.73, 1225.04, 0.03), (1166.91, 1207.48, 0.04), (1119.94, 1190.36, 0.06), (1072.72, 1173.9, 0.08), (1025.2, 1158.37, 0.1), (977.28, 1144.09, 0.13), (928.92, 1131.42, 0.17), (880.06, 1120.82, 0.18), (830.78, 1112.39, 0.18), (781.17, 1106.22, 0.16), (731.35, 1102.0, 0.13), (681.42, 1099.43, 0.1), (631.44, 1098.05, 0.07), (581.44, 1097.54, 0.05), (531.44, 1097.63, 0.03), (481.44, 1098.1, 0.02), (431.45, 1098.83, 0.01), (381.46, 1099.69, 0.0), (331.46, 1100.61, 0.0)], [(115.12, 1096.69, 0), (165.07, 1094.64, 0.0), (215.03, 1092.55, 0.01), (264.98, 1090.39, 0.01), (314.93, 1088.13, 0.01), (364.88, 1085.79, 0.01), (414.82, 1083.35, 0.01), (464.75, 1080.81, 0.01), (514.68, 1078.14, 0.01), (564.6, 1075.31, 0.01), (614.51, 1072.31, 0.02), (664.41, 1069.1, 0.02), (714.29, 1065.64, 0.02), (764.15, 1061.88, 0.04), (813.97, 1057.68, 0.05), (863.74, 1052.9, 0.07), (913.42, 1047.25, 0.08), (962.97, 1040.56, 0.25), (1012.79, 1036.95, 0.43), (1062.75, 1038.74, 0.07)], [(1153.21, 1052.83, 0), (1104.09, 1043.65, 0.41), (1056.17, 1029.5, 0.48), (1010.31, 1009.7, 0.42), (966.74, 985.22, 0.43), (926.1, 956.16, 0.37), (888.29, 923.48, 0.35), (853.49, 887.62, 0.31), (821.56, 849.16, 0.26), (792.17, 808.72, 0.27), (765.56, 766.4, 0.2), (741.12, 722.79, 0.19), (718.73, 678.09, 0.19), (698.53, 632.36, 0.15), (680.05, 585.91, 0.14), (663.19, 538.84, 0.14), (647.94, 491.22, 0.13), (634.25, 443.14, 0.11), (621.88, 394.7, 0.11), (610.79, 345.94, 0.1), (600.98, 296.92, 0.11), (592.47, 247.65, 0.09), (585.09, 198.2, 0.09), (578.83, 148.59, 0.1), (573.81, 98.85, 0.09), (569.97, 49.0, 0.09), (567.27, -0.93, 0.11), (565.93, -50.91, 0.1), (565.89, -100.9, 0.1)]]
    controller.robot.pos = [1510.0, 330.0]
    sensor.imu.set_heading(150)
    # eject red
    flags.color_setting = ColorSort.EJECT_RED

    motor.ladyBrown.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    end_timeout = brain.timer.system() + 700
    while sensor.wallEncoder.angle() < 190:
        if brain.timer.system() > end_timeout:
            break
        sleep(20, TimeUnits.MSEC)
    log("Lady brown finished or timed out. Encoder at {} degrees.".format(sensor.wallEncoder.angle()))
    motor.ladyBrown.stop(BrakeType.COAST)

    controller.fwd_speed = 7
    controller.path(paths[0], backwards=True, speed_ramp_time=400)

    AutonomousFlags.intake_auto_halt = True
    pneumatic.intake.set(True)
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    AutonomousFlags.lady_brown_autostop = True
    motor.ladyBrown.spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)

    controller.path(paths[1], speed_ramp_time=400, slowdown_distance=800)

    # grab mobile goal
    controller.path(paths[2], backwards=True, speed_ramp_time=400, slowdown_distance=800)
    pneumatic.intake.set(False)
    pneumatic.mogo.set(True)
    sleep(150, TimeUnits.MSEC)

    # turn towards ring
    drivetrain.turn_for(TurnType.LEFT, 90, RotationUnits.DEG, 80, VelocityUnits.PERCENT)
    
    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 8
    # drive into corner
    controller.path(paths[3], speed_ramp_time=400, slowdown_distance=700, timeout=3000)

    controller.fwd_speed = 6
    controller.path(paths[4], backwards=True)

    drivetrain.turn_for(TurnType.LEFT, 170, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 8
    AutonomousFlags.intake_color_kill = main["Color"].RED
    controller.path(paths[5], slowdown_distance=800, speed_ramp_time=400)
    controller.path(paths[6], backwards=True, speed_ramp_time=400, slowdown_distance=600)
    controller.path(paths[7], speed_ramp_time=400, slowdown_distance=600, timeout=2500)
