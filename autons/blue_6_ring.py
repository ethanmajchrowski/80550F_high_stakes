def gen_data():
    data = {
        "start_pos": [1500, 1500],
        "start_heading": 45,
    }
    return data

def gen_paths(main):
    paths = {
        "corner": {
                "points": ((1508.16, 1494.38), (1571.54, 1542.91), (1625.95, 1601.5), (1675.33, 1664.42), (1722.48, 1729.05), (1767.23, 1791.67)),
                "events": [],
                "checkpoints": [],
                "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
        },
        "mogo": {
            "points": ((1699.44, 1660.51), (1626.21, 1628.29), (1553.94, 1594.03), (1482.92, 1557.25), (1413.57, 1517.41), (1346.41, 1474.0), (1282.05, 1426.54), (1221.15, 1374.73), (1164.29, 1318.5), (1112.69, 1257.4), (1065.62, 1192.74), (1022.65, 1125.29), (982.56, 1056.06), (943.6, 986.19), (904.22, 916.55), (863.28, 847.83), (820.01, 780.54), (773.7, 715.33), (724.39, 652.35), (672.28, 591.66), (617.61, 533.27), (560.68, 477.07), (493.13, 415.64)),
            "events": [],
            "checkpoints": [],
            "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
        }, 
        "top_border_ring": {
            "points": ((510.12, 419.89), (550.58, 488.9), (589.3, 558.89), (625.64, 630.13), (658.69, 702.96), (687.16, 777.68), (709.21, 854.52), (722.32, 933.33), (720.32, 1013.03), (696.96, 1088.98), (648.83, 1152.14), (583.14, 1197.22), (509.09, 1227.16), (431.7, 1247.21), (353.0, 1261.47), (273.81, 1272.82), (158.62, 1289.34)),
            "events": [],
            "checkpoints": [],
            "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
        }, 
        "backup_for_bottom_border_ring": {
            "points": ((142.86, 1346.06), (222.81, 1348.81), (302.75, 1346.66), (381.88, 1335.5), (457.13, 1309.15), (521.56, 1262.2), (583.17, 1211.4), (657.61, 1183.32), (737.05, 1174.95), (824.4, 1177.86)),
            "events": [],
            "checkpoints": [],
            "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
        }, 
        "bottom_border_ring": {
            "points": ((824.4, 1177.86), (746.22, 1160.92), (669.46, 1138.49), (594.59, 1110.32), (519.76, 1082.11), (441.71, 1065.17), (361.85, 1062.02), (282.08, 1067.61), (161.33, 1084.96)),
            "events": [],
            "checkpoints": [],
            "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
        }, 
        "last_ring_reorient": {
            "points": ((139.71, 1103.44), (216.38, 1080.6), (292.75, 1056.75), (368.91, 1032.27), (444.98, 1007.51), (521.11, 982.95), (597.45, 959.02), (674.17, 936.35), (751.51, 915.91), (829.64, 898.78), (908.77, 887.19), (988.68, 885.7), (1066.04, 904.17), (1126.75, 954.32), (1152.94, 1029.13), (1156.39, 1093.4)),
            "events": [],
            "checkpoints": [],
            "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
        },
        "last_ring": {
            "points": ((1227.37, 897.57), (1225.59, 817.59), (1222.81, 737.64), (1218.51, 657.76), (1212.22, 578.01), (1203.25, 498.52), (1191.46, 419.4), (1178.58, 340.44), (1167.43, 261.23), (1159.06, 181.67), (1153.13, 101.89), (1147.99, -2.51)), 
            "events": [],
            "checkpoints": [],
            "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
        }, 
        "end_backup": {
            "points": ((1189.64, 16.42), (1189.1, 96.42), (1188.04, 176.41), (1186.94, 256.41), (1186.11, 336.4), (1185.89, 416.4), (1186.73, 496.39), (1189.64, 577.03)),
            "events": [],
            "checkpoints": [],
            "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
        }, 
    }

    return paths

def path_run(controller, path):
    print("run path")
    controller.path(path["points"], path["events"], path["checkpoints"], *path["custom_args"])

def run(main):
    paths = gen_paths(main)

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

    drivetrain.set_turn_constant(0.5)
    drivetrain.set_turn_threshold(5)
    drivetrain.set_timeout(1500) # MSEC

    main["LB_enable_PID"] = False

    print("auton testing")
    # 6 ring elims
    imu.set_heading(45)
    controller.heading = 45
    controller.dynamic_vars["position"] = [1500, 1500]
    print("{}, {}".format(controller.dynamic_vars["position"], imu.heading()))
    motors["misc"]["wall_stake"].stop(BrakeType.COAST)
    controller.dynamic_vars["intake_color_sort"] = "eject_red"

    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.dynamic_vars["fwd_speed"] = 5
    print("starting path: corner")
    controller.path(paths["corner"]["points"], [], [], False, 350, 100, 75, 1000)

    # motors["misc"]["intake_flex"].stop()
    controller.dynamic_vars["fwd_speed"] = 5.5
    print("starting path: mogo")
    controller.path(paths["mogo"]["points"], [], [], True)

    main["mogo_pneu"].set(True)
    main["sleep"](200, TimeUnits.MSEC)

    #motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    brain.timer.event(motors["misc"]["intake_chain"].spin, 300, (DirectionType.FORWARD, 65, VelocityUnits.PERCENT))
    # motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 65, VelocityUnits.PERCENT)
    controller.dynamic_vars["fwd_speed"] = 4.5
    print("starting path: top_border_ring")
    controller.path(paths["top_border_ring"]["points"], [], [], False)
    main["sleep"](200, TimeUnits.MSEC)

    controller.dynamic_vars["fwd_speed"] = 7
    print("starting path: backup_for_bottom_border_ring")
    controller.path(paths["backup_for_bottom_border_ring"]["points"], [], [], True)

    controller.dynamic_vars["fwd_speed"] = 4.5
    print("starting path: bottom_border_ring")
    controller.path(paths["bottom_border_ring"]["points"], [], [], False)
    main["sleep"](200, TimeUnits.MSEC)

    controller.dynamic_vars["fwd_speed"] = 7
    print("starting path: last_ring_reorient")
    controller.path(paths["last_ring_reorient"]["points"], [], [], True, 350, 200)

    print("intake up")
    main["intake_pneu"].set(True)

    controller.dynamic_vars["fwd_speed"] = 6
    print("starting path: last_ring")
    controller.path(paths["last_ring"]["points"], [], [], False, 350, 100, 75, 5000)

    motors["misc"]["intake_chain"].stop()
    main["sleep"](500, TimeUnits.MSEC)
    print("intake down")
    main["intake_pneu"].set(False)
    main["sleep"](250, TimeUnits.MSEC)

    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    controller.dynamic_vars["fwd_speed"] = 4
    print("starting path: end_backup")
    controller.path(paths["end_backup"]["points"], [], [], True)

    motors["misc"]["intake_flex"].stop()
    motors["misc"]["intake_chain"].stop()
    controller.kill_motors()
    main["mogo_pneu"].set(True)
