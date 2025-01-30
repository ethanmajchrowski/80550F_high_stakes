def gen_data():
    data = {
        "start_pos": [1560, 330],
        "start_heading": 150,
    }
    print(data["start_pos"])
    return data

def gen_paths(main):
        paths = {
                "grab_mogo_1": {
                     "points": ((1547.38, 312.87), (1509.84, 405.3), (1449.82, 485.02), (1373.22, 549.05), (1285.75, 597.23), (1191.58, 630.47), (1093.57, 649.61), (993.85, 654.94), (894.34, 646.14), (797.41, 621.95), (706.28, 581.18), (651.29, 545.86)),
                    #  "points": ((1547.38, 312.87), (1493.37, 397.02), (1436.66, 479.38), (1376.91, 559.55), (1313.8, 637.1), (1247.06, 711.54), (1176.48, 782.34), (1101.93, 848.93), (1023.38, 910.78), (940.89, 967.27), (854.44, 1017.5), (764.79, 1061.71), (672.36, 1099.78), (577.65, 1131.74), (481.14, 1157.76), (383.28, 1178.13), (284.46, 1193.23), (185.02, 1203.51), (-9.02, 1211.53)),
                     "events": [],
                     "checkpoints": [],
                     "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                }, 
                "grab_border_rings": {
                     "points": ((576.13, 492.17), (523.6, 577.15), (483.29, 668.58), (453.25, 763.91), (431.37, 861.45), (415.67, 960.19), (404.4, 1059.55), (395.83, 1159.18), (388.09, 1258.88), (378.5, 1358.41), (461.27, 1402.91), (559.22, 1418.49), (645.27, 1374.28), (687.09, 1284.25), (714.08, 1188.01), (795.5, 1156.71), (895.37, 1161.49), (995.24, 1157.71), (1093.89, 1142.3), (1187.11, 1107.14), (1264.9, 1045.35), (1316.95, 960.57), (1344.49, 864.61), (1356.97, 765.45), (1360.0, 665.52), (1358.19, 565.54), (1352.15, 465.74), (1341.44, 366.33), (1327.26, 267.34), (1311.34, 168.61), (1295.76, 69.84), (1283.91, -29.44), (1282.79, -129.31), (1305.12, -226.18), (1365.06, -304.81), (1413.57, -336.74)),
                     "events": [["raise intake", (1274, 0), main["intake_pneu"].set, (False,)]], # 1130, 470
                     "checkpoints": [],
                     "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                }, 
                "backup_for_stack_ring": {
                     "points": ((1446.26, -344.01), (1378.21, -271.14), (1346.34, -177.62), (1346.82, -77.7), (1352.03, 22.16), (1354.55, 100.28)),
                    #  "points": ((1547.38, 312.87), (1493.37, 397.02), (1436.66, 479.38), (1376.91, 559.55), (1313.8, 637.1), (1247.06, 711.54), (1176.48, 782.34), (1101.93, 848.93), (1023.38, 910.78), (940.89, 967.27), (854.44, 1017.5), (764.79, 1061.71), (672.36, 1099.78), (577.65, 1131.74), (481.14, 1157.76), (383.28, 1178.13), (284.46, 1193.23), (185.02, 1203.51), (-9.02, 1211.53)),
                     "events": [],
                     "checkpoints": [],
                     "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                },
                "bottom_blue_ring": {
                     "points": ((1349.18, 94.91), (1303.19, 6.14), (1264.05, -85.86), (1231.49, -180.39), (1205.05, -276.81), (1184.06, -374.57), (1167.77, -473.23), (1155.52, -572.47), (1146.6, -672.06), (1140.29, -771.86), (1136.03, -871.77), (1114.25, -967.15), (1062.54, -1052.65), (998.28, -1129.13), (922.81, -1194.55), (838.33, -1247.85), (747.2, -1288.82), (651.96, -1319.13), (554.36, -1340.78)),
                     "events": [],
                     "checkpoints": [],
                     "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                },
                "last_mogo": {
                     "points": ((377.5, -1376.02), (432.45, -1292.47), (483.2, -1206.32), (527.71, -1116.81), (563.67, -1023.57), (588.05, -926.67), (599.42, -827.41), (597.36, -727.51), (584.03, -628.46)),
                     "events": [],
                     "checkpoints": [],
                     "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                }, 
                "ladder_hit": {
                     "points": ((640.55, -640.55), (631.43, -541.01), (633.9, -441.14), (652.46, -343.03), (686.34, -249.04), (726.18, -157.33), (757.44, -62.47), (771.64, 36.36), (769.39, 137.86)),
                     "events": [],
                     "checkpoints": [],
                     "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                }
        }
        return paths

def path_run(controller, path):
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

    # imu.set_heading(150)
    # imu.set_rotation(150)
    main["LB_enable_PID"] = False
    motors["misc"]["wall_stake"].stop(BrakeType.COAST)
    # controller.heading = 150 # 150!
    main["sleep"](100, TimeUnits.MSEC)
    # controller.dynamic_vars["position"] = [1560, 330]
    controller.dynamic_vars["intake_color_sort"] = "eject_red"

    # drop preload onto alliance wall stake
    # motors["misc"]["wall_stake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    # wait for LB to move
    end_time = 7500 + brain.timer.system()
    while (main["wallEnc"].position() < 190):
        if (brain.timer.system() < end_time):
            break
        else:
            motors["misc"]["wall_stake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    # stop LB mech
    motors["misc"]["wall_stake"].stop(BrakeType.COAST)
    # path to grab mogo @ 8V
    controller.dynamic_vars["fwd_speed"] = 7
    motors["misc"]["wall_stake"].spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
    controller.path(paths["grab_mogo_1"]["points"], [], [], True, 200, 100, 75, None, 1.25, 8, 0.1, 0.01, 0, 0, None, 0)
    motors["misc"]["wall_stake"].stop(BrakeType.COAST)
    # after we are at the mogo start retracting the LB bar
    # grab mobile goal
    main["mogo_pneu"].set(True)
    # start intaking rings
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 65, VelocityUnits.PERCENT)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    # wait for mogo grabber to engage
    main["sleep"](200, TimeUnits.MSEC)
    # turn to face the rings on the auton line (w/timeout & threshold)
    drivetrain.set_timeout(1.5, TimeUnits.SECONDS)
    drivetrain.set_turn_threshold(5)
    drivetrain.set_turn_constant(0.6)
    drivetrain.turn_for(TurnType.LEFT, 90, RotationUnits.DEG, 100, VelocityUnits.PERCENT)

    controller.dynamic_vars["fwd_speed"] = 6
    print("path: grab_border_rings")
    controller.path(paths["grab_border_rings"]["points"], [
        ["raise intake", (1294, 741), main["intake_pneu"].set, (True,)], 
        ["speed up", (1294, 741), "fwd_speed", 7]
    ], [22], False, 200, 130, 120, None, 1.25, 8, 0.1, 0.01, 0, 0, None, 0)

    main["intake_pneu"].set(False)
    main["sleep"](250, TimeUnits.MSEC)
    controller.dynamic_vars["fwd_speed"] = 3
    print("path: backup_for_stack_ring")
    controller.path(paths["backup_for_stack_ring"]["points"], [], [], True)
    # main["sleep"](200, MSEC)

    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    controller.dynamic_vars["fwd_speed"] = 9
    print("path: bottom_blue_ring")
    controller.path(paths["bottom_blue_ring"]["points"], [["drop goal", (951, -1162), main["mogo_pneu"].set, [False]]], [], False, 300, 100, 200)

    motors["misc"]["intake_chain"].stop()
    main["mogo_pneu"].set(False)

    print("path: last_mogo")
    controller.dynamic_vars["fwd_speed"] = 6
    controller.path(paths["last_mogo"]["points"], [], [], True, 300, 100, 75)
    main["mogo_pneu"].set(True)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    controller.dynamic_vars["intake_color_sort"] = "none"
    
    drivetrain.turn_for(TurnType.LEFT, 150, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 65, VelocityUnits.PERCENT)
    controller.dynamic_vars["fwd_speed"] = 8
    print("path: ladder_hit")
    controller.path(paths["ladder_hit"]["points"], [], [], False, 300, 70, 75, 2000)



