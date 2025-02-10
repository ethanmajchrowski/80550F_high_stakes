def gen_data():
    data = {
        "start_pos": [-1560, 330],
        "start_heading": 210,
    }
    print(data["start_pos"])
    return data

def gen_paths(main):
        paths = {
                "grab_mogo_1": {
                     "points": ((1547.38, 312.87), (1509.84, 405.3), (1449.82, 485.02), (1373.22, 549.05), (1285.75, 597.23), (1191.58, 630.47), (1093.57, 649.61), (993.85, 654.94), (894.34, 646.14), (797.41, 621.95), (706.28, 581.18), (651.29, 545.86)),
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
                     "events": [],
                     "checkpoints": [],
                     "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                }, 
                "pinch_for_ladder": {
                    "points": ((1354.35, 85.22), (1332.98, 171.91), (1286.18, 248.6), (1231.37, 319.98), (1179.42, 393.4), (1146.15, 476.37), (1158.0, 564.09), (1211.89, 635.49), (1234.12, 656.29)),
                },
                "ladder_touch": {
                    "points": ((1230.37, 656.29), (1168.69, 590.75), (1106.69, 525.51), (1044.29, 460.65), (981.26, 396.41), (917.34, 333.05), (852.26, 270.89), (787.17, 208.73), (723.89, 144.74), (662.13, 79.28), (601.2, 13.04), (591.67, 2.56)),
                }
        }
        return paths

def path_run(controller, path):
    controller.path(path["points"], path["events"], path["checkpoints"], *path["custom_args"])

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

    controller.dynamic_vars["position"] = [-1560, 330]
    controller.heading = 210
    imu.set_heading(210)
    imu.set_rotation(210)

    main["LB_enable_PID"] = False
    motors["misc"]["wall_stake"].stop(BrakeType.COAST)
    # controller.heading = 150 # 150!
    main["sleep"](100, TimeUnits.MSEC)
    # controller.dynamic_vars["position"] = [1560, 330]
    controller.dynamic_vars["intake_color_sort"] = "none"

    paths = [
         ((-1547.38, 312.87), (-1514.69, 396.54), (-1465.18, 471.5), (-1403.42, 536.75), (-1332.6, 592.08), (-1254.68, 636.98), (-1171.15, 670.23), (-1083.91, 691.79), (-994.39, 700.0), (-904.87, 692.34), (-818.2, 668.98), (-737.58, 629.34), (-666.72, 574.2), (-606.23, 507.77), (-591.56, 488.41)), ((-527.49, 491.49), (-530.41, 581.44), (-533.87, 671.36), (-541.94, 761.0), (-553.09, 850.25), (-568.45, 938.89), (-591.21, 1025.97), (-624.14, 1109.68), (-674.74, 1183.61), (-747.47, 1234.47), (-836.4, 1236.28), (-917.15, 1197.51), (-987.16, 1141.29), (-1047.11, 1074.26), (-1099.99, 1001.52), (-1146.59, 924.59), (-1187.31, 844.37), (-1222.18, 761.43), (-1250.86, 676.15), (-1271.96, 588.69), (-1284.9, 499.66), (-1289.91, 409.82), (-1289.73, 319.84), (-1284.52, 230.02), (-1274.43, 140.6), (-1259.71, 51.82), (-1241.57, -36.32), (-1220.64, -123.85), (-1197.57, -210.84), (-1172.92, -297.4), (-1147.2, -383.64), (-1120.96, -469.73), (-1094.52, -555.76), (-1068.18, -641.82), (-1042.14, -727.97), (-1016.64, -814.28), (-991.8, -900.79), (-967.66, -987.49), (-944.28, -1074.4), (-921.65, -1161.5), (-900.06, -1248.88), (-879.41, -1336.47), (-859.02, -1425.57)), ((-1302.91, 503.96), (-1296.52, 414.18), (-1291.35, 324.33), (-1288.21, 234.39), (-1287.65, 144.4), (-1288.67, 54.4), (-1288.85, -35.6), (-1287.39, -125.58), (-1284.6, -215.54), (-1280.85, -305.46), (-1280.03, -322.95)), ((-1204.77, -355.0), (-1150.31, -284.53), (-1151.23, -196.11), (-1177.11, -109.92), (-1197.73, -22.37), (-1207.99, 66.98), (-1209.35, 98.04)), ((-1349.18, 94.91), (-1266.68, 59.04), (-1188.49, 14.66), (-1117.21, -40.02), (-1057.89, -107.48), (-1015.31, -186.47), (-991.44, -273.0), (-984.39, -362.56), (-989.91, -452.3), (-1003.92, -541.18), (-1023.44, -629.03), (-1045.81, -716.2), (-1068.93, -803.18), (-1090.95, -890.44), (-1063.68, -972.9), (-1022.31, -1052.82), (-975.58, -1129.69), (-921.45, -1201.52), (-858.66, -1265.91), (-787.32, -1320.65), (-708.92, -1364.73), (-625.15, -1397.47), (-538.2, -1420.39), (-449.4, -1434.76), (-359.7, -1441.67), (-280.37, -1442.76)), ((-309.87, -1388.96), (-389.7, -1347.49), (-465.5, -1299.03), (-537.72, -1245.36), (-605.54, -1186.24), (-668.48, -1121.99), (-724.49, -1051.61), (-769.93, -974.11), (-798.54, -889.2), (-798.79, -800.02), (-763.09, -718.46), (-700.7, -654.12), (-623.59, -608.03), (-540.67, -573.37), (-474.62, -551.51)), ((-640.55, -640.55), (-629.54, -551.24), (-622.55, -461.52), (-621.75, -371.55), (-628.8, -281.87), (-641.3, -192.74), (-650.85, -103.27), (-652.9, -13.33), (-648.53, 76.55), (-646.47, 102.62))]

    main["LB_enable_PID"] = False
    motors["misc"]["wall_stake"].stop(BrakeType.COAST)
    # controller.heading = 150 # 150!
    main["sleep"](100, TimeUnits.MSEC)
    # controller.dynamic_vars["position"] = [1560, 330]
    # controller.dynamic_vars["intake_color_sort"] = "eject_red"

    # drop preload onto alliance wall stake
    # motors["misc"]["wall_stake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    # wait for LB to move
    
    print(brain.timer.system())
    end_time = 7500 + brain.timer.system()
    print(end_time)

    main["wallEnc"].set_position(20)
    while (main["wallEnc"].position() < 380):
        print(main["wallEnc"].position())
        if (brain.timer.system() > end_time):
            break
        else:
            motors["misc"]["wall_stake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        main["sleep"](20)

    print(main["wallEnc"].position())
    # stop LB mech
    motors["misc"]["wall_stake"].stop(BrakeType.COAST)
    # path to grab mogo @ 8V
    controller.fwd_speed = 7 * 1.15
    brain.timer.event(motors["misc"]["wall_stake"].spin, 200, (DirectionType.REVERSE, 60, VelocityUnits.PERCENT))
    controller.path(paths[0], [], [], True, 200, 100, 75, None, 1.25, 8, 0.1, 0.01, 0, 0, None, 0)
    motors["misc"]["wall_stake"].stop(BrakeType.COAST)
    # after we are at the mogo start retracting the LB bar
    # grab mobile goal
    main["mogo_pneu"].set(True)
    # start intaking rings
    brain.timer.event(motors["misc"]["intake_chain"].spin, 200, (DirectionType.FORWARD, 100, VelocityUnits.PERCENT))
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    # wait for mogo grabber to engage
    main["sleep"](200, TimeUnits.MSEC)

    controller.fwd_speed = 8
    controller.path(paths[1], [], [], False, 325, 100, 75, None, 2.5)
    main["intake_pneu"].set(True)
    # motors["misc"]["intake_chain"].stop()
    controller.path(paths[2], [], [], False, 350, 200, 75, None, 1.5)
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    main["intake_pneu"].set(False)
    controller.fwd_speed = 5
    sleep(500, TimeUnits.MSEC) # sleep to allow for top stack ring to get scored
    print("backup")
    controller.path(paths[3], [], [], True)
    sleep(500, TimeUnits.MSEC) # sleep to allow for top stack ring to get scored

    controller.fwd_speed = 7
    brain.timer.event(motors["misc"]["intake_chain"].stop, 200)
    # motors["misc"]["intake_flex"].stop()
    # controller.dynamic_vars["intake_auto_halt"] = True
    # controller.dynamic_vars["drop_after_auto_halt"] = True
    print("path: bottom_blue_ring")
    controller.path(paths[4], [["drop goal", (-951, -1162), main["mogo_pneu"].set, [False]]], [], False, 250, 150, 200, None, 2.0)

    main["mogo_pneu"].set(False)

    print("path: last_mogo")
    controller.fwd_speed = 6 * 1.15
    controller.path(paths[5], [], [], True, 300, 175, 75, None, 2)
    main["mogo_pneu"].set(True)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    # controller.dynamic_vars["intake_color_sort"] = "none" # this might be needed, idk
    
    drivetrain.turn_for(TurnType.RIGHT, 150, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    sleep(500, TimeUnits.MSEC)
    controller.fwd_speed = 10
    print("path: ladder_hit")
    controller.path(paths[6], [], [], False, 300, 70, 75, 2000, 2)
