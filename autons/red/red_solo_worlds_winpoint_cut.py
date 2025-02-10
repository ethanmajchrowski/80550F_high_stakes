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

    paths = [((-1547.38, 312.87), (-1509.88, 405.33), (-1452.3, 486.82), (-1380.58, 556.21), (-1298.65, 613.28), (-1208.81, 656.95), (-1113.23, 685.8), (-1014.32, 699.16), (-914.61, 694.22), (-818.23, 668.6), (-729.22, 623.74), (-652.36, 560.13), (-591.56, 488.41)), ((-527.49, 491.49), (-529.01, 591.48), (-532.83, 691.34), (-542.3, 790.89), (-560.04, 889.23), (-587.01, 985.33), (-627.9, 1076.22), (-689.61, 1154.0), (-778.34, 1197.53), (-876.59, 1189.84), (-964.99, 1144.69), (-1038.62, 1077.42), (-1099.53, 998.31), (-1146.37, 910.09), (-1180.7, 816.33), (-1202.17, 718.85), (-1209.89, 619.35), (-1200.2, 496.18)), ((-1218.5, 496.18), (-1211.46, 396.42), (-1206.02, 296.57), (-1203.31, 196.62), (-1203.66, 96.62), (-1204.54, -3.38), (-1203.67, -103.37), (-1200.95, -203.33), (-1195.62, -330.73)), ((-1204.77, -355.0), (-1147.2, -275.02), (-1156.64, -176.84), (-1185.02, -80.96), (-1203.69, 17.2), (-1209.35, 98.04)), ((-1349.18, 94.91), (-1257.78, 54.48), (-1172.02, 3.31), (-1096.09, -61.32), (-1036.3, -141.17), (-999.44, -233.89), (-984.97, -332.65), (-987.87, -432.49), (-1002.22, -531.4), (-1023.46, -629.1), (-1048.39, -725.94), (-1073.97, -822.61), (-1089.5, -918.83), (-1045.74, -1008.74), (-997.21, -1096.14), (-940.24, -1178.24), (-873.06, -1252.15), (-795.34, -1314.83), (-708.61, -1364.27), (-615.53, -1400.47), (-518.49, -1424.36), (-419.45, -1437.68), (-280.37, -1442.76)), ((-377.5, -1376.02), (-434.22, -1293.66), (-490.05, -1210.7), (-542.89, -1125.81), (-590.15, -1037.72), (-628.49, -945.42), (-653.63, -848.74), (-661.78, -749.23), (-651.53, -649.93), (-624.37, -553.8), (-574.24, -446.51)), ((-640.55, -640.55), (-628.55, -541.28), (-621.72, -441.54), (-623.2, -341.59), (-634.12, -242.22), (-647.45, -143.12), (-653.03, -43.32), (-646.47, 102.62))]

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
    controller.path(paths[2], [], [], False, 350, 160)
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
