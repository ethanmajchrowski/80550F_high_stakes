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
    controller.dynamic_vars["intake_color_sort"] = "eject_blue"

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
    controller.dynamic_vars["fwd_speed"] = 7
    brain.timer.event(motors["misc"]["wall_stake"].spin, 200, (DirectionType.REVERSE, 100, VelocityUnits.PERCENT))
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
    drivetrain.turn_for(TurnType.RIGHT, 90, RotationUnits.DEG, 100, VelocityUnits.PERCENT)

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

    # ladder touch cause of early end
    controller.dynamic_vars["fwd_speed"] = 6
    print("path: pinch_for_ladder")
    controller.path(paths["pinch_for_ladder"]["points"], [], [], True)

    print("path: ladder_touch")
    controller.path(paths["ladder_touch"]["points"], [], [], False)
