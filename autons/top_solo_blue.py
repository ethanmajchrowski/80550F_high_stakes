def gen_data():
    data = {
        "start_pos": [1500, 300],
        "start_heading": 0,
    }
    return data

def gen_paths(main):
        paths = {
            "side_stake_align": {
                # "points": ((1500.0, 440.0), (1489.89, 350.63), (1491.16, 260.73), (1505.37, 171.98), (1534.64, 87.02), (1580.47, 9.8), (1641.7, -55.82), (1715.44, -107.02), (1797.52, -143.49), (1884.26, -167.2), (1973.43, -178.94), (2063.31, -182.55), (2094.39, -182.37)),
                "points": ((1500.0, 440.0), (1493.89, 350.22), (1496.83, 260.34), (1512.83, 171.92), (1545.19, 88.16), (1595.06, 13.53), (1660.05, -48.35), (1735.36, -97.32), (1817.89, -132.97), (1903.96, -159.0), (1992.11, -176.83), (2081.39, -187.91), (2171.17, -193.58), (2261.13, -194.96), (2358.86, -193.02)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,50,50,75,1300)            
            },
            "mogo_align": {
                "points": ((1583.0, 30.0), (1473.24, -17.19), (1354.73, -24.11), (1243.95, 17.88), (1156.52, 99.15), (1090.29, 198.8), (1043.82, 309.25), (1013.77, 425.29), (998.07, 544.1), (996.86, 663.91), (1011.33, 782.82), (1043.69, 898.16), (1099.89, 1003.71), (1192.04, 1102.49)),
                # "points": ((1583.0, 30.0), (1463.2, 35.12), (1345.97, 59.5), (1235.84, 106.13), (1141.92, 179.73), (1071.86, 276.52), (1035.56, 390.46), (1030.47, 509.77), (1053.12, 627.31), (1102.63, 736.31), (1170.92, 834.7), (1283.35, 939.45)),
                # "events": [["stop intake", (1060,130), main["motors"]["intake"].stop, (main["BrakeType"].COAST,)]],
                # "events": [[["drop intake"], (1060, 130), main["intake_pneu"].set, (False,)]],
                "events": [],
                "checkpoints": [],
                "custom_args": (False,350,100,100)
            },
            "grab_mogo": {
                "points": ((1200.0, 900.0), (1091.01, 849.79), (980.54, 802.94), (868.87, 759.02), (757.18, 715.18), (651.15, 659.47), (565.12, 576.47), (498.13, 477.08)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)    
            },
            "fill_mogo": {
                # 2 Ring:
                "points": ((500.0, 475.0), (611.26, 559.97), (714.31, 654.51), (792.3, 769.83), (801.87, 907.22), (742.33, 1032.87), (650.92, 1138.67), (614.74, 1174.19)),
                # "points": ((505.35, 462.2), (542.15, 597.28), (576.85, 732.81), (598.7, 871.09), (611.19, 1010.26), (612.14, 1150.05), (595.87, 1289.03), (554.85, 1422.65), (479.63, 1539.43), (361.04, 1603.77), (239.6, 1548.84), (174.7, 1426.47), (148.09, 1289.63), (145.86, 1149.99), (155.59, 1010.47), (173.01, 871.56), (189.61, 732.55), (204.14, 593.31), (184.96, 532.07), (159.22, 490.11)),
                "events": [[["start intake to load disc and grab next one"], (720, 700), main["motors"]["intake"].spin, (main["DirectionType"].FORWARD, 100, main["PercentUnits"].PERCENT)],],
                "checkpoints": [],
                "custom_args": (False,)
            },
            "touch_ladder": {
                "points": ((614.0, 1174.0), (623.4, 1034.34), (634.51, 894.82), (626.37, 755.35), (584.32, 622.2), (516.84, 499.71), (433.34, 387.44)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)
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
    # Custom objects
    motors = main["motors"]
    controller = main["auton"]
    drivetrain = main["drivetrain"]

    controller.dynamic_vars["fwd_speed"] = 4

    path_run(controller, paths["side_stake_align"])
    
    motors["misc"]["intake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    main["sleep"](800)
    motors["misc"]["intake"].stop(BrakeType.COAST)

    # # Prep for reverse from alliance stake
    controller.dynamic_vars["position"] = [1570, 50]
    main["intake_pneu"].set(True,)
    motors["misc"]["intake"].spin(DirectionType.FORWARD,75,VelocityUnits.PERCENT,)
    controller.dynamic_vars["fwd_speed"] = 5
    controller.dynamic_vars["intake_auto_halt"] = True
    controller.dynamic_vars["drop_after_auto_halt"] = False
    controller.dynamic_vars["raise_after_auto_halt"] = True

    # Actually drive
    path_run(controller, paths["mogo_align"])
    controller.dynamic_vars["position"] = [1200, 1000]
    # # Stop the intake in the case it didnt automatically
    motors["misc"]["intake"].stop(BrakeType.COAST)

    # # Prep to grab the pneumatic
    main["mogo_pneu"].set(True)
    controller.dynamic_vars["mogo_grab_tolerance"] = 65
    main["intake_pneu"].set(False)
    controller.dynamic_vars["mogo_listen"] = True

    path_run(controller, paths["grab_mogo"])
    # Grab w/pneu regardless of if it auto
    main["mogo_pneu"].set(False)
    # # Start intaking to load the blue disc
    controller.dynamic_vars["intake_auto_halt"] = False
    main["intake_pneu"].set(False)
    # motors["misc"]["intake"].spin(DirectionType.FORWARD,100,VelocityUnits.PERCENT)
    path_run(controller, paths["fill_mogo"])
    main["sleep"](1000)
    motors["misc"]["intake"].stop(BrakeType.COAST)

    path_run(controller, paths["touch_ladder"])
    # main["mogo_pneu"].set(True)
    motors["misc"]["intake"].spin(DirectionType.REVERSE,25,VelocityUnits.PERCENT,)
    main["sleep"](1500)
    motors["misc"]["intake"].stop(BrakeType.HOLD)


    # # drivetrain.turn_for(TurnType.RIGHT, 15, RotationUnits.DEG, 60, VelocityUnits.PERCENT)
    # # main["sleep"](100)
    # # drivetrain.turn_for(TurnType.LEFT, 30, RotationUnits.DEG, 60, VelocityUnits.PERCENT)
    # # main["sleep"](100)