def gen_data():
    data = {
        "start_pos": [1500, 200],
        "start_heading": 360,
    }
    return data

def gen_paths(main):
        paths = {
            "side_stake_align": {
                # "points": ((1500.0, 440.0), (1489.89, 350.63), (1491.16, 260.73), (1505.37, 171.98), (1534.64, 87.02), (1580.47, 9.8), (1641.7, -55.82), (1715.44, -107.02), (1797.52, -143.49), (1884.26, -167.2), (1973.43, -178.94), (2063.31, -182.55), (2094.39, -182.37)),
                # "points": ((1500.0, 440.0), (1492.96, 310.2), (1482.19, 180.65), (1485.61, 51.76), (1597.76, 3.44), (1727.7, -0.12), (1774.73, -0.87)),
                "points": ((1500.0, 440.0), (1504.07, 360.11), (1505.35, 280.13), (1503.81, 200.15), (1501.15, 120.19), (1510.97, 41.55), (1578.49, 5.57), (1658.29, 0.57), (1774.73, -0.87)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,150,50,75,1300,1.2)            
            },
            "mogo_align": {
                "points": ((1760.9, 0), (1634.36, -15.27), (1506.47, -25.92), (1377.25, -50.21), (1247.58, -25.31), (1120.47, -15.5), (1003.41, 0.63), (918.32, 50.23), (875.99, 214.76), (875.28, 344.24), (900.57, 471.45), (944.48, 593.69), (1001.8, 710.34), (1069.07, 821.55), (1110.46, 883.46)),
                # "points": ((1583.0, 30.0), (1463.2, 35.12), (1345.97, 59.5), (1235.84, 106.13), (1141.92, 179.73), (1071.86, 276.52), (1035.56, 390.46), (1030.47, 509.77), (1053.12, 627.31), (1102.63, 736.31), (1170.92, 834.7), (1283.35, 939.45)),
                # "events": [["stop intake", (1060,130), main["motors"]["intake"].stop, (main["BrakeType"].COAST,)]],
                # "events": [[["drop intake"], (1060, 130), main["intake_pneu"].set, (False,)]],
                "events": [],
                "checkpoints": [],
                "custom_args": (False,350,100,100)
            },
            "grab_mogo": {
                "points": ((1082.18, 1060.21), (983.1, 976.05), (884.02, 891.88), (784.94, 807.72), (685.86, 723.56), (586.79, 639.39), (487.71, 555.23)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)    
            },
            "fill_mogo": {
                # 2 Ring:
                "points": ((417.5, 374.42), (465.6, 473.34), (510.74, 573.64), (551.37, 675.84), (585.31, 780.44), (609.44, 887.69), (620.63, 997.04), (640.85, 1106.89), (680.86, 1207.16), (700, 1400),),
                # "points": ((505.35, 462.2), (542.15, 597.28), (576.85, 732.81), (598.7, 871.09), (611.19, 1010.26), (612.14, 1150.05), (595.87, 1289.03), (554.85, 1422.65), (479.63, 1539.43), (361.04, 1603.77), (239.6, 1548.84), (174.7, 1426.47), (148.09, 1289.63), (145.86, 1149.99), (155.59, 1010.47), (173.01, 871.56), (189.61, 732.55), (204.14, 593.31), (184.96, 532.07), (159.22, 490.11)),
                "events": [
                    [["start intake to load disc and grab next one"], (720, 700), main["motors"]["misc"]["intake_flex"].spin, (main["DirectionType"].FORWARD, 100, main["PercentUnits"].PERCENT)],
                  ],
                "checkpoints": [],
                "custom_args": (False,350,100,100)
            },
            "touch_ladder": {
                "points": ((608.49, 1180.4), (633.7, 1052.93), (650.36, 924.01), (674.4, 796.39), (732.97, 681.78), (839.05, 609.28), (966.0, 582.71), (1103.39, 579.45)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)
            },
            "touch": {
                # "points": ((940.78, 586.52), (888.16, 467.64), (832.52, 350.18), (757.24, 244.57), (685.64, 136.1), (608.49, 6.77)),
                "points": ((940.78, 586.52), (888.16, 467.64), (832.52, 350.18), (757.24, 244.57), (685.64, 136.1)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False,350,100,75,)
            },
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

    controller.dynamic_vars["fwd_speed"] = 3

    path_run(controller, paths["side_stake_align"])
    drivetrain.drive(DirectionType.REVERSE, 50, VelocityUnits.PERCENT)
    main["sleep"](300)
    drivetrain.stop()
    
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    main["sleep"](800)
    motors["misc"]["intake_chain"].stop(BrakeType.COAST)
    motors["misc"]["intake_flex"].stop(BrakeType.COAST)

    # # Prep for reverse from alliance stake
    controller.dynamic_vars["position"] = [1570, 50]
    main["intake_pneu"].set(True,)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD,75,VelocityUnits.PERCENT,)
    controller.dynamic_vars["fwd_speed"] = 5
    controller.dynamic_vars["intake_auto_halt"] = True
    controller.dynamic_vars["drop_after_auto_halt"] = False
    controller.dynamic_vars["raise_after_auto_halt"] = True

    # Actually drive
    
    path_run(controller, paths["mogo_align"])
      # controller.dynamic_vars["position"] = [1200, 1000]
    # # Stop the intake in the case it didnt automatically
    # motors["misc"]["intake_flex"].stop(BrakeType.COAST)
    # motors["misc"]["intake_chain"].stop(BrakeType.COAST)

    # # Prep to grab the pneumatic
    controller.dynamic_vars["mogo_grab_tolerance"] = 65
    main["intake_pneu"].set(False)
    # controller.dynamic_vars["mogo_listen"] = True

    path_run(controller, paths["grab_mogo"])
    # Grab w/pneu regardless of if it auto
    main["mogo_pneu"].set(True)
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD,75,VelocityUnits.PERCENT,)
    # # Start intaking to load the blue disc
    controller.dynamic_vars["intake_auto_halt"] = False
    main["intake_pneu"].set(False)
    # motors["misc"]["intake"].spin(DirectionType.FORWARD,100,VelocityUnits.PERCENT)
    path_run(controller, paths["fill_mogo"])
    main["sleep"](200)

    # Intake after fill mogo path just in case we didn't hit the event trigger for it
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 65, VelocityUnits.PERCENT)
    path_run(controller, paths["touch_ladder"])
    motors["misc"]["intake_flex"].spin(DirectionType.REVERSE,25,VelocityUnits.PERCENT,)
    motors["misc"]["intake_chain"].spin(DirectionType.REVERSE,25,VelocityUnits.PERCENT,)
    path_run(controller, paths["touch"])
    main["sleep"](200)
    motors["misc"]["intake_flex"].stop(BrakeType.HOLD)
    motors["misc"]["intake_chain"].stop(BrakeType.HOLD)


    # # drivetrain.turn_for(TurnType.RIGHT, 15, RotationUnits.DEG, 60, VelocityUnits.PERCENT)
    # # main["sleep"](100)
    # # drivetrain.turn_for(TurnType.LEFT, 30, RotationUnits.DEG, 60, VelocityUnits.PERCENT)
    # # main["sleep"](100)