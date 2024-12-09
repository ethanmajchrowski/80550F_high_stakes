def gen_data():
    data = {
        "start_pos": [-1590, 0],
        "start_heading": 90,
    }
    return data

def gen_paths(main):
        paths = {
             # Grab ring at (-600, -600)
            "ring_1": {
                "points": ((-1752.91, 6.77), (-1624.69, -13.98), (-1497.68, -41.68), (-1369.97, -65.87), (-1240.74, -78.85), (-1112.32, -64.04), (-1024.19, 24.0), (-1020.45, 42.12)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False, 350, 150, 75, None, 1.5)            
            },
            # Positive corners side
            "mogo_1": {
                # "points": ((-317.0, -997.0), (-413.31, -925.72), (-524.79, -882.81), (-643.48, -865.67), (-761.36, -843.94), (-874.23, -803.79), (-980.94, -749.09), (-1081.83, -684.2), (-1178.16, -612.65), (-1270.92, -536.52), (-1358.14, -459.91)),
                "points": ((-1215.59, -63.93), (-1214.52, -193.92), (-1212.07, -323.9), (-1209.86, -453.88), (-1208.71, -583.87), (-1208.52, -664.88), (-1208, -750)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)    
            },
            "bottom_red_rings": {
                #"points": ((-1350.0, -460.0), (-1243.54, -534.61), (-1137.15, -609.31), (-1030.87, -684.18), (-924.93, -759.52), (-819.68, -835.82), (-716.35, -914.69), (-619.65, -1001.33), (-582.2, -1120.97), (-605.35, -1238.07), (-719.57, -1219.71), (-800.34, -1118.02), (-881.51, -1016.56), (-976.83, -928.53), (-1088.49, -862.62), (-1132.9, -982.24), (-1153.07, -1110.58), (-1162.57, -1240.23), (-1173.35, -1369.76), (-1203.25, -1492.41), (-1331.07, -1475.27), (-1432.13, -1396.49), (-1485.3, -1278.84), (-1501.76, -1150.24), (-1494.38, -1030.98)),
                "points": ((-1187.31, -601.25), (-1059.4, -578.03), (-930.06, -568.38), (-800.47, -572.96), (-672.07, -591.86), (-546.35, -624.62), (-424.92, -671.04), (-309.2, -730.2), (-199.68, -800.07), (-97.45, -880.19), (-4.0, -970.39), (78.31, -1070.87), (141.38, -1184.17), (173.37, -1309.29), (123.42, -1424.68), (-1.06, -1456.08), (-130.6, -1462.33), (-254.53, -1427.02), (-358.27, -1348.97), (-462.0, -1270.77), (-575.85, -1208.71), (-704.57, -1190.66), (-834.24, -1182.4), (-964.13, -1186.84), (-1093.89, -1194.68), (-1223.77, -1200.3), (-1353.73, -1203.41), (-1421.74, -1289.05), (-1414.67, -1415.74), (-1309.46, -1484.49), (-1180.4, -1497.28), (-1061.82, -1493.79),(-960, -1495), (-900, -1495)),
                "events": [],
                "checkpoints": [9,15,18,],
                "custom_args": (False, 150,)
            },
            "score_first_mogo": {
                "points": ((-1116.61, -1506.22), (-1246.37, -1512.35), (-1363.1, -1561.98), (-1450.7, -1657.84), (-1554.48, -1735.84), (-1646.86, -1789.02)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True, 350, 150, 75, 1200,) 
            },
            "drive_second_mogo": {
                "points": ((-1696.35, -1690.04), (-1604.02, -1598.61), (-1520.2, -1499.29), (-1441.02, -1396.2), (-1363.07, -1292.16), (-1282.74, -1189.96), (-1196.3, -1092.93), (-1100.19, -1005.62), (-991.43, -934.98), (-925.72, -905.26)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False,)
            },
            "mogo_2": {
                "points": ((-883.3, -884.05), (-987.33, -806.81), (-1070.44, -707.5), (-1127.01, -590.88), (-1162.67, -466.06), (-1183.8, -337.89), (-1194.71, -208.37), (-1198.59, -78.44), (-1198.68, 51.56), (-1196.71, 181.55), (-1194.01, 311.52), (-1191.67, 441.5), (-1190.77, 571.49), (-1192.08, 701.48), (-1194.38, 798.62)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)  
            },
            "top_red_rings": {
                "points": ((-1215.59, 607.73), (-1095.62, 557.66), (-969.73, 525.64), (-840.22, 516.42), (-711.45, 532.4), (-587.56, 570.79), (-471.14, 628.07), (-363.22, 700.25), (-263.76, 783.84), (-172.96, 876.82), (-89.12, 976.12), (-11.23, 1080.16), (61.34, 1188.0), (109.13, 1302.8), (33.58, 1394.51), (-93.44, 1375.2), (-214.84, 1328.79), (-334.94, 1279.03), (-456.54, 1233.15), (-581.01, 1195.85), (-708.65, 1171.98), (-838.37, 1167.59), (-968.1, 1175.86), (-1098.04, 1179.63), (-1227.97, 1176.05), (-1357.89, 1175.78), (-1487.39, 1187.04), (-1489.98, 1308.8), (-1399.85, 1400.02), (-1282.06, 1454.11), (-1155.57, 1483.01), (-1024.7, 1484.41)),
                "events": [],
                "checkpoints": [8,13,17,],
                "custom_args": (False,150)  
            },
            "top_mogo": {
                "points": ((-1470.46, 1120.31), (-1499.69, 1206.98), (-1528.92, 1473.65), (-1558.16, 1600.32), (-1650.76, 1750.92)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,350, 150, 75, 1000,)
            },
            "ring_3": {
                "points": ((-1689.28, 1661.17), (-1566.02, 1619.86), (-1443.06, 1577.68), (-1320.69, 1533.78), (-1198.92, 1488.29), (-1077.48, 1441.89), (-956.57, 1394.14), (-835.99, 1345.55), (-715.62, 1296.46), (-595.4, 1246.98), (-475.15, 1197.58), (-354.66, 1148.78), (-233.74, 1101.06), (-112.13, 1055.14), (10.51, 1012.04), (134.55, 973.18), (260.34, 940.51), (388.05, 916.62), (517.37, 904.79), (647.29, 905.73), (777.16, 904.24), (902.12, 871.91), (987.89, 778.0), (1032.08, 655.84), (1089.11, 539.47), (1175.54, 442.93), (1266.45, 350.42), (1387.73, 323.74), (1467.23, 423.63), (1511.07, 545.69), (1538.91, 672.66), (1559.09, 801.08), (1574.76, 930.13), (1587.78, 1059.47), (1599.23, 1188.96), (1610.04, 1318.51), (1621.37, 1448.02), (1635.32, 1577.25), (1661.92, 1717.73)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False,350, 150, 75, 6000,)
            },
            "side_stake": {
                "points": ((280.0, 1390.0), (169.43, 1457.81), (80.14, 1551.69), (21.48, 1667.11), (2.5, 1747.6)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,150,100,75,1000)
            },
            "ring_4": {
                "points": ((550.95, 909.31), (622.58, 810.15), (696.46, 703.22), (774.6, 599.33), (854.55, 496.82), (930.47, 391.31), (1009.33, 288.12), (1096.65, 192.1), (1200.1, 114.69), (1322.32, 110.18), (1393.47, 215.48), (1422.76, 341.82), (1436.17, 471.06), (1440.4, 600.96), (1437.95, 730.92), (1431.17, 860.74), (1421.9, 990.37), (1431.36, 1119.73), (1470.88, 1243.23), (1530.79, 1358.54), (1591.55, 1473.44), (1625.98, 1552.38)),
                "events": [],
                "checkpoints": [10,],
                "custom_args": (False,)
            },
            "mogo_3": {
                "points": ((1471.03, 1350.08), (1397.42, 1243.19), (1341.41, 1126.04), (1297.32, 1003.81), (1261.72, 878.81), (1234.13, 751.78), (1212.41, 623.63), (1196.09, 494.68), (1184.94, 365.18), (1178.93, 235.33), (1178.25, 105.35), (1184.08, -24.51), (1195.3, -148.77)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,200)
            },
            "score_mogo_3": {
                "points": ((1560.02, -1235.3), (1580.91, -1363.59), (1615.55, -1488.75), (1654.26, -1612.81), (1659.97, -1635.11)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,150,100,75,750,)
            },
            "fill_mogo_3": {
                "points": ((1196.58, 30.67), (1158.84, 155.08), (1115.14, 277.19), (1059.95, 394.89), (983.11, 499.74), (884.34, 583.68), (761.64, 623.46), (632.99, 616.69), (511.12, 573.44), (400.25, 506.12), (300.22, 423.27), (210.0, 329.75), (128.93, 228.16), (57.31, 119.7), (-2.79, 4.51), (-44.17, -114.34), (79.66, -153.92), (200.86, -199.96), (317.2, -257.98), (419.71, -337.9), (502.43, -437.7), (553.29, -556.77), (571.01, -685.19), (565.86, -814.88), (547.95, -943.59), (526.88, -1071.87), (518.1, -1201.31), (542.87, -1327.87), (634.21, -1417.01), (751.36, -1472.26), (871.72, -1520.27), (999.72, -1538.94), (1128.0, -1521.18), (1245.9, -1467.89), (1347.1, -1387.1), (1427.91, -1285.65), (1484.29, -1168.77), (1509.74, -1041.7), (1503.0, -963.64)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False,)
            },
            "elevate": {
                "points": ((1570.01, -1527.43), (1508.02, -1413.17), (1435.57, -1305.26), (1353.95, -1204.13), (1264.87, -1109.5), (1170.08, -1020.57), (1071.23, -936.17), (969.72, -854.96), (866.69, -775.69), (763.05, -697.21), (659.53, -618.57), (556.84, -538.85), (455.6, -457.31), (356.12, -373.64), (258.63, -287.67), (163.21, -199.39), (70.75, -108.01), (0.46, -35.65)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False, 450)
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
    brain = main["brain"]
    controller = main["auton"]
    drivetrain = main["drivetrain"]

    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    main["sleep"](500)
    motors["misc"]["intake_chain"].stop(BrakeType.COAST)
    main["sleep"](100)

    # controller.dynamic_vars["intake_auto_halt"] = True
    controller.dynamic_vars["fwd_speed"] = 5.5
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 40, VelocityUnits.PERCENT)
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 40, VelocityUnits.PERCENT)
    path_run(controller, paths["ring_1"])
    main["mogo_pneu"].set(False)
    motors["misc"]["intake_chain"].stop()
    motors["misc"]["intake_flex"].stop()
    main["sleep"](100)

    # controller.dynamic_vars["mogo_listen"] = True
    path_run(controller, paths["mogo_1"])
    main["sleep"](150)
    main["mogo_pneu"].set(True)

    controller.dynamic_vars["intake_auto_halt"] = False
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.dynamic_vars["fwd_speed"] = 3.5
    main["sleep"](100)
    path_run(controller, paths["bottom_red_rings"])
    # controller.dynamic_vars["mogo_listen"] = False
    main["sleep"](100)
    path_run(controller, paths["score_first_mogo"])
    motors["misc"]["intake_chain"].spin(DirectionType.REVERSE, 60, VelocityUnits.PERCENT)
    motors["misc"]["intake_flex"].spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
    controller.dynamic_vars["fwd_speed"] = 5.5
    main["sleep"](100)
    main["mogo_pneu"].set(False)
    main["sleep"](300)
    # motors["misc"]["intake_chain"].stop()
    # motors["misc"]["intake_flex"].stop()
    
    # motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 40, VelocityUnits.PERCENT)    
    # motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 40, VelocityUnits.PERCENT)    
    controller.dynamic_vars["intake_auto_halt"] = True    
    path_run(controller, paths["drive_second_mogo"])
    # controller.dynamic_vars["mogo_listen"] = True 
    controller.dynamic_vars["fwd_speed"] = 3.5
    path_run(controller, paths["mogo_2"])
    main["sleep"](100)
    main["mogo_pneu"].set(True)
    controller.dynamic_vars["intake_auto_halt"] = False  
    
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    path_run(controller, paths["top_red_rings"])
    main["sleep"](500)
    # controller.dynamic_vars["mogo_listen"] = False 
    path_run(controller, paths["top_mogo"])
    motors["misc"]["intake_flex"].spin(DirectionType.REVERSE, 20, VelocityUnits.PERCENT)
    motors["misc"]["intake_chain"].spin(DirectionType.REVERSE, 20, VelocityUnits.PERCENT)
    main["mogo_pneu"].set(False)
    main["sleep"](300)
    motors["misc"]["intake_chain"].stop()
    motors["misc"]["intake_flex"].stop()
    #controller.dynamic_vars["position"] = [-1370, 1480]

    # controller.dynamic_vars["intake_auto_halt"] = True      
    # motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    # motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    controller.dynamic_vars["fwd_speed"] = 5.5
    path_run(controller, paths["ring_3"])
    controller.dynamic_vars["fwd_speed"] = 3.5
    # controller.dynamic_vars["intake_auto_halt"] = False      
    #motors["misc"]["intake"].spin(DirectionType.FORWARD, 20, VelocityUnits.PERCENT)
    #main["sleep"](500)
    #motors["misc"]["intake"].spin(DirectionType.REVERSE, 20, VelocityUnits.PERCENT)
    #main["sleep"](1000)
    #path_run(controller, paths["side_stake"])
    #main["side_scoring_a"].set(True)
    #main["side_scoring_b"].set(True)
    #main["sleep"](750)
    #main["side_scoring_a"].set(False)
    #main["side_scoring_b"].set(False)

    # controller.dynamic_vars["fwd_speed"] = 5.5
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    # controller.dynamic_vars["intake_auto_halt"] = True      
    # path_run(controller, paths["ring_4"])
    drivetrain.drive(DirectionType.REVERSE, 50, VelocityUnits.PERCENT)
    # main["intake_pneu"].set(True)
    # controller.dynamic_vars["mogo_listen"] = True 
    main["sleep"](500)
    path_run(controller, paths["mogo_3"])
    # main["intake_pneu"].set(False)
    main["sleep"](300)
    main["mogo_pneu"].set(True)
    # controller.dynamic_vars["intake_auto_halt"] = False      
    
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    main["sleep"](100)
    path_run(controller, paths["fill_mogo_3"])
    path_run(controller, paths["score_mogo_3"])
    main["mogo_pneu"].set(False)
    motors["misc"]["intake"].spin(DirectionType.REVERSE, 20, VelocityUnits.PERCENT)
    # main["elevation_pneu"].set(True)
    # controller.dynamic_vars["fwd_speed"] = 7.5
    # path_run(controller, paths["elevate"])