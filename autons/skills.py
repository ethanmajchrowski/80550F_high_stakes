def gen_data():
    data = {
        "start_pos": [-1549.4, 0],
        "start_heading": 90,
    }
    return data

def gen_paths(main):
        paths = {
             # Grab ring at (-600, -600)
            "ring_1": {
                "points": ((-1497.68, -41.68), (-1369.97, -65.87), (-1140.74, -78.85), (-912.32, -64.04), (-824.19, 24.0), (-820.45, 42.12), (-800, 100)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False, 350, 150, 75, None, 1.5)            
            },
            # Positive corners side
            "mogo_1": {
                # "points": ((-317.0, -997.0), (-413.31, -925.72), (-524.79, -882.81), (-643.48, -865.67), (-761.36, -843.94), (-874.23, -803.79), (-980.94, -749.09), (-1081.83, -684.2), (-1178.16, -612.65), (-1270.92, -536.52), (-1358.14, -459.91)),
                "points": ((-1215.59, -63.93), (-1114.52, -193.92), (-1112.07, -323.9), (-1109.86, -453.88), (-1108.71, -583.87), (-1108.52, -664.88), (-1108, -750)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)    
            },
            "bottom_red_rings": {
                #"points": ((-1350.0, -460.0), (-1243.54, -534.61), (-1137.15, -609.31), (-1030.87, -684.18), (-924.93, -759.52), (-819.68, -835.82), (-716.35, -914.69), (-619.65, -1001.33), (-582.2, -1120.97), (-605.35, -1238.07), (-719.57, -1219.71), (-800.34, -1118.02), (-881.51, -1016.56), (-976.83, -928.53), (-1088.49, -862.62), (-1132.9, -982.24), (-1153.07, -1110.58), (-1162.57, -1240.23), (-1173.35, -1369.76), (-1203.25, -1492.41), (-1331.07, -1475.27), (-1432.13, -1396.49), (-1485.3, -1278.84), (-1501.76, -1150.24), (-1494.38, -1030.98)),
                "points": ((-1187.31, -601.25), (-1059.48, -577.61), (-930.2, -566.84), (-800.52, -569.57), (-671.75, -585.9), (-545.24, -615.52), (-422.55, -658.52), (-304.98, -713.9), (-193.15, -780.0), (-88.21, -856.56), (8.21, -943.59), (93.07, -1041.9), (157.66, -1154.27), (184.92, -1279.44), (121.02, -1386.31), (-8.54, -1388.25), (-138.42, -1383.38), (-266.03, -1360.25), (-383.95, -1306.06), (-498.72, -1245.02), (-619.91, -1201.86), (-748.95, -1186.34), (-878.82, -1182.8), (-1008.63, -1189.64), (-1138.43, -1196.95), (-1268.34, -1201.71), (-1395.56, -1207.86), (-1428.81, -1332.98), (-1386.79, -1450.09), (-1265.61, -1492.35), (-1135.82, -1496.83), (-1061.82, -1493.79), (-900, -1492), (-800, -1490)),
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
                "points": ((-883.3, -884.05), (-987.33, -806.81), (-1070.44, -707.5), (-1100.01, -590.88), (-1100.67, -466.06), (-1100.8, -337.89), (-1100.71, -208.37), (-1100.59, -78.44), (-1100.68, 51.56), (-1100.71, 181.55), (-1100.01, 311.52), (-1100.67, 441.5), (-1100.77, 571.49), (-1100.08, 701.48)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)  
            },
            "top_red_rings": {
                "points": ((-1215.59, 607.73), (-1095.62, 557.66), (-969.73, 525.64), (-840.22, 516.42), (-711.45, 532.4), (-587.56, 570.79), (-471.14, 628.07), (-363.22, 700.25), (-263.76, 783.84), (-172.96, 876.82), (-89.12, 976.12), (-11.23, 1080.16), (61.34, 1188.0), (106.02, 1302.18), (-7.43, 1339.63), (-133.73, 1309.58), (-257.77, 1270.68), (-382.2, 1233.05), (-508.07, 1200.63), (-635.95, 1177.66), (-765.4, 1166.79), (-895.19, 1171.64), (-1025.01, 1178.27), (-1154.95, 1176.89), (-1284.76, 1173.09), (-1414.24, 1184.54), (-1424.2, 1308.69), (-1343.53, 1407.82), (-1227.08, 1464.32), (-1099.58, 1487.35), (-1024.7, 1484.41), (-900, 1484), (-800, 1485)),
                "events": [],
                "checkpoints": [8,13,17,],
                "custom_args": (False,150)  
            },
            "top_mogo": {
                "points": ((-1470.46, 1120.31), (-1499.69, 1206.98), (-1528.92, 1473.65), (-1558.16, 1600.32)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,350, 150, 75, 1000,)
            },
            "blue_mogo_neg_corner": {
                "points": ((-1689.28, 1661.17), (-1565.84, 1620.38), (-1442.6, 1579.03), (-1319.71, 1536.62), (-1197.19, 1493.17), (-1074.87, 1449.14), (-952.79, 1404.47), (-830.87, 1359.34), (-708.97, 1314.19), (-586.97, 1269.27), (-464.75, 1225.0), (-342.07, 1181.99), (-218.65, 1141.14), (-94.22, 1103.52), (31.53, 1070.56), (158.81, 1044.21), (287.54, 1026.51), (417.26, 1019.65), (544.82, 1006.96), (668.15, 965.88), (791.84, 925.87), (913.42, 880.05), (1025.42, 814.71), (1119.88, 725.76), (1200.02, 623.47), (1325.5, 601.57), (1448.66, 636.75), (1531.91, 734.7), (1580.09, 854.9), (1607.18, 981.95), (1624.77, 1110.73), (1637.89, 1240.06), (1651.48, 1369.34), (1671.44, 1497.74), (1714.25, 1619.75), (1736.92, 1653.48)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False,270, 150, 75, 9000,)
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
            "grab_mogo_4": {
                "points": ((1676.06, 1512.69), (1641.66, 1418.82), (1602.41, 1326.86), (1558.94, 1236.81), (1513.61, 1147.68), (1468.9, 1058.23), (1427.03, 967.43), (1390.19, 874.49), (1358.63, 779.62), (1333.41, 682.88), (1313.76, 584.85), (1298.97, 485.97), (1288.47, 386.54), (1281.59, 286.79), (1277.74, 186.87), (1277.34, 86.87), (1281.2, -89.65)),
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
            "fill_mogo_4": {
                 # Drives under ladder w/mogo 4, filling and ends by positive corner,
                 # ready to drop into positive
                # "points": ((1195.3, -7.37), (1181.88, 91.45), (1149.64, 186.03), (1106.64, 276.26), (1056.75, 362.88), (1000.36, 445.44), (938.41, 523.88), (870.05, 596.82), (792.58, 659.88), (699.91, 693.32), (629.59, 631.22), (548.17, 573.2), (471.55, 508.99), (398.95, 440.23), (328.95, 368.83), (259.98, 296.41), (190.99, 224.03), (121.36, 152.25), (50.77, 81.42), (8.42, 21.58), (107.89, 11.31), (207.13, -0.93), (305.54, -18.47), (401.87, -45.04), (493.04, -85.63), (572.14, -146.18), (626.13, -229.61), (648.52, -326.57), (644.76, -426.29), (636.26, -523.42), (665.09, -619.02), (666.97, -718.42), (634.64, -812.72), (598.5, -905.86), (581.74, -1004.23), (581.49, -1104.08), (583.9, -1203.55), (621.26, -1295.56), (689.48, -1367.94), (776.02, -1400.48), (870.68, -1400.36), (1030.89, -1400.95), (1167.18, -1400.6), (1246.25, -1400.23), (1280.53, -1312.27), (1280.95, -1212.59), (1260.15, -1114.91), (1223.86, -1021.84), (1189.9, -960.49)),
                "points": ((1195.3, -7.37), (1181.88, 91.45), (1149.64, 186.03), (1106.64, 276.26), (1056.75, 362.88), (1000.36, 445.44), (938.41, 523.88), (870.05, 596.82), (792.58, 659.88), (699.91, 693.32), (629.59, 631.22), (548.17, 573.2), (471.55, 508.99), (398.95, 440.23), (328.95, 368.83), (259.98, 296.41), (190.99, 224.03), (121.36, 152.25), (50.77, 81.42), (8.36, 21.13), (106.83, 3.97), (203.32, -22.13), (296.8, -57.5), (385.7, -103.15), (467.29, -160.78), (537.68, -231.52), (591.37, -315.54), (622.7, -410.19), (630.99, -509.47), (664.93, -603.49), (684.07, -701.38), (672.74, -800.34), (653.25, -898.36), (647.88, -998.12), (646.78, -1097.75), (646.22, -1197.47), (677.7, -1291.68), (744.35, -1365.34), (831.21, -1414.29), (926.22, -1445.06), (1024.46, -1463.42), (1123.87, -1473.98), (1215.43, -1437.36), (1270.63, -1355.79), (1283.84, -1257.16), (1271.42, -1158.15), (1241.74, -1062.79), (1189.9, -960.49)),
                "events": [],
                "checkpoints": [24,40,],
                "custom_args": (False,300)
            },
            "4th_mogo_corner_drop": {
                "points": ((1255.08, -1224.22), (1297.65, -1291.92), (1346.44, -1355.26), (1402.37, -1412.38), (1463.98, -1463.38), (1527.21, -1512.38), (1586.83, -1565.66), (1639.44, -1625.85), (1673.06, -1673.71)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True, 350, 100, 75, 3000)
            },
            "elevate": {
                "points": ((1570.01, -1527.43), (1508.02, -1413.17), (1435.57, -1305.26), (1353.95, -1204.13), (1264.87, -1109.5), (1170.08, -1020.57), (1071.23, -936.17), (969.72, -854.96), (866.69, -775.69), (763.05, -697.21), (659.53, -618.57), (556.84, -538.85), (455.6, -457.31), (356.12, -373.64), (258.63, -287.67), (163.21, -199.39), (70.75, -108.01), (0.46, -35.65)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False, 450)
            }, "score_mogo_2_corner": {
                "points": ((-1020.0, 1480.0), (-1099.04, 1468.26), (-1178.93, 1469.8), (-1257.68, 1483.42), (-1333.91, 1507.48), (-1406.75, 1540.45), (-1475.6, 1581.11), (-1539.91, 1628.62), (-1611.44, 1695.93)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False, 450)
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
    brain = main["brain"]
    controller = main["auton"]
    drivetrain = main["drivetrain"]
    imu = main["imu"]

    imu.set_heading(90)
    controller.dynamic_vars["position"] = [-1549.4, 0]

    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    main["sleep"](500)
    motors["misc"]["intake_chain"].stop(BrakeType.COAST)
    main["sleep"](100)

    # controller.dynamic_vars["intake_auto_halt"] = True
    controller.fwd_speed = 5.5
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
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 3.5
    main["sleep"](100)
    path_run(controller, paths["bottom_red_rings"])
    # controller.dynamic_vars["mogo_listen"] = False
    main["sleep"](100)
    path_run(controller, paths["score_first_mogo"])
    motors["misc"]["intake_chain"].spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
    motors["misc"]["intake_flex"].spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 5.5
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
    controller.fwd_speed = 5
    path_run(controller, paths["mogo_2"])
    main["sleep"](100)
    main["mogo_pneu"].set(True)
    controller.dynamic_vars["intake_auto_halt"] = False  
    
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 4
    path_run(controller, paths["top_red_rings"])
    main["sleep"](500)
    # controller.dynamic_vars["mogo_listen"] = False 
    path_run(controller, paths["score_mogo_2_corner"])
    motors["misc"]["intake_flex"].spin(DirectionType.REVERSE, 20, VelocityUnits.PERCENT)
    motors["misc"]["intake_chain"].spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
    main["mogo_pneu"].set(False)
    main["sleep"](300)
    motors["misc"]["intake_chain"].stop()
    motors["misc"]["intake_flex"].stop()
    #controller.dynamic_vars["position"] = [-1370, 1480]

    controller.dynamic_vars["intake_auto_halt"] = True      
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    controller.fwd_speed = 5.5
    path_run(controller, paths["blue_mogo_neg_corner"])
    controller.fwd_speed = 3.5

    controller.dynamic_vars["intake_auto_halt"] = False      

    drivetrain.drive(DirectionType.REVERSE, 50, VelocityUnits.PERCENT)
 
    main["sleep"](500)
    path_run(controller, paths["grab_mogo_4"])

    main["sleep"](300)
    main["mogo_pneu"].set(True)
    
    motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    main["sleep"](100)
    path_run(controller, paths["fill_mogo_4"])
    main["sleep"](300)

    path_run(controller, paths["4th_mogo_corner_drop"])
    main["mogo_pneu"].set(False)
    motors["misc"]["intake_chain"].spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
    drivetrain.drive(DirectionType.FORWARD, 50, VelocityUnits.PERCENT)
    main["sleep"](2000)
    drivetrain.drive(DirectionType.FORWARD, 0, VelocityUnits.PERCENT)
    
