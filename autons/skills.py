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
                "points": ((-1590.0, 0.0), (-1460.52, -11.26), (-1332.05, -30.99), (-1205.2, -59.33), (-1080.69, -96.5), (-959.89, -144.36), (-846.48, -207.58), (-746.48, -289.94), (-673.36, -396.28), (-622.42, -515.34), (-599.41, -642.05), (-620.32, -770.09), (-555.34, -876.76), (-443.26, -942.34), (-317.15, -997.86)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False, 350, 150, 75, None, 1.5)            
            },
            # Positive corners side
            "mogo_1": {
                # "points": ((-317.0, -997.0), (-413.31, -925.72), (-524.79, -882.81), (-643.48, -865.67), (-761.36, -843.94), (-874.23, -803.79), (-980.94, -749.09), (-1081.83, -684.2), (-1178.16, -612.65), (-1270.92, -536.52), (-1358.14, -459.91)),
                "points": ((-317.0, -997.0), (-412.75, -924.9), (-521.62, -875.26), (-638.06, -846.5), (-754.6, -818.03), (-867.12, -776.52), (-974.02, -722.2), (-1075.84, -658.8), (-1173.35, -588.94), (-1267.3, -514.33), (-1400.0, -399.97)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)    
            },
            "bottom_red_rings": {
                #"points": ((-1350.0, -460.0), (-1243.54, -534.61), (-1137.15, -609.31), (-1030.87, -684.18), (-924.93, -759.52), (-819.68, -835.82), (-716.35, -914.69), (-619.65, -1001.33), (-582.2, -1120.97), (-605.35, -1238.07), (-719.57, -1219.71), (-800.34, -1118.02), (-881.51, -1016.56), (-976.83, -928.53), (-1088.49, -862.62), (-1132.9, -982.24), (-1153.07, -1110.58), (-1162.57, -1240.23), (-1173.35, -1369.76), (-1203.25, -1492.41), (-1331.07, -1475.27), (-1432.13, -1396.49), (-1485.3, -1278.84), (-1501.76, -1150.24), (-1494.38, -1030.98)),
                "points": ((-1350.0, -460.0), (-1226.28, -499.91), (-1102.74, -540.38), (-979.45, -581.61), (-856.82, -624.74), (-734.84, -669.7), (-613.85, -717.21), (-494.37, -768.38), (-377.3, -824.79), (-264.1, -888.58), (-157.2, -962.36), (-62.9, -1051.44), (12.85, -1156.71), (59.02, -1277.81), (79.33, -1405.92), (-21.07, -1461.09), (-147.77, -1435.26), (-267.45, -1384.77), (-382.29, -1323.92), (-493.94, -1257.35), (-603.64, -1187.6), (-712.03, -1115.84), (-819.66, -1042.94), (-927.15, -969.82), (-1035.24, -897.59), (-1112.91, -921.93), (-1131.46, -1050.21), (-1128.9, -1180.12), (-1120.24, -1309.83), (-1119.4, -1439.74), (-1171.11, -1538.82), (-1283.59, -1479.38), (-1366.96, -1379.95), (-1436.35, -1270.09), (-1496.92, -1155.1), (-1549.94, -1036.43), (-1560.05, -1009.53)),
                "events": [],
                "checkpoints": [9,15,18,],
                "custom_args": (False, 150,)
            },
            "score_first_mogo": {
                "points": ((-1500.0, -1000.0), (-1481.01, -1128.59), (-1470.96, -1258.12), (-1500.01, -1387.51), (-1575.46, -1484.62)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,) 
            },
            "drive_second_mogo": {
                "points": ((-1630.0, -1610.0), (-1599.58, -1442.74), (-1580.43, -1273.85), (-1565.74, -1104.5), (-1550.91, -935.15), (-1532.58, -766.18), (-1508.04, -598.01), (-1471.45, -431.99), (-1419.59, -270.27), (-1352.31, -114.64), (-1269.21, 33.2), (-1166.76, 168.86), (-1045.6, 287.53), (-914.64, 395.03), (-771.32, 486.47), (-492.68, 623.44)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False,)
            },
            "mogo_2": {
                "points": ((-500.0, 620.0), (-629.47, 728.31), (-791.07, 771.67), (-957.37, 743.47), (-1111.56, 672.73), (-1254.72, 581.18), (-1405.72, 476.18), (-1450, 400)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)  
            },
            "top_red_rings": {
                "points": ((-1022.84, 764.53), (-904.94, 819.3), (-789.9, 879.63), (-680.91, 950.1), (-590.51, 1041.72), (-605.96, 1155.66), (-727.26, 1196.82), (-855.42, 1182.33), (-939.63, 1093.3), (-993.51, 979.96), (-1082.52, 906.83), (-1163.48, 961.37), (-1171.75, 1090.55), (-1175.25, 1217.93), (-1219.63, 1334.77), (-1344.64, 1335.59), (-1449.31, 1260.73), (-1503.48, 1144.74), (-1470.46, 1020.31), (-1470, 900)),
                "events": [],
                "checkpoints": [8,13,17,],
                "custom_args": (False,150)  
            },
            "top_mogo": {
                "points": ((-1470.46, 1020.31), (-1499.69, 1146.98), (-1528.92, 1273.65), (-1558.16, 1400.32), (-1650.76, 1450.92)),
                "events": [],
                "checkpoints": [],
                "custom_args": (True,)
            },
            "ring_3": {
                "points": ((-1370.0, 1480.0), (-1300.47, 1370.16), (-1215.33, 1271.98), (-1116.81, 1187.31), (-1007.84, 1116.63), (-891.32, 1059.22), (-769.66, 1013.65), (-644.64, 978.24), (-517.51, 951.29), (-389.08, 931.28), (-259.91, 916.83), (-130.31, 906.73), (-0.49, 899.86), (129.42, 895.13), (259.36, 891.37), (388.03, 887.5)),
                "events": [],
                "checkpoints": [],
                "custom_args": (False,)
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

    motors["intake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    main["sleep"](500)
    motors["intake"].stop(BrakeType.COAST)
    main["sleep"](100)

    controller.dynamic_vars["intake_auto_halt"] = True
    controller.dynamic_vars["fwd_speed"] = 5.5
    motors["intake"].spin(DirectionType.FORWARD, 40, VelocityUnits.PERCENT)
    path_run(controller, paths["ring_1"])
    main["mogo_pneu"].set(True)
    motors["intake"].stop()
    main["sleep"](100)

    controller.dynamic_vars["mogo_listen"] = True
    path_run(controller, paths["mogo_1"])
    main["sleep"](150)
    main["mogo_pneu"].set(False)

    controller.dynamic_vars["intake_auto_halt"] = False
    motors["intake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.dynamic_vars["fwd_speed"] = 3.5
    main["sleep"](100)
    path_run(controller, paths["bottom_red_rings"])
    controller.dynamic_vars["mogo_listen"] = False
    main["sleep"](100)
    path_run(controller, paths["score_first_mogo"])
    motors["intake"].spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
    controller.dynamic_vars["fwd_speed"] = 5.5
    main["sleep"](100)
    main["mogo_pneu"].set(True)
    main["sleep"](300)
    motors["intake"].stop()
    
    motors["intake"].spin(DirectionType.FORWARD, 40, VelocityUnits.PERCENT)    
    controller.dynamic_vars["intake_auto_halt"] = True    
    path_run(controller, paths["drive_second_mogo"])
    controller.dynamic_vars["mogo_listen"] = True 
    controller.dynamic_vars["fwd_speed"] = 3.5
    path_run(controller, paths["mogo_2"])
    main["sleep"](100)
    main["mogo_pneu"].set(False)
    controller.dynamic_vars["intake_auto_halt"] = False  
    
    motors["intake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    path_run(controller, paths["top_red_rings"])    
    controller.dynamic_vars["mogo_listen"] = False 
    path_run(controller, paths["top_mogo"])
    motors["intake"].spin(DirectionType.REVERSE, 20, VelocityUnits.PERCENT)
    main["mogo_pneu"].set(True)
    main["sleep"](300)
    motors["intake"].stop()
    #controller.dynamic_vars["position"] = [-1370, 1480]

    controller.dynamic_vars["intake_auto_halt"] = True      
    motors["intake"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    path_run(controller, paths["ring_3"])
    controller.dynamic_vars["intake_auto_halt"] = False      
    #motors["intake"].spin(DirectionType.FORWARD, 20, VelocityUnits.PERCENT)
    #main["sleep"](500)
    #motors["intake"].spin(DirectionType.REVERSE, 20, VelocityUnits.PERCENT)
    #main["sleep"](1000)
    #path_run(controller, paths["side_stake"])
    #main["side_scoring_a"].set(True)
    #main["side_scoring_b"].set(True)
    #main["sleep"](750)
    #main["side_scoring_a"].set(False)
    #main["side_scoring_b"].set(False)

    controller.dynamic_vars["fwd_speed"] = 5.5
    motors["intake"].spin(DirectionType.FORWARD, 60, VelocityUnits.PERCENT)
    controller.dynamic_vars["intake_auto_halt"] = True      
    path_run(controller, paths["ring_4"])
    drivetrain.drive(DirectionType.REVERSE, 50, VelocityUnits.PERCENT)
    main["intake_pneu"].set(True)
    controller.dynamic_vars["mogo_listen"] = True 
    main["sleep"](1000)
    path_run(controller, paths["mogo_3"])
    main["intake_pneu"].set(False)
    main["mogo_pneu"].set(False)
    main["sleep"](100)
    controller.dynamic_vars["intake_auto_halt"] = False      
    
    motors["intake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    main["sleep"](100)
    path_run(controller, paths["fill_mogo_3"])
    path_run(controller, paths["score_mogo_3"])
    main["mogo_pneu"].set(True)
    motors["intake"].spin(DirectionType.REVERSE, 20, VelocityUnits.PERCENT)
    main["elevation_pneu"].set(True)
    controller.dynamic_vars["fwd_speed"] = 7.5
    path_run(controller, paths["elevate"])