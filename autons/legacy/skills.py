def gen_data():
    data = {
        "start_pos": [-1545.0, 0.0],
        "start_heading": 90,
    }
    return data

def run(main):
    #region Bindings
    motor = main["motor"]
    pneumatic = main["pneumatic"]
    drivetrain = main["drivetrain"]
    
    BrakeType = main["BrakeType"]
    DirectionType = main["DirectionType"]
    VelocityUnits = main["VelocityUnits"]
    TimeUnits = main["TimeUnits"]
    RotationUnits = main["RotationUnits"]
    TurnType = main["TurnType"]
    VoltageUnits = main["VoltageUnits"]
    # Custom objects
    robot = main["robot"]
    controller = robot.autonomous_controller
    color_sorter = robot.color_sort_controller
    drivetrain = main["drivetrain"]
    sensor = main["sensor"]
    imu = sensor.imu
    brain = main["brain"]
    log = main["log"]
    flags = main["flags"]
    ColorSort = main["ColorSort"]
    sleep = main["sleep"]
    math = main["math"]
    #endregion Bindings

    controller.robot.pos = [-1545.0, 0.0]
    sensor.imu.set_heading(90)

    # controller.mcl_controller.start()

    # place temporary / testing code here
    main["AutonomousFlags"].intake_anti_jam = True

    #* Skills
    controller = controller
    drivetrain.set_turn_threshold(2)
    drivetrain.set_turn_constant(0.28)
    paths = [[(-1593.76, 6.33, 0), (-1523.77, 5.76, 0.02), (-1453.78, 4.66, 0.01), (-1383.79, 3.36, 0.0), (-1313.8, 2.02, 0.0), (-1243.81, 0.76, 0.01), (-1173.82, -0.25, 0.04)], [(-1201.9, -9.15, 0), (-1201.64, -79.15, 0.04), (-1202.38, -149.14, 0.05), (-1204.43, -219.11, 0.05), (-1207.76, -289.03, 0.02), (-1211.54, -358.93, 0.03), (-1214.66, -428.86, 0.05), (-1216.65, -498.83, 0.04), (-1217.63, -568.82, 0.03), (-1217.77, -638.82, 0.03), (-1217.26, -708.82, 0.03)], [(-1244.4, -726.06, 0), (-1239.23, -656.36, 0.62), (-1219.12, -589.65, 0.93), (-1178.74, -533.1, 1.11), (-1119.66, -496.47, 0.91), (-1051.8, -480.43, 0.63), (-981.94, -479.64, 0.43), (-912.71, -489.49, 0.32), (-844.99, -507.01, 0.25), (-779.06, -530.46, 0.23), (-715.23, -559.17, 0.21), (-653.69, -592.48, 0.21), (-594.82, -630.29, 0.25), (-539.42, -673.02, 0.32), (-489.23, -721.71, 0.35), (-434.68, -763.2, 0.52), (-372.14, -794.63, 0.08), (-310.47, -827.75, 0.04), (-249.22, -861.62, 0.08), (-188.9, -897.14, 0.07), (-129.43, -934.04, 0.05), (-70.67, -972.07, 0.08), (-13.05, -1011.82, 0.09), (43.27, -1053.38, 0.08), (98.37, -1096.53, 0.1), (151.98, -1141.51, 0.11), (203.77, -1188.58, 0.14), (253.28, -1238.05, 0.16), (299.87, -1290.27, 0.2), (342.6, -1345.66, 0.26), (380.07, -1404.72, 0.46), (407.56, -1468.92, 0.81), (415.8, -1537.77, 1.83), (380.8, -1595.35, 1.94), (314.74, -1615.29, 1.01), (245.1, -1610.63, 0.44), (176.88, -1595.25, 0.29), (110.55, -1572.94, 0.2), (45.92, -1546.07, 0.15), (-17.22, -1515.85, 0.34), (-75.55, -1478.9, 0.72), (-123.29, -1427.86, 0.42), (-178.05, -1384.38, 0.33), (-237.49, -1347.49, 0.24), (-299.82, -1315.7, 0.2), (-364.24, -1288.36, 0.18), (-430.24, -1265.06, 0.13), (-497.23, -1244.79, 0.11), (-564.96, -1227.14, 0.1), (-633.24, -1211.78, 0.08), (-701.96, -1198.45, 0.07), (-771.0, -1186.92, 0.07), (-840.29, -1177.0, 0.06), (-909.79, -1168.65, 0.06), (-979.44, -1161.68, 0.05), (-1049.2, -1155.95, 0.05), (-1119.05, -1151.37, 0.05), (-1188.96, -1147.93, 0.05), (-1258.93, -1145.72, 0.05), (-1328.92, -1144.63, 0.06), (-1398.92, -1144.91, 0.01), (-1468.88, -1145.47, 0.06), (-1538.84, -1144.64, 0.3), (-1608.45, -1151.12, 0.6), (-1674.88, -1171.93, 1.69), (-1716.31, -1224.59, 2.41), (-1695.65, -1290.05, 1.13), (-1650.04, -1342.89, 0.48), (-1596.09, -1387.38, 0.28), (-1537.96, -1426.33, 0.23), (-1476.84, -1460.42, 0.16), (-1413.94, -1491.11, 0.15), (-1349.51, -1518.45, 0.14), (-1283.8, -1542.55, 0.12), (-1217.13, -1563.82, 0.12), (-1149.63, -1582.32, 0.12), (-1081.43, -1598.06, 0.12), (-1012.66, -1611.04, 0.11)], [(-1006.48, -1634.53, 0), (-1054.61, -1583.7, 0.08), (-1104.11, -1534.22, 0.16), (-1156.29, -1487.59, 0.25), (-1212.29, -1445.65, 0.36), (-1273.12, -1411.15, 0.55), (-1339.34, -1388.98, 0.74), (-1408.88, -1384.53, 0.85), (-1476.57, -1400.85, 0.78), (-1537.32, -1435.07, 0.57), (-1590.08, -1480.89, 0.45), (-1635.0, -1534.5, 0.28), (-1674.48, -1592.24, 0.22), (-1709.44, -1652.86, 0.17), (-1740.71, -1715.46, 0.13), (-1769.03, -1779.46, 0.1), (-1795.0, -1844.44, 0.06)], [(-1587.87, -1546.77, 0), (-1557.23, -1483.83, 0.14), (-1529.8, -1419.43, 0.12), (-1505.12, -1353.93, 0.1), (-1482.74, -1287.61, 0.08), (-1462.21, -1220.69, 0.06), (-1443.15, -1153.34, 0.05), (-1425.18, -1085.68, 0.03), (-1408.0, -1017.83, 0.02), (-1391.31, -949.84, 0.01), (-1374.87, -881.8, 0.0), (-1358.45, -813.76, 0.01), (-1341.77, -745.77, 0.02), (-1324.59, -677.91, 0.03), (-1306.78, -610.22, 0.03), (-1288.2, -542.73, 0.04), (-1268.72, -475.5, 0.05), (-1247.99, -408.64, 0.06), (-1225.86, -342.23, 0.06), (-1202.43, -276.28, 0.06), (-1177.64, -210.82, 0.07), (-1151.21, -146.0, 0.09), (-1122.74, -82.06, 0.07), (-1092.71, -18.84, 0.07), (-1061.14, 43.63, 0.1), (-1027.39, 104.95, 0.09), (-991.64, 165.12, 0.07), (-954.38, 224.36, 0.09), (-915.26, 282.41, 0.11), (-873.85, 338.83, 0.07), (-831.1, 394.24, 0.09), (-786.58, 448.26, 0.11), (-740.0, 500.49, 0.06), (-692.39, 551.79, 0.11), (-642.75, 601.14, 0.08), (-591.83, 649.15, 0.07), (-539.81, 695.99, 0.08)], [(-468.96, 756.87, 0), (-508.51, 699.12, 0.12), (-550.5, 643.13, 0.19), (-596.08, 590.04, 0.28), (-646.59, 541.67, 0.4), (-703.28, 500.76, 0.54), (-766.61, 471.28, 0.62), (-834.8, 456.3, 0.59), (-904.65, 455.66, 0.49), (-973.62, 467.09, 0.37), (-1040.56, 487.35, 0.26), (-1105.39, 513.69, 0.22), (-1168.0, 544.96, 0.14), (-1228.95, 579.36, 0.12), (-1288.45, 616.22, 0.1), (-1346.68, 655.07, 0.08), (-1403.81, 695.52, 0.09)], [(-1305.43, 655.02, 0), (-1242.98, 623.4, 0.12), (-1179.28, 594.42, 0.15), (-1114.2, 568.76, 0.18), (-1047.65, 547.21, 0.21), (-979.67, 530.71, 0.25), (-910.52, 520.27, 0.29), (-840.67, 516.89, 0.34), (-770.92, 521.85, 0.41), (-702.58, 536.66, 0.36), (-636.66, 559.98, 0.42), (-574.86, 592.7, 0.32), (-517.13, 632.2, 0.33), (-464.3, 678.08, 0.22), (-415.19, 727.92, 0.2), (-369.65, 781.07, 0.12), (-326.36, 836.07, 0.07), (-284.39, 892.09, 0.01), (-242.31, 948.04, 0.1), (-198.26, 1002.42, 0.27), (-149.4, 1052.44, 0.85), (-88.67, 1085.53, 0.1), (-26.12, 1116.89, 0.28), (33.0, 1154.32, 0.24), (88.82, 1196.5, 0.23), (141.1, 1243.01, 0.25), (189.05, 1293.97, 0.31), (231.2, 1349.81, 0.43), (264.47, 1411.27, 0.76), (280.14, 1479.05, 1.49), (259.38, 1544.14, 1.81), (201.98, 1582.32, 1.04), (133.83, 1597.25, 0.51), (63.94, 1599.71, 0.28), (-5.89, 1595.38, 0.2), (-75.28, 1586.18, 0.36), (-142.61, 1568.44, 0.59), (-205.11, 1537.08, 0.5), (-261.11, 1495.31, 0.59), (-307.39, 1442.98, 0.4), (-346.02, 1384.63, 0.12), (-386.98, 1327.94, 0.55), (-437.97, 1280.23, 0.6), (-497.93, 1244.35, 0.44), (-562.73, 1218.06, 0.29), (-629.9, 1198.48, 0.21), (-698.33, 1183.79, 0.14), (-767.38, 1172.35, 0.11), (-836.82, 1163.53, 0.01), (-906.25, 1154.91, 0.11), (-975.34, 1143.6, 0.1), (-1044.77, 1134.79, 0.08), (-1114.41, 1127.9, 0.09), (-1184.25, 1123.3, 0.11), (-1254.22, 1121.46, 0.14), (-1324.2, 1123.01, 0.16), (-1394.0, 1128.36, 0.19), (-1463.27, 1138.3, 0.23), (-1531.48, 1153.86, 0.28), (-1597.83, 1175.99, 0.41), (-1660.25, 1207.34, 0.61), (-1714.39, 1251.21, 1.04), (-1748.95, 1311.21, 1.73), (-1741.11, 1379.41, 1.67), (-1694.78, 1430.87, 1.01), (-1632.96, 1463.04, 0.6), (-1565.58, 1481.47, 0.42), (-1496.13, 1489.79, 0.29), (-1426.18, 1491.12, 0.18), (-1357.15, 1496.83, 0.18), (-1287.96, 1507.06, 0.34), (-1218.02, 1508.99, 0.28), (-1148.23, 1504.0, 0.21), (-1078.98, 1493.83, 0.14), (-1010.31, 1480.27, 0.07), (-942.01, 1464.94, 0.02)], [(-862.7, 1442.86, 0), (-931.01, 1458.07, 0.19), (-1000.17, 1468.76, 0.21), (-1069.92, 1474.31, 0.2), (-1139.9, 1474.95, 0.17), (-1209.8, 1471.31, 0.06), (-1279.6, 1466.11, 0.2), (-1349.53, 1465.72, 0.83), (-1416.05, 1485.35, 1.33), (-1465.99, 1533.43, 0.9), (-1498.48, 1595.26, 0.41), (-1521.89, 1661.18, 0.23), (-1540.02, 1728.78, 0.12), (-1555.39, 1797.07, 0.08), (-1568.85, 1865.76, 0.05), (-1580.99, 1934.69, 0.03)], [(-1775.99, 1786.6, 0), (-1728.56, 1735.11, 0.31), (-1676.04, 1689.09, 0.27), (-1619.37, 1648.23, 0.23), (-1559.43, 1612.1, 0.31), (-1495.95, 1582.6, 0.15), (-1431.04, 1556.58, 0.13), (-1364.99, 1533.44, 0.19), (-1297.53, 1514.75, 0.09), (-1229.54, 1498.26, 0.08), (-1161.07, 1483.67, 0.13), (-1092.04, 1472.13, 0.04), (-1022.86, 1461.49, 0.09), (-953.38, 1452.96, 0.04), (-883.79, 1445.47, 0.05), (-814.08, 1439.08, 0.05), (-744.28, 1433.88, 0.02), (-674.44, 1429.16, 0.05), (-604.54, 1425.55, 0.01), (-534.62, 1422.14, 0.03), (-464.67, 1419.46, 0.01), (-394.72, 1416.91, 0.01), (-324.75, 1414.68, 0.0), (-254.79, 1412.48, 0.0), (-184.82, 1410.31, 0.01), (-114.86, 1407.91, 0.0), (-44.91, 1405.41, 0.03), (25.02, 1402.2, 0.01), (94.93, 1398.65, 0.04), (164.78, 1394.15, 0.05), (234.55, 1388.55, 0.03), (304.25, 1382.12, 0.1), (373.7, 1373.3, 0.08), (442.84, 1362.52, 0.1), (511.57, 1349.39, 0.15), (579.55, 1332.67, 0.24), (645.89, 1310.4, 0.28), (709.69, 1281.81, 0.4), (768.8, 1244.66, 0.56), (819.39, 1196.68, 0.73), (856.05, 1137.38, 0.84), (873.81, 1069.82, 0.76), (873.12, 1000.04, 0.11), (869.81, 930.2, 0.3), (873.73, 860.4, 0.33), (885.72, 791.53, 0.36), (906.36, 724.72, 0.46), (937.37, 662.06, 0.41), (977.11, 604.57, 0.44), (1025.12, 553.8, 0.46), (1080.77, 511.48, 0.42), (1142.0, 477.79, 0.44), (1207.75, 454.01, 0.45), (1276.38, 440.91, 0.47), (1346.22, 439.27, 0.56), (1415.01, 451.38, 0.72), (1478.47, 480.38, 0.94), (1528.75, 528.44, 1.1), (1556.86, 591.92, 1.0), (1561.21, 661.45, 0.71), (1548.27, 729.95, 1.05), (1561.38, 798.71, 0.09), (1572.4, 867.83, 0.09), (1581.19, 937.27, 0.09), (1587.71, 1006.96, 0.11), (1591.57, 1076.85, 0.08), (1593.58, 1146.82, 0.03), (1594.8, 1216.81, 0.06), (1597.61, 1286.75, 0.2), (1605.35, 1356.28, 0.32), (1620.88, 1424.49, 0.36), (1644.95, 1490.18, 0.31), (1676.05, 1552.86, 0.22), (1711.91, 1612.95, 0.17), (1751.23, 1670.85, 0.11), (1792.82, 1727.14, 0.09), (1836.15, 1782.12, 0.07), (1880.78, 1836.05, 0.05), (1926.39, 1889.14, 0.04)], [(1816.33, 2050.0, 0), (1782.62, 1988.65, 0.06), (1750.1, 1926.66, 0.05), (1718.7, 1864.1, 0.05), (1688.39, 1801.01, 0.05), (1659.1, 1737.43, 0.05), (1630.89, 1673.37, 0.05), (1603.88, 1608.79, 0.04), (1577.8, 1543.84, 0.04), (1552.61, 1478.53, 0.04), (1528.29, 1412.89, 0.05), (1505.03, 1346.87, 0.04), (1482.73, 1280.52, 0.03), (1461.23, 1213.9, 0.03), (1440.51, 1147.04, 0.04), (1420.73, 1079.9, 0.04), (1401.91, 1012.48, 0.03), (1383.82, 944.85, 0.03), (1366.47, 877.04, 0.04), (1350.06, 808.99, 0.04), (1334.53, 740.74, 0.03), (1319.72, 672.32, 0.03), (1305.6, 603.76, 0.04), (1292.52, 535.0, 0.03), (1280.21, 466.09, 0.03), (1268.61, 397.06, 0.03), (1257.75, 327.91, 0.04), (1247.97, 258.59, 0.03), (1238.91, 189.18, 0.03), (1230.56, 119.69, 0.04), (1223.11, 50.08, 0.04), (1216.66, -19.62, 0.03), (1210.95, -89.38, 0.03), (1206.0, -159.21, 0.04), (1202.11, -229.1, 0.04), (1199.16, -299.03, 0.03), (1197.02, -369.0, 0.03)], [(1123.34, -482.5, 0), (1133.58, -413.26, 0.05), (1142.59, -343.84, 0.06), (1150.1, -274.25, 0.06), (1156.06, -204.51, 0.07), (1160.43, -134.65, 0.07), (1163.03, -64.7, 0.08), (1163.66, 5.29, 0.09), (1162.06, 75.27, 0.11), (1157.77, 145.13, 0.13), (1150.36, 214.73, 0.15), (1139.43, 283.85, 0.17), (1124.35, 352.18, 0.21), (1104.37, 419.23, 0.27), (1078.2, 484.1, 0.36), (1044.06, 545.13, 0.39), (1001.87, 600.84, 0.53), (950.21, 647.79, 0.58), (890.13, 683.33, 0.54), (824.39, 706.93, 0.57), (755.21, 716.89, 0.42), (685.31, 716.54, 0.34), (615.93, 707.92, 0.25), (547.54, 693.17, 3.21), (530.56, 642.3, 0.81), (527.94, 572.71, 0.76), (506.86, 506.23, 0.57), (472.96, 445.17, 0.45), (429.79, 390.16, 0.33), (380.61, 340.39, 0.21), (327.98, 294.27, 0.14), (273.11, 250.83, 0.09), (216.82, 209.22, 0.05), (159.79, 168.64, 0.01), (102.59, 128.28, 0.04), (45.99, 87.1, 0.12), (-8.88, 43.66, 0.22), (-60.29, -3.78, 0.55), (-101.53, -60.0, 1.22), (-115.17, -127.19, 1.82), (-83.78, -188.31, 1.08), (-30.89, -233.68, 0.52), (29.6, -268.79, 0.25), (92.95, -298.53, 0.18), (158.09, -324.14, 0.12), (224.23, -347.04, 0.05), (290.77, -368.73, 0.04), (356.81, -391.22, 0.55), (417.5, -426.0, 0.3), (474.13, -467.05, 0.35), (525.23, -514.74, 0.39), (569.38, -568.9, 0.4), (605.44, -628.79, 0.45), (631.63, -693.6, 0.38), (649.03, -761.33, 0.31), (659.06, -830.56, 0.29), (662.03, -900.47, 0.19), (660.28, -970.43, 0.14), (655.16, -1040.23, 0.1), (647.53, -1109.81, 0.04), (639.03, -1179.3, 0.02), (630.97, -1248.83, 0.09), (625.19, -1318.58, 0.19), (624.14, -1388.53, 0.35), (631.62, -1458.04, 0.64), (654.44, -1524.02, 0.95), (697.62, -1578.68, 1.07), (757.53, -1613.19, 1.17), (827.17, -1620.3, 0.01), (896.78, -1627.66, 0.02), (966.45, -1634.49, 0.05), (1036.21, -1640.19, 0.08), (1106.11, -1643.92, 0.11), (1176.09, -1644.86, 0.16), (1246.01, -1641.99, 0.22), (1315.49, -1633.75, 0.32), (1383.61, -1617.87, 0.46), (1448.24, -1591.33, 0.64), (1505.26, -1551.17, 0.79), (1549.06, -1496.99, 0.8), (1575.98, -1432.66, 0.66), (1587.39, -1363.76, 0.39)], [(1578.64, -1280.07, 0), (1576.95, -1350.04, 0.2), (1580.09, -1419.95, 0.3), (1590.45, -1489.12, 0.43), (1610.97, -1555.95, 0.56), (1644.06, -1617.45, 0.63), (1689.88, -1670.14, 0.6), (1745.74, -1712.11, 0.49), (1808.01, -1743.92, 0.37), (1873.92, -1767.38, 0.28), (1941.82, -1784.31, 0.23)], [(1975.49, -1926.9, 0), (1915.22, -1891.3, 0.0), (1854.95, -1855.7, 0.11), (1796.1, -1817.81, 0.02), (1737.56, -1779.43, 0.12), (1680.68, -1738.67, 0.03), (1624.19, -1697.32, 0.15), (1569.95, -1653.08, 0.04), (1516.41, -1608.02, 0.14), (1465.17, -1560.33, 0.1), (1415.74, -1510.82, 0.09), (1367.86, -1459.76, 0.19), (1323.52, -1405.59, 0.12), (1281.59, -1349.6, 0.13), (1242.3, -1291.7, 0.16), (1206.35, -1231.64, 0.2), (1174.61, -1169.26, 0.15), (1146.3, -1105.29, 0.16), (1121.53, -1039.86, 0.15), (1100.33, -973.18, 0.15), (1082.61, -905.47, 0.14), (1068.28, -836.95, 0.13), (1057.19, -767.84, 0.1), (1048.53, -698.39, 0.08), (1041.89, -628.71, 0.07), (1036.87, -558.89, 0.05), (1033.03, -489.0, 0.03), (1029.95, -419.07, 0.01), (1027.2, -349.12, 0.01), (1024.3, -279.18, 0.03), (1020.74, -209.27, 0.04), (1016.13, -139.43, 0.06), (1010.14, -69.69, 0.07), (1002.42, -0.13, 0.08), (992.68, 69.18, 0.1), (980.49, 138.11, 0.13), (965.14, 206.4, 0.12), (946.95, 273.98, 0.09)], [(906.26, 358.94, 0), (925.99, 291.89, 0.63), (959.98, 230.85, 0.66), (1007.08, 179.24, 0.56), (1063.42, 137.84, 0.44), (1125.55, 105.69, 0.33), (1190.97, 80.91, 0.23), (1258.19, 61.44, 0.19), (1326.53, 46.37, 0.13), (1395.49, 34.36, 0.11), (1464.86, 25.07, 0.07), (1534.46, 17.58, 0.06), (1604.2, 11.57, 0.05), (1674.03, 6.75, 0.04), (1743.92, 2.88, 0.03), (1813.85, -0.33, 0.02), (1883.8, -3.06, 0.01), (1953.76, -5.45, 0.01), (2023.72, -7.64, 0.0), (2093.69, -9.74, 0.0)], [(2046.79, -45.01, 0), (1976.83, -47.4, 0.1), (1906.84, -47.26, 0.13), (1836.93, -43.92, 0.16), (1767.32, -36.73, 0.19), (1698.34, -24.94, 0.27), (1630.85, -6.59, 0.33), (1565.91, 19.35, 0.45), (1505.87, 55.21, 0.54), (1453.71, 101.74, 0.59), (1412.19, 157.95, 0.53), (1381.79, 220.89, 0.43), (1361.12, 287.7, 0.31), (1347.92, 356.42, 0.22), (1340.04, 425.96, 0.16), (1335.99, 495.84, 0.09), (1334.15, 565.81, 0.05), (1333.58, 635.81, 0.05), (1334.25, 705.8, 0.13), (1338.09, 775.69, 0.1), (1344.3, 845.41, 0.04), (1351.47, 915.04, 0.07), (1356.83, 984.83, 0.31), (1354.5, 1054.67, 0.81), (1332.71, 1120.58, 1.3), (1283.67, 1169.46, 1.02), (1219.99, 1197.91, 0.53), (1151.97, 1214.11, 0.27), (1082.67, 1223.84, 0.18), (1012.87, 1229.08, 0.1), (942.93, 1231.79, 0.07), (872.94, 1232.68, 0.05), (802.94, 1232.23, 0.04), (732.96, 1230.82, 0.06), (663.03, 1227.82, 0.06), (593.06, 1226.36, 0.27), (523.28, 1231.41, 0.28), (454.34, 1243.19, 0.36), (387.46, 1263.65, 0.45), (324.63, 1294.37, 0.51), (268.33, 1335.81, 0.53), (220.64, 1386.9, 0.5), (182.52, 1445.48, 0.43), (153.65, 1509.16, 0.36), (132.92, 1575.97, 0.29), (118.97, 1644.53, 0.23), (110.49, 1714.0, 0.18), (106.42, 1783.86, 0.14), (105.86, 1853.85, 0.14)], [(-7.83, 1891.55, 0), (-14.8, 1821.9, 0.17), (-17.66, 1752.0, 0.19), (-15.76, 1682.09, 0.22), (-8.46, 1612.55, 0.25), (4.82, 1543.89, 0.27), (24.52, 1476.79, 0.29), (50.88, 1412.03, 0.3), (83.92, 1350.41, 0.31), (123.4, 1292.7, 0.31), (168.86, 1239.58, 0.3), (219.7, 1191.56, 0.29), (275.22, 1149.0, 0.3), (334.91, 1112.5, 0.28), (397.91, 1082.07, 0.26), (463.37, 1057.41, 0.25), (530.71, 1038.49, 0.24), (599.41, 1025.27, 0.24), (668.98, 1017.76, 0.27), (738.95, 1016.8, 0.25), (808.72, 1021.85, 0.25), (877.77, 1033.05, 0.27), (945.47, 1050.69, 0.35), (1010.51, 1076.42, 0.34), (1072.03, 1109.64, 0.35)], [(1133.88, 1148.65, 0), (1086.49, 1097.13, 0.03), (1039.67, 1045.1, 0.01), (993.07, 992.86, 0.01), (946.59, 940.53, 0.0), (900.19, 888.11, 0.0), (853.86, 835.64, 0.0), (807.58, 783.12, 0.0), (761.36, 730.55, 0.0), (715.17, 677.95, 0.0), (669.03, 625.31, 0.0), (622.93, 572.63, 0.0), (576.86, 519.93, 0.0), (530.84, 467.19, 0.0), (484.85, 414.41, 0.0), (438.91, 361.6, 0.0), (393.01, 308.74, 0.0), (347.17, 255.85, 0.0), (301.38, 202.89, 0.0), (255.67, 149.88, 0.01), (210.06, 96.78, 0.01), (164.57, 43.58, 0.01), (119.29, -9.81, 0.02), (74.42, -63.53, 0.08)]]

    controller.robot.pos[0] = -1545.0

    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    sleep(350, TimeUnits.MSEC)        
    motor.intakeChain.stop(BrakeType.COAST)

    log(controller.robot.pos)

    # drive away from wall
    controller.fwd_speed = 7
    controller.path(paths[0], 
            backwards=False,
            heading_authority=2.5,
            finish_margin=200, slowdown_distance=0)
        
    # turn to face close mogo with rear of robot
    drivetrain.set_turn_threshold(10)
    drivetrain.set_turn_constant(0.8)
    drivetrain.turn_for(TurnType.LEFT, 65, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
    sleep(100, TimeUnits.MSEC)

    # grab mogo path
    controller.fwd_speed = 6
    controller.path(paths[1], backwards=True, heading_authority=1, slowdown_distance=0, speed_ramp_time=300)
    
    pneumatic.mogo.set(True)
    sleep(300, TimeUnits.MSEC)

    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    controller.fwd_speed = 5
    # drive across 1st ring and into 2nd ring and wall stake
    controller.path(paths[2], backwards=False, 
            heading_authority=1.3, look_ahead_dist=350, event_look_ahead_dist=300, 
            finish_margin=100, checkpoints=[15, 20, 30, 50])

    # drop mogo in corner
    controller.path(paths[3], backwards=True, timeout=1300, heading_authority=2)
    pneumatic.mogo.set(False)

    # recalibrate x and y since we're in the corner
    sleep(200, TimeUnits.MSEC)
    h = math.radians(sensor.imu.heading())
    d1 = sensor.wallLeftDistance.object_distance()
    if d1 < 900:
        x = d1 * math.cos(h)
        left_offset = 172.6 * math.cos(math.radians(-10) + h)
        x = -1785 + x + left_offset

        diff = x - controller.robot.pos[0]
        log("Recalibrate x pos from {} to {}. Diff: {}".format(controller.robot.pos[0], x, diff))
        controller.robot.pos[0] = x
    else:
        log("Couldn't calibrate left sensor!")

    d2 = sensor.wallRightDistance.object_distance()
    if d2 < 900:
        y = d2*math.sin(h)
        right_offset = 172.6 * math.sin(math.radians(-10) - h)
        y = -1785 + y - right_offset
        diff = y - controller.robot.pos[1]
        log("Recalibrate y pos from {} to {}. Diff: {}".format(controller.robot.pos[1], y, diff))
        controller.robot.pos[1] = y
    else:
        log("Couldn't calibrate right distance!")
    log("L: {} R: {}".format(d1, d2))   

    motor.intakeChain.spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
    sleep(250, TimeUnits.MSEC)
    motor.intakeChain.stop(BrakeType.COAST)

    # cross field from bottom left goal after drop
    controller.fwd_speed = 7
    controller.path(paths[4], slowdown_distance=1000)
    # grab top left mogo
    controller.fwd_speed = 6
    controller.path(paths[5], backwards=True, speed_ramp_time=400)
    pneumatic.mogo.set(True)
    sleep(300, TimeUnits.MSEC)
    # Intake and grab top left rings
    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.fwd_speed = 6
    controller.path(paths[6], speed_ramp_time=400, checkpoints=[15, 30, 40, 60, 70], slowdown_distance=700, min_slow_voltage=5)
    # drop mogo in corner
    controller.path(paths[7], backwards=True, timeout=1000)
    pneumatic.mogo.set(False)

    sleep(200, TimeUnits.MSEC)

    #* Top left corner recalibration
    # h = sensor.imu.heading()
    # theta_1 = h%90
    # print(theta_1)
    # h = math.radians(h)
    # d1 = sensor.wallRightDistance.object_distance()
    # if d1 < 900:
    #     x = abs(d1 * math.sin(math.radians(theta_1)))
    #     left_offset = 172.6 * abs(math.sin(math.radians(theta_1 + 10)))
    #     print("Right offset: {}, x: {}".format(left_offset, x))
    #     x = -1785 + x + left_offset

    #     diff = x - controller.robot.pos[0]
    #     log("Recalibrate x pos from {} to {}. Diff: {}".format(controller.robot.pos[0], x, diff))
    #     controller.robot.pos[0] = x
    # else:
    #     log("Couldn't calibrate left sensor!")

    # d2 = sensor.wallLeftDistance.object_distance()
    # if d2 < 900:
    #     y = abs(d2*math.cos(math.radians(theta_1)))
    #     right_offset = 172.6 * abs(math.cos(math.radians(theta_1-10)))
    #     print("Left offset: {}, y: {}".format(right_offset, y))
    #     y = 1785 - y - right_offset
    #     diff = y - controller.robot.pos[1]
    #     log("Recalibrate y pos from {} to {}. Diff: {}".format(controller.robot.pos[1], y, diff))
    #     controller.robot.pos[1] = y
    # else:
    #     log("Couldn't calibrate right distance!")

    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    # extake to ensure we aren't caught on the goal
    motor.intakeChain.spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
    sleep(350, TimeUnits.MSEC)
    motor.intakeChain.stop(BrakeType.COAST)

    controller.fwd_speed = 7
    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    main["AutonomousFlags"].intake_auto_halt = True
    main["AutonomousFlags"].intake_flex_auto_halt = True
    # top blue goal corner 
    controller.path(paths[8], speed_ramp_time=1000, min_start_voltage=2.5, timeout=7000, slowdown_distance=1000, look_ahead_dist=400, events=[["pres intake", (920.0, 689.8), pneumatic.intake.set, (True,)]])
    motor.intakeChain.stop(BrakeType.COAST)

    #* Top right distance recalibration
    # h = math.radians(sensor.imu.heading())
    # d1 = sensor.wallRightDistance.object_distance()
    # if d1 < 900:
    #     x = d1 * math.cos(h)
    #     left_offset = 172.6 * math.cos(math.radians(10) + h)
    #     x = 1785 - x - left_offset

    #     diff = x - controller.robot.pos[0]
    #     log("Recalibrate x pos from {} to {}. Diff: {}".format(controller.robot.pos[0], x, diff))
    #     controller.robot.pos[0] = x
    # else:
    #     log("Couldn't calibrate left sensor!")

    # d2 = sensor.wallFrontDistance.object_distance()
    # if d2 < 900:
    #     y = d2*math.cos(h)
    #     offset = 210 * math.sin(math.radians(38) + h)
    #     y = 1785 - y - offset
    #     diff = y - controller.robot.pos[1]
    #     log("Recalibrate y pos from {} to {}. Diff: {}".format(controller.robot.pos[1], y, diff))
    #     controller.robot.pos[1] = y
    # else:
    #     log("Couldn't calibrate right distance!")
    # log("R: {} F: {}".format(d1, d2))   
    # log("Recalibrated to {}".format(controller.robot.pos))

    main["AutonomousFlags"].intake_auto_halt = False
    main["AutonomousFlags"].intake_flex_auto_halt = False
    controller.fwd_speed = 6
    # back out of corner
    controller.path(paths[9], backwards=True, speed_ramp_time=300, heading_authority=1.8)
    pneumatic.intake.set(False)

    pneumatic.mogo.set(True)
    sleep(300, TimeUnits.MSEC)

    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    controller.path(paths[10], speed_ramp_time=400, checkpoints=[30, 50], event_look_ahead_dist=150,
                events=[
                        ["reverse", (1016.0, 508.0), motor.intakeChain.spin, (DirectionType.REVERSE, 100, VelocityUnits.PERCENT)],
                        ["stop", (510.0, 510.0), motor.intakeChain.stop, (BrakeType.COAST,)],
                        ["spin", (264.0, 299.7), motor.intakeChain.spin, (DirectionType.FORWARD, 100, VelocityUnits.PERCENT)],
                        ["reverse", (150.0, -170.0), motor.intakeChain.spin, (DirectionType.REVERSE, 100, VelocityUnits.PERCENT)],
                        ["spin", (630.0, -630.0), motor.intakeChain.spin, (DirectionType.FORWARD, 100, VelocityUnits.PERCENT)]
                    ])
    
    motor.intakeChain.stop()
    controller.path(paths[11], backwards=True, timeout=1200)
    pneumatic.mogo.set(False)

    # sleep(200, TimeUnits.MSEC)
    # #* Bottom right corner calibration
    # h = math.radians((sensor.imu.heading()))
    # d1 = sensor.wallRightDistance.object_distance()
    # if d1 < 900:
    #     x = d1 * abs(math.cos(h))
    #     left_offset = 172.6 * abs(math.cos(math.radians(-10) - h))
    #     # print("left offset: {}".format(left_offset))
    #     x = 1785 - x - left_offset

    #     diff = x - controller.robot.pos[0]
    #     log("Recalibrate x pos from {} to {}. Diff: {}".format(controller.robot.pos[0], x, diff))
    #     controller.robot.pos[0] = x
    # else:
    #     log("Couldn't calibrate right sensor!")

    # d2 = sensor.wallLeftDistance.object_distance()
    # if d2 < 900:
    #     y = abs(d2*math.sin(h))
    #     right_offset = 172.6 * abs(math.sin(math.radians(-10) + h)) #! - h???
    #     y = -1785 + y + right_offset
    #     diff = y - controller.robot.pos[1]
    #     log("Recalibrate y pos from {} to {}. Diff: {}".format(controller.robot.pos[1], y, diff))
    #     controller.robot.pos[1] = y
    # else:
    #     log("Couldn't calibrate left distance!")
    # log(controller.robot.pos)

    motor.intakeChain.spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
    motor.intakeFlex.spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
    sleep(300, TimeUnits.MSEC)
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    brain.timer.event(motor.intakeChain.spin, 350, (DirectionType.FORWARD, 100, VelocityUnits.PERCENT))
    main["AutonomousFlags"].intake_auto_halt = True
    # motor.intakeFlex.stop(BrakeType.BRAKE)

    controller.path(paths[12], speed_ramp_time=800, slowdown_distance=800)
    motor.intakeChain.stop()
    controller.fwd_speed = 7
    controller.path(paths[13], backwards=True, speed_ramp_time=400, timeout=1500)
    drivetrain.drive_for(DirectionType.FORWARD, 3, main["DistanceUnits"].IN, 50, VelocityUnits.PERCENT)
    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    sleep(1000, TimeUnits.MSEC)
    motor.intakeChain.spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
    sleep(200, TimeUnits.MSEC)
    motor.intakeChain.stop()

    ## flip lady brown guide off
    # motor.ladyBrown.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    # sleep(450, TimeUnits.MSEC)
    controller.robot.LB_PID.home()

    # position off the wall
    x = sensor.wallBackDistance.object_distance()
    x += 140
    controller.robot.pos[0] = 1785 - x
    log("Recalibrated x to {}".format(controller.robot.pos[0]))

    main["AutonomousFlags"].intake_anti_jam = False
    main["AutonomousFlags"].intake_auto_halt = False
    ### start intake and drive to grab last ring
    motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

    controller.fwd_speed = 8
    controller.path(paths[14], speed_ramp_time=600, slowdown_distance=800, timeout=3500, heading_authority=1.5)

    # extake slightly to make sure we aren't caught on the ring
    motor.intakeChain.spin(DirectionType.REVERSE, 50, VelocityUnits.PERCENT)
    sleep(100, TimeUnits.MSEC)
    motor.intakeChain.stop(BrakeType.COAST)

    ## spin lady brown to put ring on wall stake
    controller.robot.LB_PID.enabled = False
    motor.ladyBrown.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
    sleep(800)
    motor.ladyBrown.stop(BrakeType.HOLD)
    # motor.ladyBrown.spin_for(DirectionType.REVERSE, 500, RotationUnits.DEG, False)

    controller.fwd_speed = 8
    controller.path(paths[15], backwards=True, speed_ramp_time=500, slowdown_distance=600)
    main["Thread"](main["ControllerFunctions"].elevation_bar)
    sleep(500, TimeUnits.MSEC)

    main["AutonomousFlags"].intake_anti_jam = True
    motor.intakeChain.spin(DirectionType.FORWARD, 20, VelocityUnits.PERCENT)
    controller.fwd_speed = 11
    # controller.robot.pos = [650.0, 758.0]
    controller.path(paths[16], timeout=2200, speed_ramp_time=400)
    pneumatic.doinker.set(True)
    pneumatic.elevatoinBarLift.set(False)

    motor.intakeChain.stop()
    motor.intakeFlex.stop()

    motor.ladyBrown.spin(DirectionType.FORWARD, 20, VelocityUnits.PERCENT)
    sleep(300, TimeUnits.MSEC)
    motor.ladyBrown.stop(BrakeType.COAST)
