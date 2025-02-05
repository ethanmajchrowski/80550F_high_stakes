# paths = [((1284.27, -1460.87), (1204.34, -1464.06), (1124.36, -1465.88), (1044.37, -1466.57), (964.37, -1466.26), (884.38, -1464.96), (804.42, -1462.56), (724.49, -1459.08), (644.63, -1454.35), (564.88, -1448.06), (485.29, -1440.01), (405.97, -1429.67), (327.13, -1416.17), (233.06, -1393.36)), ((233.06, -1393.36), (310.01, -1415.16), (388.07, -1432.66), (466.8, -1446.84), (546.0, -1458.04), (625.58, -1466.17), (705.44, -1470.78), (785.42, -1470.83), (866.36, -1464.08)), ((866.36, -1464.08), (902.43, -1392.78), (926.52, -1316.64), (938.65, -1237.68), (939.66, -1157.75), (930.1, -1078.37), (911.78, -1000.56), (885.68, -924.99), (852.43, -852.27), (812.17, -783.18), (765.1, -718.54), (711.1, -659.59), (649.57, -608.61), (560.96, -563.96)), ((570.61, -563.96), (564.64, -643.68), (566.54, -723.64), (572.95, -803.38), (581.57, -882.91), (590.64, -962.4), (598.85, -1041.97), (605.17, -1121.72), (608.47, -1201.64), (608.05, -1281.63), (605.97, -1325.85)), ((615.61, -1329.06), (536.52, -1317.18), (458.59, -1299.18), (381.75, -1276.94), (305.19, -1253.73), (227.87, -1233.24), (159.12, -1219.76)), ((159.12, -1216.55), (169.26, -1295.63), (202.46, -1367.69), (257.68, -1424.79), (328.7, -1461.06), (405.28, -1483.71), (484.06, -1497.17), (563.68, -1504.51), (643.6, -1507.83), (723.59, -1508.57), (803.58, -1507.74), (883.56, -1506.1), (963.54, -1504.27), (1043.53, -1502.92), (1123.53, -1502.86), (1203.5, -1504.68), (1283.36, -1509.19), (1362.9, -1517.43), (1441.71, -1530.79), (1518.98, -1551.14), (1593.09, -1580.8), (1661.22, -1622.26), (1719.17, -1677.04), (1785.77, -1811.27)), ((1788.99, -1808.06), (1736.7, -1747.51), (1684.65, -1686.77), (1637.58, -1622.14), (1600.63, -1551.3), (1577.78, -1474.77), (1569.19, -1395.33), (1570.39, -1341.92)), ((1576.81, -1338.71), (1521.76, -1396.43), (1452.88, -1436.41), (1375.97, -1457.57), (1296.41, -1464.75), (1216.45, -1463.41), (1136.71, -1457.04), (1057.14, -1448.8), (977.56, -1440.54), (897.86, -1433.76), (817.94, -1430.41), (737.96, -1431.24), (658.28, -1437.97), (579.57, -1452.0), (502.57, -1473.47), (428.45, -1503.4), (319.86, -1566.95))]   

#         imu.set_heading(270)
#         self.dynamic_vars["position"] = [1285, -1461]
#         self.dynamic_vars["intake_color_sort"] = "eject_red"

#         self.fwd_speed = 10
#         # rush goal grab
#         self.path(paths[0])


#         mogo_pneu.set(True)
#         wait(100, TimeUnits.MSEC)

#         LB_enable_PID = True
#         wall_setpoint = 2 # let the PID work while we drive
#         self.fwd_speed = 8 # slow down on reverse? thats what they did at rumble
#         # drag rushed goal back to our side
#         self.path(paths[1], [], [], True)
#         LB_enable_PID = False
#         motors["misc"]["wall_stake"].stop(BrakeType.BRAKE)

#         mogo_pneu.set(False)

#         # turn to face home goal
#         drivetrain.turn_for(TurnType.LEFT, 75, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
#         self.fwd_speed = 9

#         # grab home goal
#         self.path(paths[2], [], [], True)
#         mogo_pneu.set(True)
#         wait(100, TimeUnits.MSEC)

#         motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
#         motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
#         # grab bottom of ring stack
#         self.path(paths[3], [], [], False)
#         sleep(250, TimeUnits.MSEC)

#         # drop mogo to turn and grab the border one
#         mogo_pneu.set(False)
#         drivetrain.turn_for(TurnType.LEFT, 80, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
#         motors["misc"]["intake_flex"].stop()
#         motors["misc"]["intake_chain"].stop()


#         # gab mogo that we (hopefully) have stolen
#         self.path(paths[4], [], [], True)
#         mogo_pneu.set(True)
#         wait(100, TimeUnits.MSEC)
        
#         motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
#         motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
#         # run into corner for bottom ring
#         self.path(paths[5], [], [], False)
#         sleep(200, TimeUnits.MSEC)
#         self.fwd_speed = 7
#         # slowly backup for a second to try and get the corner ring
#         self.path(paths[6], [], [], True, 350, 100, 75, 750) # with timeout

#         drivetrain.turn_for(TurnType.LEFT, 80, RotationUnits.DEG, 100, VelocityUnits.PERCENT)

#         self.path(paths[7], [], [], False)

#         motors["misc"]["wall_stake"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
#         sleep(1500, TimeUnits.MSEC)
#         motors["misc"]["wall_stake"].stop(BrakeType.HOLD)
