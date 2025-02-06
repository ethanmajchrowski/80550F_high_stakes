# imu.set_heading(70)  
#         self.dynamic_vars["position"] = [-1244.6, -1533.906]
        
#         paths = [((-1300.0, -1500.0), (-1227.21, -1466.88), (-1157.59, -1427.52), (-1090.11, -1384.56), (-1021.76, -1343.02), (-949.86, -1308.04), (-874.04, -1282.7), (-795.81, -1266.24), (-716.35, -1257.2), (-636.45, -1253.68), (-556.46, -1254.35), (-480.25, -1257.87)), ((-477.38, -1257.87), (-556.08, -1243.58), (-633.37, -1223.09), (-707.12, -1192.42), (-772.43, -1146.55), (-833.61, -1095.15), (-906.27, -1062.49), (-977.85, -1047.9)), ((-977.85, -1050.17), (-922.18, -992.71), (-866.42, -935.34), (-811.2, -877.47), (-758.26, -817.5), (-708.5, -754.87), (-660.29, -691.02), (-612.56, -626.82), (-564.92, -562.55), (-523.4, -506.56))]

#         motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

#         self.dynamic_vars["intake_color_sort"] = "eject_blue"
#         self.fwd_speed = 11
#         self.path(paths[0])
#         doinker_pneu.set(True)

#         motors["misc"]["wall_stake"].set_stopping(BrakeType.HOLD)
#         motors["misc"]["wall_stake"].set_velocity(100, VelocityUnits.PERCENT)
#         motors["misc"]["wall_stake"].spin_for(DirectionType.FORWARD, 50, RotationUnits.DEG, False)

#         self.fwd_speed = 7.5
#         self.path(paths[1], backwards=True, finish_margin=200)
#         motors["misc"]["intake_flex"].stop()

#         doinker_pneu.set(False)
#         drivetrain.turn_for(TurnType.RIGHT, 70, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
#         self.fwd_speed = 8
#         self.path(paths[2], backwards=True)
#         mogo_pneu.set(True)
#         sleep(150, TimeUnits.MSEC)
#         motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
#         motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

#         sleep(500, TimeUnits.MSEC)
#         motors["misc"]["intake_chain"].stop()
#         motors["misc"]["intake_flex"].stop()
