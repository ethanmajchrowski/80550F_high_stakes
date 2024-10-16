# Filename: auton.py
# Devices & variables last updated:
	# 2024-10-14 18:32:52.641639
####################
#region Devices
from vex import *

brain = Brain()
con = Controller()

controls = {
    "DRIVE_FORWARD_AXIS":  con.axis3,
    "DRIVE_TURN_AXIS":     con.axis4,
    "INTAKE_IN_HOLD":      con.buttonR1,
    "INTAKE_OUT_HOLD":     con.buttonR2,
    "INTAKE_HEIGHT_TOGGLE":con.buttonL1,
    "SIDE_SCORING_TOGGLE": con.buttonB,
    "MOGO_GRABBER_TOGGLE": con.buttonA,
    "AUTO_MOGO_ENGAGE_TOGGLE": con.buttonY,
    "ELEVATION_RELEASE_1": con.buttonDown,
    "ELEVATION_RELEASE_2": con.buttonLeft,
}
motors = {
    "left": {
        "A": Motor(Ports.PORT12, GearSetting.RATIO_6_1), # stacked top
        "B": Motor(Ports.PORT11, GearSetting.RATIO_6_1, True), # stacked bottom
        "C": Motor(Ports.PORT15, GearSetting.RATIO_6_1, True), # front
        "D": Motor(Ports.PORT13, GearSetting.RATIO_18_1) # 5.5w
    },
    "right": {
        # D motor is 5.5w
        "A": Motor(Ports.PORT16, GearSetting.RATIO_6_1, True), # stacked top
        "B": Motor(Ports.PORT18, GearSetting.RATIO_6_1), # stacked bottom
        "C": Motor(Ports.PORT9, GearSetting.RATIO_6_1), # front
        "D": Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
    },
    "intake": Motor(Ports.PORT19, GearSetting.RATIO_6_1, True)
}

# PNEUMATICS
mogo_pneu = DigitalOut(brain.three_wire_port.c)
mogo_pneu.set(1)
intake_pneu = DigitalOut(brain.three_wire_port.b)
side_scoring_a = DigitalOut(brain.three_wire_port.a)
side_scoring_b = DigitalOut(brain.three_wire_port.d)
elevation_pneu = DigitalOut(brain.three_wire_port.e)

# wire_expander = Triport(Ports.PORT5)
# DigitalOut(wire_expander.a)

# SENSORS
leftEnc = motors["left"]["A"]
rightEnc = motors["right"]["A"]
leftDistance = Distance(Ports.PORT14)
rightDistance = Distance(Ports.PORT17)

imu = Inertial(Ports.PORT9)

imu.calibrate()
while imu.is_calibrating(): 
    wait(5)

mogo_pneu_engaged = False
mogo_pneu_status = False
#endregion Devices####################
#DO NOT CHANGE THE FOLLOWING LINE:#
#end_1301825#

# Horseshoe
# Start at (1200, -1200) facing 270*
# path = ((1193.69, -1193.69, 270), (993.69, -1194.01), (793.69, -1194.33), (593.7, -1193.09), (393.72, -1190.23), (193.83, -1184.7), (-5.85, -1173.48), (-204.8, -1154.67), (-402.9, -1127.2), (-595.66, -1074.5), (-777.68, -994.58), (-936.09, -876.15), (-1055.72, -718.63), (-1134.5, -535.92), (-1179.09, -340.95), (-1196.48, -141.99), (-1202.14, 57.34), (-1177.15, 255.77), (-1122.2, 446.42), (-1035.91, 625.2), (-911.32, 781.65), (-750.56, 900.2), (-573.66, 991.29), (-386.65, 1060.62), (-192.84, 1109.99), (4.62, 1140.87), (203.08, 1164.79), (402.61, 1178.53), (602.39, 1186.92), (802.33, 1191.92), (1002.32, 1193.25), (1186.52, 1193.96))

# Squiggle
# Start at (0, -1500) facing 45*
# path = ((0, -1500, 45), (118.3, -1377.91), (230.86, -1250.6), (333.28, -1115.14), (417.26, -967.72), (461.85, -804.82), (450.58, -636.41), (386.98, -479.63), (293.2, -338.12), (185.49, -206.73), (70.63, -81.43), (-46.54, 41.73), (-160.22, 168.01), (-264.05, 302.44), (-351.13, 448.06), (-410.98, 606.51), (-429.96, 774.55), (-396.12, 940.44), (-327.09, 1095.23), (-234.85, 1237.75), (-128.66, 1370.39), (4.01, 1517.04))

# # Square
# # Start at (1200, 0) facing 0*
# path = ((1200, 0, 0), (1196.73, 179.97), (1192.3, 359.91), (1185.61, 539.78), (1169.45, 719.05), (1130.75, 894.45), (1039.26, 1046.97), (890.4, 1144.09), (715.35, 1184.94), (536.48, 1204.7), (356.71, 1213.77), (176.8, 1218.94), (-3.16, 1222.92), (-183.14, 1225.12), (-363.13, 1225.76), (-543.1, 1222.74), (-722.5, 1208.37), (-898.78, 1173.13), (-1051.87, 1082.88), (-1147.67, 933.4), (-1188.23, 758.57), (-1201.33, 579.04), (-1204.03, 399.09), (-1203.36, 219.1), (-1200.81, 39.12), (-1197.44, -140.85), (-1193.71, -320.81), (-1186.71, -500.67), (-1172.69, -680.06), (-1142.86, -857.26), (-1075.97, -1023.1), (-937.5, -1134.21), (-766.54, -1187.01), (-587.87, -1207.33), (-408.1, -1215.2), (-228.15, -1218.86), (-48.15, -1220.45), (131.81, -1223.6), (311.76, -1227.76), (491.62, -1234.97), (671.55, -1239.86), (851.3, -1233.99), (1022.06, -1185.38), (1133.71, -1049.25), (1177.71, -875.45), (1192.67, -696.07), (1198.16, -516.2), (1200.69, -336.23), (1200.41, -156.23), (1200, 0))

# Figure 8
# Start at (1200, -1200) facing 315* (45* to the left of 0)
# Checkpoint at (-1200, 0) facing 180
path = ((1200, -1200, 315), (1094.98, -1092.9), (989.96, -985.8), (884.93, -878.7), (780.62, -770.92), (677.71, -661.79), (574.8, -552.66), (471.89, -443.53), (370.24, -333.22), (268.62, -222.89), (167.01, -112.55), (65.56, -2.06), (-35.88, 108.44), (-137.68, 218.61), (-240.56, 327.76), (-343.44, 436.92), (-449.98, 542.51), (-556.82, 647.79), (-671.75, 744.18), (-793.89, 830.26), (-930.82, 884.26), (-1064.69, 841.42), (-1133.5, 711.0), (-1164.68, 564.96), (-1185.06, 416.55), (-1192.55, 266.74), (-1199.44, 116.91), (-1199.15, -33.08), (-1195.32, -183.03), (-1191.48, -332.98), (-1175.38, -482.04), (-1153.78, -630.16), (-1107.69, -772.91), (-1003.28, -872.22), (-859.18, -855.5), (-729.66, -781.82), (-610.63, -690.65), (-501.02, -588.25), (-392.95, -484.28), (-288.59, -376.53), (-184.49, -268.54), (-82.45, -158.6), (19.6, -48.66), (121.4, 61.51), (222.99, 171.87), (324.58, 282.23), (426.67, 392.13), (529.05, 501.75), (631.43, 611.38), (734.44, 720.41), (838.51, 828.44), (942.57, 936.47), (1046.64, 1044.5), (1200, 1200))

checkpoints = [
    # position, heading, index
    ((-1199.44, 116.91), 180, 26),
    ((1200, 1200), 45, 53)
]

events = [
    # ["description", (5, 5), function, ("arg1", "arg2")],
    # WHEN MAKING DEVICES, IF YOU HAVE 1 ARGUMENT YOU NEED A COMMA AT THE END:
    #                                                         VV
    # ["intake stop", (0, 1130), motors["intake"].stop, (BRAKE, )]

    # ["intake", (-1200, 0), motors["intake"].spin, (FORWARD, 100, PERCENT)],
    # ["intake stop", (0, 1130), motors["intake"].stop, (BRAKE,)]
]

def dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

class Logger:
    def __init__(self) -> None:
        if brain.sdcard.is_inserted():
            # get number of existing files from info.txt
            info_file = open("data/info.txt", "r")
            num_files = int(info_file.read())
            info_file.close()

            # open info and increment number of files
            f = open("data/info.txt", "w")
            f.write(str(num_files + 1))
            f.close()

            # get latest.txt and the new file for archiving
            recent_file = open("data/latest.txt", "r")
            new_file = open("data/" + str(num_files) + ".txt", "w")

            # open new_file to write to it
            new_file.write(recent_file.read())
            recent_file.close()
            new_file.close()

            # clear latest.txt
            f = open("data/latest.txt", "w")
            # write the heading line to the file - this needs to be different for all the data we collect
            # f.write("time, L1 TEMP, L2 TEMP, L3 TEMP, R1 TEMP, R2 TEMP, R3 TEMP")
            # f.write("time, L1 VOLT, L2 VOLT, L3 VOLT, R1 VOLT, R2 VOLT, R3 VOLT")
            f.write("time, x, y, heading, heading_to_target, heading_pid, target_x, target_y")
            f.close()
        else:
            print("No SD card inserted!")

    def log(self, data):
        if brain.sdcard.is_inserted():
            file = open("data/latest.txt", "a")
            file.write("\n" + data)
            file.close()
        else:
            print("No SD card inserted")

class PurePursuit():
    def __init__(self, look_ahead_dist, finish_margin, path: list[tuple[float, float]]) -> None:
        """
        Init variables. Robot position is able to be passed in from another source, like a GPS sensor or odometry algorithm.
        Arguments:
            path (list[tuple[float, float]])

        """
        self.path = path
        self.look_dist = look_ahead_dist
        self.finish_margin = finish_margin
        self.last_found_point = 0

        self.path_complete = False

    def goal_search(self, current_pos):
        """
        Run every time you want to update your goal position. 
        Returns the point along our path that is look_dist away from the robot
        and that is closest to the end of the path.
        """
        start_point = self.last_found_point
        goal = path[self.last_found_point+1][:2]

        # Iterate over every un-crossed point in our path.

        # Iterate from our current point to the next checkpoint. 
        # Once we reach that checkpoint, move the checkpoint to the next and keep going.
        for i in range(start_point, len(self.path)-1):
            # step 1: line and circle intersection
            h, k = current_pos
            point1 = self.path[i][:2]
            point2 = self.path[i+1][:2]
            ax, ay = point1[:2]
            bx, by = point2[:2]
            
            m = (by - ay) / (bx - ax) # slope of line between point a and b
            b = m*(-ax) + ay # y-intercept of line
            r = self.look_dist
            # quadratic terms
            A = (m*m) + 1
            B = (2*m*(b-k)-(2*h))
            C = ((h*h) + ((b-k)*(b-k)) - (r*r))

            discriminant = (B*B) - (4*A*C)
            if discriminant >= 0:
                sol1_x = (-B + math.sqrt(discriminant)) / (2*A)
                sol1_y = m*sol1_x + b

                sol2_x = (-B - math.sqrt(discriminant)) / (2*A)
                sol2_y = m*sol2_x + b

                sol1 = (sol1_x, sol1_y)
                sol2 = (sol2_x, sol2_y)

                minX, minY = min(ax, bx), min(ay, by)
                maxX, maxY = max(ax, bx), max(ay, by)
                # general check to see if either point is on the line 
                if ((minX < sol1_x < maxX) and (minY < sol1_y < maxY)) or ((minX < sol2_x < maxX) and (minY < sol2_y < maxY)):
                    if ((minX < sol1_x < maxX) and (minY < sol1_y < maxY)) and ((minX < sol2_x < maxX) and (minY < sol2_y < maxY)):
                        # both solutions are within bounds, so we need to compare and decide which is better
                        # choose based on distance to pt2
                        sol1_distance = dist(sol1, point2)
                        sol2_distance = dist(sol2, point2)

                        if sol1_distance < sol2_distance:
                            goal = sol1
                        else:
                            goal = sol2
                    else:
                        if (minX < sol1_x < maxX) and (minY < sol1_y < maxY):
                            # solution 1 is within bounds
                            goal = sol1
                        else:
                            goal = sol2
                    
                # first, check if the robot is not close to the end point in the path
                if dist(current_pos, self.path[len(self.path)-1]) < self.look_dist:
                    goal = self.path[len(self.path)-1]
                else:
                    # update last_found_point
                    # only keep the goal if the goal point is closer to the target than our robot
                    if dist(goal[:2], self.path[self.last_found_point+1][:2]) < dist(current_pos, self.path[self.last_found_point+1][:2]):
                        # found point is closer to the target than we are, so we keep it
                        goal = goal
                    else:
                        self.last_found_point = i 

            # only if we are 80% through the path 
            # if self.last_found_point >= int(len(self.path) * 0.8):
            # if we are close to the finish point, regardless of what has happened, finish the path
            if dist(current_pos, self.path[len(self.path)-1]) < self.finish_margin:
                self.path_complete = True
       
        return goal

class DeltaPositioning():
    def __init__(self, leftEnc, rightEnc, imu) -> None:
        self.last_time = brain.timer.time()
        self.leftEnc = leftEnc
        self.rightEnc = rightEnc
        self.imu = imu

        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.last_heading = 0

        # formula:
        # wheel diamter * pi
        # convert to mm
        # multiply by motor to wheel ratio
        self.circumference = 219.46 * (3/4)

    def update(self):
        # Call constantly to update position.

        # change in left & right encoders (degrees)
        dl = self.leftEnc.position() - self.last_left_encoder
        dr = self.rightEnc.position() - self.last_right_encoder
        # dHeadingTheta = self.imu.rotation() - self.last_heading

        # proportion
        # (dTheta / 360) = (x mm / Circumference mm)
        # x mm = (dTheta / 360) * Circumference
        dl = (dl / 360) * self.circumference
        dr = (dr / 360) * self.circumference

        # average the position of left & right to get the center of the robot
        dNet = (dl + dr) / 2 

        # x = cos
        # y = sin
        dx = dNet * math.sin(math.radians(self.imu.heading()))
        dy = dNet * math.cos(math.radians(self.imu.heading()))

        self.last_time = brain.timer.time()
        self.last_heading = self.imu.heading()
        self.last_right_encoder = self.rightEnc.position()
        self.last_left_encoder = self.leftEnc.position()
        return [dx, dy]

class MultipurposePID:
    def __init__(self, KP, KD, KI, KI_MAX, MIN = None) -> None:
        '''
        Create a multipurpose PID that has individually tuned control variables.

        Args:
            KP (int): kP value for tuning controller
            KD (int): kD value for tuning controller
            KI (int): kI value for tuning controller
            KI_MAX (int): integral will not be allowed to pass this value
            MIN (int): a minimum value that the PID will output

        Returns:
            None
        '''
        
        self.kP, self.kD, self.kI = KP, KD, KI
        self.kI_max = KI_MAX
        self.error = 0
        self.integral = 0
        self.derivative = 0
        self.last_error = 0
        self.minimum = MIN
    def calculate(self, TARGET, INPUT, DELAY = 0) -> int:
        '''
        Calculates the output based on the PID's tuning and the provided target & input

        Args:
            TARGET (int): a number that the PID will try to reach
            INPUT (int): the current sensor or other input
            DELAY (int): the delay for the loop

        Returns:
            PID outpit (int)
        '''
        self.error = TARGET - INPUT
        if self.kI != 0:
            if abs(self.error) < self.kI_max:
                self.integral += self.error
            else:
                self.integral = 0
        else:
            self.integral = 0

        self.derivative = self.error - self.last_error
        
        output = (self.error * self.kP) + (self.integral * self.kI) + (self.derivative * self.kD)
        if self.minimum is not None:
            if abs(output) < self.minimum:
                if self.error < 0: output = -self.minimum
                else: output = self.minimum
        
        self.last_error = self.error
        wait(DELAY, MSEC)
        return output

class AutonomousController():
    def __init__(self, path, events, constant_fwd_volt = 3.5,
                 look_ahead_dist = 200, finish_margin = 100,
                 event_look_dist = 75,
                 hPID_KP = 0.1, hPID_KD = 0, hPID_KI = 0, hPID_KI_MAX = 0, hPID_MIN_OUT = None,
                 clock_time = 10,
                 log = True) -> None:
        # loop wait time in msec
        self.clock_time = clock_time

        self.running = True
        self.position = [0, 0]
        self.position = list(path[0][:2])
        self.constant_fwd_folt = constant_fwd_volt
        self.path = path

        self.events = events
        self.event_look_dist = event_look_dist

        if len(self.path[0]) == 3:
            # we know data[0] has a heading
            imu.set_heading(self.path[0][2])

        self.heading_pid = MultipurposePID(hPID_KP, hPID_KD, hPID_KI, hPID_KI_MAX, hPID_MIN_OUT)
        self.heading = imu.heading()

        self.position_controller = DeltaPositioning(leftEnc, rightEnc, imu)
        self.path_controller = PurePursuit(look_ahead_dist, finish_margin, self.path)

        self.logging = log
        if self.logging:
            self.logger = Logger()
            self.logger.log("Starting Position: {}, Starting Heading: {}".format(self.position, self.heading, 100, 10))
            sleep(5)
            self.logger.log("Look ahead: {}, Finish margin: {}, Event trigger distance: {}".format(look_ahead_dist, finish_margin, event_look_dist))
            sleep(5)
            self.logger.log("Constant fwd volts: {}".format(constant_fwd_volt))
            sleep(5)
            self.logger.log("HPID | KP: {}, KD: {}, KI: {}, KI_MAX: {}, MIN_OUT: {}".format(hPID_KP, hPID_KD, hPID_KI, hPID_KI_MAX, hPID_MIN_OUT))
            sleep(5)
            self.logger.log("Path: {}".format(self.path))
            sleep(5)
            self.logger.log("Event positions: {}".format(self.events))
            sleep(5)
            self.logger.log("Clock time: {}".format(clock_time))
            sleep(5)
            # wait for file operations :)
            sleep(100)

    def run(self):
        dx, dy = self.position_controller.update()
        self.position[0] += dx
        self.position[1] += dy
        target_point = self.path_controller.goal_search(self.position)

        dx, dy = target_point[0] - self.position[0], target_point[1] - self.position[1] # type: ignore
        heading_to_target = math.degrees(math.atan2(dx, dy))
        if dx < 0:
            heading_to_target = 360 + heading_to_target

        self.heading = imu.heading()

        # logic to handle the edge case of the target heading rolling over
        heading_error = self.heading - heading_to_target
        rollover = False

        if heading_error > 180:
            heading_error = 360 - heading_error
            rollover = True
        if heading_error < -180:
            heading_error = -360 - heading_error
            rollover = True

        # heading_output = self.heading_pid.calculate(heading_to_target, self.heading)
        heading_output = self.heading_pid.calculate(0, heading_error)

        constant_forwards_speed = self.constant_fwd_folt
        turn_max_speed = 5

        heading_output = (heading_output / 2) * turn_max_speed
        if rollover:
            heading_output *= -1

        motors["left"]["A"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
        motors["left"]["B"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
        motors["left"]["C"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
        motors["left"]["D"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
        # leftMotorC.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motors["right"]["A"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
        motors["right"]["B"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
        motors["right"]["C"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
        motors["right"]["D"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)

        for event in self.events:
            if dist(self.position, event[1]) < self.event_look_dist:
                # commands[0][2](*commands[0][3])
                # Call the function (at index 2) with the unpacked (*) args (at index 3)
                
                # ["intake", (1200, 0), motors["intake"].spin, (FORWARD, 50)],

                event[2](*event[3])

        #time, x, y, heading, heading_to_target, point_x, point_y, point, len_points
        if self.logging:
            time = str(brain.timer.time())
            posx, posy = str(self.position[0]), str(self.position[1])
            tx, ty = str(target_point[0]), str(target_point[1]) #type: ignore
            hpid = str(heading_output)
            self.logger.log(time + ", " + posx + ", " + posy + ", " + str(self.heading) + ", " + str(heading_to_target) + ", " + hpid + ", " + tx + ", " + ty)

        if self.path_controller.path_complete:
            self.running = False

        scr = brain.screen
        scr.clear_screen()
        scr.set_font(FontType.MONO30)
        scr.set_cursor(1, 1)

        scr.print("X: " + str(self.position[0]))
        scr.new_line()
        scr.print("Y: " + str(self.position[1]))
        scr.new_line()
        scr.print("Target X: " + str(target_point[0]))
        scr.new_line()
        scr.print("Target Y: " + str(target_point[1]))
        scr.new_line()
        scr.print("Heading: " + str(self.heading))
        scr.new_line()
        scr.print("Target Heading: " + str(heading_to_target))
        scr.new_line()
        scr.print("hPID output: " + str(heading_output))
        scr.new_line()
        if heading_output < -0.1:
            scr.print("Turn Left")
            scr.new_line()
        elif heading_output > 0.1:
            scr.print("Turn Right")
            scr.new_line()
        else:
            scr.print("Drive Straight")
            scr.new_line

        scr.render()

        # queue next run, or stop
        if self.running:
            brain.timer.event(self.run, self.clock_time)
        else:
            # kill the motors
            motors["left"]["A"].stop(BRAKE)
            motors["left"]["B"].stop(BRAKE)
            motors["left"]["C"].stop(BRAKE)
            motors["left"]["D"].stop(BRAKE)

            motors["right"]["A"].stop(BRAKE)
            motors["right"]["B"].stop(BRAKE)
            motors["right"]["C"].stop(BRAKE)
            motors["right"]["D"].stop(BRAKE)

            if self.logging:
                self.logger.log("END")

    def test(self):
        dpos = self.position_controller.update()
        self.position[0] += dpos[0]
        self.position[1] += dpos[1]

        scr = brain.screen
        scr.clear_screen()
        scr.set_cursor(1, 1)

        scr.print("({}mm, {}mm)".format(int(self.position[0]), int(self.position[1])))
        scr.next_row()

        scr.render()

        if self.running:
            brain.timer.event(self.test, self.clock_time)
        else:
            if self.logging:
                self.logger.log("END")

auton = AutonomousController(path, events, log=True)
auton.logger.log("Start\n")
auton.run()
# auton.test()