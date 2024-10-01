#####################################################
from vex import *

# region variables
brain = Brain()
con = Controller()

controls = {
    "DRIVE_FORWARD_AXIS":  con.axis3,
    "DRIVE_TURN_AXIS":     con.axis4,
    "MOGO_GRABBER_TOGGLE": con.buttonA,
    "INTAKE_IN_HOLD":      con.buttonR1,
    "INTAKE_OUT_HOLD":     con.buttonR2,
    "INTAKE_HEIGHT_TOGGLE":con.buttonL1,
    "SIDE_SCORING_TOGGLE": con.buttonB
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
# wire_expander = Triport(Ports.PORT5)
# DigitalOut(wire_expander.a)
misc_devices = {
    # pneumatics
    "mogo_pneu": DigitalOut(brain.three_wire_port.c),
    "intake_pneu": DigitalOut(brain.three_wire_port.b),
    "side_scoring_a": DigitalOut(brain.three_wire_port.a), 
    "side_scoring_b": DigitalOut(brain.three_wire_port.d), 
}
# pneumatics
mogo_pneu = DigitalOut(brain.three_wire_port.c)
intake_pneu = DigitalOut(brain.three_wire_port.b)
side_scoring_a = DigitalOut(brain.three_wire_port.a)
side_scoring_b = DigitalOut(brain.three_wire_port.d)

leftEnc = motors["left"]["A"]
rightEnc = motors["right"]["A"]

imu = Inertial(Ports.PORT9)

imu.calibrate()
while imu.is_calibrating():
    wait(5)

# end variables
#####################################################

# slow right turn
"""
path = (
    (-118.566,122.15),
    (-118.612,97.15),
    (-118.775,72.15),
    (-119.826,47.197),
    (-119.961,22.198),
    (-119.472,-2.766),
    (-116.029,-27.528),
    (-107.835,-50.782),
    (-94.482,-71.549),
    (-75.985,-88.367),
    (-53.475,-99.231),
    (-29.858,-107.076),
    (-5.541,-112.77),
    (19.272,-115.817),
    (44.172,-117.843),
    (69.146,-118.971),
    (94.141,-119.267),
    (117.936,-119.369),
)
"""
# loop
"""
path = (
    (-120,121.433),
    (-95.003,120.993),
    (-70.015,120.327),
    (-45.05,119.015),
    (-20.197,116.477),
    (4.497,112.761),
    (28.58,106.05),
    (51.012,95.261),
    (70.281,79.821),
    (84.641,59.804),
    (93.84,36.655),
    (96.542,11.802),
    (96.322,-13.032),
    (93.283,-37.846),
    (83.895,-60.932),
    (69.432,-80.885),
    (50.058,-96.18),
    (27.511,-106.696),
    (3.364,-113.172),
    (-21.395,-116.47),
    (-46.283,-118.626),
    (-71.263,-119.62),
    (-96.259,-119.953),
    (-121.258,-120.085),
    (-121.433,-120.086),
)
"""
# squiggle
path = (
    (35.518,148.666),
    (29.59,129.79),
    (16.636,114.679),
    (1.595,101.504),
    (-12.853,87.698),
    (-23.093,70.69),
    (-25.801,51.107),
    (-21.511,31.763),
    (-9.756,15.863),
    (5.787,3.299),
    (20.616,-10.065),
    (30.747,-27.095),
    (34.795,-46.543),
    (35.518,-54.868),
)


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
    
    def dist(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))


    def goal_search(self, current_pos):
        """
        Run every time you want to update your goal position. 
        Returns the point along our path that is look_dist away from the robot
        and that is closest to the end of the path.
        """
        start_point = self.last_found_point
        goal = ()
        # Iterate over every un-crossed point in our path.
        for i in range(start_point, len(self.path)-1):
            # step 1: line and circle intersection
            h, k = current_pos
            point1 = self.path[i]
            point2 = self.path[i+1]
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
                        sol1_distance = self.dist(sol1, point2)
                        sol2_distance = self.dist(sol2, point2)

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
                    if self.dist(current_pos, self.path[len(self.path)-1]) < self.look_dist:
                        goal = self.path[len(self.path)-1]
                        if self.dist(current_pos, self.path[len(self.path)-1]) < self.finish_margin:
                            path_complete = True
                    else:
                        # update last_found_point
                        # only keep the goal if the goal point is closer to the target than our robot
                        if self.dist(goal, self.path[self.last_found_point+1]) < self.dist(current_pos, self.path[self.last_found_point+1]):
                            # found point is closer to the target than we are, so we keep it
                            goal = goal
                        else:
                            self.last_found_point = i        
            elif goal == ():
                # we didn't find any point on the line, so return the last found index
                goal = self.path[self.last_found_point][:2]
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
        dHeadingTheta = self.imu.rotation() - self.last_heading

        # proportion
        # (dTheta / 360) = (x mm / Circumference mm)
        # x mm = (dTheta / 360) * Circumference
        dl = (dl / 360) * self.circumference
        dr = (dr / 360) * self.circumference

        # average the position of left & right to get the center of the robot
        dNet = (dl + dr) / 2 

        # x = cos
        # y = sin
        dx = dNet * math.cos(math.radians(dHeadingTheta))
        dy = dNet * math.sin(math.radians(dHeadingTheta))

        self.last_time = brain.timer.time()
        self.last_heading = self.imu.rotation()
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
    def __init__(self, path, log = True) -> None:
        # loop wait time in msec
        self.clock_time = 10

        self.running = True
        self.position = [0, 0]
        self.position = list(path[0])
        self.path = path
        imu.set_rotation(-180)


        self.heading_pid = MultipurposePID(0.1, 0, 0, 0)
        self.heading = imu.rotation()

        self.position_controller = DeltaPositioning(leftEnc, rightEnc, imu)
        self.path_controller = PurePursuit(100, 10, self.path)

        self.logging = log
        if self.logging:
            self.logger = Logger()
            self.logger.log("Position: {}, Heading: {}, Look Ahead: {}, Finish Margin: {}".format(self.position, self.heading, 100, 10))

    def run(self):
        dpos = self.position_controller.update()
        self.position[0] += dpos[0]
        self.position[1] += dpos[1]
        target_point = self.path_controller.goal_search(self.position)

        dx, dy = target_point[0] - self.position[0], target_point[1] - self.position[1] # type: ignore
        heading_to_target = math.degrees(math.atan2(dy, dx))

        self.heading = imu.rotation()
        heading_output = self.heading_pid.calculate(heading_to_target, self.heading)

        constant_forwards_speed = 5
        turn_max_speed = 5

        heading_output = (heading_output / 50) * turn_max_speed

        motors["left"]["A"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
        motors["left"]["B"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
        motors["left"]["C"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
        motors["left"]["D"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
        # leftMotorC.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motors["right"]["A"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
        motors["right"]["B"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
        motors["right"]["C"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
        motors["right"]["D"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)

        #time, x, y, heading, heading_to_target, point_x, point_y, point, len_points
        if self.logging:
            time = str(brain.timer.time())
            posx, posy = str(self.position[0]), str(self.position[1])
            tx, ty = str(target_point[0]), str(target_point[1]) #type: ignore
            hpid = str(heading_output)
            self.logger.log(time + ", " + posx + ", " + posy + ", " + str(self.heading) + ", " + str(heading_to_target) + ", " + hpid + ", " + tx + ", " + ty)

        scr = brain.screen
        scr.clear_screen()
        scr.set_font(FontType.MONO30)
        scr.set_cursor(1, 1)

        scr.print("X: " + str(self.position[0]))
        scr.new_line()
        scr.print("Y: " + str(self.position[1]))
        scr.new_line()

        scr.render()

        # queue next run, or stop
        if self.running:
            brain.timer.event(self.run, self.clock_time)
        else:
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

auton = AutonomousController(path)
# auton.run()
auton.test()