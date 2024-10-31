# Filename: comp.py
# Devices & variables last updated:
	# 2024-10-30 18:19:48.436423
####################
#region Devices
calibrate_imu = True
# ██████  ███████ ██    ██ ██  ██████ ███████ ███████ 
# ██   ██ ██      ██    ██ ██ ██      ██      ██      
# ██   ██ █████   ██    ██ ██ ██      █████   ███████ 
# ██   ██ ██       ██  ██  ██ ██      ██           ██ 
# ██████  ███████   ████   ██  ██████ ███████ ███████ 

from vex import *
from json import load, dump

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
intakeDistance = Distance(Ports.PORT6)

imu = Inertial(Ports.PORT9)

if calibrate_imu:
    imu.calibrate()
    while imu.is_calibrating(): 
        wait(5)

mogo_pneu_engaged = False
mogo_pneu_status = False

lmg = MotorGroup(*motors["left"].values())
rmg = MotorGroup(*motors["right"].values())
"""
wheelTravel - The circumference of the driven wheels. The default is 300 mm.
trackWidth - The track width of the drivetrain. The default is 320 mm. 
    (distance between left and right parts of the drivetrain)
wheelBase - The wheel base of the Drivetrain. The default is 320 mm.
    (distance from center of front and rear wheels)
units - A valid DistanceUnit for the units that wheelTravel, trackWidth and wheelBase are specified in. The default is MM.
externalGearRatio - The gear ratio used to compensate drive distances if gearing is used.
"""
drivetrain = SmartDrive(lmg, rmg, imu)
"""
Over Under Settings:
    drivetrain.set_timeout(2, SECONDS)
    drivetrain.set_turn_constant(0.28)
    drivetrain.set_turn_threshold(0.25)
"""

class Logger:
    def __init__(self, interval: int, data: list[tuple[Callable, str]]) -> None:
        """
        Initializes the logger. The "data" list contains functions that return the data,
        and a string to label the data. You do not need to pass time into data.
        Interval is the mS interval between logging operations.
        """
        self.data = data
        self.interval = interval

        self.functions = [brain.timer.system]
        self.labels = ["time"]
        
        for d in self.data:
            self.functions.append(d[0])
            self.labels.append(d[1])
    
    def setup(self):
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
        f.write(", ".join(self.labels))
        f.close()

    def log(self):
        file = open("data/latest.txt", "a")
        # file.write("\n" + data)
        data = []
        for i in self.functions:
            data.append(i())
        file.write("\n" + ", ".join(data))
        file.close()

        brain.timer.event(self.log, self.interval)

    def start(self):
        Thread(self.log)
#endregion Devices####################
#DO NOT CHANGE THE FOLLOWING LINE:#
#end_1301825#
####################

#region Helpers
# ██   ██ ███████ ██      ██████  ███████ ██████  ███████ 
# ██   ██ ██      ██      ██   ██ ██      ██   ██ ██      
# ███████ █████   ██      ██████  █████   ██████  ███████ 
# ██   ██ ██      ██      ██      ██      ██   ██      ██ 
# ██   ██ ███████ ███████ ██      ███████ ██   ██ ███████ 

def dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))
#endregion Helpers

#region auton
#  █████  ██    ██ ████████  ██████  ███    ██ 
# ██   ██ ██    ██    ██    ██    ██ ████   ██ 
# ███████ ██    ██    ██    ██    ██ ██ ██  ██ 
# ██   ██ ██    ██    ██    ██    ██ ██  ██ ██ 
# ██   ██  ██████     ██     ██████  ██   ████ 

class PurePursuit():
    def __init__(self, look_ahead_dist, finish_margin, 
                 path: list[tuple[float, float]], checkpoints) -> None:
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

        self.checkpoints = checkpoints
        if len(self.checkpoints) != 0:
            self.current_checkpoint = checkpoints[0]
            self.checkpoints_complete = False
        else:
            self.checkpoints_complete = True

    def goal_search(self, current_pos):
        """
        Run every time you want to update your goal position. 
        Returns the point along our path that is look_dist away from the robot
        and that is closest to the end of the path.
        """
        goal = self.path[self.last_found_point+1][:2]

        # Iterate over every un-crossed point in our path.
        #if we are close to the finish point, regardless of what has happened, finish the path
        if dist(current_pos, self.path[len(self.path)-1]) < self.finish_margin:
            self.path_complete = True      
        else:
            # Iterate from our current point to the next checkpoint, or the end of the path.
            # Once we reach that checkpoint, move the checkpoint to the next and keep going.
            if self.checkpoints_complete:
                end = len(self.path)-1
            else:
                end = self.current_checkpoint
            
            start = self.last_found_point
            
            for i in range(start, end):
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

                if not self.checkpoints_complete:
                    if (self.last_found_point + 2 >= self.current_checkpoint):
                        # If we are done with our checkpoints,
                        if (self.checkpoints.index(self.current_checkpoint)+1) >= len(self.checkpoints):
                            self.checkpoints_complete = True
                        else:
                            # Set the current checkpoint to the next checkpoint in the checkpoints list
                            self.current_checkpoint = self.checkpoints[self.checkpoints.index(self.current_checkpoint) + 1]
       
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

class AutonomousHandler:
    def __init__(self, selected_filename) -> None:
        ### Load paths
        # DO NOT turn the brain off when this is running! This may corrupt the files :)
        # self.autonomous = self.read_auto("autons/{}.txt".format(selected_filename))
        # Import auton module
        try:
            auton_module = __import__("module.{}".format(selected_filename), None, None, ["run"])

            self.sequence = getattr(auton_module, "run")
            
            gen_data = getattr(auton_module, "gen_data")
            self.autonomous_data = gen_data()
        except:
            print("Auton not recognized!")

        ### Pathing
        # Reference the object so we can create new handlers for new paths
        self.path_controller = PurePursuit

        ### Global positioning
        ## Position controller
        self.position_controller = DeltaPositioning(leftEnc, rightEnc, imu)
        ## Position data
        imu.set_heading(self.autonomous_data["start_heading"])
        self.heading = imu.heading()

        ### Logging
        # deal with this later :)

        ### Variables
        self.dynamic_vars = {
            "fwd_speed": 9.6,
            # common speeds:
            # 25%: 3
            # 50%: 6
            # 60%: 7.2
            # 75%: 9
            # 80%: 9.6
            # 90%: 10.8
            "position": list(self.autonomous_data["start_pos"]),
            #### Thread flags
            # Mogo
            "mogo_listen": False,
            "mogo_grab_tolerance": 60,
            # Intake
            "intake_auto_halt": False,
            "drop_after_auto_halt": False,
        }
        self.end_time = 0

    def misc_listeners(self):
        #thread
        while True:
            if self.dynamic_vars["mogo_listen"]:
                if leftDistance.object_distance() < self.dynamic_vars["mogo_grab_tolerance"] and rightDistance.object_distance() < self.dynamic_vars["mogo_grab_tolerance"]:
                    mogo_pneu.set(True)

            if self.dynamic_vars["intake_auto_halt"]:
                if intakeDistance.object_distance() < 50:
                    motors["intake"].stop(BRAKE)

                    if self.dynamic_vars["drop_after_auto_halt"]:
                        intake_pneu.set(False)
            
            sleep(10, MSEC)

    # decent settings: 
    """
    speed: 65%
    self, path, events, checkpoints=[], backwards = False,
    look_ahead_dist=250, finish_margin=100, event_look_ahead_dist=75,
    heading_authority=1, max_turn_volts = 5,
    hPID_KP = 0.1, hPID_KD = 0.01, hPID_KI = 0, hPID_KI_MAX = 0, hPID_MIN_OUT = None,"""
    def position_thread(self):
        while True:
            scr = brain.screen
            scr.clear_screen()
            scr.set_cursor(1,1)
            scr.set_font(FontType.MONO40)
            scr.print(str(self.dynamic_vars["position"]))
            scr.new_line()
            scr.print(str(brain.timer.system() / 1000))
            scr.new_line()
            scr.print(str(self.end_time / 1000))
            scr.new_line()
            scr.render()

            self.heading = imu.heading()
            dx, dy = self.position_controller.update()
            self.dynamic_vars["position"][0] += dx
            self.dynamic_vars["position"][1] += dy

            wait(20, MSEC)

    def run(self):
        """
        Call once at start of auton. This is where all the sequential commands are located.
        """
        Thread(self.position_thread)
        Thread(self.misc_listeners)

        self.sequence(globals())
        
        self.end_time = brain.timer.system()

    def kill_motors(self, brake_type=BrakeType.BRAKE):
        motors["left"]["A"].stop(brake_type)
        motors["left"]["B"].stop(brake_type)
        motors["left"]["C"].stop(brake_type)
        motors["left"]["D"].stop(brake_type)

        motors["right"]["A"].stop(brake_type)
        motors["right"]["B"].stop(brake_type)
        motors["right"]["C"].stop(brake_type)
        motors["right"]["D"].stop(brake_type)

    def path(self, path, events, checkpoints=[], backwards = False,
             look_ahead_dist=350, finish_margin=100, event_look_ahead_dist=75, timeout=None,
             heading_authority=1, max_turn_volts = 8,
             hPID_KP = 0.1, hPID_KD = 0.01, hPID_KI = 0, hPID_KI_MAX = 0, hPID_MIN_OUT = None,) -> None:

        if timeout is not None:
            time_end = brain.timer.system() + timeout

        path_handler = self.path_controller(look_ahead_dist, finish_margin, path, checkpoints)
        heading_pid = MultipurposePID(hPID_KP, hPID_KD, hPID_KI, hPID_KI_MAX, hPID_MIN_OUT)

        done = path_handler.path_complete
        waiting = False
        wait_stop = 0

        while not done:
            target_point = path_handler.goal_search(self.dynamic_vars["position"])
            dx, dy = target_point[0] - self.dynamic_vars["position"][0], target_point[1] - self.dynamic_vars["position"][1] # type: ignore
            heading_to_target = math.degrees(math.atan2(dx, dy))

            if dx < 0:
                heading_to_target = 360 + heading_to_target

            # logic to handle the edge case of the target heading rolling over
            heading_error = self.heading - heading_to_target
            rollover = False

            if backwards:
                if heading_error > 180:
                    heading_error -= 180
                else:
                    heading_error += 180

            if heading_error > 180:
                heading_error = 360 - heading_error
                rollover = True
            if heading_error < -180:
                heading_error = -360 - heading_error
                rollover = True

            heading_output = heading_pid.calculate(0, heading_error)

            # Grab constant speed from dynamic variables
            constant_forwards_speed = self.dynamic_vars["fwd_speed"]
            if backwards:
                constant_forwards_speed *= -1
                # heading_output *= -1

            # Do some stuff that lowers the authority of turning, no idea if this is reasonable
            heading_output *= heading_authority
            if heading_output > max_turn_volts: heading_output = max_turn_volts
            if heading_output < -max_turn_volts: heading_output = -max_turn_volts
            # if we rollover, fix it
            if rollover:
                heading_output *= -1
            if not waiting:
                motors["left"]["A"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
                motors["left"]["B"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
                motors["left"]["C"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
                motors["left"]["D"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)

                motors["right"]["A"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
                motors["right"]["B"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
                motors["right"]["C"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
                motors["right"]["D"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
            else:
                if brain.timer.system() >= wait_stop:
                    waiting = False

            for event in events:
                if dist(self.dynamic_vars["position"], event[1]) < event_look_ahead_dist:
                    if type(event[2]) == str:
                        if event[2] == "wait_function":
                            if not event[4]:
                                # This tells us to wait
                                # format: ["description", (x, y), EventWaitType(), duration, completed]
                                waiting = True
                                wait_stop = brain.timer.system() + event[3]

                                # make sure we dont get stuck in a waiting loop
                                event[4] = True

                                self.kill_motors()
                        else:
                            # this is a variable change
                            # format: ["speed down", (0, 1130), "speed", 3.5]
                            self.dynamic_vars[event[2]] = event[3]
                    elif callable(event[2]):
                        # Call the function (at index 2) with the unpacked (*) args (at index 3)
                        # ["intake", (1200, 0), motors["intake"].spin, (FORWARD, 50)],
                        event[2](*event[3])

            done = path_handler.path_complete

            if timeout is not None:
                if brain.timer.system() > time_end:
                    done = True

            if not done:
                # brain.timer.event(self.run, self.clock_time)
                sleep(20, MSEC)
            else:
                self.kill_motors()

# Select proper autonomous
with open("cfg/config.json", 'r') as f:
    data = load(f)

auton = AutonomousHandler(data["selected_auton"])
print(auton.run)
#endregion auton

#region driver
# ██████  ██████  ██ ██    ██ ███████ ██████  
# ██   ██ ██   ██ ██ ██    ██ ██      ██   ██ 
# ██   ██ ██████  ██ ██    ██ █████   ██████  
# ██   ██ ██   ██ ██  ██  ██  ██      ██   ██ 
# ██████  ██   ██ ██   ████   ███████ ██   ██ 

def driver():
    pass
#endregion driver

#region comp
#  ██████  ██████  ███    ███ ██████  
# ██      ██    ██ ████  ████ ██   ██ 
# ██      ██    ██ ██ ████ ██ ██████  
# ██      ██    ██ ██  ██  ██ ██      
#  ██████  ██████  ██      ██ ██      

if brain.sdcard.is_inserted():
    if brain.battery.capacity() < 15:
        brain.screen.set_fill_color(Color(255, 255, 0))
        brain.screen.draw_rectangle(0, 0, 480, 240)
    else:
        auton.run()
    # comp = Competition(driver, auton.run)
else:
    while True:
        brain.screen.set_cursor(0, 0)
        # warn the user that there is no SD card inserted
        brain.screen.set_fill_color(Color(255, 0, 0))
        brain.screen.draw_rectangle(0, 0, 480, 240)
        for i in range(4):
            brain.screen.set_font(FontType.PROP60)
            brain.screen.print("     NO SD CARD")
            brain.screen.new_line()

        wait(400)

        brain.screen.clear_screen()

        wait(400)
#endregion comp