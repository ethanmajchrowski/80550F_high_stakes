# Filename: comp.py
# Devices & variables last updated:
	# 2024-12-18 17:16:58.231995
####################
#region Devices
calibrate_imu = False
# ██████  ███████ ██    ██ ██  ██████ ███████ ███████ 
# ██   ██ ██      ██    ██ ██ ██      ██      ██      
# ██   ██ █████   ██    ██ ██ ██      █████   ███████ 
# ██   ██ ██       ██  ██  ██ ██      ██           ██ 
# ██████  ███████   ████   ██  ██████ ███████ ███████ 

from vex import *
from json import load, dump

sd_fail = False
# load config data from SD card
try:
    with open("cfg/config.json", 'r') as f:
        data = load(f)
except:
    sd_fail = True
    print("ERROR LOADING SD CARD DATA")

brain = Brain()
con = Controller(PRIMARY)
con_2 = Controller(PARTNER)

controls = {
    "DRIVE_FORWARD_AXIS":      con.axis3,
    "DRIVE_TURN_AXIS":         con.axis1,
    "INTAKE_IN_HOLD":          con.buttonR1,
    "INTAKE_OUT_HOLD":         con.buttonR2,
    "INTAKE_HEIGHT_TOGGLE":    con.buttonL1,
    "SIDE_SCORING_TOGGLE":     con.buttonB,
    "MOGO_GRABBER_TOGGLE":     con.buttonA,
    "CYCLE_EJECTOR_COLOR":     con.buttonLeft,
    "DOINKER":                 con.buttonRight,
    "INTAKE_FLEX_HOLD":        con.buttonL2,
    "SIDE_STAKE_MANUAL_UP":    con_2.buttonL1,
    "SIDE_STAKE_MANUAL_DOWN":  con_2.buttonL2,
    "LADY_BROWN_MACRO_UP_A":  con.buttonL1,
    "LADY_BROWN_MACRO_UP_B":  con.buttonR1,
    "LADY_BROWN_MACRO_DOWN_A": con.buttonR1,  
    "LADY_BROWN_MACRO_DOWN_B": con.buttonL2,  
}

motors = {
    "left": {
        "A": Motor( Ports.PORT3, GearSetting.RATIO_6_1, True), # stacked top
        "B": Motor(Ports.PORT12, GearSetting.RATIO_6_1, True), # rear
        "C": Motor( Ports.PORT4, GearSetting.RATIO_6_1, True), # front
    },
    "right": {
        "A": Motor(Ports.PORT18, GearSetting.RATIO_6_1, False), # stacked top
        "B": Motor(Ports.PORT16, GearSetting.RATIO_6_1, False), # rear
        "C": Motor(Ports.PORT15, GearSetting.RATIO_6_1, False), # front
    },
    "misc": {
        "intake_chain": Motor(Ports.PORT14, GearSetting.RATIO_6_1, True),  
        "intake_flex": Motor(Ports.PORT5, GearSetting.RATIO_6_1, False), # 5.5 W flexwheel hinge
        "wall_stake": Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
    }
}

# PNEUMATICS
mogo_pneu = DigitalOut(brain.three_wire_port.a)

intake_pneu = DigitalOut(brain.three_wire_port.h)
doinker_pneu = DigitalOut(brain.three_wire_port.b)

# SENSORS
leftEnc = Rotation(Ports.PORT2)
rightEnc = Rotation(Ports.PORT17)
wallEnc = Rotation(Ports.PORT6)
wall_setpoint = 2
wall_control_cooldown = 0
wall_positions = [-0.65*360, -2*360, -2.2*360] # wall_setpoint is an INDEX used to grab from THIS LIST

leftDistance = Distance(Ports.PORT13)
rightDistance = Distance(Ports.PORT19)
intakeDistance = Distance(Ports.PORT9)
intakeColor = Optical(Ports.PORT10)

imu = Inertial(Ports.PORT11)

if calibrate_imu:
    imu.calibrate()

if not sd_fail:
    enable_macro_lady_brown = data["config"]["enable_macro_lady_brown"]
else:
    enable_macro_lady_brown = False

if enable_macro_lady_brown:
    motors["misc"]["wall_stake"].spin_for(FORWARD, 1000, MSEC, 100, PERCENT)
    wallEnc.set_position(0)
    # put wall stake back up
    motors["misc"]["wall_stake"].spin_for(REVERSE, 1000, MSEC, 100, PERCENT)

while imu.is_calibrating() and calibrate_imu: 
    wait(5)

mogo_pneu_engaged = False
mogo_pneu_status = False
elevation_status = False

# Intake ejector related booleans
allow_intake_input = True
queued_sort = False
eject_prep = False

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

# Set initial color sort from SD card
if not sd_fail:
    color_setting = data["config"]["initial_color_sorting"]["selected"]
else:
    color_setting = "none"

# Turn on intake color sensor LED for consistency
intakeColor.set_light_power(100, PERCENT)
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
                    distance_to_end = dist(current_pos, self.path[len(self.path)-1])
                    if (distance_to_end < self.look_dist) and self.checkpoints_complete:
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
        # self.circumference = 219.46 * (3/4)
        self.circumference = (2 * 3.14159) * 25.4

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

class LaserPositioning():
    def __init__(self) -> None:
        pass

    def update(self):
        dx, dy = 0, 0

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
        print("Loading module {}".format(selected_filename))
        try:
            print("Importing module...")
            auton_module = __import__("module.{}".format(selected_filename), None, None, ["run"])

            print("Importing sequence...")
            self.sequence = getattr(auton_module, "run")
            
            print("Importing data...")
            gen_data = getattr(auton_module, "gen_data")
            self.autonomous_data = gen_data()
        except:
            print("Auton not recognized!")
            raise ImportError("Can't find auton file")

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
            "intake_halt_tolerance": 60,
            "intake_auto_halt": False,
            "drop_after_auto_halt": False,
            "raise_after_auto_halt": False,
            "jam_listen": False,
            # Color sorting
            "intake_color_sort": "none", # options: "none", "eject_blue", "eject_red"
            "queued_sort": False,
            "eject_prep": False,
            "last_intake_command": None,
            "intake_color_kill": "none" # options: "none", "kill_on_blue", "kill_on_red"
        }
        self.end_time = 0

    def misc_listeners(self):
        #thread

        while True:
            if self.dynamic_vars["mogo_listen"]:
                if leftDistance.object_distance() < self.dynamic_vars["mogo_grab_tolerance"] and rightDistance.object_distance() < self.dynamic_vars["mogo_grab_tolerance"]:
                    mogo_pneu.set(False)

            # Intake Distance auto halt
            if self.dynamic_vars["intake_auto_halt"]:
                if intakeDistance.object_distance() < self.dynamic_vars["mogo_grab_tolerance"]:
                    motors["misc"]["intake_chain"].stop(BRAKE)

                    if self.dynamic_vars["drop_after_auto_halt"]:
                        intake_pneu.set(False)
                    if self.dynamic_vars["raise_after_auto_halt"]:
                        intake_pneu.set(True)
            
            # Intake color sort
            if self.dynamic_vars["intake_color_sort"] != "none":
                if ((self.dynamic_vars["intake_color_sort"] == "eject_blue") 
                    and intakeColor.hue() > 180 and not self.dynamic_vars["eject_prep"]
                    and intakeColor.is_near_object()):

                    self.dynamic_vars["eject_prep"] = True
                
                if ((self.dynamic_vars["intake_color_sort"] == "eject_red") 
                    and intakeColor.hue() < 18 and not self.dynamic_vars["eject_prep"]
                    and intakeColor.is_near_object()):

                    self.dynamic_vars["eject_prep"] = True

                if ((intakeDistance.object_distance() < 70) 
                        and (not self.dynamic_vars["queued_sort"]) 
                        and (self.dynamic_vars["eject_prep"])):
                    self.dynamic_vars["intake_last_command"] = (motors["misc"]["intake_chain"].command(PERCENT), 
                                                        motors["misc"]["intake_chain"].direction())
                    motors["misc"]["intake_chain"].spin(FORWARD, 100, PERCENT)
                    brain.timer.event(self.color_sort, 210)

                    self.dynamic_vars["queued_sort"] = True

            # Intake kill on red
            if ((
                    self.dynamic_vars["intake_color_kill"] == "kill_on_red") 
                    and intakeColor.hue() < 18
                    and intakeColor.is_near_object()
                ):
                motors["misc"]["intake_chain"].stop()
                motors["misc"]["intake_flex"].stop()
            # Intake kill on blue
            if ((
                    self.dynamic_vars["intake_color_kill"] == "kill_on_blue") 
                    and intakeColor.hue() > 180
                    and intakeColor.is_near_object()
                ):
                motors["misc"]["intake_chain"].stop()
                motors["misc"]["intake_flex"].stop()

            sleep(10, MSEC)
            # end of thread loop

    def color_sort(self):
        # Intake is spinning and it's time to engage the stop to eject
        if motors["misc"]["intake_chain"].command(PERCENT) != 0:
            motors["misc"]["intake_chain"].stop(BRAKE)

            # At this point, allow_intake_input will be false
            # so the current code won't run in a loop
            # blue color timings
            # brain.timer.event(intake_sorter, 170)
            # distance timings
            brain.timer.event(self.color_sort, 150)
        else:
            # Re-enable intake
            last_command = self.dynamic_vars["intake_last_command"]
            # spin motor with last DIRECTION and last SPEED
            motors["misc"]["intake_chain"].spin(last_command[1], last_command[0], PERCENT)
            self.dynamic_vars["queued_sort"] = False
            self.dynamic_vars["eject_prep"] = False

    # decent settings: 
    """
    speed: 65%
    self, path, events, checkpoints=[], backwards = False,
    look_ahead_dist=250, finish_margin=100, event_look_ahead_dist=75,
    heading_authority=1, max_turn_volts = 5,
    hPID_KP = 0.1, hPID_KD = 0.01, hPID_KI = 0, hPID_KI_MAX = 0, hPID_MIN_OUT = None,"""
    def position_thread(self):
        while True:
            # scr = brain.screen
            # scr.clear_screen()
            # scr.set_cursor(1,1)
            # scr.set_font(FontType.MONO20)
            # scr.print(str(self.dynamic_vars["position"]))
            # scr.new_line()
            # scr.print(str(brain.timer.system() / 1000))
            # scr.new_line()
            # scr.print(str(self.end_time / 1000))
            # scr.new_line()
            
            # # scr.print("Change Pos: {}".format(str(self.change_pos)))
            # # scr.new_line()
            # scr.render()

            self.heading = imu.heading()
            dx, dy = self.position_controller.update()
            self.dynamic_vars["position"][0] += dx
            self.dynamic_vars["position"][1] += dy

            wait(20, MSEC)

    def run(self):
        global driver_thread
        """
        Call once at start of auton. This is where all the sequential commands are located.
        """
        t1 = Thread(self.position_thread)
        t2 = Thread(self.misc_listeners)

        self.sequence(globals())
        
        self.end_time = brain.timer.system()

        t1.stop()
        t1 = None
        t2.stop()
        t2 = None

    def kill_motors(self, brake_type=BrakeType.BRAKE):
        motors["left"]["A"].stop(brake_type)
        motors["left"]["B"].stop(brake_type)
        motors["left"]["C"].stop(brake_type)

        motors["right"]["A"].stop(brake_type)
        motors["right"]["B"].stop(brake_type)
        motors["right"]["C"].stop(brake_type)

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

                motors["right"]["A"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
                motors["right"]["B"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
                motors["right"]["C"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
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

auton = AutonomousHandler(data["autons"]["selected"])
print(auton.run)
#endregion auton

#region driver
def switch_mogo_engaged():
    global mogo_pneu_engaged
    mogo_pneu_engaged = not mogo_pneu_engaged

def switch_mogo():
    global mogo_pneu_engaged
    if mogo_pneu.value() == 0 and mogo_pneu_engaged:
        mogo_pneu_engaged = False

    mogo_pneu.set(not mogo_pneu.value())

def switch_intake_height():
    intake_pneu.set(not intake_pneu.value())

def switch_doinker():
    doinker_pneu.set(not doinker_pneu.value())

def cycle_ejector_color():
    global color_setting
    # print("old: " + color_setting)
    l = ["none", "eject_blue", "eject_red"]
    index = l.index(color_setting)
    if index + 1 < len(l):
        index += 1
    else:
        index = 0
    color_setting = l[index]

controls["DOINKER"].pressed(switch_doinker)
controls["MOGO_GRABBER_TOGGLE"].pressed(switch_mogo)
# controls["AUTO_MOGO_ENGAGE_TOGGLE"].pressed(switch_mogo_engaged)
controls["INTAKE_HEIGHT_TOGGLE"].pressed(switch_intake_height)
controls["CYCLE_EJECTOR_COLOR"].pressed(cycle_ejector_color)

def intake_sorter():
    global allow_intake_input, queued_sort, eject_prep 

    # check if we are moving from manual --> auto or auto --> manual
    if allow_intake_input:
        motors["misc"]["intake_chain"].stop(BRAKE)
        # motors["misc"]["intake_chain"].spin(REVERSE, 100, PERCENT)
        allow_intake_input = False

        # At this point, allow_intake_input will be false
        # so the current code won't run in a loop
        # blue color timings
        # brain.timer.event(intake_sorter, 170)
        # distance timings
        brain.timer.event(intake_sorter, 150)
    else:
        # Re-enable intake
        allow_intake_input = True
        queued_sort = False
        eject_prep = False

def driver():
    global eject_prep, queued_sort
    while True:
        intakeColor.set_light_power(100, PERCENT)
        brain.screen.clear_screen()

        # Movement controls
        turnVolts = (controls["DRIVE_TURN_AXIS"].position() * 0.12) * 0.9
        forwardVolts = controls["DRIVE_FORWARD_AXIS"].position() * 0.12
        if elevation_status == True and controls["DRIVE_FORWARD_AXIS"].position() > 25:
            forwardVolts = 7.5
        elif elevation_status == True and controls["DRIVE_FORWARD_AXIS"].position() < -25:
            forwardVolts = -7.5

        # Spin motors and combine controller axes
        motors["left"]["A"].spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motors["left"]["B"].spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motors["left"]["C"].spin(FORWARD, forwardVolts + turnVolts, VOLT)
        # leftMotorC.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motors["right"]["A"].spin(FORWARD, forwardVolts - turnVolts, VOLT)
        motors["right"]["B"].spin(FORWARD, forwardVolts - turnVolts, VOLT)
        motors["right"]["C"].spin(FORWARD, forwardVolts - turnVolts, VOLT)

        #NORMAL INTAKE, NO COLOR
        if controls["INTAKE_FLEX_HOLD"].pressing():
            motors["misc"]["intake_flex"].spin(FORWARD, 100, PERCENT)
        elif controls["INTAKE_IN_HOLD"].pressing():
            motors["misc"]["intake_flex"].spin(FORWARD, 100, PERCENT)
            motors["misc"]["intake_chain"].spin(FORWARD, 65, PERCENT)
        elif controls["INTAKE_OUT_HOLD"].pressing():
            motors["misc"]["intake_flex"].spin(REVERSE, 100, PERCENT)
            motors["misc"]["intake_chain"].spin(REVERSE, 65, PERCENT)
        else:
            motors["misc"]["intake_flex"].stop()
            motors["misc"]["intake_chain"].stop()

        # COLOR SENSING
        # if color_setting == "eject_blue" and intakeColor.hue() > 180 and not eject_prep and intakeColor.is_near_object():
        #     eject_prep = True
        # if color_setting == "eject_red" and intakeColor.hue() < 18 and not eject_prep and intakeColor.is_near_object():
        #     eject_prep = True

        # if (intakeDistance.object_distance() < 70) and (not queued_sort) and (eject_prep):
        #     motors["misc"]["intake_chain"].spin(FORWARD, 100, PERCENT)
        #     brain.timer.event(intake_sorter, 210)

        #     queued_sort = True

        # if controls["INTAKE_FLEX_HOLD"].pressing():
        #     motors["misc"]["intake_flex"].spin(FORWARD, 100, PERCENT)
        # elif controls["INTAKE_IN_HOLD"].pressing():
        #     motors["misc"]["intake_flex"].spin(FORWARD, 100, PERCENT)
        #     if allow_intake_input:
        #         motors["misc"]["intake_chain"].spin(FORWARD, 65, PERCENT)
        # elif controls["INTAKE_OUT_HOLD"].pressing():
        #     motors["misc"]["intake_flex"].spin(REVERSE, 100, PERCENT)
        #     motors["misc"]["intake_chain"].spin(REVERSE, 65, PERCENT)
        # else:
        #     motors["misc"]["intake_flex"].stop()
        #     motors["misc"]["intake_chain"].stop()

        # Lady Brown controls
        if wall_control_cooldown == 0:
            if controls["LADY_BROWN_MACRO_DOWN_A"].pressing() and controls["LADY_BROWN_MACRO_DOWN_B"].pressing():
                wall_control_cooldown = 25
                if wall_setpoint > 0:
                    wall_setpoint -= 1
                    
            elif controls["LADY_BROWN_MACRO_UP_A"].pressing() and controls["LADY_BROWN_MACRO_UP_B"].pressing():
                wall_control_cooldown = 25
                if wall_setpoint < len(wall_positions) - 1:
                    wall_setpoint += 1
                    
        elif wall_control_cooldown > 0:
            wall_control_cooldown -= 1

        # Grabber sensors
        if mogo_pneu_engaged == True:
            if leftDistance.object_distance() < 80 and rightDistance.object_distance() < 80:
                mogo_pneu.set(False)

        # Screen debugging
        scr = brain.screen
        scr.clear_screen()
        scr.set_cursor(1,1)

        scr.print("Left distance: {}".format(leftDistance.object_distance()))
        scr.next_row()
        scr.print("Right distance: {}".format(rightDistance.object_distance()))
        scr.next_row()
        scr.print("Timer: {}".format(brain.timer.time()))
        scr.next_row()

        scr = con.screen
        scr.clear_screen()
        scr.set_cursor(1,1)

        scr.print(color_setting)
        scr.new_line()

        if mogo_pneu_engaged: 
            scr.print("MOGO ENGAGED")
            scr.new_line()

        brain.screen.render()

#endregion driver

def lady_brown_PID():
    pid = MultipurposePID(0.1, 0, 0, 0, None)

    while True:
        output = pid.calculate(wall_positions[wall_setpoint], wallEnc.position())

        motors["misc"]["wall_stake"].spin(FORWARD, output, VOLT)

        sleep(10)

if data["config"]["enable_lady_brown"]:
    Thread(lady_brown_PID)

#region comp
#  ██████  ██████  ███    ███ ██████  
# ██      ██    ██ ████  ████ ██   ██ 
# ██      ██    ██ ██ ████ ██ ██████  
# ██      ██    ██ ██  ██  ██ ██      
#  ██████  ██████  ██      ██ ██      

# driver_thread = None

def no_auton():
    pass

# def override_comp(comp):
#     while True:
#         if comp.is_driver_control():
#             driver()
#             driver_thread.stop() #type: ignore
if brain.sdcard.is_inserted():
    if data["config"]["auton_test"]:
        auton.run()
    else:
        comp = Competition(driver, auton.run)
    # auton.run()

    # driver_thread = Thread(override_comp, (comp,))
else:
    comp = Competition(driver, no_auton)
    # driver_thread = Thread(override_comp, (comp,))

    for i in range(5):
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
    brain.screen.clear_screen()
#endregion comp