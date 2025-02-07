# Filename: comp.py
# Devices & variables last updated:
	# 2025-02-07 14:09:26.433413
####################
#region Devices
calibrate_imu = True
# ██████  ███████ ██    ██ ██  ██████ ███████ ███████ 
# ██   ██ ██      ██    ██ ██ ██      ██      ██      
# ██   ██ █████   ██    ██ ██ ██      █████   ███████ 
# ██   ██ ██       ██  ██  ██ ██      ██           ██ 
# ██████  ███████   ████   ██  ██████ ███████ ███████ 

from vex import *
from json import load, dump, dumps
from math import radians
import sys

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
    "DRIVE_FORWARD_AXIS":          con.axis3,
    "DRIVE_TURN_AXIS":             con.axis1,
    "INTAKE_IN_HOLD":              con.buttonR1,
    "INTAKE_OUT_HOLD":             con.buttonR2,
    "INTAKE_HEIGHT_TOGGLE":        con.buttonLeft,
    # "SIDE_SCORING_TOGGLE":         con.buttonB,
    "MOGO_GRABBER_TOGGLE":         con.buttonA,
    # "CYCLE_EJECTOR_COLOR":         con.buttonLeft,
    "DOINKER":                     con.buttonRight,
    "INTAKE_FLEX_HOLD":            con.buttonL2,
    "LB_MANUAL_UP":                con.buttonL1,
    "LB_MANUAL_DOWN":              con.buttonL2, 
    "MANUAL_ELEVATION_PNEUMATICS": con.buttonUp,
    "LB_MACRO_HOME":           con.buttonDown,
}

controls2 = {
    "ELEVATION_PRIMARY_PNEUMATICS": con_2.buttonUp,
    "DOINKER": con_2.buttonRight,
    "DRIVE_MODE": con_2.buttonDown,
    "AXIS_FORWARDS": con_2.axis3,
    "AXIS_TILT": con_2.axis1,
    "PTO_TOGGLE": con_2.buttonB,
    "INTAKE_IN": con_2.buttonR1,
    "INTAKE_OUT": con_2.buttonR2,
    "LB_MANUAL_UP": con_2.buttonL1,
    "LB_MANUAL_DOWN": con_2.buttonL2,
    "LB_RAISE_MACRO": con_2.buttonA
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
        "intake_chain": Motor(Ports.PORT10, GearSetting.RATIO_6_1, False),  
        "intake_flex": Motor(Ports.PORT5, GearSetting.RATIO_6_1, True), # 5.5 W flexwheel hinge
        "wall_stake": Motor(Ports.PORT7, GearSetting.RATIO_18_1, False)
    }
}

# PNEUMATICS
mogo_pneu = DigitalOut(brain.three_wire_port.a)
elevation_hook_release = DigitalOut(brain.three_wire_port.b)
elevation_bar_lift = DigitalOut(brain.three_wire_port.c)
PTO_left_pneu = DigitalOut(brain.three_wire_port.d) #! left/right not done yet
PTO_right_pneu = DigitalOut(brain.three_wire_port.e)
passive_hook_release_pneu = DigitalOut(brain.three_wire_port.f)
doinker_pneu = DigitalOut(brain.three_wire_port.g)
intake_pneu = DigitalOut(brain.three_wire_port.h)

#### SENSORS
# ENCODERS
leftEnc = Rotation(Ports.PORT2)
rightEnc = Rotation(Ports.PORT17)
wallEnc = Rotation(Ports.PORT1)
# DISTANCE SENSORS
intakeDistance = Distance(Ports.PORT9)

leftWallDistance = Distance(Ports.PORT6)
backWallDistance = Distance(Ports.PORT13)

elevationDistance = Distance(Ports.PORT20)

# MISC SENSORS
intakeColor = Optical(Ports.PORT14)
imu = Inertial(Ports.PORT11)

# SENSOR VARIABLES
wall_setpoint = 0
wall_control_cooldown = 0
wall_positions = [30, 130, 400] # wall_setpoint is an INDEX used to grab from THIS LIST
LB_enable_PID = False

if calibrate_imu:
    imu.calibrate()

if not sd_fail:
    enable_macro_lady_brown = data["config"]["enable_macro_lady_brown"]
else:
    enable_macro_lady_brown = False

if enable_macro_lady_brown:
    print("calibrating wall stake")
    motors["misc"]["wall_stake"].spin_for(REVERSE, 1000, MSEC, 20, PERCENT)
    sleep(100, MSEC) # sleep to allow motor to spin to idle
    wallEnc.set_position(0)
    print(wallEnc.position())

while imu.is_calibrating() and calibrate_imu:
    wait(5)

mogo_pneu_engaged = False
mogo_pneu_status = False

# Intake ejector related booleans
allow_intake_input = True
queued_sort = False
eject_prep = False

tank_drive = False
elevating = False

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

def log(data):
    print("=" + ", ".join(data) + "*")

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
        try:
            if dist(current_pos, self.path[len(self.path)-1][:2]) < self.finish_margin:
                self.path_complete = True
                print("path complete within finish margin")      
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
                        distance_to_end = dist(current_pos, self.path[len(self.path)-1][:2])
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
        except ZeroDivisionError:
            raise NameError
    
        return goal

class DeltaPositioning():
    def __init__(self, leftEnc, rightEnc, imu) -> None:
        self.last_time = brain.timer.time()
        self.leftEnc = leftEnc
        self.rightEnc = rightEnc
        self.leftEnc.reset_position()
        self.rightEnc.reset_position()
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
        global LB_enable_PID
        ### Load paths
        # DO NOT turn the brain off when this is running! This may corrupt the files :)
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

        # base stuff
        drivetrain.set_timeout(1.5, TimeUnits.SECONDS)
        drivetrain.set_turn_threshold(5)
        drivetrain.set_turn_constant(0.6)

        ### Pathing
        # Reference the object so we can create new handlers for new paths
        self.path_controller = PurePursuit

        ### Global positioning
        ## Position controller
        self.position_controller = DeltaPositioning(leftEnc, rightEnc, imu)

        ### Variables
        self.fwd_speed = None
        self.dynamic_vars = {
            "fwd_speed": 10,
            # "position": list(self.autonomous_data["start_pos"]),
            "position": [-1244.6, -1533.906],
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
            print("E: Trigger sort")
        else:
            print("E: re-enable intake from sort")
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
            self.heading = imu.heading()
            dx, dy = self.position_controller.update()
            self.dynamic_vars["position"][0] += dx
            self.dynamic_vars["position"][1] += dy

            wait(20, MSEC)

    def run(self):
        global driver_thread, LB_enable_PID, wall_setpoint
        """
        Call once at start of auton. This is where all the sequential commands are located.
        """
        self.start_time = brain.timer.system()
        
        ## Position data
        imu.set_heading(self.autonomous_data["start_heading"])
        self.heading = self.autonomous_data["start_heading"]

        # disable LB PID unless turned on in auton
        LB_enable_PID = False
        motors["misc"]["wall_stake"].stop(COAST)
        print("disabled LB PID! will auto re-enable after auton")

        print("start pos thread")
        t1 = Thread(self.position_thread)
        print("started pos thread")
        print("start listener thread")
        t2 = Thread(self.misc_listeners)
        print("started listener thread")

        print("starting sequence")

        #! !!!!!!!!!!!!!!!!!!!!!!
        self.sequence(globals())        

        #! sequence done!!!!!!!!!!!!!!!!!!!!!!!!!
        print("sequence done at {}/15000 msec".format(brain.timer.system() - self.start_time))
        self.end_time = brain.timer.system()

        sleep(2000, MSEC)
        try:
            log_thread.stop() # type: ignore
        except:
            pass
        scr = brain.screen
        scr.clear_screen()
        scr.set_cursor(1,1)
        scr.print(str(self.end_time - self.start_time))
        scr.render()

        sleep(2, SECONDS)
        
        motors["misc"]["intake_chain"].stop()
        motors["misc"]["intake_flex"].stop()
        # LB_enable_PID = True
        # print("re-enabled LB PID")

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

    def turn_to_heading(self, target, threshold = 5, speed_constant = 1.0):
        pid = MultipurposePID(0.1, 0.01, 0, 0)
        # use absolute heading
        heading = imu.rotation()

        error = heading - target
        while abs(error) > threshold:
            heading = imu.rotation()
            error = heading - target
            output = pid.calculate(0, error) * speed_constant

            motors["left"]["A"].spin(FORWARD, output, VOLT)
            motors["left"]["B"].spin(FORWARD, output, VOLT)
            motors["left"]["C"].spin(FORWARD, output, VOLT)

            motors["right"]["A"].spin(REVERSE, output, VOLT)
            motors["right"]["B"].spin(REVERSE, output, VOLT)
            motors["right"]["C"].spin(REVERSE, output, VOLT)
        self.kill_motors()

    def path(self, path, events=[], checkpoints=[], backwards = False,
             look_ahead_dist=350, finish_margin=100, event_look_ahead_dist=75, timeout=None,
             heading_authority=1.0, max_turn_volts = 8,
             hPID_KP = 0.1, hPID_KD = 0.01, hPID_KI = 0, hPID_KI_MAX = 0, hPID_MIN_OUT = None,
             turn_speed_weight = 0.0) -> None:

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

            # dynamic forwards speed
            dynamic_forwards_speed = abs(heading_output) * turn_speed_weight
            unclamped = dynamic_forwards_speed
            # clamp speed
            if dynamic_forwards_speed < -4:
                dynamic_forwards_speed = -4
            elif dynamic_forwards_speed > 4:
                dynamic_forwards_speed = 4

            # Grab constant speed from dynamic variables
            if self.fwd_speed is None:
                forwards_speed = self.dynamic_vars["fwd_speed"]
                print("dynamic vars fwd speed deprecated, use self.fwd_speed")
            else:
                forwards_speed = self.fwd_speed
            if backwards:
                forwards_speed *= -1

            # Do some stuff that lowers the authority of turning, no idea if this is reasonable
            heading_output *= heading_authority
            if heading_output > max_turn_volts: heading_output = max_turn_volts
            if heading_output < -max_turn_volts: heading_output = -max_turn_volts
            # if we rollover, fix it
            if rollover:
                heading_output *= -1
            if not waiting:
                left_speed = forwards_speed - dynamic_forwards_speed + heading_output
                right_speed = forwards_speed - dynamic_forwards_speed - heading_output
    
                motors["left"]["A"].spin(FORWARD, left_speed, VOLT)
                motors["left"]["B"].spin(FORWARD, left_speed, VOLT)
                motors["left"]["C"].spin(FORWARD, left_speed, VOLT)

                motors["right"]["A"].spin(FORWARD, right_speed, VOLT)
                motors["right"]["B"].spin(FORWARD, right_speed, VOLT)
                motors["right"]["C"].spin(FORWARD, right_speed, VOLT)
            else:
                if brain.timer.system() >= wait_stop:
                    waiting = False

            for event in events:
                if dist(self.dynamic_vars["position"], event[1]) < event_look_ahead_dist:
                    print(event)
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
            if done:
                print("path complete")

            if timeout is not None:
                if brain.timer.system() > time_end:
                    done = True
                    print("path timed out")

            data = [
                str(brain.timer.system() - self.start_time), 
                # str(left_speed), 
                # str(right_speed), 
                # str(dynamic_forwards_speed),
                # str(unclamped),
                # str(heading_output),
                # str(target_point[0]),
                # str(target_point[1]),
                # str(path_handler.last_found_point),
                # # str(abs(dynamic_speed_heading_error)),
                # str(heading_error),
                str(round(self.heading)),
                str(round(self.dynamic_vars["position"][0])),
                str(round(self.dynamic_vars["position"][1])),
            ]
            # Thread(log, (data,))

            if not done:
                sleep(20, MSEC)
            else:
                self.kill_motors(BRAKE)
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
    if not elevating:
        intake_pneu.set(not intake_pneu.value())

def switch_doinker():
    print("doinker to {}".format(not doinker_pneu.value()))
    doinker_pneu.set(not doinker_pneu.value())

def manual_elevation():
    print("manual elevation")
    elevation_bar_lift.set(not elevation_bar_lift.value())
    # doinker_pneu.set(elevation_bar_lift.value())

def toggle_tank():
    global tank_drive
    tank_drive = not tank_drive

def drivebase_command(command, speed):
    motors["left"]["A"].spin(command, speed, PERCENT)
    motors["left"]["B"].spin(command, speed, PERCENT)
    motors["left"]["C"].spin(command, speed, PERCENT)

    motors["right"]["A"].spin(command, speed, PERCENT)
    motors["right"]["B"].spin(command, speed, PERCENT)
    motors["right"]["C"].spin(command, speed, PERCENT)

def stop_drivebase(BrakeType):
    motors["left"]["A"].stop(BrakeType)
    motors["left"]["B"].stop(BrakeType)
    motors["left"]["C"].stop(BrakeType)

    motors["right"]["A"].stop(BrakeType)
    motors["right"]["B"].stop(BrakeType)
    motors["right"]["C"].stop(BrakeType)

def unbind_button():
    pass

def toggle_PTO():
    PTO_left_pneu.set(not PTO_left_pneu.value())
    PTO_right_pneu.set(PTO_left_pneu.value()) # right mimics left
    print("PTO L: {}, PTO R: {}".format(PTO_left_pneu.value(), PTO_left_pneu.value()))

def elevation_raise_LB():
    global LB_enable_PID, wall_setpoint
    wall_setpoint = 2
    LB_enable_PID = True

def elevation_macro():
    global LB_enable_PID
    enable_PTO = False
    print("start elevation")

    LB_enable_PID = False
    mogo_pneu.set(True)
    # motors["misc"]["wall_stake"].stop()
    # motors["misc"]["wall_stake"].spin_for(FORWARD, 400, MSEC, 100, PERCENT)

    roll_pid = MultipurposePID(0.1, 0, 0, 0)

    elevation_hook_release.set(True)
    # wait and close these pistons cause leak :(
    sleep(200, MSEC)
    # intake_pneu.set(False)
    # doinker_pneu.set(True)
    elevation_hook_release.set(False)
    # elevation_bar_lift.set(True)

    # elevation_bar_lift.set(False)
    # sleep(100, MSEC)

    controls2["DOINKER"].pressed(switch_doinker)
    controls2["PTO_TOGGLE"].pressed(toggle_PTO)
    controls2["ELEVATION_PRIMARY_PNEUMATICS"].pressed(manual_elevation)
    controls2["LB_RAISE_MACRO"].pressed(elevation_raise_LB)
    LB_braketype = BrakeType.COAST

    while True:
        # DRIVE MOTORS
        if abs(con_2.axis2.position()) > 1 or abs(con_2.axis3.position()) > 1:
            motors["left"]["A"].spin(FORWARD, (con_2.axis3.position() / 100) * 12, VOLT)
            motors["left"]["B"].spin(FORWARD, (con_2.axis3.position() / 100) * 12, VOLT)
            motors["left"]["C"].spin(FORWARD, (con_2.axis3.position() / 100) * 12, VOLT)

            motors["right"]["A"].spin(FORWARD, (con_2.axis2.position() / 100) * 12, VOLT)
            motors["right"]["B"].spin(FORWARD, (con_2.axis2.position() / 100) * 12, VOLT)
            motors["right"]["C"].spin(FORWARD, (con_2.axis2.position() / 100) * 12, VOLT)
            print("driving")
            LB_braketype = BrakeType.COAST
        else:
            motors["left"]["A"].stop(BrakeType.COAST)
            motors["left"]["B"].stop(BrakeType.COAST)
            motors["left"]["C"].stop(BrakeType.COAST)

            motors["right"]["A"].stop(BrakeType.COAST)
            motors["right"]["B"].stop(BrakeType.COAST)
            motors["right"]["C"].stop(BrakeType.COAST)

        # lady brown controls
        if con.buttonL1.pressing() or controls2["LB_MANUAL_DOWN"].pressing():
            motors["misc"]["wall_stake"].spin(REVERSE, 100, PERCENT)
            LB_enable_PID = False
            LB_braketype = BrakeType.HOLD
        elif con.buttonL2.pressing() or controls2["LB_MANUAL_UP"].pressing():
            motors["misc"]["wall_stake"].spin(FORWARD, 100, PERCENT)
            LB_enable_PID = False
            LB_braketype = BrakeType.HOLD
        else:
            motors["misc"]["wall_stake"].stop(LB_braketype)

        # intake controls
        if con.buttonR1.pressing() or controls2["INTAKE_IN"].pressing():
            motors["misc"]["intake_flex"].spin(FORWARD, 100, PERCENT)
            motors["misc"]["intake_chain"].spin(FORWARD, 100, PERCENT)
        elif con.buttonR2.pressing() or controls2["INTAKE_OUT"].pressing():
            motors["misc"]["intake_flex"].spin(REVERSE, 100, PERCENT)
            motors["misc"]["intake_chain"].spin(REVERSE, 100, PERCENT)
        else:
            motors["misc"]["intake_flex"].stop()
            motors["misc"]["intake_chain"].stop()

        roll = imu.orientation(OrientationType.PITCH)
        pid_output = round(roll_pid.calculate(0, roll), 3)
        # data = {
        #     "roll": round(roll, 2),
        #     "output": round(pid_output, 2),
        #     "height": round(elevationDistance.object_distance(), 2)
        # }
        # payload_manager.send_data("elevation", data)

        scr = con_2.screen
        scr.set_cursor(1,1)
        if elevation_bar_lift.value():
            scr.print("UP__")
        else:
            scr.print("DOWN")

        sleep(35, MSEC)

def home_lady_brown_PID():
    print("run home_lady_brown_PID")
    global wall_setpoint, LB_enable_PID
    LB_enable_PID = True
    wall_setpoint = 1

controls["DOINKER"].pressed(switch_doinker)
controls["MOGO_GRABBER_TOGGLE"].pressed(switch_mogo)
controls["INTAKE_HEIGHT_TOGGLE"].pressed(switch_intake_height)
controls["LB_MACRO_HOME"].pressed(home_lady_brown_PID)

allow_intake_input = True
queued_sort = False
eject_prep = False

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

def lady_brown_PID():
    pid = MultipurposePID(0.2, 0.015, 0.02, 5, None)

    while True:
        if LB_enable_PID:
            output = pid.calculate(wall_positions[wall_setpoint], wallEnc.position())
            # print(wallEnc.position(), output)

            motors["misc"]["wall_stake"].spin(FORWARD, output*2, VOLT)
            # print(motors["misc"]["wall_stake"].command(VOLT))
            
        sleep(20)

if enable_macro_lady_brown:
    print("starting LB thread")
    Thread(lady_brown_PID)

def driver():
    global eject_prep, queued_sort, wall_control_cooldown, wall_setpoint, elevating, LB_enable_PID
    print("starting driver")
    elevation_hold_duration = 5
    while True:
        intakeColor.set_light_power(100, PERCENT)
        brain.screen.clear_screen()

        turnVolts = (controls["DRIVE_TURN_AXIS"].position() * 0.12) * 0.9
        forwardVolts = controls["DRIVE_FORWARD_AXIS"].position() * 0.12

        # Spin motors and combine controller axes
        motors["left"]["A"].spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motors["left"]["B"].spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motors["left"]["C"].spin(FORWARD, forwardVolts + turnVolts, VOLT)
        # leftMotorC.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motors["right"]["A"].spin(FORWARD, forwardVolts - turnVolts, VOLT)
        motors["right"]["B"].spin(FORWARD, forwardVolts - turnVolts, VOLT)
        motors["right"]["C"].spin(FORWARD, forwardVolts - turnVolts, VOLT)

        if color_setting == "eject_blue" and intakeColor.hue() > 100 and not eject_prep and intakeColor.is_near_object():
            eject_prep = True
        if color_setting == "eject_red" and intakeColor.hue() < 18 and not eject_prep and intakeColor.is_near_object():
            eject_prep = True

        if (intakeDistance.object_distance() < 70) and (not queued_sort) and (eject_prep):
            motors["misc"]["intake_chain"].spin(FORWARD, 100, PERCENT)
            brain.timer.event(intake_sorter, 210)

            queued_sort = True

        if (controls["INTAKE_IN_HOLD"].pressing()):
            motors["misc"]["intake_flex"].spin(FORWARD, 100, PERCENT)
            if allow_intake_input:
                motors["misc"]["intake_chain"].spin(FORWARD, 100, PERCENT)
        elif (controls["INTAKE_OUT_HOLD"].pressing()):
            motors["misc"]["intake_flex"].spin(REVERSE, 100, PERCENT)
            motors["misc"]["intake_chain"].spin(REVERSE, 100, PERCENT)
        else:
            motors["misc"]["intake_flex"].stop()
            motors["misc"]["intake_chain"].stop()

        # WALL STAKES MOTORS
        if controls["LB_MANUAL_UP"].pressing():
            motors["misc"]["wall_stake"].spin(FORWARD, 100, PERCENT)
            LB_enable_PID = False
        elif controls["LB_MANUAL_DOWN"].pressing():
            motors["misc"]["wall_stake"].spin(REVERSE, 60, PERCENT)
            LB_enable_PID = False
        elif not LB_enable_PID:
            motors["misc"]["wall_stake"].stop(HOLD)
        
        #     # Lady Brown controls
        # if wall_control_cooldown == 0:
        #     if controls["LB_MACRO_DECREASE"].pressing():
        #         LB_enable_PID = True
        #         wall_control_cooldown = 2
        #         if wall_setpoint > 0:
        #             wall_setpoint -= 1
            
        #     elif controls["LB_MACRO_INCREASE"].pressing():
        #         LB_enable_PID = True
        #         wall_control_cooldown = 2
        #         if wall_setpoint < len(wall_positions) - 1:
        #             wall_setpoint += 1

        # elif wall_control_cooldown > 0:
        #     wall_control_cooldown -= 1

        # 3levation hold button
        if (con.buttonUp.pressing() or con_2.buttonUp.pressing()) and not elevating:
            print(elevation_hold_duration)
            elevation_hold_duration -= 1
            if elevation_hold_duration <= 0:
                elevating = True
                elevation_macro()
        else:
            elevation_hold_duration = 5

        print(wallEnc.position())

        brain.screen.render()

#endregion driver

# logger
class PayloadManager():
    def __init__(self) -> None:
        pass

    def send_data(self, topic, payload: dict):
        data = {
            "topic": topic,
            "payload": payload
        }
        print(dumps(data))
        # print(sys.__dict__)

#region comp
#  ██████  ██████  ███    ███ ██████  
# ██      ██    ██ ████  ████ ██   ██ 
# ██      ██    ██ ██ ████ ██ ██████  
# ██      ██    ██ ██  ██  ██ ██      
#  ██████  ██████  ██      ██ ██      

# driver_thread = None
# print(sys.__dict__)
payload_manager = PayloadManager()

def odom_logging_thread():
    while True:
        pos = auton.dynamic_vars["position"]
        data = {
            "x": round(pos[0] / 25.4, 2),
            "y": round(pos[1] / 25.4, 2),
            "theta": round(radians(imu.heading()), 2)
        }

        payload_manager.send_data("odometry", data)

        sleep(35, MSEC)

# log_thread = Thread(odom_logging_thread)
# data["config"]["auton_test"] = True

def no_auton():
    pass

if brain.sdcard.is_inserted():
    auton = AutonomousHandler(data["autons"]["selected"])
    comp = Competition(driver, auton.run)
    if comp.is_competition_switch() or comp.is_field_control():
        print("competition")
        print("field control: " + str(comp.is_field_control()))
        print("autonomous: " + str(comp.is_autonomous()))
        print("driver: " + str(comp.is_driver_control()))
    elif data["config"]["auton_test"]:
        sleep(1000, MSEC)
        print("run auton")
        auton.run()
        sleep(1, SECONDS)
        motors["left"]["A"].stop(COAST)
        motors["left"]["B"].stop(COAST)
        motors["left"]["C"].stop(COAST)

        motors["right"]["A"].stop(COAST)
        motors["right"]["B"].stop(COAST)
        motors["right"]["C"].stop(COAST)
    else:
        pass
else:
    comp = Competition(driver, no_auton)

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
