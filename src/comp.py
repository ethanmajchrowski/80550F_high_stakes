# Filename: comp.py
# Devices & variables last updated:
	# 2025-01-29 15:57:19.364118
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
    "INTAKE_HEIGHT_TOGGLE":        con.buttonL1,
    # "SIDE_SCORING_TOGGLE":         con.buttonB,
    "MOGO_GRABBER_TOGGLE":         con.buttonA,
    "CYCLE_EJECTOR_COLOR":         con.buttonLeft,
    "DOINKER":                     con.buttonRight,
    "INTAKE_FLEX_HOLD":            con.buttonL2,
    "SIDE_STAKE_MANUAL_UP":        con_2.buttonL1,
    "SIDE_STAKE_MANUAL_DOWN":      con_2.buttonL2, 
    "MANUAL_ELEVATION_PNEUMATICS": con.buttonUp,
    "LB_MACRO_INCREASE":           con.buttonB,
    "LB_MACRO_DECREASE":           con.buttonDown,
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
intakeColor = Optical(Ports.PORT10)
imu = Inertial(Ports.PORT11)

# SENSOR VARIABLES
wall_setpoint = 0
wall_control_cooldown = 0
wall_positions = [15, 125, 400, 600] # wall_setpoint is an INDEX used to grab from THIS LIST
LB_enable_PID = True

if calibrate_imu:
    imu.calibrate()

if not sd_fail:
    enable_macro_lady_brown = data["config"]["enable_macro_lady_brown"]
else:
    enable_macro_lady_brown = False

if enable_macro_lady_brown:
    motors["misc"]["wall_stake"].spin_for(REVERSE, 1000, MSEC, 100, PERCENT)
    wallEnc.set_position(0)

while imu.is_calibrating() and calibrate_imu: 
    wait(5)

mogo_pneu_engaged = False
mogo_pneu_status = False

# Intake ejector related booleans
allow_intake_input = True
queued_sort = False
eject_prep = False

tank_drive = False

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

# Elevation macro vars
enable_elevation_macro = True
if enable_elevation_macro:
    controls["START_ELEVATE"] = con.buttonUp
manual_elevation_controls = True
elevating = False
elevation_hold_duration = 20

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

        ### Pathing
        # Reference the object so we can create new handlers for new paths
        self.path_controller = PurePursuit

        ### Global positioning
        ## Position controller
        self.position_controller = DeltaPositioning(leftEnc, rightEnc, imu)
        ## Position data
        imu.set_heading(self.autonomous_data["start_heading"])
        self.heading = self.autonomous_data["start_heading"]

        ### Variables
        self.dynamic_vars = {
            "fwd_speed": 10,
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
        global driver_thread, LB_enable_PID
        """
        Call once at start of auton. This is where all the sequential commands are located.
        """
        self.start_time = brain.timer.system()

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
        if not data["config"]["auton_test"]:
            self.sequence(globals())
        else:
            # self.sequence(globals())
            
            print("auton testing")
            paths = {
                "corner": {
                     "points": ((1508.16, 1494.38), (1571.54, 1542.91), (1625.95, 1601.5), (1675.33, 1664.42), (1722.48, 1729.05), (1767.23, 1791.67)),
                     "events": [],
                     "checkpoints": [],
                     "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                },
                "mogo": {
                    "points": ((1699.44, 1660.51), (1626.21, 1628.29), (1553.94, 1594.03), (1482.92, 1557.25), (1413.57, 1517.41), (1346.41, 1474.0), (1282.05, 1426.54), (1221.15, 1374.73), (1164.29, 1318.5), (1112.69, 1257.4), (1065.62, 1192.74), (1022.65, 1125.29), (982.56, 1056.06), (943.6, 986.19), (904.22, 916.55), (863.28, 847.83), (820.01, 780.54), (773.7, 715.33), (724.39, 652.35), (672.28, 591.66), (617.61, 533.27), (560.68, 477.07), (493.13, 415.64)),     
                    "events": [],
                    "checkpoints": [],
                    "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                }, 
                "top_border_ring": {
                    "points": ((510.12, 419.89), (551.03, 488.63), (590.8, 558.04), (628.93, 628.35), (664.74, 699.88), (697.1, 773.02), (724.32, 848.19), (743.79, 925.71), (747.05, 1005.4), (723.7, 1081.2), (669.35, 1139.09), (599.92, 1178.38), (525.24, 1206.92), (448.8, 1230.49), (371.84, 1252.32), (294.96, 1274.47), (218.58, 1298.23), (105.03, 1338.91)), 
                    "events": [],
                    "checkpoints": [],
                    "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                }, 
                "backup_for_bottom_border_ring": {
                    "points": ((105.03, 1344.28), (184.97, 1347.05), (264.92, 1345.12), (344.21, 1334.89), (420.67, 1311.88), (489.41, 1271.31), (553.43, 1223.38), (625.76, 1190.02), (704.44, 1176.51), (824.4, 1177.86)),
                    "events": [],
                    "checkpoints": [],
                    "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                }, 
                "bottom_border_ring": {
                    "points": ((824.4, 1177.86), (746.18, 1161.1), (669.14, 1139.62), (593.37, 1113.95), (517.9, 1087.42), (440.77, 1066.39), (361.58, 1055.54), (281.63, 1054.51), (201.88, 1060.53), (94.3, 1075.86)),
                    "events": [],
                    "checkpoints": [],
                    "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                }, 
                "ladder_reorient": {
                    "points": ((824.4, 1177.86), (746.18, 1161.1), (669.14, 1139.62), (593.37, 1113.95), (517.9, 1087.42), (440.77, 1066.39), (361.58, 1055.54), (281.63, 1054.51), (201.88, 1060.53), (94.3, 1075.86)),
                    "events": [],
                    "checkpoints": [],
                    "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                },
                "ladder_touch": {
                    "points": ((110.4, 1081.23), (186.02, 1055.23), (258.75, 1022.06), (326.81, 980.21), (388.58, 929.5), (441.84, 869.95), (486.42, 803.65), (521.96, 732.04), (549.27, 656.9), (570.27, 579.73), (585.38, 501.18), (596.61, 421.99), (604.5, 342.38), (609.94, 262.57), (613.72, 182.66), (616.33, 102.71), (618.13, 22.73), (620.4, -99.81)),
                    "events": [],
                    "checkpoints": [],
                    "custom_args": () #!!!!!!!!!!!!!!!!!!!!!!!!!!
                }, 
            }

            # 6 ring elims
            imu.set_heading(45)
            self.heading = 45
            self.dynamic_vars["position"] = [1500, 1500]
            print("{}, {}".format(self.dynamic_vars["position"], imu.heading()))
            motors["misc"]["wall_stake"].stop(BrakeType.COAST)
            self.dynamic_vars["intake_color_sort"] = "eject_red"

            motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
            self.dynamic_vars["fwd_speed"] = 10
            print("starting path: corner")
            self.path(paths["corner"]["points"], [], [], False, 350, 100, 75, 2000)

            motors["misc"]["intake_flex"].stop()
            self.dynamic_vars["fwd_speed"] = 8
            print("starting path: mogo")
            self.path(paths["mogo"]["points"], [], [], True)

            mogo_pneu.set(True)
            sleep(200, TimeUnits.MSEC)

            motors["misc"]["intake_flex"].spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
            motors["misc"]["intake_chain"].spin(DirectionType.FORWARD, 65, VelocityUnits.PERCENT)
            self.dynamic_vars["fwd_speed"] = 5
            print("starting path: top_border_ring")
            self.path(paths["top_border_ring"]["points"], [], [], False)

            self.dynamic_vars["fwd_speed"] = 7
            print("starting path: backup_for_bottom_border_ring")
            self.path(paths["backup_for_bottom_border_ring"]["points"], [], [], True)

            self.dynamic_vars["fwd_speed"] = 5
            print("starting path: bottom_border_ring")
            self.path(paths["bottom_border_ring"]["points"], [], [], False)

            self.dynamic_vars["fwd_speed"] = 7
            print("starting path: ladder_reorient")
            self.path(paths["ladder_reorient"]["points"], [], [], True)
            mogo_pneu.set(False)

            motors["misc"]["intake_flex"].stop()
            motors["misc"]["intake_chain"].stop()

            self.dynamic_vars["fwd_speed"] = 7
            print("starting path: ladder_touch")
            self.path(paths["ladder_touch"]["points"], [], [], False, 350, 100, 75, 5000)

        #! sequence done!!!!!!!!!!!!!!!!!!!!!!!!!
        print("sequence done at {}/15000 msec".format(brain.timer.system() - self.start_time))
        
        self.end_time = brain.timer.system()

        sleep(2, SECONDS)
        
        motors["misc"]["intake_chain"].stop()
        motors["misc"]["intake_flex"].stop()
        LB_enable_PID = True
        print("re-enabled LB PID")

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

    def path(self, path, events, checkpoints=[], backwards = False,
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
            forwards_speed = self.dynamic_vars["fwd_speed"]
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
                # left_speed /= 1.2
                # right_speed /= 1.2
                # print("\nL: {} - {} + {} = {}".format(forwards_speed, dynamic_forwards_speed, heading_output, left_speed))
                # print("R: {} - {} - {} = {}".format(forwards_speed, dynamic_forwards_speed, heading_output, right_speed))
                    
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
            Thread(log, (data,))

            if not done:
                # brain.timer.event(self.run, self.clock_time)
                sleep(20, MSEC)
            else:
                self.kill_motors(BRAKE)
            
            # scr = brain.screen
            # scr.clear_screen()
            # scr.set_cursor(1,1)
            # scr.print(path_handler.last_found_point)
            # scr.new_line()
            # scr.print(dx, dy)
            # scr.new_line()
            # scr.print(heading_error)
            # scr.new_line()
            # scr.print(left_speed)
            # scr.new_line()
            # scr.print(right_speed)
            # scr.new_line()
            # scr.print(dynamic_forwards_speed)
            # scr.new_line()
            # # scr.print(path.index(target_point))
            # # scr.new_line()

            # scr.render()

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

def manual_elevation():
    elevation_bar_lift.set(not elevation_bar_lift.value())

def toggle_tank():
    global tank_drive
    tank_drive = not tank_drive

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

    # print("new: " + color_setting)

controls["DOINKER"].pressed(switch_doinker)
controls["MOGO_GRABBER_TOGGLE"].pressed(switch_mogo)
# controls["AUTO_MOGO_ENGAGE_TOGGLE"].pressed(switch_mogo_engaged)
controls["INTAKE_HEIGHT_TOGGLE"].pressed(switch_intake_height)
controls["CYCLE_EJECTOR_COLOR"].pressed(cycle_ejector_color)
controls["MANUAL_ELEVATION_PNEUMATICS"].pressed(manual_elevation)
con.buttonLeft.pressed(toggle_tank)

allow_intake_input = True
queued_sort = False
# This will let us look lower in the intake and allow the ejector to work
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
        brain.timer.event(intake_sorter, 175)
    else:
        # Re-enable intake
        allow_intake_input = True
        queued_sort = False
        eject_prep = False

def lady_brown_PID():
    pid = MultipurposePID(0.15, 0.015, 0.01, 25, None)

    while True:
        if LB_enable_PID:
            output = pid.calculate(wall_positions[wall_setpoint], wallEnc.position())

            motors["misc"]["wall_stake"].spin(FORWARD, output/2, VOLT)
            
        sleep(20)

if enable_macro_lady_brown:
    Thread(lady_brown_PID)

def driver():
    global eject_prep, queued_sort, wall_control_cooldown, wall_setpoint, LB_enable_PID
    if enable_macro_lady_brown: LB_enable_PID = True
    while True:
        intakeColor.set_light_power(100, PERCENT)
        brain.screen.clear_screen()

        if not tank_drive:
            # Movement controls
            turnVolts = (controls["DRIVE_TURN_AXIS"].position() * 0.12) * 0.9
            forwardVolts = controls["DRIVE_FORWARD_AXIS"].position() * 0.12
            # if elevation_status == True and controls["DRIVE_FORWARD_AXIS"].position() > 25:
            #     forwardVolts = 7.5
            # elif elevation_status == True and controls["DRIVE_FORWARD_AXIS"].position() < -25:
            #     forwardVolts = -7.5

            # Spin motors and combine controller axes
            motors["left"]["A"].spin(FORWARD, forwardVolts + turnVolts, VOLT)
            motors["left"]["B"].spin(FORWARD, forwardVolts + turnVolts, VOLT)
            motors["left"]["C"].spin(FORWARD, forwardVolts + turnVolts, VOLT)
            # leftMotorC.spin(FORWARD, forwardVolts + turnVolts, VOLT)
            motors["right"]["A"].spin(FORWARD, forwardVolts - turnVolts, VOLT)
            motors["right"]["B"].spin(FORWARD, forwardVolts - turnVolts, VOLT)
            motors["right"]["C"].spin(FORWARD, forwardVolts - turnVolts, VOLT)
        else:
            leftVolts = con.axis3.position() * 0.12
            rightVolts = con.axis2.position() * 0.12
            motors["left"]["A"].spin(FORWARD, leftVolts, VOLT)
            motors["left"]["B"].spin(FORWARD, leftVolts, VOLT)
            motors["left"]["C"].spin(FORWARD, leftVolts, VOLT)
            # leftMotorC.spin(FORWARD, forwardVolts + turnVolts, VOLT)
            motors["right"]["A"].spin(FORWARD, rightVolts, VOLT)
            motors["right"]["B"].spin(FORWARD, rightVolts, VOLT)
            motors["right"]["C"].spin(FORWARD, rightVolts, VOLT)

        if color_setting == "eject_blue" and intakeColor.hue() > 100 and not eject_prep and intakeColor.is_near_object():
            eject_prep = True
        if color_setting == "eject_red" and intakeColor.hue() < 18 and not eject_prep and intakeColor.is_near_object():
            eject_prep = True

        if (intakeDistance.object_distance() < 70) and (not queued_sort) and (eject_prep):
            motors["misc"]["intake_chain"].spin(FORWARD, 100, PERCENT)
            brain.timer.event(intake_sorter, 200)

            queued_sort = True

        if controls["INTAKE_FLEX_HOLD"].pressing():
            motors["misc"]["intake_flex"].spin(FORWARD, 100, PERCENT)
        elif (
                (controls["INTAKE_IN_HOLD"].pressing() and not enable_macro_lady_brown) or
                (controls["INTAKE_IN_HOLD"].pressing())): #and 
                    # ((not controls["LADY_BROWN_MACRO_UP_A"].pressing()) and enable_macro_lady_brown))):
            motors["misc"]["intake_flex"].spin(FORWARD, 100, PERCENT)
            if allow_intake_input:
                motors["misc"]["intake_chain"].spin(FORWARD, 65, PERCENT)
        elif (
                (controls["INTAKE_OUT_HOLD"].pressing() and not enable_macro_lady_brown) or
                (controls["INTAKE_OUT_HOLD"].pressing())): #and 
                   # ((not controls["LADY_BROWN_MACRO_DOWN_B"].pressing()) and enable_macro_lady_brown))):
            motors["misc"]["intake_flex"].spin(REVERSE, 100, PERCENT)
            motors["misc"]["intake_chain"].spin(REVERSE, 65, PERCENT)
        else:
            motors["misc"]["intake_flex"].stop()
            motors["misc"]["intake_chain"].stop()

        # WALL STAKES MOTORS
        # if not enable_macro_lady_brown:
        #     if controls["SIDE_STAKE_MANUAL_UP"].pressing():
        #         motors["misc"]["wall_stake"].spin(FORWARD, 100, PERCENT)
        #     elif controls["SIDE_STAKE_MANUAL_DOWN"].pressing():
        #         motors["misc"]["wall_stake"].spin(REVERSE, 30, PERCENT)
        #     else:
        #         motors["misc"]["wall_stake"].stop(BRAKE)
        # else:
        #     # Lady Brown controls
        if wall_control_cooldown == 0:
            if controls["LB_MACRO_DECREASE"].pressing():
                wall_control_cooldown = 5
                if wall_setpoint > 0:
                    wall_setpoint -= 1

            elif controls["LB_MACRO_INCREASE"].pressing():
                wall_control_cooldown = 5
                if wall_setpoint < len(wall_positions) - 1:
                    wall_setpoint += 1

        elif wall_control_cooldown > 0:
            wall_control_cooldown -= 1

        # # Grabber sensors
        # if mogo_pneu_engaged == True:
        #     if leftDistance.object_distance() < 80 and rightDistance.object_distance() < 80:
        #         mogo_pneu.set(False)

        # Screen debugging
        scr = brain.screen
        scr.clear_screen()
        scr.set_cursor(1,1)

        scr.print("Timer: {}".format(brain.timer.time()))
        scr.next_row()
        scr.print(wall_setpoint)
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
