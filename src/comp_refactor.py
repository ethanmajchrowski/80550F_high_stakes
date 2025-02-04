# Filename: comp.py
# Devices & variables last updated:
	# 2025-01-31 14:05:11.676754
####################
#region Devices
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
    print("SUCCESS LOADING SD CARD")
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

class MotorCollection():
    def __init__(self) -> None:
        self.leftA = Motor( Ports.PORT3, GearSetting.RATIO_6_1, True) # stacked top
        self.leftB = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True), # rear
        self.leftC = Motor( Ports.PORT4, GearSetting.RATIO_6_1, True), # front

        self.rightA = Motor(Ports.PORT18, GearSetting.RATIO_6_1, False), # stacked top
        self.rightB = Motor(Ports.PORT16, GearSetting.RATIO_6_1, False), # rear
        self.rightC = Motor(Ports.PORT15, GearSetting.RATIO_6_1, False), # front

        self.intakeChain = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)
        self.intakeFlex = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)
        self.ladyBrown = Motor(Ports.PORT7, GearSetting.RATIO_18_1, False)

motors = MotorCollection()

# PNEUMATICS
class PneumaticCollection():
    def __init__(self) -> None:
        self.mogo = DigitalOut(brain.three_wire_port.a)
        self.elevation_hook_release = DigitalOut(brain.three_wire_port.b)
        self.elevation_bar_lift = DigitalOut(brain.three_wire_port.c)
        self.PTO_left = DigitalOut(brain.three_wire_port.d) #! left/right not done yet
        self.PTO_right = DigitalOut(brain.three_wire_port.e)
        self.passive_hook_release = DigitalOut(brain.three_wire_port.f)
        self.doinker = DigitalOut(brain.three_wire_port.g)
        self.intake = DigitalOut(brain.three_wire_port.h)

pneumatics = PneumaticCollection()

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

# Turn on intake color sensor LED for consistency
intakeColor.set_light_power(100, PERCENT)
#endregion Devices####################
#DO NOT CHANGE THE FOLLOWING LINE:#
#end_1301825#
####################

def log(msg: Any):
    if do_logging:
        try:
            print(str(msg))
        except:
            print("LogError: Could not convert message to string.")

# organizational functions


# global functions (no I/O)
def calibrate_lady_brown():
    log("calibrating lady brown")

    motors["misc"]["wall_stake"].spin_for(REVERSE, 1000, MSEC, 100, PERCENT)
    wallEnc.set_position(0)

    log("lady brown calibration complete")

def calibrate_imu():
    log("calibrating IMU")

    imu.calibrate()
    while imu.is_calibrating():
        wait(5)

    log("IMU calibration complete")

def dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

# classes

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
        external_gear_ratio = 1 # fraction
        wheel_diameter = 2 # inches
        self.circumference = (wheel_diameter * 3.14159) * 25.4 * external_gear_ratio

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
            KP (float): kP value for tuning controller
            KD (float): kD value for tuning controller
            KI (float): kI value for tuning controller
            KI_MAX (float): integral will not be allowed to pass this value
            MIN (float): a minimum value that the PID will output

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

        if DELAY != 0:
            wait(DELAY, MSEC)
        
        return output

# robot states
class Autonomous():
    def __init__(self, parent) -> None:
        """
        Setup autonomous. Runs at start of program!
        """
        self.robot = parent

        self.positioning_algorithm = DeltaPositioning()

        self.fwd_speed = 8

        log("Autonomous object setup")
    
    def autonomous_setup(self) -> None:
        """
        Call at start of auton. Sets up the coordinates n stuff for the loaded path.
        """
        pass

    def run(self) -> None:
        """
        Runs autonomous. Do not modify.
        """
        self.autonomous_setup()

    def test(self) -> None:
        """
        Run a test version of autonomous. This is NOT run in competition!
        """
        self.autonomous_setup()

class Driver():
    def __init__(self, parent) -> None:
        """
        Setup driver. Runs at start of program!
        """
        self.robot = parent

        log("Driver object setup")
    
    def driver_setup(self) -> None:
        """
        Setup driver runtime things
        """
        pass

    def run(self) -> None:
        """
        Runs driver control loop.
        """
        self.driver_setup()
        
        while True:
            self.loop()
            
    def loop(self) -> None:
        """
        Runs every loop cycle
        """
        pass

# control objects
class LadyBrown():
    def __init__(self, parent) -> None:
        self.robot = parent
        # SENSOR VARIABLES
        self.wall_setpoint = 0
        self.wall_positions = [15, 125, 400, 600] # wall_setpoint is an INDEX used to grab from THIS LIST
        self.LB_enable_PID = True

class ColorSort():
    def __init__(self, parent) -> None:
        self.robot = parent
        # Intake ejector related booleans
        self.allow_intake_input = True
        self.queued_sort = False
        self.eject_prep = False

        self.tank_drive = False

        # Set initial color sort from SD card
        self.color_setting = data["config"]["initial_color_sorting"]["selected"]

class Robot():
    def __init__(self) -> None:
        self.mogo_pneu_engaged = False
        self.mogo_pneu_status = False

        self.lady_brown_controller = LadyBrown(parent=self)
        self.color_sort_controller = ColorSort(parent=self)

        self.autonomous_controller = Autonomous(parent=self)
        self.driver_controller = Driver(parent=self)

        # Autonomous - related variables
        self.pos = [0, 0]
        self.heading = 0

    def driver(self):
        log("Starting driver")
        self.driver_controller.run()
    
    def autonomous(self):
        log("Starting autonomous")
        self.autonomous_controller.run()

do_logging = True

# run file
def main():
    if not sd_fail:
        robot = Robot()
        comp = Competition(robot.driver, robot.autonomous)
        # when connected to the field, do everything normally
        if comp.is_field_control() or comp.is_competition_switch():
            log("Connected to field or comp switch.")
    else:
        brain.screen.set_fill_color(Color.RED)
        log("Robot object not created. (SD Load Error)")
        raise ImportError("Robot object not created. (SD Load Error)")

main()
# Reviewed code:
# [ ] Autonomous Handler
# [X] Delta Positioning
# [ ] Driver
# [ ] Pure Pursuit
# [ ] Path Loading
