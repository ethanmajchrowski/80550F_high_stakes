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

class ImmutableMeta(type):
    def __setattr__(self, key, value):
        raise AttributeError("Cannot modify constant '{}'".format(key))

class LogLevel(metaclass=ImmutableMeta):
    # LogLevel constants for better ID
    FATAL = 5
    ERROR = 4
    WARNING = 3
    INFO = 2
    DEBUG = 1
    UNKNOWN = 0

def log(msg: Any, level = LogLevel.INFO):
    if do_logging:
        tag = str()
        try:
            if level == 0: tag = "[UNKNOWN] "
            if level == 1: tag = "[DEBUG] "
            if level == 2: tag = "[INFO] "
            if level == 3: tag = "[WARNING] "
            if level == 4: tag = "[FATAL] "
            print(tag + str(msg))
        except:
            print("LogError: Could not convert message to string.")

brain = Brain()
con = Controller(PRIMARY)
con_2 = Controller(PARTNER)

class control():
    DRIVE_FORWARD_AXIS =           con.axis3
    DRIVE_TURN_AXIS =              con.axis1
    INTAKE_IN_HOLD =               con.buttonR1
    INTAKE_OUT_HOLD =              con.buttonR2
    INTAKE_HEIGHT_TOGGLE =         con.buttonLeft
    MOGO_GRABBER_TOGGLE =          con.buttonA
    DOINKER =                      con.buttonRight
    INTAKE_FLEX_HOLD =             con.buttonL2
    LB_MANUAL_UP =                 con.buttonL1
    LB_MANUAL_DOWN =               con.buttonL2
    MANUAL_ELEVATION_PNEUMATICS =  con.buttonUp
    LB_MACRO_HOME =                con.buttonDown

class motor():
    leftA = Motor( Ports.PORT3, GearSetting.RATIO_6_1, True) # stacked top
    leftB = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True) # rear
    leftC = Motor( Ports.PORT4, GearSetting.RATIO_6_1, True) # front

    rightA = Motor(Ports.PORT18, GearSetting.RATIO_6_1, False) # stacked top
    rightB = Motor(Ports.PORT16, GearSetting.RATIO_6_1, False) # rear
    rightC = Motor(Ports.PORT15, GearSetting.RATIO_6_1, False) # front

    intakeChain = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)
    intakeFlex = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)
    ladyBrown = Motor(Ports.PORT7, GearSetting.RATIO_18_1, False)

# PNEUMATICS
class pneumatic():
    mogo = DigitalOut(brain.three_wire_port.a)
    elevation_hook_release = DigitalOut(brain.three_wire_port.b)
    elevation_bar_lift = DigitalOut(brain.three_wire_port.c)
    PTO_left = DigitalOut(brain.three_wire_port.d) #! left/right not done yet
    PTO_right = DigitalOut(brain.three_wire_port.e)
    passive_hook_release = DigitalOut(brain.three_wire_port.f)
    doinker = DigitalOut(brain.three_wire_port.g)
    intake = DigitalOut(brain.three_wire_port.h)

#### SENSORS
# ENCODERS
class sensor():
    leftEncoder = Rotation(Ports.PORT2)
    rightEncoder = Rotation(Ports.PORT17)
    wallEncoder = Rotation(Ports.PORT1)
    # DISTANCE SENSORS
    intakeDistance = Distance(Ports.PORT9)

    leftWallDistance = Distance(Ports.PORT6)
    backWallDistance = Distance(Ports.PORT13)

    elevationDistance = Distance(Ports.PORT20)

    # MISC SENSORS
    intakeColor = Optical(Ports.PORT10)
    imu = Inertial(Ports.PORT11)

lmg = MotorGroup(motor.leftA, motor.leftB, motor.leftC)
rmg = MotorGroup(motor.rightA, motor.rightB, motor.rightC)
"""
wheelTravel - The circumference of the driven wheels. The default is 300 mm.
trackWidth - The track width of the drivetrain. The default is 320 mm. 
    (distance between left and right parts of the drivetrain)
wheelBase - The wheel base of the Drivetrain. The default is 320 mm.
    (distance from center of front and rear wheels)
units - A valid DistanceUnit for the units that wheelTravel, trackWidth and wheelBase are specified in. The default is MM.
externalGearRatio - The gear ratio used to compensate drive distances if gearing is used.
"""
drivetrain = SmartDrive(lmg, rmg, sensor.imu)
"""
Over Under Settings:
    drivetrain.set_timeout(2, SECONDS)
    drivetrain.set_turn_constant(0.28)
    drivetrain.set_turn_threshold(0.25)
"""

# Turn on intake color sensor LED for consistency
sensor.intakeColor.set_light_power(100, PERCENT)
#endregion Devices####################
#DO NOT CHANGE THE FOLLOWING LINE:#
#nada you dont get to mess up my refactor#
####################

# global functions (no I/O)
def calibrate_lady_brown():
    log("calibrating lady brown")

    motor.ladyBrown.spin_for(REVERSE, 1000, MSEC, 100, PERCENT)
    sensor.wallEncoder.set_position(0)

    log("lady brown calibration complete")

def calibrate_imu():
    log("calibrating IMU")

    sensor.imu.calibrate()
    while sensor.imu.is_calibrating():
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

class Robot():
    def __init__(self) -> None:
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
    
# robot states
class Autonomous():
    def __init__(self, parent) -> None:
        """
        Setup autonomous. Runs at start of program!
        """
        self.robot = parent

        self.positioning_algorithm = DeltaPositioning(sensor.leftEncoder, sensor.rightEncoder, sensor.imu)

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
    def __init__(self, parent: Robot) -> None:
        """
        Setup driver. Runs at start of program!
        """
        self.robot = parent

        log("Driver object setup")
    
    def driver_setup(self) -> None:
        """
        Setup driver runtime things
        """
        sensor.intakeColor.set_light_power(100, PERCENT)
        brain.screen.clear_screen()

    def run(self) -> None:
        """
        Runs driver control loop.
        """
        self.driver_setup()
        
        while True:
            self.loop()

    def drive_controls(self):
        turnVolts = (control.DRIVE_TURN_AXIS.position() * 0.12) * 0.9
        forwardVolts = control.DRIVE_FORWARD_AXIS.position() * 0.12

        # Spin motors and combine controller axes
        motor.leftA.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motor.leftB.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motor.leftC.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        
        motor.rightA.spin(FORWARD, forwardVolts - turnVolts, VOLT)
        motor.rightB.spin(FORWARD, forwardVolts - turnVolts, VOLT)
        motor.rightC.spin(FORWARD, forwardVolts - turnVolts, VOLT)
    
    def intake_controls(self):
        if (control.INTAKE_IN_HOLD.pressing()):
            motor.intakeFlex.spin(FORWARD, 100, PERCENT)
            if flags.allow_intake_input:
                motor.intakeChain.spin(FORWARD, 100, PERCENT)
        elif (control.INTAKE_OUT_HOLD.pressing()):
            motor.intakeFlex.spin(REVERSE, 100, PERCENT)
            motor.intakeChain.spin(REVERSE, 100, PERCENT)
        else:
            motor.intakeFlex.stop()
            motor.intakeChain.stop()
    
    def lady_brown_controls(self):
        # WALL STAKES MOTORS
        if control.LB_MANUAL_UP.pressing():
            motor.ladyBrown.spin(FORWARD, 100, PERCENT)
            flags.LB_enable_PID = False
        elif control.LB_MANUAL_DOWN.pressing():
            motor.ladyBrown.spin(REVERSE, 30, PERCENT)
            flags.LB_enable_PID = False
        else:
            motor.ladyBrown.stop(HOLD)

    def loop(self) -> None:
        """
        Runs every loop cycle
        """

        self.drive_controls()
        self.intake_controls()

        self.robot.color_sort_controller.sense()

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
        if con.buttonUp.pressing() and not elevating:
            elevation_hold_duration -= 1
            if elevation_hold_duration <= 0:
                elevating = True
                elevation_macro()
        else:
            elevation_hold_duration = 10

# control objects
class LadyBrown():
    def __init__(self, parent: Robot) -> None:
        self.robot = parent
        # SENSOR VARIABLES
        self.wall_setpoint = 0
        self.wall_positions = [15, 125, 400, 600] # wall_setpoint is an INDEX used to grab from THIS LIST

class ColorSort():
    def __init__(self, parent: Robot) -> None:
        self.robot = parent
        
        self.queued_sort = False
        self.eject_next_ring = False

        self.blue_hue = 100

        self.wait_time_ms = 210
        self.hold_time_ms = 150
    
    def sense(self):
        """
        Run to detect if we have an incorrectly colored disk.
        If so, queue an eject of the next dist to the top of the intake.
        """
        # If we eject blue, 
        if ((flags.color_setting == "eject_blue") and
        sensor.intakeColor.hue() > self.blue_hue and 
        sensor.intakeColor.is_near_object() and 
        not self.eject_next_ring):

            self.eject_next_ring = True
            log("Found [blue] ring to eject.")
        
        if (flags.color_setting == "eject_red" and 
        sensor.intakeColor.hue() < 18 and 
        sensor.intakeColor.is_near_object()and 
        not self.eject_next_ring):
            
            self.eject_next_ring = True
            log("Found [red] ring to eject.")

        if ((sensor.intakeDistance.object_distance() < 70) and 
        (not self.queued_sort) and 
        (self.eject_next_ring)):

            self.allow_intake_input = False
            motor.intakeChain.spin(FORWARD, 100, PERCENT)

            brain.timer.event(self.intake_sorter, self.wait_time_ms)

            self.queued_sort = True
    
    def intake_sorter(self):
        # check if we are moving from manual --> auto or auto --> manual
        if flags.allow_intake_input:
            motor.intakeChain.stop(BRAKE)
            # stop chain and don't let driver loop re-enable it until we're done with it
            flags.allow_intake_input = False

            # At this point, allow_intake_input will be false
            # so the current code won't run in a loop
            # schedule this again to re-enable to intake in ms
            brain.timer.event(self.intake_sorter, self.hold_time_ms)
            log("Ejecting ring.")
        else:
            # Re-enable intake
            flags.allow_intake_input = True
            # reset timing flags
            self.queued_sort = False
            self.eject_next_ring = False

class flags():
    """
    'global' booleans for various states.
    """
    LB_enable_PID = False
    mogo_pneu_engaged = False
    mogo_pneu_status = False
    color_setting = "none" # "none", "eject_blue", "eject_red"
    allow_intake_input = True

do_logging = True

# load SD card
sd_fail = False

try:
    with open("cfg/config.json", 'r') as f:
        data = load(f)
    log("SUCCESS LOADING SD CARD")
except:
    sd_fail = True
    log("ERROR LOADING SD CARD DATA")


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
