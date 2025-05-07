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
from json import load, dumps
from random import random

class LogLevel():
    # LogLevel constants for better ID
    FATAL = 5
    ERROR = 4
    WARNING = 3
    INFO = 2
    DEBUG = 1
    UNKNOWN = 0

def threaded_log(msg: Any, level):
    tag = str()
    try:
        str(msg)
    except:
        print("Log Error: couldn't convert msg to str")
        return
    
    time_abs_ms = brain.timer.system()
    time_s = time_abs_ms % 60000 // 1000
    time_min = time_abs_ms // 60000
    time_ms = time_abs_ms - (time_s * 1000) + (time_min * 60000)

    time_str = "{minutes:02}:{seconds:02}.{milliseconds:03} ".format(
                minutes=time_min, seconds=time_s, milliseconds=time_ms) # MM:SS.mS

    if level == 0: tag = "UNKNOWN"
    if level == 1: tag = "DEBUG"
    if level == 2: tag = "INFO"
    if level == 3: tag = "WARNING"
    if level == 4: tag = "ERROR"
    if level == 5: tag = "FATAL"
    # MM:SS.0000 [WARNING] Unable to load SD card!
    # print(time_str + tag + str(msg))
    foxglove_logLevel = level
    if foxglove_log:
        packet_mgr.add_packet("foxglove.Log", {"timestamp": {"sec": time_s, "nsec": time_ms * 1000000}, "message": msg, "level": foxglove_logLevel, "name": "main", "file": "main", "line": 0})
    else:
        print("[{}] {}: {}".format(tag, time_abs_ms, msg))

def log(msg: Any, level = LogLevel.INFO):
    if do_logging:
        Thread(threaded_log, (msg, level))

class PacketTiming:
    CONTROLLER = 50
    BRAIN = 20

class PacketManager():
    def __init__(self, timing: PacketTiming | int) -> None:
        self.thread: Thread
        self.queue = []
        self.delay = timing
        self.allow_sending = True

    def start(self) -> None:
        #print newline to ensure we don't get a decode error from previous print
        print("") 
        # global do_logging
        self.thread = Thread(self.loop)
        # do_logging = False
    
    def loop(self) -> None:
        while True:
            if self.allow_sending:
                if len(self.queue) != 0:
                    print(self.queue.pop(0)[1])
                    # print(brain.timer.system())
            
            sleep(self.delay, TimeUnits.MSEC) #type: ignore
    
    def add_packet(self, topic, payload: dict) -> bool:
        """
        Adds item to packet queue.
        Returns:
            False if packet topic already in queue.
            True if packet was added to queue.
        """
        if topic not in ["LOG", "foxglove.LOG"]: # we don't want to drop any log packets
            for item in self.queue:
                if item[0] == topic:
                    return False
        
        data = {
            "topic": topic,
            "payload": payload
        }
        self.queue.append((topic, dumps(data)))
        return True

do_logging = True
packet_mgr = PacketManager(PacketTiming.CONTROLLER)
packet_mgr.start()

brain = Brain()
con = Controller(PRIMARY)
con_2 = Controller(PARTNER)

class control():
    DRIVE_FORWARD_AXIS =            con.axis3
    DRIVE_TURN_AXIS =               con.axis1
    INTAKE_IN_HOLD =                con.buttonR1
    INTAKE_OUT_HOLD =               con.buttonR2
    INTAKE_HEIGHT_TOGGLE =          con.buttonLeft
    MOGO_GRABBER_TOGGLE =           con.buttonA
    DOINKER =                       con.buttonRight
    INTAKE_FLEX_HOLD =              con.buttonL2
    LB_MANUAL_UP =                  con.buttonL1
    LB_MANUAL_DOWN =                con.buttonL2
    LB_MACRO_HOME =                 con.buttonDown
    BACK_OFF_ALLIANCE_STAKE =       con.buttonX
    LB_MACRO_DESCORE =              con.buttonUp

class control_2():
    ELEVATION_PRIMARY_PNEUMATICS =  con_2.buttonUp
    DOINKER =                       con_2.buttonRight
    DRIVE_MODE =                    con_2.buttonDown
    AXIS_FORWARDS =                 con_2.axis3
    AXIS_TILT =                     con_2.axis1
    PTO_TOGGLE =                    con_2.buttonB
    INTAKE_IN =                     con_2.buttonR1
    INTAKE_OUT =                    con_2.buttonR2
    LB_MANUAL_UP =                  con_2.buttonL1
    LB_MANUAL_DOWN =                con_2.buttonL2
    ZERO_ROT_SENSOR =               con_2.buttonLeft

class motor():
    leftA = Motor( Ports.PORT3, GearSetting.RATIO_6_1, True) # stacked top
    leftB = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True) # rear
    leftC = Motor( Ports.PORT4, GearSetting.RATIO_6_1, True) # front

    rightA = Motor(Ports.PORT18, GearSetting.RATIO_6_1, False) # stacked top
    rightB = Motor(Ports.PORT16, GearSetting.RATIO_6_1, False) # rear
    rightC = Motor(Ports.PORT15, GearSetting.RATIO_6_1, False) # front

    intakeChain = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
    intakeFlex = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
    ladyBrown = Motor(Ports.PORT18, GearSetting.RATIO_36_1,  False)

# PNEUMATICS
class pneumatic():
    mogo = DigitalOut(brain.three_wire_port.e)
    doinker_left = DigitalOut(brain.three_wire_port.f)
    doinker = DigitalOut(brain.three_wire_port.h)
    intake = DigitalOut(brain.three_wire_port.g)

#### SENSORS
# ENCODERS
class sensor():
    groundEncoder = Rotation(Ports.PORT2)

    wallEncoder = Rotation(Ports.PORT20)
    # DISTANCE SENSORS
    intakeDistance = Distance(Ports.PORT21)

    wallLeftDistance = Distance(Ports.PORT6)
    wallBackDistance = Distance(Ports.PORT1)
    wallFrontDistance = Distance(Ports.PORT9)
    wallRightDistance = Distance(Ports.PORT19)

    # elevationDistance = Distance(Ports.PORT20) # unplugged

    # MISC SENSORS
    intakeColor = Optical(Ports.PORT4)
    imu = Inertial(Ports.PORT3)

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

    # motor.ladyBrown.spin_for(REVERSE, 1000, MSEC, 100, PERCENT)
    sensor.wallEncoder.set_position(0)

    log("lady brown calibration complete")

def calibrate_imu():
    log("calibrating IMU. Minimum calibration time 2.5 seconds.")
    min_duration = brain.timer.system() + 2500

    sensor.imu.calibrate()
    while brain.timer.system() < min_duration:
        sleep(10, TimeUnits.MSEC)

    log("IMU calibration complete")

def dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

def gauss(mu = 0, sigma = 1):
    u1 = random()
    u2 = random()

    z = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
    return mu + sigma * z

def sign(num):
    if num == 0: return 0
    if num < 0: return -1
    else: return 1
# classes

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
        self.integral_processed = 0
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
            self.integral += self.error

            if self.integral > 0:
                self.integral = min(self.integral, 5 / self.kI)
            else:
                self.integral = max(self.integral, 5 / self.kI * -1)
            self.integral_processed = self.integral * self.kI

            if self.kI_max is not None:
                if self.integral_processed > 0:
                    self.integral_processed = min(self.integral_processed, self.kI_max)
                else:
                    self.integral_processed = max(self.integral_processed, self.kI_max * -1)

        self.derivative = (self.error - self.last_error) * self.kD
        
        output = (self.error * self.kP) + self.integral_processed + self.derivative
        if self.minimum is not None:
            if abs(output) < self.minimum:
                if self.error < 0: output = -self.minimum
                else: output = self.minimum
        
        self.last_error = self.error
        wait(DELAY, MSEC)
        return output
    
class Robot():
    def __init__(self) -> None:
        self.color_sort_controller = ColorSortController(parent=self)

        self.driver_controller = Driver(parent=self)

        self.LB_PID = LadyBrown(motor.ladyBrown)

    def driver(self) -> None:
        log("Starting driver")
        self.driver_controller.run()

class ControllerFunctions():
    @staticmethod
    def switch_mogo():
        log("Switched mogo pneumatic")
        pneumatic.mogo.set(not pneumatic.mogo.value())

    @staticmethod
    def switch_intake_height():
        if not flags.elevating:
            log("Switched intake height")
            pneumatic.intake.set(not pneumatic.intake.value())

    @staticmethod
    def switch_doinker():
        log("Switched doinker")
        # pneumatic.doinker.set(not pneumatic.doinker.value())
        # pneumatic.doinker_left.set(pneumatic.doinker_left.value())
        pneumatic.doinker.set(False)
        pneumatic.doinker_left.set(False)

    @staticmethod
    def manual_elevation():
        log("Elevation not on robot!", LogLevel.WARNING)
        # if pneumatic.elevation_bar_lift_left.value(): # elevation is up
        #     pneumatic.elevation_bar_lift_left.set(False)
        #     pneumatic.elevation_bar_lift_right.set(False)

        #     pneumatic.elevation_bar_lower_left.set(True)
        #     pneumatic.elevation_bar_lower_right.set(True)
        # else:
        #     pneumatic.elevation_bar_lift_left.set(True)
        #     pneumatic.elevation_bar_lift_right.set(True)

        #     pneumatic.elevation_bar_lower_left.set(False)
        #     pneumatic.elevation_bar_lower_right.set(False)
    
    @staticmethod
    def toggle_PTO():
        log("Toggled PTO pneumatics")
        log("PTOs not on robot!", LogLevel.WARNING)
        # pneumatic.PTO.set(not pneumatic.PTO.value())
    
    @staticmethod
    def zero_lady_brown():
        log("Zeroed wall stake rotation sensor")
        sensor.wallEncoder.set_position(0)

    @staticmethod
    def wallstake_macro():
        log("Driving off wall stake")
        flags.disable_drive = True
        drivetrain.drive_for(DirectionType.FORWARD, 1.6, DistanceUnits.IN, 80, VelocityUnits.PERCENT)
        flags.disable_drive = False

class Driver():
    def __init__(self, parent: Robot) -> None:
        """
        Setup driver. Runs at start of program!
        """
        self.robot = parent
        self.elevation_hold_duration = 0
        self.elevation_hold_duration_reset = 7
        self.LB_braketype = BrakeType.HOLD

        log("Driver object setup")
    
    def driver_setup(self) -> None:
        """
        Setup driver runtime things
        """
        sensor.intakeColor.set_light_power(100, PERCENT)
        pneumatic.doinker.set(False)
        pneumatic.doinker.set(False)

        # bind controller functions
        control.DOINKER.pressed(ControllerFunctions.switch_doinker)
        control.MOGO_GRABBER_TOGGLE.pressed(ControllerFunctions.switch_mogo)
        # control.INTAKE_HEIGHT_TOGGLE.pressed(ControllerFunctions.switch_intake_height)
        control.LB_MACRO_HOME.pressed(self.robot.LB_PID.home)
        control.LB_MACRO_DESCORE.pressed(self.robot.LB_PID.descore)
        # control.BACK_OFF_ALLIANCE_STAKE.pressed(ControllerFunctions.wallstake_macro)

        # just in case
        flags.color_setting = ColorSort.NONE
        self.robot.LB_PID.run()

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
        # if flags.elevating:
        #     self.elevation_loop()
        # else:
        self.driver_loop()

    def drive_controls(self) -> None:
        turnVolts = -(control.DRIVE_TURN_AXIS.position() * 0.12) * 0.9
        forwardVolts = -control.DRIVE_FORWARD_AXIS.position() * 0.12

        # Spin motors and combine controller axes
        motor.leftA.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motor.leftB.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motor.leftC.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        
        motor.rightA.spin(FORWARD, forwardVolts - turnVolts, VOLT)
        motor.rightB.spin(FORWARD, forwardVolts - turnVolts, VOLT)
        motor.rightC.spin(FORWARD, forwardVolts - turnVolts, VOLT)
    
    def intake_controls(self) -> None:
        if (control.INTAKE_IN_HOLD.pressing()):
            motor.intakeFlex.spin(FORWARD, 12, VOLT)

            if flags.allow_intake_input:
                motor.intakeChain.spin(FORWARD, 100, PERCENT)
        elif (control.INTAKE_OUT_HOLD.pressing()):
            motor.intakeFlex.spin(REVERSE, 100, PERCENT)
            motor.intakeChain.spin(REVERSE, 100, PERCENT)
        elif not flags.elevating:
            motor.intakeFlex.stop()
            motor.intakeChain.stop()
    
    def lady_brown_controls(self) -> None:
        # WALL STAKES MOTORS
        if control.LB_MANUAL_UP.pressing():
            print("up")
            if not 250 < sensor.wallEncoder.angle() < 355:
                motor.ladyBrown.spin(FORWARD, 100, PERCENT)
                self.robot.LB_PID.enabled = False
            else:
                motor.ladyBrown.stop(BRAKE)
        elif control.LB_MANUAL_DOWN.pressing():
            if not (sensor.wallEncoder.angle() < 5 or sensor.wallEncoder.angle() > 350): # arm is lowered
                motor.ladyBrown.spin(REVERSE, 90, PERCENT)
                self.robot.LB_PID.enabled = False
            else:
                motor.ladyBrown.stop(BRAKE)
        elif not self.robot.LB_PID.enabled and not flags.elevating:
            motor.ladyBrown.stop(BRAKE)

    def driver_loop(self) -> None:
        if not flags.disable_drive:
            self.drive_controls()
        self.intake_controls()
        self.lady_brown_controls()

        if flags.elevating and self.robot.LB_PID.enabled:
            self.robot.LB_PID.enabled = False
            print("stopped PID")

        self.robot.color_sort_controller.sense()

# control objects
class LadyBrown():
    POS_LOAD = 28
    POS_DESCORE = 155
    def __init__(self, wall_motor) -> None:
        # self.pid = MultipurposePID(0.3, 0.5, 0.01, 2, None)
        # self.pid = MultipurposePID(0.035, 0.05, 0.004, 2, None)
        self.pid = MultipurposePID(0.25, 0.1, 0, 2, None)
        self.enabled = False
        self.autostop = False
        self.threshold = 5
        self.motor: Motor = wall_motor
        self.target_rotation = 0
        self.thread: Thread
        self.in_range = False

        self.sensor: Rotation | Encoder = sensor.wallEncoder
        self.last_enabled = self.enabled
    
    def home(self):
        """
        Homes PID to ready to load position
        """
        self.target_rotation = LadyBrown.POS_LOAD
        self.enabled = True
        self.in_range = True
        log("Homing lady brown")
    
    def descore(self):
        """
        Run PID to try and unwinch the winches and prep for next ladder
        """
        self.target_rotation = LadyBrown.POS_DESCORE
        self.enabled = True
        self.in_range = True
        log("Descore lady brown")

    def run(self):
        """
        Run once to start PID thread.
        """
        log("Starting lady brown thread")
        while True:
            if self.enabled and self.in_range:
                self.loop()

            if (not self.enabled) and self.last_enabled:
                self.pid.integral = 0
            
            self.last_enabled = self.enabled
            # motor.ladyBrown.stop(BrakeType.COAST)
            
            sleep(20, MSEC)

    def loop(self):
        """
        Runs once every loop.
        """
        angle = self.sensor.position()
        if angle > 270:
            angle = 360 - angle
        
        output = self.pid.calculate(self.target_rotation, angle)
        # if abs(self.pid.error) > 40: 
        #     self.pid.integral = 0
        #     output *= 4
        # print(sensor.wallEncoder.angle(), output)

        # # debug data
        if foxglove_log:
            data = {
                "o": str(round(output, 2)),
                "error": str(round(self.target_rotation-angle, 2)),
                "integral": str(round(self.pid.integral, 2)),
                "integralp": str(round(self.pid.integral_processed, 2)),
                "d": str(round(self.pid.error * self.pid.kP, 2)),
                "p": str(round(self.pid.derivative * self.pid.kD))
            }
            packet_mgr.add_packet("lb", data)

        # limit output
        if output < -8:
            output = -8
        elif output > 8:
            output = 8

        self.motor.spin(FORWARD, output, VOLT)

        if abs(self.pid.error) < self.threshold and self.autostop:
            self.autostop = False
            self.enabled = False
            self.motor.stop(BrakeType.HOLD)
            log("LB PID Autostop")
            self.pid.integral = 0
            self.threshold = 5

class ColorSort:
    NONE = 0
    EJECT_BLUE = 1
    EJECT_RED = 2

class ColorSortController():
    BLUE = 195
    RED = 20
    def __init__(self, parent: Robot) -> None:
        self.robot = parent
        
        self.ejecting = False
        self.eject_next_ring = False
        self.give_up_threshold = 20

        self.blue_hue = ColorSortController.BLUE
        self.red_hue = ColorSortController.RED

        self.wait_time_ms = 210
        self.hold_time_ms = 150
    
    def sense(self) -> None:
        """
        Run to detect if we have an incorrectly colored disk.
        If so, queue an eject of the next dist to the top of the intake.
        """
        if not flags.color_setting == ColorSort.NONE and not self.ejecting:
            # Color flagging
            if sensor.intakeColor.is_near_object() and not self.eject_next_ring:
                if sensor.intakeColor.hue() < self.red_hue:
                    if flags.color_setting == ColorSort.EJECT_RED:
                        self.eject_next_ring = True
                        log("Queued [red] ring to eject. Hue: {}".format(sensor.intakeColor.hue()))
                
                if sensor.intakeColor.hue() > self.blue_hue:
                    if flags.color_setting == ColorSort.EJECT_BLUE:
                        self.eject_next_ring = True
                        log("Queued [blue] ring to eject. Hue: {}".format(sensor.intakeColor.hue()))

            if self.eject_next_ring:
                self.give_up_threshold -= 1
                
                if self.give_up_threshold <= 0:
                    self.eject_next_ring = False
                    log("Color sort timed out!")
            else:
                self.give_up_threshold = 100

            # color flag is on, now ring is up intake
            if ((sensor.intakeDistance.object_distance() < 70) and (self.eject_next_ring)):
                log("Ejecting ring!")

                self.allow_intake_input = False
                self.eject_next_ring = False
                self.ejecting = True
                command = motor.intakeChain.command(VelocityUnits.PERCENT)

                sleep(160, TimeUnits.MSEC)
                motor.intakeChain.stop(HOLD)
                brain.timer.event(self.stop_eject, 200, (command,))

    def stop_eject(self, speed):
        log("Done ejecting ring!")
        
        if speed > 0: motor.intakeChain.spin(FORWARD, speed, VelocityUnits.PERCENT)
        else: motor.intakeChain.spin(REVERSE, speed, VelocityUnits.PERCENT)
        
        self.allow_intake_input = True
        self.ejecting = False

class flags():
    """
    'global' booleans enum for various states.
    """
    color_setting = ColorSort.NONE
    allow_intake_input = True
    elevating = False
    wall_setpoint = 1
    disable_drive = False

def pull_data(data: dict, robot: Robot):
    """
    Assigns data variables to different objects.
    """
    pass

def null_function():
    pass

# run file
foxglove_log = False
do_logging = True

log("Battery at {}".format(brain.battery.capacity()))
brain.screen.clear_screen(Color.CYAN)
robot = Robot()
robot.driver()
