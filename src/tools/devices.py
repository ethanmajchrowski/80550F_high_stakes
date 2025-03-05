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
    
    time_ms = brain.timer.system()
    time_s = time_ms % 60000 // 1000
    time_min = time_ms // 60000
    time_ms -= (time_s * 1000) + (time_min * 60000)

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
    if foxglove_log: #type:ignore
        packet_mgr.add_packet("foxglove.Log", {"timestamp": {"sec": time_s, "nsec": time_ms * 1000000}, "message": msg, "level": foxglove_logLevel, "name": "main", "file": "main", "line": 0})
    else:
        print("[{}] {}: {}".format(tag, time_ms + (time_s + time_min*60)*1000, msg))

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
    MANUAL_ELEVATION_PNEUMATICS =   con.buttonUp
    LB_MACRO_HOME =                 con.buttonDown

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

    intakeChain = Motor(Ports.PORT10, GearSetting.RATIO_6_1, False)
    intakeFlex = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)
    ladyBrown = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)

# PNEUMATICS
class pneumatic():
    mogo = DigitalOut(brain.three_wire_port.a)
    doinker = DigitalOut(brain.three_wire_port.h)
    elevatoinBarLift = DigitalOut(brain.three_wire_port.d)
    intake = DigitalOut(brain.three_wire_port.g)

#### SENSORS
# ENCODERS
class sensor():
    leftEncoder = Rotation(Ports.PORT2)
    rightEncoder = Rotation(Ports.PORT17)
    driftEncoder = Rotation(Ports.PORT13)

    wallEncoder = Rotation(Ports.PORT20)
    # DISTANCE SENSORS
    intakeDistance = Distance(Ports.PORT9)

    wallLeftDistance = Distance(Ports.PORT6)
    wallBackDistance = Distance(Ports.PORT20)
    wallFrontDistance = Distance(Ports.PORT9)
    wallRightDistance = Distance(Ports.PORT19)

    # elevationDistance = Distance(Ports.PORT20) # unplugged

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
