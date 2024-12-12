# Filename: driver.py
# Devices & variables last updated:
	# 2024-12-11 18:24:40.197123
####################
#region Devices
# Filename: driver.py
# Devices & variables last updated:
    # 2024-10-30 18:19:48.449465
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
    "DRIVE_TURN_AXIS":     con.axis1,
    "INTAKE_IN_HOLD":      con.buttonR1,
    "INTAKE_OUT_HOLD":     con.buttonR2,
    "INTAKE_HEIGHT_TOGGLE":con.buttonL1,
    "SIDE_SCORING_TOGGLE": con.buttonB,
    "MOGO_GRABBER_TOGGLE": con.buttonA,
    "AUTO_MOGO_ENGAGE_TOGGLE": con.buttonY,
    "ELEVATION_RELEASE_1": con.buttonDown,
    "ELEVATION_RELEASE_2": con.buttonLeft,
    "CYCLE_EJECTOR_COLOR": con.buttonLeft,
    "DOINKER":             con.buttonRight,
    "INTAKE_FLEX_HOLD":    con.buttonL2,
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
        "intake_flex": Motor(Ports.PORT5, GearSetting.RATIO_6_1, False) # 5.5 W flexwheel hinge
    }
}

# PNEUMATICS
mogo_pneu = DigitalOut(brain.three_wire_port.a)

intake_pneu = DigitalOut(brain.three_wire_port.h)
doinker_pneu = DigitalOut(brain.three_wire_port.b)

# SENSORS
leftEnc = Rotation(Ports.PORT2)
rightEnc = Rotation(Ports.PORT17)

leftDistance = Distance(Ports.PORT13)
rightDistance = Distance(Ports.PORT19)
intakeDistance = Distance(Ports.PORT9)
intakeColor = Optical(Ports.PORT10)

imu = Inertial(Ports.PORT11)

if calibrate_imu:
    imu.calibrate()
    while imu.is_calibrating(): 
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

sd_fail = False
# load config data from SD card
try:
    with open("cfg/config.json", 'r') as f:
        data = load(f)
except:
    sd_fail = True
    print("ERROR LOADING SD CARD DATA")

# Set initial color sort from SD card
if not sd_fail:
    color_setting = data["config"]["initial_color_sorting"]["selected"]
else:
    color_setting = "none"
#endregion Devices####################
#DO NOT CHANGE THE FOLLOWING LINE:#
#end_1301825#
####################

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

    # print("new: " + color_setting)

controls["DOINKER"].pressed(switch_doinker)
controls["MOGO_GRABBER_TOGGLE"].pressed(switch_mogo)
controls["AUTO_MOGO_ENGAGE_TOGGLE"].pressed(switch_mogo_engaged)
controls["INTAKE_HEIGHT_TOGGLE"].pressed(switch_intake_height)
controls["CYCLE_EJECTOR_COLOR"].pressed(cycle_ejector_color)

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

        if color_setting == "eject_blue" and intakeColor.hue() > 100 and not eject_prep:
            eject_prep = True
        if color_setting == "eject_red" and intakeColor.hue() < 18 and not eject_prep:
            eject_prep = True

        if (intakeDistance.object_distance() < 70) and (not queued_sort) and (eject_prep):
            motors["misc"]["intake_chain"].spin(FORWARD, 100, PERCENT)
            brain.timer.event(intake_sorter, 210)

            queued_sort = True

        if controls["INTAKE_FLEX_HOLD"].pressing():
            motors["misc"]["intake_flex"].spin(FORWARD, 100, PERCENT)
        elif controls["INTAKE_IN_HOLD"].pressing():
            motors["misc"]["intake_flex"].spin(FORWARD, 100, PERCENT)
            if allow_intake_input:
                motors["misc"]["intake_chain"].spin(FORWARD, 100, PERCENT)
        elif controls["INTAKE_OUT_HOLD"].pressing():
            motors["misc"]["intake_flex"].spin(REVERSE, 100, PERCENT)
            motors["misc"]["intake_chain"].spin(REVERSE, 100, PERCENT)
        else:
            motors["misc"]["intake_flex"].stop()
            motors["misc"]["intake_chain"].stop()

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
driver()
