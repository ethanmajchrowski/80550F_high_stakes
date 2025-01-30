# Filename: driver.py
# Devices & variables last updated:
	# 2025-01-29 15:57:19.380787
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


# def elevation_macro():
#     global LB_enable_PID
#     print("start elevation")
#     # PTO
#     PTO_left_pneu.set(True)
#     PTO_right_pneu.set(True)

#     LB_enable_PID = False
#     motors["misc"]["wall_stake"].spin_for(FORWARD, 500, MSEC, 100, PERCENT)

#     pitch_pid = MultipurposePID(0.1, 0, 0, 0)

#     elevation_hook_release.set(True)
#     # wait and close these pistons cause leak :(
#     sleep(200, MSEC)
#     intake_pneu.set(True)
#     doinker_pneu.set(True)
#     elevation_hook_release.set(False)
#     elevation_bar_lift.set(True)

#     # wait for matics
#     sleep(100, MSEC)
#     motors["misc"]["wall_stake"].spin_for(REVERSE, 1200, MSEC, 100, PERCENT)
#     elevation_bar_lift.set(False)

#     while True:
#         pitch = imu.orientation(OrientationType.PITCH, RotationUnits.DEG)
#         pid_output = round(pitch_pid.calculate(0, pitch), 3)

#         print(pid_output, elevationDistance.object_distance())

#         motors["left"]["A"].spin(REVERSE, 6 - pid_output, VOLT)
#         motors["left"]["B"].spin(REVERSE, 6 - pid_output, VOLT)
#         motors["left"]["C"].spin(REVERSE, 6 - pid_output, VOLT)

#         motors["right"]["A"].spin(REVERSE, 6 + pid_output, VOLT)
#         motors["right"]["B"].spin(REVERSE, 6 + pid_output, VOLT)
#         motors["right"]["C"].spin(REVERSE, 6 + pid_output, VOLT)

#         if elevationDistance.object_distance() > 115:
#             sleep(50, MSEC)
#             break

#         sleep(10)

#     stop_drivebase(BrakeType.HOLD)

controls["DOINKER"].pressed(switch_doinker)
controls["MOGO_GRABBER_TOGGLE"].pressed(switch_mogo)
# controls["AUTO_MOGO_ENGAGE_TOGGLE"].pressed(switch_mogo_engaged)
controls["INTAKE_HEIGHT_TOGGLE"].pressed(switch_intake_height)
controls["CYCLE_EJECTOR_COLOR"].pressed(cycle_ejector_color)
# if not enable_elevation_macro:
#     controls["MANUAL_ELEVATION_PNEUMATICS"].pressed(manual_elevation)
#     con.buttonLeft.pressed(toggle_tank)
# else:
#     controls["START_ELEVATE"].pressed(elevation_macro)

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
    global eject_prep, queued_sort, wall_control_cooldown, wall_setpoint
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
            brain.timer.event(intake_sorter, 210)

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
driver()
