# Filename: driver.py
# Devices & variables last updated:
	# 2025-02-05 11:56:14.818664
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
    "LB_MANUAL_DOWN": con_2.buttonL2
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
        "intake_chain": Motor(Ports.PORT10, GearSetting.RATIO_6_1, True),  
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
wall_positions = [30, 125, 400, 600] # wall_setpoint is an INDEX used to grab from THIS LIST
LB_enable_PID = True

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

def elevation_macro():
    global LB_enable_PID
    enable_PTO = False
    print("start elevation")

    LB_enable_PID = False
    # mogo_pneu.set(True)
    # motors["misc"]["wall_stake"].stop()
    # motors["misc"]["wall_stake"].spin_for(FORWARD, 400, MSEC, 100, PERCENT)

    roll_pid = MultipurposePID(0.1, 0, 0, 0)

    # elevation_hook_release.set(True)
    # wait and close these pistons cause leak :(
    sleep(200, MSEC)
    # intake_pneu.set(False)
    # doinker_pneu.set(True)
    # elevation_hook_release.set(False)
    # elevation_bar_lift.set(True)

    # wait for matics
    sleep(100, MSEC)
    # motors["misc"]["wall_stake"].spin_for(REVERSE, 1200, MSEC, 100, PERCENT)
    sleep(200, MSEC)
    # elevation_bar_lift.set(False)
    # sleep(100, MSEC)

    controls2["DOINKER"].pressed(switch_doinker)
    controls2["PTO_TOGGLE"].pressed(toggle_PTO)
    controls2["ELEVATION_PRIMARY_PNEUMATICS"].pressed(manual_elevation)
    LB_braketype = BrakeType.COAST

    while True:
        # DRIVE MOTORS
        fwd_volt = (controls2["AXIS_FORWARDS"].position() / 100) * 12
        tilt_volt = (controls2["AXIS_TILT"].position() / 100) * 12
        # if abs(fwd_volt) > 1 or abs(tilt_volt) > 1:
        if abs(con_2.axis2.position()) > 1 or abs(con_2.axis3.position()) > 1:
            motors["left"]["A"].spin(FORWARD, (con_2.axis2.position() / 100) * 12, VOLT)
            motors["left"]["B"].spin(FORWARD, (con_2.axis2.position() / 100) * 12, VOLT)
            motors["left"]["C"].spin(FORWARD, (con_2.axis2.position() / 100) * 12, VOLT)
            # motors["left"]["A"].spin(FORWARD, fwd_volt + tilt_volt, VOLT)
            # motors["left"]["B"].spin(FORWARD, fwd_volt + tilt_volt, VOLT)
            # motors["left"]["C"].spin(FORWARD, fwd_volt + tilt_volt, VOLT)

            # motors["right"]["A"].spin(FORWARD, fwd_volt - tilt_volt, VOLT)
            # motors["right"]["B"].spin(FORWARD, fwd_volt - tilt_volt, VOLT)
            # motors["right"]["C"].spin(FORWARD, fwd_volt - tilt_volt, VOLT)
            motors["right"]["A"].spin(FORWARD, (con_2.axis3.position() / 100) * 12, VOLT)
            motors["right"]["B"].spin(FORWARD, (con_2.axis3.position() / 100) * 12, VOLT)
            motors["right"]["C"].spin(FORWARD, (con_2.axis3.position() / 100) * 12, VOLT)
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
            LB_braketype = BrakeType.HOLD
        elif con.buttonL2.pressing() or controls2["LB_MANUAL_UP"].pressing():
            motors["misc"]["wall_stake"].spin(FORWARD, 100, PERCENT)
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
        data = {
            "roll": round(roll, 2),
            "output": round(pid_output, 2),
            "height": round(elevationDistance.object_distance(), 2)
        }
        # payload_manager.send_data("elevation", data)
        sleep(35, MSEC)

    # while True:
    #     pitch = imu.orientation(OrientationType.PITCH, RotationUnits.DEG)

    #     print(pid_output, elevationDistance.object_distance())

    #     motors["left"]["A"].spin(REVERSE, 6 - pid_output, VOLT)
    #     motors["left"]["B"].spin(REVERSE, 6 - pid_output, VOLT)
    #     motors["left"]["C"].spin(REVERSE, 6 - pid_output, VOLT)

    #     motors["right"]["A"].spin(REVERSE, 6 + pid_output, VOLT)
    #     motors["right"]["B"].spin(REVERSE, 6 + pid_output, VOLT)
    #     motors["right"]["C"].spin(REVERSE, 6 + pid_output, VOLT)

    #     if elevationDistance.object_distance() > 115:
    #         sleep(50, MSEC)
    #         print("height reached")
    #         break

    #     sleep(10)

    # stop_drivebase(BrakeType.HOLD)

def home_lady_brown_PID():
    print("run home_lady_brown_PID")
    global wall_setpoint, LB_enable_PID
    LB_enable_PID = True
    wall_setpoint = 1

controls["DOINKER"].pressed(switch_doinker)
controls["MOGO_GRABBER_TOGGLE"].pressed(switch_mogo)
# controls["AUTO_MOGO_ENGAGE_TOGGLE"].pressed(switch_mogo_engaged)
controls["INTAKE_HEIGHT_TOGGLE"].pressed(switch_intake_height)
controls["LB_MACRO_HOME"].pressed(home_lady_brown_PID)

# if not enable_elevation_macro:
#     controls["MANUAL_ELEVATION_PNEUMATICS"].pressed(manual_elevation)
#     con.buttonLeft.pressed(toggle_tank)

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
        else:
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
        if con.buttonUp.pressing() and not elevating:
            print(elevation_hold_duration)
            elevation_hold_duration -= 1
            if elevation_hold_duration <= 0:
                elevating = True
                elevation_macro()
        else:
            elevation_hold_duration = 5

        brain.screen.render()

#endregion driver

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

payload_manager = PayloadManager()

# def logging_thread():
#     while True:
#         data = {
#             "roll": round(imu.orientation(OrientationType.PITCH), 2)
#         }
#         payload_manager.send_data("elevation", data)

#         sleep(35, MSEC)

# def log(msg: str):
#     data = {
#         "level": 2,
#         "message": msg,
#         "name": "main",
#         "file": "main",
#         "line": 1
#     }
#     payload_manager.send_data("log", data)

# Thread(logging_thread)

# log("starting driver")

driver()