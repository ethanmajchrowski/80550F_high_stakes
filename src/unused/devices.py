# Use this file to define all devices. Copy & paste when updating 
# to other files
# TODO: maybe set this to automatically update all other files?

#####################################################
from vex import *

# region variables
brain = Brain()
con = Controller()

controls = {
    "DRIVE_FORWARD_AXIS":  con.axis3,
    "DRIVE_TURN_AXIS":     con.axis4,
    "MOGO_GRABBER_TOGGLE": con.buttonA,
    "INTAKE_IN_HOLD":      con.buttonR1,
    "INTAKE_OUT_HOLD":     con.buttonR2,
    "INTAKE_HEIGHT_TOGGLE":con.buttonL1,
    "SIDE_SCORING_TOGGLE": con.buttonB
}
motors = {
    "left": {
        "A": Motor(Ports.PORT12, GearSetting.RATIO_6_1), # stacked top
        "B": Motor(Ports.PORT11, GearSetting.RATIO_6_1, True), # stacked bottom
        "C": Motor(Ports.PORT15, GearSetting.RATIO_6_1, True), # front
        "D": Motor(Ports.PORT13, GearSetting.RATIO_18_1) # 5.5w
    },
    "right": {
        # D motor is 5.5w
        "A": Motor(Ports.PORT16, GearSetting.RATIO_6_1, True), # stacked top
        "B": Motor(Ports.PORT18, GearSetting.RATIO_6_1), # stacked bottom
        "C": Motor(Ports.PORT9, GearSetting.RATIO_6_1), # front
        "D": Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
    },
    "intake": Motor(Ports.PORT19, GearSetting.RATIO_6_1, True)
}
# wire_expander = Triport(Ports.PORT5)
# DigitalOut(wire_expander.a)
misc_devices = {
    # pneumatics
    "mogo_pneu": DigitalOut(brain.three_wire_port.c),
    "intake_pneu": DigitalOut(brain.three_wire_port.b),
    "side_scoring_a": DigitalOut(brain.three_wire_port.a), 
    "side_scoring_b": DigitalOut(brain.three_wire_port.d), 
}
# pneumatics
mogo_pneu = DigitalOut(brain.three_wire_port.c)
intake_pneu = DigitalOut(brain.three_wire_port.b)
side_scoring_a = DigitalOut(brain.three_wire_port.a)
side_scoring_b = DigitalOut(brain.three_wire_port.d)

leftEnc = motors["left"]["A"]
rightEnc = motors["right"]["A"]

imu = Inertial(Ports.PORT9)

imu.calibrate()
while imu.is_calibrating():
    wait(5)

# end variables
#####################################################