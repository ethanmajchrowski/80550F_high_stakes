# Filename: test_files.py
# Devices & variables last updated:
	# 2024-10-01 12:27:39.097644
####################
from vex import *

# region variables
brain = Brain()
con = Controller()

controls = {
    "DRIVE_FORWARD_AXIS":  con.axis3,
    "DRIVE_TURN_AXIS":     con.axis4,
    "INTAKE_IN_HOLD":      con.buttonR1,
    "INTAKE_OUT_HOLD":     con.buttonR2,
    "INTAKE_HEIGHT_TOGGLE":con.buttonL1,
    "SIDE_SCORING_TOGGLE": con.buttonB,
    "MOGO_GRABBER_TOGGLE": con.buttonA,
    "AUTO_MOGO_ENGAGE_TOGGLE": con.buttonY,
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

# PNEUMATICS
mogo_pneu = DigitalOut(brain.three_wire_port.c)
mogo_pneu.set(1)
intake_pneu = DigitalOut(brain.three_wire_port.b)
side_scoring_a = DigitalOut(brain.three_wire_port.a)
side_scoring_b = DigitalOut(brain.three_wire_port.d)

# wire_expander = Triport(Ports.PORT5)
# DigitalOut(wire_expander.a)

# SENSORS
leftEnc = motors["left"]["A"]
rightEnc = motors["right"]["A"]
leftDistance = Distance(Ports.PORT14)
rightDistance = Distance(Ports.PORT17)

imu = Inertial(Ports.PORT9)

imu.calibrate()
while imu.is_calibrating(): wait(5)

mogo_pneu_engaged = False
mogo_pneu_status = False
####################
#DO NOT CHANGE THE FOLLOWING LINE:#
#end_1301825

print(1)

print(2)

for i in range(5):
    print(i)