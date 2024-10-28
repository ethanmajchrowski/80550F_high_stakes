# Filename: driver.py
# Devices & variables last updated:
	# 2024-10-28 10:34:42.436638
####################
#region Devices
from vex import *

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
    "ELEVATION_RELEASE_1": con.buttonDown,
    "ELEVATION_RELEASE_2": con.buttonLeft,
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
elevation_pneu = DigitalOut(brain.three_wire_port.e)

# wire_expander = Triport(Ports.PORT5)
# DigitalOut(wire_expander.a)

# SENSORS
leftEnc = motors["left"]["A"]
rightEnc = motors["right"]["A"]
leftDistance = Distance(Ports.PORT14)
rightDistance = Distance(Ports.PORT17)

imu = Inertial(Ports.PORT9)

imu.calibrate()
while imu.is_calibrating(): 
    wait(5)

mogo_pneu_engaged = False
mogo_pneu_status = False

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
#endregion Devices####################
#DO NOT CHANGE THE FOLLOWING LINE:#
#end_1301825#
####################

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

def switch_mogo_engaged():
    global mogo_pneu_engaged
    mogo_pneu_engaged = not mogo_pneu_engaged

def switch_mogo():
    global mogo_pneu_engaged
    if mogo_pneu.value() == 1 and mogo_pneu_engaged:
        mogo_pneu_engaged = False

    mogo_pneu.set(not mogo_pneu.value())

def switch_intake_height():
    intake_pneu.set(not intake_pneu.value())

def toggle_side_scoring():
    side_scoring_a.set(not side_scoring_a.value())
    side_scoring_b.set(not side_scoring_b.value())

controls["MOGO_GRABBER_TOGGLE"].pressed(switch_mogo)
controls["AUTO_MOGO_ENGAGE_TOGGLE"].pressed(switch_mogo_engaged)
controls["INTAKE_HEIGHT_TOGGLE"].pressed(switch_intake_height)
controls["SIDE_SCORING_TOGGLE"].pressed(toggle_side_scoring)

while True:
    brain.screen.clear_screen()

    # Movement controls
    turnVolts = (controls["DRIVE_TURN_AXIS"].position() * 0.12) * 0.9
    forwardVolts = controls["DRIVE_FORWARD_AXIS"].position() * 0.12

    # Spin motors and combine controller axes
    motors["left"]["A"].spin(FORWARD, forwardVolts + turnVolts, VOLT)
    motors["left"]["B"].spin(FORWARD, forwardVolts + turnVolts, VOLT)
    motors["left"]["C"].spin(FORWARD, forwardVolts + turnVolts, VOLT)
    motors["left"]["D"].spin(FORWARD, forwardVolts + turnVolts, VOLT)
    # leftMotorC.spin(FORWARD, forwardVolts + turnVolts, VOLT)
    motors["right"]["A"].spin(FORWARD, forwardVolts - turnVolts, VOLT)
    motors["right"]["B"].spin(FORWARD, forwardVolts - turnVolts, VOLT)
    motors["right"]["C"].spin(FORWARD, forwardVolts - turnVolts, VOLT)
    motors["right"]["D"].spin(FORWARD, forwardVolts - turnVolts, VOLT)

    # Intake Controls
    if controls["INTAKE_IN_HOLD"].pressing():
        motors["intake"].spin(FORWARD, 100, PERCENT)
    elif controls["INTAKE_OUT_HOLD"].pressing():
        motors["intake"].spin(REVERSE, 100, PERCENT)
    else:
        motors["intake"].stop()
    
    # Pneumatic Hold Controls

    # Grabber sensors
    if mogo_pneu_engaged == True:
        if leftDistance.object_distance() < 50 and rightDistance.object_distance() < 50:
            mogo_pneu.set(True)

    # Screen debugging
    scr = brain.screen
    scr.clear_screen()
    scr.set_cursor(1,1)

    scr.print("Left distance: {}".format(leftDistance.object_distance()))
    scr.next_row()
    scr.print("Right distance: {}".format(rightDistance.object_distance()))
    scr.next_row()

    scr = con.screen
    scr.clear_screen()
    scr.set_cursor(1,1)

    if mogo_pneu_engaged: scr.print("MOGO ENGAGED")

    brain.screen.render()