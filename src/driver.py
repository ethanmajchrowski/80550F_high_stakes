# driver code

from vex import *

# Brain should be defined by default
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

distance_sensor = Distance(Ports.PORT16)

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

def switch_mogo():
    misc_devices["mogo_pneu"].set(not misc_devices["mogo_pneu"].value())

def switch_intake_height():
    misc_devices["intake_pneu"].set(not misc_devices["intake_pneu"].value())

def toggle_side_scoring():
    misc_devices["side_scoring_a"].set(not misc_devices["side_scoring_a"].value())
    misc_devices["side_scoring_b"].set(not misc_devices["side_scoring_b"].value())

controls["MOGO_GRABBER_TOGGLE"].pressed(switch_mogo)
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


    brain.screen.render()