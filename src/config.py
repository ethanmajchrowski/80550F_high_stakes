# Filename: config.py
# Devices & variables last updated:
	# 2024-12-11 17:11:57.554230
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
    "AUTO_SIDE_LOADER":    con.buttonL2,
    "CYCLE_EJECTOR_COLOR": con.buttonLeft,
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
    color_setting = data["config"]["initial_color_sorting"]
else:
    color_setting = "none"
#endregion Devices####################
#DO NOT CHANGE THE FOLLOWING LINE:#
#end_1301825#
####################

def button(text, x, y, width, height, align = "center"):
    string_width = brain.screen.get_string_width(text)
    string_height = brain.screen.get_string_height(text)
        
    brain.screen.draw_rectangle(x, y, width, height)
    if align == "center":
        brain.screen.print_at(text, x = (x + (width / 2)) - (string_width / 2), y = (y + (height * 0.5)) + string_height // 4)
    if align == "left":
        brain.screen.print_at(text, x = x+5, y = (y + (height * 0.5)) + string_height // 4)

class Config_GUI:
    def __init__(self) -> None:
        self.state = "main" # main, auton, thermals, options
        self.page = 0
        self.max_page = 0
        self.height = 40
        self.title = "Robot Configuration"
        self.nested_menu = False
        self.display_list = ["Autonomous", "Options", "Thermals"]

        # data = data = {
        #     "autons": {
        #         "selected": "none",
        #         "options": ["none", "right", "left", "2", "3"]
        #     },
        #     "config": {
        #         "boolean option1": True,
        #         "boolean option2": True,
        #         "boolean option3": True,
        #         "boolean option4": True,
        #         "multi options": {
        #             "selected": "1",
        #             "options": ["1", "2", "3"]
        #         }
        #     }
        # }
        # initial run to draw first frame
        self.run()

    def run(self):
        # File structure is:
        #   /autons/testing.txt
        #   /config.json
        scr = brain.screen

        # OS error 5?????
        f = open('cfg/config.json', 'r')
        data = load(f)
        f.close()
        
        self.selected_auton = data["autons"]["selected"]
        
        #######################################
        # All button logic
        x, y = scr.x_position(), scr.y_position()
        if self.nested_menu and (0 < x < 120) and (200 < y < 240):
            if self.page > 0:
                self.page -= 1
        if self.nested_menu and (360 < x < 480) and (200 < y < 240):
            if self.page < self.max_page:
                self.page += 1

        if self.nested_menu and (0 < y and y < 40 and 0 < x and x < 120):
            self.state = "main"
            self.title = "Robot Configuration"
            self.nested_menu = False
            self.display_list = ["Autonomous", "Options", "Thermals"]
            self.max_page = 0
            self.page = 0
        elif self.state == "main":
            if 40 < y and y < 80:
                # row 1
                self.state = "auton"
                self.nested_menu = True
                self.display_list = data["autons"]["options"]
                self.title = self.selected_auton

                self.max_page = len(self.display_list) // 4
                self.page = 0
            if 80 < y and y < 120:
                self.state = "options"
                self.nested_menu = True
                self.title = "Options"
                self.display_list = list(data["config"].keys())

                self.page = 0
                self.max_page = len(self.display_list) // 4
            
            if 120 < y and y < 160:
                self.state = "thermals"
                self.nested_menu = True
                self.title = "Thermals"
                # combine motors dict into one list
                self.display_list = []
                for key in motors.keys():
                    if type(motors[key]) == dict:
                        # print(motors[key].values())
                        for motor in list(motors[key].values()):
                            self.display_list.append((motor, key, motor.temperature(), motor.installed()))
                    else:
                        self.display_list.append(motors[key])
                # print(self.display_list)
                # # sort list by temperature (comment out for sorting by dictionary group)
                # self.display_list.sort(key=lambda motor: motor[2])
                self.page = 0
                self.max_page = len(self.display_list) // 4

        elif self.state == "auton":
            # button press
            if 40 < y and y < 200:
                # y press is within list
                pressed_auto = ((y // 40) - 1) + (self.page * 4)
                if pressed_auto < len(self.display_list):
                    selected = data["autons"]["options"][pressed_auto]
                    self.selected_auton, self.title, data["autons"]["selected"] = selected, selected, selected                  
        elif self.state == "options":
            # Options menu
            if 40 < y and y < 200:
                pressed_index = ((y // 40)-1)+(self.page*4)

                if pressed_index < len(self.display_list):
                    key = self.display_list[pressed_index]
                    key_type = type(data['config'][key])
                    if key_type == bool:
                        data['config'][key] = not data['config'][key]
                    if key_type == dict:
                        options_list = data['config'][key]['options']
                        selected_index = options_list.index(data['config'][key]['selected'])
                        
                        if selected_index == len(options_list) - 1:
                            new_index = 0
                        else:
                            new_index = selected_index + 1
                        
                        data['config'][key]['selected'] = options_list[new_index]

        scr.clear_screen()
        #######################################
        # Draw all buttons
        if self.nested_menu:
            # draw back button
            button("Back", 0, 0, 120, 40)
            button(self.title, 120, 0, 480-120, 40)
        else:
            button(self.title, 0, 0, 480, 40)
        
        # list section
        if len(self.display_list) < 4:
            l = self.display_list
        else:
            l = self.display_list[self.page*4:(self.page*4)+4]
        
        for i in range(min(len(self.display_list), 4)):
            # Index (number next to button)
            button(i+1+(self.page*4), 0, (i+1)*40, 40, 40)

            if self.state == "thermals":
                if i < len(l):
                    item = l[i] # Motor, main dictionary (left, right, misc), temp, Installed
                    if len(str(item[0])) == 9:
                        # motor is plugged into double digit port (10+)
                        port_string = str(item[0])[6:8]
                    else:
                        port_string = str(item[0])[6]
                        
                    if item[3]:
                        button("{}   {}   {}*C".format(port_string, item[1], item[2]), 40, (i+1)*40, 440, 40, align="left")
                    else:
                        button("{}   {}   UNINSTALLED".format(port_string, item[1]), 40, (i+1)*40, 440, 40, align="left")

            else:
                # Title
                if i < len(l):
                    button(l[i], 40, (i+1)*40, 440, 40, align="left")
                # If options, we want to show the current option
                if self.state == 'options':
                    try:
                        key = l[i]
                        key_type = type(data['config'][key])
                        if key_type == dict:
                            data_title = data['config'][key]['selected']
                        if key_type == bool:
                            data_title = data['config'][key]
                        
                        strw = brain.screen.get_string_width(data_title)
                        strw = max(strw, 40) # if strw is less than 40 set it to 40
                        button(data_title, 480-(strw+10), (i+1)*40, strw+10, 40)
                    except:
                        pass
        
        button("<", 0, 200, 120, 40)
        button("Page {} of {}".format(self.page+1, self.max_page+1), 120, 200, 240, 40)
        button(">", 480-120, 200, 120, 40)

        # save data
        with open("cfg/config.json", "w") as f:
            dump(data, f)

        scr.render()

if brain.sdcard.is_inserted():
    cgui = Config_GUI()
    brain.screen.pressed(cgui.run)
else:
    # warn the user that there is no SD card inserted
    brain.screen.set_fill_color(Color(255, 0, 0))
    brain.screen.draw_rectangle(0, 0, 480, 240)
    for i in range(4):
        brain.screen.set_font(FontType.PROP60)
        brain.screen.print("     NO SD CARD")
        brain.screen.new_line()