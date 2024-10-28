# Filename: auton.py
# Devices & variables last updated:
	# 2024-10-28 16:26:09.614366
####################
#region Devices
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

#region Helpers
# ██   ██ ███████ ██      ██████  ███████ ██████  ███████ 
# ██   ██ ██      ██      ██   ██ ██      ██   ██ ██      
# ███████ █████   ██      ██████  █████   ██████  ███████ 
# ██   ██ ██      ██      ██      ██      ██   ██      ██ 
# ██   ██ ███████ ███████ ██      ███████ ██   ██ ███████ 

def dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))
#endregion Helpers

class Logger:
    def __init__(self) -> None:
        if brain.sdcard.is_inserted():
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
            # write the heading line to the file - this needs to be different for all the data we collect
            # f.write("time, L1 TEMP, L2 TEMP, L3 TEMP, R1 TEMP, R2 TEMP, R3 TEMP")
            # f.write("time, L1 VOLT, L2 VOLT, L3 VOLT, R1 VOLT, R2 VOLT, R3 VOLT")
            f.write("time, x, y, heading, heading_to_target, heading_pid, target_x, target_y")
            f.close()
        else:
            print("No SD card inserted!")

    def log(self, data):
        if brain.sdcard.is_inserted():
            file = open("data/latest.txt", "a")
            file.write("\n" + data)
            file.close()
        else:
            print("No SD card inserted")

class PurePursuit():
    def __init__(self, look_ahead_dist, finish_margin, 
                 path: list[tuple[float, float]], checkpoints) -> None:
        """
        Init variables. Robot position is able to be passed in from another source, like a GPS sensor or odometry algorithm.
        Arguments:
            path (list[tuple[float, float]])

        """
        self.path = path
        self.look_dist = look_ahead_dist
        self.finish_margin = finish_margin
        self.last_found_point = 0

        self.path_complete = False

        self.checkpoints = checkpoints
        if len(self.checkpoints) != 0:
            self.current_checkpoint = checkpoints[0]
            self.checkpoints_complete = False
        else:
            self.checkpoints_complete = True

    def goal_search(self, current_pos):
        """
        Run every time you want to update your goal position. 
        Returns the point along our path that is look_dist away from the robot
        and that is closest to the end of the path.
        """
        goal = self.path[self.last_found_point+1][:2]

        # Iterate over every un-crossed point in our path.
        #if we are close to the finish point, regardless of what has happened, finish the path
        if dist(current_pos, self.path[len(self.path)-1]) < self.finish_margin:
            self.path_complete = True      
        else:
            # Iterate from our current point to the next checkpoint, or the end of the path.
            # Once we reach that checkpoint, move the checkpoint to the next and keep going.
            if self.checkpoints_complete:
                end = len(self.path)-1
            else:
                end = self.current_checkpoint
            
            start = self.last_found_point
            
            for i in range(start, end):
                # step 1: line and circle intersection
                h, k = current_pos
                point1 = self.path[i][:2]
                point2 = self.path[i+1][:2]
                ax, ay = point1[:2]
                bx, by = point2[:2]
                
                m = (by - ay) / (bx - ax) # slope of line between point a and b
                b = m*(-ax) + ay # y-intercept of line
                r = self.look_dist
                # quadratic terms
                A = (m*m) + 1
                B = (2*m*(b-k)-(2*h))
                C = ((h*h) + ((b-k)*(b-k)) - (r*r))

                discriminant = (B*B) - (4*A*C)
                if discriminant >= 0:
                    sol1_x = (-B + math.sqrt(discriminant)) / (2*A)
                    sol1_y = m*sol1_x + b

                    sol2_x = (-B - math.sqrt(discriminant)) / (2*A)
                    sol2_y = m*sol2_x + b

                    sol1 = (sol1_x, sol1_y)
                    sol2 = (sol2_x, sol2_y)

                    minX, minY = min(ax, bx), min(ay, by)
                    maxX, maxY = max(ax, bx), max(ay, by)
                    # general check to see if either point is on the line 
                    if ((minX < sol1_x < maxX) and (minY < sol1_y < maxY)) or ((minX < sol2_x < maxX) and (minY < sol2_y < maxY)):
                        if ((minX < sol1_x < maxX) and (minY < sol1_y < maxY)) and ((minX < sol2_x < maxX) and (minY < sol2_y < maxY)):
                            # both solutions are within bounds, so we need to compare and decide which is better
                            # choose based on distance to pt2
                            sol1_distance = dist(sol1, point2)
                            sol2_distance = dist(sol2, point2)

                            if sol1_distance < sol2_distance:
                                goal = sol1
                            else:
                                goal = sol2
                        else:
                            if (minX < sol1_x < maxX) and (minY < sol1_y < maxY):
                                # solution 1 is within bounds
                                goal = sol1
                            else:
                                goal = sol2
                        
                    # first, check if the robot is not close to the end point in the path
                    if dist(current_pos, self.path[len(self.path)-1]) < self.look_dist:
                        goal = self.path[len(self.path)-1]
                    else:
                        # update last_found_point
                        # only keep the goal if the goal point is closer to the target than our robot
                        if dist(goal[:2], self.path[self.last_found_point+1][:2]) < dist(current_pos, self.path[self.last_found_point+1][:2]):
                            # found point is closer to the target than we are, so we keep it
                            goal = goal
                        else:
                            self.last_found_point = i 

                if not self.checkpoints_complete:
                    if (self.last_found_point + 2 >= self.current_checkpoint):
                        # If we are done with our checkpoints,
                        if (self.checkpoints.index(self.current_checkpoint)+1) >= len(self.checkpoints):
                            self.checkpoints_complete = True
                        else:
                            # Set the current checkpoint to the next checkpoint in the checkpoints list
                            self.current_checkpoint = self.checkpoints[self.checkpoints.index(self.current_checkpoint) + 1]
       
        return goal

class DeltaPositioning():
    def __init__(self, leftEnc, rightEnc, imu) -> None:
        self.last_time = brain.timer.time()
        self.leftEnc = leftEnc
        self.rightEnc = rightEnc
        self.imu = imu

        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.last_heading = 0

        # formula:
        # wheel diamter * pi
        # convert to mm
        # multiply by motor to wheel ratio
        self.circumference = 219.46 * (3/4)

    def update(self):
        # Call constantly to update position.

        # change in left & right encoders (degrees)
        dl = self.leftEnc.position() - self.last_left_encoder
        dr = self.rightEnc.position() - self.last_right_encoder
        # dHeadingTheta = self.imu.rotation() - self.last_heading

        # proportion
        # (dTheta / 360) = (x mm / Circumference mm)
        # x mm = (dTheta / 360) * Circumference
        dl = (dl / 360) * self.circumference
        dr = (dr / 360) * self.circumference

        # average the position of left & right to get the center of the robot
        dNet = (dl + dr) / 2 

        # x = cos
        # y = sin
        dx = dNet * math.sin(math.radians(self.imu.heading()))
        dy = dNet * math.cos(math.radians(self.imu.heading()))

        self.last_time = brain.timer.time()
        self.last_heading = self.imu.heading()
        self.last_right_encoder = self.rightEnc.position()
        self.last_left_encoder = self.leftEnc.position()
        return [dx, dy]

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

#endregion Helper Classes

class AutonomousHandler:
    def __init__(self, selected_filename) -> None:
        ### Load paths
        # DO NOT turn the brain off when this is running! This may corrupt the files :)
        self.autonomous = self.read_auto("autons/{}.txt".format(selected_filename))

        ### Pathing
        # Reference the object so we can create new handlers for new paths
        self.path_controller = PurePursuit

        ### Global positioning
        ## Position controller
        self.position_controller = DeltaPositioning(leftEnc, rightEnc, imu)
        ## Position data
        self.position = list(self.autonomous["setup"][0])
        imu.set_heading(self.autonomous["setup"][1])

        ### Logging
        # deal with this later :)

        ### Variables
        self.dynamic_vars = {
            "fwd_speed": int(12*0.8)
        }
    
    # decent settings: 
    """
    speed: 65%
    self, path, events, checkpoints=[], backwards = False,
    look_ahead_dist=250, finish_margin=100, event_look_ahead_dist=75,
    heading_authority=1, max_turn_volts = 5,
    hPID_KP = 0.1, hPID_KD = 0.01, hPID_KI = 0, hPID_KI_MAX = 0, hPID_MIN_OUT = None,"""
    def position_thread(self):
        while True:
            self.heading = imu.heading()
            dx, dy = self.position_controller.update()
            self.position[0] += dx
            self.position[1] += dy

            wait(20, MSEC)

    def run(self) -> None:
        """
        Call once at start of auton. This is where all the sequential commands are located.
        """
        # Start our positioning thread
        Thread(self.position_thread)

        for command in self.autonomous["sequence"]:
            if callable(command[0]):
                # function, args
                # "path" is only there if it is a path!
                # rfs.append((self.path, (points, ev, chck, args), "path"))
                if len(command) == 3:
                    # path, events, checkpoints, unpack custom settings
                    self.path(command[1][0], command[1][1], command[1][2], *command[1][3])
                else:
                    command[0](*command[1])
            else:
                if command[0] is None:
                    # This is a variable under self.dynamic vars
                    self.dynamic_vars[command[1]] = command[2]
                else:
                    # This is a variable of some other class.
                    setattr(command[0], command[1], command[2])

    def read_auto(self, fp):
        # Setup everything we need
        output = dict()

        with open(fp) as f:
            data = f.readlines()
        
        nd = []
        for line in data:
            nl = line.strip()
            nd.append(nl)
        data = nd
        
        section = 0

        setup = []
        sequence = []
        paths_n = []

        t = []
        # Split data into sections
        for line in data:
            t.append(line)
            if line == "":
                # t[1:-1] only gives us the content: title blocks and line separators are ignored
                if section == 1: setup = t[1:-1]
                if section == 2: sequence = t[1:-1]
                if section == 3: paths_n = t[1:-1]
                
                section += 1
                t = []

        # Setup data
        for line in setup:
            split = line.split(" ")
            if split[0] == "at":
                pos = (float(split[1]), float(split[2]))
            if split[0] == "towards":
                heading = float(split[1])
        output["setup"] = (pos, heading)
        
        # Path data
        # Due to the format of the path (name, points, events, checkpoints) being each on a new line, we have to grab 
        # it all at once to get it in one item per path
        nps = []
        for line in paths_n:
            i = paths_n.index(line)
            if type(eval(line)) == str:
                # grabs from name to custom arguments
                nps.append(paths_n[i:i+5])
        paths = nps

        # Now we have to convert the data within each path into something valid.
        # Since it should be valid python code, we can just use eval()
        nps = []
        for path in paths:
            np = []
            for line in path:
                np.append(eval(line))
            nps.append(np)
        paths = nps

        # At this point, we have our paths in one item each, and the data in the path is valid.

        # path[0] is the name of the path
        # path[1] is the path point data
        # path[2] is the events list
        # path[3] is the checkpoints for the path
        # Convert to dictionary for simpler grabbing
        pd = dict()
        for path in paths:
            # Remove the name so that when we set the dictionary value it doesn't have a duplicate of the name
            name = path.pop(0)
            # print(path[2])
            pd[name] = {
                "points": path[0],
                "events": path[1],
                "checkpoints": path[2],
                "custom_settings": path[3]
            }
        output["paths"] = pd
        # print(output["paths"]["horseshoe"]["points"])

        # Sequence data
        rfs = []
        for line in sequence:
            split = line.split(" ")

            if split[0] == "call":
                args = eval(split[3])  
                func = eval(split[1])
                rfs.append((func, args))          

            if split[0] == "set":
                # set class name value
                if split[1] == "var":
                    # this is a "generic" variable within vars_dict
                    name = split[2]
                    value = eval(split[3])
                    rfs.append((None, name, value))
                else:
                    # this is under some sort of class
                    var_class = eval(split[1])
                    name = split[2]
                    # print(getattr(var_class, name))
                    value = eval(split[3])
                    rfs.append((var_class, name, value))
            
            if split[0] == "path":
                p = output["paths"][split[1]]
                points = p["points"]
                ev = p["events"]
                chck = p["checkpoints"]
                args = p["custom_settings"]

                command = (self.path, (points, ev, chck, args), "path")
                rfs.append(command)
                # Need to unpack args LATER
                # rfs.append(tuple((self.path, tuple((points, ev, chck, *args)))))

        output["sequence"] = tuple(rfs)
        
        return output

    def kill_motors(self, brake_type=BrakeType.BRAKE):
        motors["left"]["A"].stop(brake_type)
        motors["left"]["B"].stop(brake_type)
        motors["left"]["C"].stop(brake_type)
        motors["left"]["D"].stop(brake_type)

        motors["right"]["A"].stop(brake_type)
        motors["right"]["B"].stop(brake_type)
        motors["right"]["C"].stop(brake_type)
        motors["right"]["D"].stop(brake_type)

    def path(self, path, events, checkpoints=[], backwards = False,
             look_ahead_dist=350, finish_margin=100, event_look_ahead_dist=75,
             heading_authority=1, max_turn_volts = 8,
             hPID_KP = 0.1, hPID_KD = 0.01, hPID_KI = 0, hPID_KI_MAX = 0, hPID_MIN_OUT = None,) -> None:
        
        path_handler = self.path_controller(look_ahead_dist, finish_margin, path, checkpoints)
        heading_pid = MultipurposePID(hPID_KP, hPID_KD, hPID_KI, hPID_KI_MAX, hPID_MIN_OUT)

        done = path_handler.path_complete
        waiting = False
        wait_stope = 0

        while not done:
            target_point = path_handler.goal_search(self.position)
            dx, dy = target_point[0] - self.position[0], target_point[1] - self.position[1] # type: ignore
            heading_to_target = math.degrees(math.atan2(dx, dy))

            if dx < 0:
                heading_to_target = 360 + heading_to_target

            # logic to handle the edge case of the target heading rolling over
            heading_error = self.heading - heading_to_target
            rollover = False

            if heading_error > 180:
                heading_error = 360 - heading_error
                rollover = True
            if heading_error < -180:
                heading_error = -360 - heading_error
                rollover = True

            # PID to drive heading_error to 0
            if backwards:
                if heading_error > 180:
                    heading_error -= 180
                else:
                    heading_error += 180
            heading_output = heading_pid.calculate(0, heading_error)

            # Grab constant speed from dynamic variables
            constant_forwards_speed = self.dynamic_vars["fwd_speed"]
            if backwards:
                constant_forwards_speed *= -1
                # heading_output *= -1

            # Do some stuff that lowers the authority of turning, no idea if this is reasonable
            heading_output *= heading_authority
            if heading_output > max_turn_volts: heading_output = max_turn_volts
            if heading_output < -max_turn_volts: heading_output = -max_turn_volts
            # if we rollover, fix it
            if rollover:
                heading_output *= -1
            if not waiting:
                motors["left"]["A"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
                motors["left"]["B"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
                motors["left"]["C"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)
                motors["left"]["D"].spin(FORWARD, constant_forwards_speed + heading_output, VOLT)

                motors["right"]["A"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
                motors["right"]["B"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
                motors["right"]["C"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
                motors["right"]["D"].spin(FORWARD, constant_forwards_speed - heading_output, VOLT)
            else:
                if brain.timer.system() >= wait_stop:
                    waiting = False

            for event in events:
                if dist(self.position, event[1]) < event_look_ahead_dist:
                    if type(event[2]) == str:
                        if event[2] == "wait_function":
                            if not event[4]:
                                # This tells us to wait
                                # format: ["description", (x, y), EventWaitType(), duration, completed]
                                waiting = True
                                wait_stop = brain.timer.system() + event[3]

                                # make sure we dont get stuck in a waiting loop
                                event[4] = True

                                self.kill_motors()
                        else:
                            # this is a variable change
                            # format: ["speed down", (0, 1130), "speed", 3.5]
                            self.dynamic_vars[event[2]] = event[3]
                    elif callable(event[2]):
                        # Call the function (at index 2) with the unpacked (*) args (at index 3)
                        # ["intake", (1200, 0), motors["intake"].spin, (FORWARD, 50)],
                        event[2](*event[3])
            done = path_handler.path_complete

            if not path_handler.path_complete:
                # brain.timer.event(self.run, self.clock_time)
                sleep(20, MSEC)
            else:
                self.kill_motors()

with open("cfg/config.json", 'r') as f:
    data = load(f)

auton = AutonomousHandler(data["selected_auton"])
auton.run()

# comp = Competition(driver, auton.run)