# Filename: comp.py
# Devices & variables last updated:
	# 2025-01-31 14:05:11.676754
####################
#region Devices
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
    
    time_abs_ms = brain.timer.system()
    time_s = time_abs_ms % 60000 // 1000
    time_min = time_abs_ms // 60000
    time_ms = time_abs_ms - (time_s * 1000) + (time_min * 60000)

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
    if foxglove_log:
        packet_mgr.add_packet("foxglove.Log", {"timestamp": {"sec": time_s, "nsec": time_ms * 1000000}, "message": msg, "level": foxglove_logLevel, "name": "main", "file": "main", "line": 0})
    else:
        print("[{}] {}: {}".format(tag, time_abs_ms, msg))

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
    LB_MACRO_HOME =                 con.buttonDown
    BACK_OFF_ALLIANCE_STAKE =       con.buttonX
    LB_MACRO_DESCORE =              con.buttonUp

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
    leftA = Motor( Ports.PORT14, GearSetting.RATIO_6_1, False) # stacked top
    leftB = Motor(Ports.PORT15, GearSetting.RATIO_6_1, False) # rear
    leftC = Motor( Ports.PORT16, GearSetting.RATIO_6_1, False) # front

    rightA = Motor(Ports.PORT11, GearSetting.RATIO_6_1, True) # stacked top
    rightB = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True) # rear
    rightC = Motor(Ports.PORT13, GearSetting.RATIO_6_1, True) # front

    intakeChain = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
    intakeFlex = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
    ladyBrown = Motor(Ports.PORT18, GearSetting.RATIO_36_1,  False)

# PNEUMATICS
class pneumatic():
    mogo = DigitalOut(brain.three_wire_port.e)
    doinker_left = DigitalOut(brain.three_wire_port.f)
    doinker = DigitalOut(brain.three_wire_port.h)
    intake = DigitalOut(brain.three_wire_port.g)

#### SENSORS
# ENCODERS
class sensor():
    groundEncoder = Rotation(Ports.PORT2)

    wallEncoder = Rotation(Ports.PORT17)
    # DISTANCE SENSORS
    intakeDistance = Distance(Ports.PORT21)

    wallLeftDistance = Distance(Ports.PORT6)
    wallBackDistance = Distance(Ports.PORT1)
    wallFrontDistance = Distance(Ports.PORT9)
    wallRightDistance = Distance(Ports.PORT19)

    # elevationDistance = Distance(Ports.PORT20) # unplugged

    # MISC SENSORS
    intakeColor = Optical(Ports.PORT4)
    imu = Inertial(Ports.PORT3)

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
#endregion Devices####################
#DO NOT CHANGE THE FOLLOWING LINE:#
#nada you dont get to mess up my refactor#
####################

# global functions (no I/O)
def calibrate_lady_brown():
    log("calibrating lady brown")

    # motor.ladyBrown.spin_for(REVERSE, 1000, MSEC, 100, PERCENT)
    sensor.wallEncoder.set_position(0)

    log("lady brown calibration complete")

def calibrate_imu():
    log("calibrating IMU. Minimum calibration time 2.5 seconds.")
    min_duration = brain.timer.system() + 2500

    sensor.imu.calibrate()
    while brain.timer.system() < min_duration:
        sleep(10, TimeUnits.MSEC)

    log("IMU calibration complete")

def dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

def gauss(mu = 0, sigma = 1):
    u1 = random()
    u2 = random()

    z = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
    return mu + sigma * z

def sign(num):
    if num == 0: return 0
    if num < 0: return -1
    else: return 1
# classes

class PurePursuit():
    def __init__(self, look_ahead_dist, finish_margin, 
                 path: list[tuple[float, float, float]], checkpoints) -> None:
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
            log("Setup path with checkpoints")
            self.checkpoint_index = 0
            self.current_checkpoint = checkpoints[self.checkpoint_index]
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
        curvature = 0.0

        # Iterate over every un-crossed point in our path.
        #if we are close to the finish point, regardless of what has happened, finish the path
        try:
            if dist(current_pos, self.path[len(self.path)-1][:2]) < self.finish_margin:
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
                            sol1_distance = dist(sol1, point2)
                            sol2_distance = dist(sol2, point2)

                            if ((minX < sol1_x < maxX) and (minY < sol1_y < maxY)) and ((minX < sol2_x < maxX) and (minY < sol2_y < maxY)):
                                # both solutions are within bounds, so we need to compare and decide which is better
                                # choose based on distance to pt2

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
                            
                            # we have a goal solution at this point in time
                            # we can now get that solutions completion of the line segment
                            # this lets us linearly interpolate between the curvature of point[i] and point[i+1]
                            segment_length = dist(self.path[i][:2], self.path[i+1][:2])
                            if goal == sol1: completed_length = sol1_distance
                            else: completed_length = sol2_distance

                            percent = (1 - (completed_length/segment_length))
                            if len(self.path[i]) >= 3: # we check this to make sure it works with a legacy path
                                try:
                                    curvature = (self.path[i][2] * (1 - percent) + self.path[i+1][2] * percent)
                                except:
                                    curvature = 0.0
                            
                        # first, check if the robot is not close to the end point in the path
                        distance_to_end = dist(current_pos, self.path[len(self.path)-1][:2])
                        if (distance_to_end < self.look_dist) and self.checkpoints_complete:
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
                        if (self.last_found_point + 5 >= self.current_checkpoint):
                            log("Checkpoint {} reached!".format(self.checkpoint_index))
                            # If we are done with our checkpoints,
                            if (self.checkpoint_index+1) >= len(self.checkpoints):
                                self.checkpoints_complete = True
                                log("Path done with checkpoints")
                            else:
                                # Set the current checkpoint to the next checkpoint in the checkpoints list
                                log("Path next checkpoint")
                                self.checkpoint_index += 1
                                self.current_checkpoint = self.checkpoints[self.checkpoint_index]
        except ZeroDivisionError:
            log("Pather tried to divide by zero", LogLevel.FATAL)
    
        # data = {
        #     "ix" : self.last_found_point,
        #     "ip": goal
        #     # "look_ahead": path_handler.look_dist
        # }
        # packet_mgr.add_packet("pix", data)
        # print("Last: {}. Target: {}, {}".format(self.last_found_point, goal[0], goal[1]))

        return goal[:2], curvature

class DeltaPositioning():
    def __init__(self, enc: Rotation | Motor, imu: Inertial) -> None:
        """Odometry based on tracking wheels."""
        self.last_time = brain.timer.time()
        self.enc = enc

        self.enc.reset_position()

        self.imu = imu

        self.last_encoder = 0
        self.last_heading = 0

        # formula:
        # wheel diamter * pi
        # convert to mm
        # multiply by motor to wheel ratio
        # self.circumference = 219.46 * (3/4)
        external_gear_ratio = 1 # fraction
        wheel_diameter = 2 # inches
        self.circumference = (wheel_diameter * 3.14159) * 25.4 * external_gear_ratio

        drift_dist = 80
        self.drift_circumference = 2 * 3.14159 * drift_dist
        self.drift_accum = 0

    def update(self) -> list[float]:
        """Call constantly to update position."""

        h = self.imu.heading()
        h_rad = math.radians(h)

        # change in left & right encoders (degrees)
        d = self.enc.position() - self.last_encoder
        # change in heading
        dh = h - self.last_heading

        # get change in encode wheel in distance
        d = (d / 360) * self.circumference

        # average the position of left & right to get the center of the robot
        # dNet = (dl + dr) / 2 
        dNet = d

        dx = (dNet * math.sin(h_rad))# + drift_x
        dy = (dNet * math.cos(h_rad))# + drift_y

        # print(round(dh, 2), round(dl, 2), round(dr, 2), round(d_drift, 2))

        self.last_time = brain.timer.time()
        self.last_heading = self.imu.heading()
        self.last_encoder = self.enc.position()
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
        self.integral_processed = 0
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
            self.integral += self.error

            if self.integral > 0:
                self.integral = min(self.integral, 5 / self.kI)
            else:
                self.integral = max(self.integral, 5 / self.kI * -1)
            self.integral_processed = self.integral * self.kI

            if self.kI_max is not None:
                if self.integral_processed > 0:
                    self.integral_processed = min(self.integral_processed, self.kI_max)
                else:
                    self.integral_processed = max(self.integral_processed, self.kI_max * -1)

        self.derivative = (self.error - self.last_error) * self.kD
        
        output = (self.error * self.kP) + self.integral_processed + self.derivative
        if self.minimum is not None:
            if abs(output) < self.minimum:
                if self.error < 0: output = -self.minimum
                else: output = self.minimum
        
        self.last_error = self.error
        wait(DELAY, MSEC)
        return output
    
class Laser:
    def __init__(self, sensor: Distance, offset) -> None:
        """
        Create a MCL laser with offsets for build location.

        Arguments:
        x_offset: offset from the forwards/backwards axis of the robot. forwards on the robot is positive
        y_offset: offset from the center of the robot left/right. right on the robot is positive.
        distance_offset: distance LOCAL to the sensor forwards/backwards. Away from the center is positive.
        """
        self.offset = offset
        self.sensor = sensor
        # self.forward_offset = distance_offset

        # self.distance = math.sqrt((x_offset**2) + (y_offset**2))
        # self.angle = math.degrees(math.atan2(y_offset, x_offset))
    
    def get_distance(self):
        return self.sensor.object_distance() + self.offset

class MCL:
    def __init__(self):
        self.num_lasers = 4

        self.field_size = 3600
        self.num_particles = 100
        self.avg_score = 0
        self.avg_pos = [0, 0]

        self.simulation = set()

    def update_avg_position(self):
        """Returns the average position of all the points in the simulation set."""
        avg_x, avg_y = self.avg_pos
        for point in self.simulation:
            #* linear average
            # avg_x = (avg_x + point[1][0]) / 2
            # avg_y = (avg_y + point[1][1]) / 2

            #! weighted average based on score ( not working?)
            # if point[0] > 2000: continue
            # print(point)
            if self.avg_score:
                weight = point[0] / self.avg_score * 1.8
            else:
                weight = 0

            weight = round(weight, 3) * 5
            if 1-weight < 0:
                weight = 1
            
            # print(weight)

            avg_x = (avg_x * (weight)) + (point[1][0] * (1-weight))
            avg_y = (avg_y * (weight)) + (point[1][1] * (1-weight))
            # print(avg_x, avg_y, weight, 1-weight)
        
        self.avg_pos = [avg_x, avg_y]
        return((avg_x, avg_y))

    def redistribute(self, angle, target, lasers, clear = True, count = None):
        """
        Used to continue the algorithm. Resets the current simulation and evenly distributes the points across the map.
        """
        if count is None:
            count = self.num_particles
        
        if clear:
            self.simulation = set()
        num = int(math.sqrt(count))
        dist = self.field_size / num

        for x in range(num):
            for y in range(num):
                pos = (int(dist*x - 1800 + 5), int(dist*y - 1800 + 5))

                self.simulate_robot(pos, angle, target, lasers)

    def simulate_robot(self, pos: list | tuple, angle, target: list[int], lasers: list):
        """
        Returns the error score of a simulated robot at the given pos / angle.
        Compares to the given target lasers, a list of 4 integers.
        If a laser is over 2.5m, it should be handled as None.
        Target list of lasers should move counter clockwise starting at 0*, and have the same length as num_lasers
        """
        score = 0
        for i in range(self.num_lasers):
            corrected_angle = (angle + i*(360//self.num_lasers) + 90) % 360
            dist = self.simulate_laser(pos, corrected_angle, lasers[i])
            # None checking (if a sim laser is out of range, the same real lasers should also be out of range)
            if target[i] is None:
                if dist > 2500:
                    # this means that our found distance is correct (out of range, so keep going)
                    continue 
                else:
                    # one of our points is technically within range, but it should be none
                    # return
                    score = 9000
                    break

            # linear error from the target
            error = abs(target[i] - dist)
            score += error
        
        self.simulation.add((score, pos))

    def motion_offset(self, motion):
        dx, dy = motion
        ns = set()
        for point in self.simulation:
            x = point[1][0] + dx + gauss(0, 2)
            y = point[1][1] - dy - gauss(0, 2)
            if -1800 <= x <= 1800 and -1800 <= y <= 1800:
                ns.add((point[0], (x, y)))
        self.simulation = ns.copy()
        del ns

    def resample(self, tolerance, angle, target, lasers):
        """Iterate through our simulated points and destroy they ones with high error.
        Re-run robot simulation with some noise"""
        angle = angle % 360

        survived = set()
        avg_score = 0
        for point in self.simulation:
            avg_score = (avg_score + point[0]) / 2
        self.avg_score = avg_score
        for point in self.simulation:
            if point[0] < avg_score:
                if random() > 0.2:
                    survived.add(point)
        
        if len(survived) != 0:
            children = self.num_particles // len(survived)

            self.simulation = survived.copy()
            for point in survived:
                for i in range(children):
                    pos = (point[1][0] + (gauss()) * 50, point[1][1] + (gauss()) * 50)
                    self.simulate_robot(pos, angle, target, lasers)

            # # add in small number of distributed points
            # self.redistribute(angle, target, 100, False)

            # print(f"Number of points: {len(self.simulation)}")
    
    def simulate_laser(self, pos: list | tuple, angle, laser: Laser):
        """
        Fire a laser in the direction of the angle at the given pos.
        Returns the distance that the laser gives from the wall.
        """
        # Get the equation of the laser based on slope.
        # We can then plug in the coordinate of the wall to get the pos that it hit.
        angle %= 360
        r_angle = math.radians(angle)
        x, y = pos

        west_intersect = None
        east_intersect = None
        north_intersect = None
        south_intersect = None

        sin_out = math.sin(r_angle)
        cos_out = math.cos(r_angle)

        # get the x/y offset based off the lasers position on the robot
        # we need to have the simulated lasers at that position, not in the center of the robot
        # x = x - (laser.distance*math.cos(math.radians((laser.angle + angle - 90) % 360)))
        # y = y + (laser.distance*math.sin(math.radians((laser.angle + angle - 90) % 360)))

        # using an equation we can find the intersection with a certain coordinate
        # this can give us the intersection with the wall from this simulated position
        if cos_out != 0:
            west_intersect = (-1800, y - (((1800 + x) / cos_out) * sin_out))
            east_intersect = (1800, y + (((1800 - x) / cos_out) * sin_out))
        if sin_out != 0:
            north_intersect = (x + (((1800 - y) / sin_out) * cos_out), 1800)
            south_intersect = (x - (((1800 + y) / sin_out) * cos_out), -1800)

        points = [west_intersect, east_intersect, north_intersect, south_intersect]
        valid_points = []
        # start be excluding the points outside of our bounds
        # because of the order of points, valid_points[0] will be west or east
        # and valid_points[1] will be north or south
        for point in points:
            if point is not None:
                if (-1800 <= point[0] <= 1800) and (-1800 <= point[1] <= 1800):
                    valid_points.append((round(point[0]), round(point[1])))
        
        # sort by point Y - coordinate
        valid_points.sort(key=lambda x: x[1], reverse=True)
        # now determine which point we are looking at via angle
        # if we are looking up, we take the pos that has a higher y coordinate
        if len(valid_points) == 0:
            return 9999
        if 0 < angle <= 180: laser_point = valid_points[0]
        else: laser_point = valid_points[1]

        return int(dist(laser_point, pos))

class MCL_Handler:
    def __init__(self) -> None:
        self.front = Laser(sensor.wallBackDistance, 60)
        self.left = Laser(sensor.wallLeftDistance, 0)
        self.back = Laser(sensor.wallFrontDistance, 180)
        self.right = Laser(sensor.wallRightDistance, 345)
        self.all_directions = [self.front, self.left, self.back, self.right]
        self.lasers = [None, None, None, None]
        self.prev_pos = [0.0, 0.0]

        # self.sim_center_offset = (60, 165)
        # self.sim_center_offset = math.sqrt(self.sim_center_offset[0]**2 + self.sim_center_offset[1]**2)
        self.sim_center_distance = 175.5
        self.sim_center_angle = 290

        print("ange : " + str(self.sim_center_angle))

        self.mcl = MCL()

        self.is_running = False

    def start(self):
        self.mcl.redistribute(sensor.imu.heading(), self.lasers, self.all_directions)
        self.thread = Thread(self.main)
        self.is_running = True

    def main(self):
        while True:
            if self.is_running:
                # print("start")
                self.loop()
                # print("end")
                sleep(35, TimeUnits.MSEC)

    def filter_lasers(self):
        # filter each laser and distance it
        # get distance and add in distance from the center of the robot
        fd = self.front.get_distance()
        ld = self.left.get_distance()
        bd = self.back.get_distance()
        rd = self.right.get_distance()
        if fd >= 9000:
            self.lasers[0] = None
        elif self.lasers[0] is not None:
            self.lasers[0] = (self.lasers[0] * 0.9 + fd * 0.1)
        else:
            if fd < 300:
                # don't change anything because we have a ring in the intake
                #TODO change this to not simulate the front laser if we have a ring in the intake
                # self.lasers[0] = self.lasers[0]
                self.lasers[0] = fd
                # pass
            else:
                self.lasers[0] = fd

        if ld >= 9000:
            self.lasers[1] = None
        elif self.lasers[1] is not None:
            self.lasers[1] = (self.lasers[1] * 0.9 + ld * 0.1)
        else:
            self.lasers[1] = ld

        if bd >= 9000:
            self.lasers[2] = None
        elif self.lasers[2] is not None:
            self.lasers[2] = (self.lasers[2] * 0.9 + bd * 0.1)
        else:
            self.lasers[2] = bd

        if rd >= 9000:
            self.lasers[3] = None
        elif self.lasers[3] is not None:
            self.lasers[3] = (self.lasers[3] * 0.9 + rd * 0.1) # type: ignore
        else:
            self.lasers[3] = rd # type: ignore

    def loop(self):
        self.mcl.resample(1000, sensor.imu.heading(), self.lasers, [self.front, self.left, self.back, self.right])
        self.mcl.redistribute(sensor.imu.heading(), self.lasers, [self.front, self.left, self.back, self.right], False, 20)
        # print(len(self.mcl.simulation))
        out_x, out_y = self.mcl.update_avg_position()
        # print("")
        # print(self.mcl.simulation)
        out_x = (0.6 * self.prev_pos[0]) + (0.4 * out_x)
        out_y = (0.6 * self.prev_pos[1]) + (0.4 * out_y)

        self.prev_pos = (out_x, out_y)

        # Adjust the calculated center to our robot center based on the offset of the sim center
        # and robot heading.
        # self.sim_center_distance = 175.5
        # self.sim_center_angle = 290
        out_x = self.prev_pos[0] + self.sim_center_distance * math.sin(math.radians((self.sim_center_angle + 180 + sensor.imu.heading()) % 360))
        out_y = self.prev_pos[1] + self.sim_center_distance * math.cos(math.radians((self.sim_center_angle + 180 + sensor.imu.heading()) % 360))

        data = {
            "x": round(out_x / 25.4, 2),
            "y": round(-out_y / 25.4, 2),
            "theta": round(math.radians(sensor.imu.heading()), 2)
        }
        # packet_mgr.add_packet("odometry", data)
        # print("out: {}, {}".format(out_x, out_y))
        # print("raw: {}, {}".format(*self.prev_pos))

        data = {
            "f": self.lasers[0],
            "l": self.lasers[1],
            "b": self.lasers[2],
            "r": self.lasers[3]
        }
        # packet_mgr.add_packet("lsr", data)
        sleep(35, MSEC)

class Robot():
    def __init__(self) -> None:
        self.color_sort_controller = ColorSortController(parent=self)

        self.autonomous_controller = Autonomous(parent=self)
        self.driver_controller = Driver(parent=self)

        self.LB_PID = LadyBrown(motor.ladyBrown)

        # Autonomous variables
        self.pos = [0.0, 0.0]
        self.heading = 0.0

    def driver(self) -> None:
        log("Starting driver")
        self.driver_controller.run()
    
    def autonomous(self) -> None:
        log("Starting autonomous")
        self.autonomous_controller.run()

class AutonomousCommands:
    @staticmethod
    def kill_motors(brake = BrakeType.COAST):
        motor.leftA.stop(brake)
        motor.leftB.stop(brake)
        motor.leftC.stop(brake)

        motor.rightA.stop(brake)
        motor.rightB.stop(brake)
        motor.rightC.stop(brake)

class AutonomousFlags:
    intake_halt_tolerance = 60
    intake_auto_halt = False
    intake_flex_auto_halt = False
    drop_after_auto_halt = False
    raise_after_auto_halt = False
    last_intake_command = 0.0
    intake_anti_jam = False
    intake_auto_stop_type = BrakeType.BRAKE
    intake_color_kill: None | Color.DefinedColor = None

    lady_brown_autostop = False

# robot states``
class AutonomousRoutines:
    red_positive_solo_AWP = [[0,{'pos':[-1550,-350],'angle':330}],[3,(('intake_anti_jam',True),)],[6,'alliance_wall_stake'],[1,'ladyBrown',-1,9.6],[2,[],(('fwd_volt',8.0),('backwards',True)),[(-1567.0,-333.0),(-1538.6,-374.2),(-1509.9,-415.1),(-1480.8,-455.7),(-1450.9,-495.9),(-1420.4,-535.4),(-1389.3,-574.6),(-1358.0,-613.6),(-1337.0,-640.0)]],[4,100],[1,'ladyBrown',0,'BRAKE'],[7,['RIGHT',160,30]],[5,True,'doinker_left'],[1,'intakeChain',1,12.0],[1,'intakeFlex',1,12.0],[3,(('intake_auto_halt',True),)],[2,[],(('fwd_volt',6.8),('speed_ramp_time',400),('slowdown_distance',350),('heading_authority',1.2),('look_ahead_dist',400),('finish_margin',150)),[(-1563.0,-323.0),(-1529.8,-360.4),(-1496.7,-397.8),(-1463.7,-435.4),(-1430.9,-473.1),(-1398.1,-510.9),(-1365.4,-548.7),(-1332.8,-586.6),(-1300.1,-624.5),(-1267.4,-662.3),(-1234.6,-700.1),(-1201.7,-737.7),(-1168.5,-775.1),(-1135.0,-812.2),(-1101.0,-848.9),(-1066.5,-885.0),(-1031.2,-920.4),(-995.0,-954.9),(-957.7,-988.2),(-919.1,-1019.9),(-879.0,-1049.9),(-837.4,-1077.6),(-794.2,-1102.7),(-749.4,-1125.0),(-703.3,-1144.3),(-656.0,-1160.5),(-607.8,-1173.9),(-559.0,-1184.5),(-509.7,-1192.7),(-460.0,-1198.8),(-410.2,-1203.0),(-360.3,-1205.6),(-310.3,-1206.8),(-270.0,-1207.0)]],[1,'intakeFlex',-1,12.0],[2,[('set_pneumatic',{'pneumatic':'doinker_left','state':False},[-763,-1119])],(('fwd_volt',5.7),('speed_ramp_time',400),('slowdown_distance',250),('backwards',True)),[(-268.0,-1192.0),(-318.0,-1193.0),(-368.0,-1192.8),(-418.0,-1191.3),(-467.9,-1188.3),(-517.6,-1183.6),(-567.2,-1176.8),(-616.4,-1167.8),(-665.0,-1156.3),(-713.0,-1142.1),(-760.0,-1125.1),(-805.8,-1105.1),(-843.0,-1086.0)]],[4,100],[5,False,'doinker_left'],[7,['LEFT',160,100]],[4,100],[2,[],(('fwd_volt',5.0),('backwards',True)),[(-856.0,-1090.0),(-806.0,-1090.2),(-756.0,-1090.9),(-706.0,-1091.8),(-656.0,-1092.8),(-606.0,-1094.1),(-556.1,-1095.4),(-506.1,-1096.7),(-456.1,-1098.1),(-406.1,-1099.4),(-356.1,-1100.6),(-306.1,-1101.5),(-256.1,-1102.0),(-254.0,-1102.0)]],[5,True,'mogo'],[1,'intakeChain',1,12.0],[1,'intakeFlex',1,12.0],[2,[('set_flag',{'intake_auto_halt':True},[-1172,-1237]),('set_pneumatic',{'pneumatic':'mogo','state':False},[-686,-1171])],(('fwd_volt',5.7),('speed_ramp_time',400),('timeout',2400),('look_ahead_dist',300),('event_look_ahead_dist',250)),[(-253.0,-1096.0),(-302.2,-1104.9),(-351.4,-1114.1),(-400.5,-1123.5),(-449.5,-1133.0),(-498.6,-1142.6),(-547.8,-1151.8),(-597.0,-1160.2),(-646.6,-1166.7),(-696.5,-1170.5),(-746.5,-1171.3),(-796.4,-1169.8),(-827.0,-1168.0),(-876.3,-1176.3),(-925.3,-1186.3),(-973.9,-1197.9),(-1022.2,-1210.9),(-1070.1,-1225.3),(-1117.5,-1241.1),(-1164.5,-1258.3),(-1210.8,-1277.0),(-1256.5,-1297.3),(-1301.5,-1319.2),(-1345.5,-1342.8),(-1388.5,-1368.4),(-1430.2,-1396.0),(-1470.3,-1425.8),(-1508.6,-1457.9),(-1544.7,-1492.5),(-1578.2,-1529.6),(-1608.6,-1569.3),(-1635.4,-1611.5),(-1658.1,-1656.0),(-1676.2,-1702.6),(-1687.0,-1740.0)]],[2,[],(('fwd_volt',7.0),('backwards',True),('speed_ramp_time',600),('min_start_voltage',2.0),('slowdown_distance',350),('look_ahead_dist',375)),[(-1768.0,-1787.0),(-1738.4,-1760.1),(-1708.3,-1733.8),(-1677.8,-1707.9),(-1646.9,-1682.5),(-1615.5,-1657.7),(-1583.6,-1633.6),(-1551.2,-1610.0),(-1518.4,-1587.2),(-1485.0,-1565.1),(-1451.2,-1543.8),(-1416.8,-1523.4),(-1381.9,-1503.9),(-1346.5,-1485.3),(-1310.5,-1467.7),(-1274.1,-1451.2),(-1237.2,-1435.8),(-1199.8,-1421.5),(-1162.1,-1408.4),(-1123.9,-1396.3),(-1085.4,-1385.4),(-1046.7,-1375.4),(-1007.8,-1366.2),(-968.7,-1357.5),(-929.7,-1348.9),(-890.8,-1339.6),(-852.3,-1328.7),(-814.9,-1314.5),(-780.0,-1295.2),(-749.6,-1269.3),(-725.6,-1237.4),(-708.1,-1201.4),(-695.9,-1163.4),(-690.0,-1138.0),(-681.2,-1099.0),(-672.5,-1059.9),(-663.8,-1020.9),(-655.2,-981.8),(-646.5,-942.8),(-638.0,-903.7),(-629.4,-864.6),(-620.9,-825.5),(-612.4,-786.5),(-604.0,-747.4),(-595.5,-708.3),(-587.1,-669.2),(-578.7,-630.1),(-570.2,-591.0),(-561.7,-551.9),(-553.0,-512.8),(-544.0,-473.9),(-534.7,-435.0),(-524.9,-396.2),(-514.5,-357.6),(-503.5,-319.1),(-499.0,-304.0)]],[5,True,'mogo'],[3,(('intake_auto_halt',False),)],[1,'intakeChain',1,12.0],[1,'intakeChain',1,12.0],[5,True,'intake'],[7,['RIGHT',85,0]],[3,(('intake_auto_halt',False),)],[2,[('set_pneumatic',{'pneumatic':'intake','state':False},[-908,179])],(('fwd_volt',6.0),('speed_ramp_time',300),('slowdown_distance',600),('timeout',4500),('event_look_ahead_dist',120)),[(-541.0,-374.0),(-609.5,-359.6),(-677.7,-343.7),(-745.4,-326.0),(-812.5,-306.1),(-878.7,-283.4),(-943.7,-257.4),(-1006.7,-227.0),(-1066.5,-190.6),(-1120.3,-146.0),(-1162.0,-90.1),(-1180.8,-23.2),(-1181.0,-15.0),(-1163.4,52.2),(-1118.3,105.1),(-1057.7,139.6),(-991.2,161.3),(-922.6,175.2),(-853.2,184.3),(-783.5,190.2),(-713.6,194.1),(-643.6,196.8),(-573.7,198.8),(-503.7,200.7),(-433.7,202.9),(-430.0,203.0)]],[1,'intakeChain',0,'COAST'],[1,'intakeFlex',0,'COAST'],[5,False,'intake']]
    blue_positive_solo_AWP = [[0,{'pos':[1550,-350],'angle':30}],[3,(('intake_anti_jam',True),)],[6,'alliance_wall_stake'],[1,'ladyBrown',-1,9.6],[2,[],(('fwd_volt',8.0),('backwards',True)),[(1567.0,-333.0),(1538.6,-374.2),(1509.9,-415.1),(1480.8,-455.7),(1450.9,-495.9),(1420.4,-535.4),(1389.3,-574.6),(1358.0,-613.6),(1337.0,-640.0)]],[4,150],[1,'ladyBrown',0,'BRAKE'],[7,['LEFT',160,30]],[5,True,'doinker_left'],[1,'intakeChain',1,12.0],[1,'intakeFlex',1,12.0],[3,(('intake_auto_halt',True),)],[2,[],(('fwd_volt',6.0),('speed_ramp_time',400),('slowdown_distance',300),('heading_authority',1.2),('look_ahead_dist',400)),[(1563.0,-323.0),(1529.7,-360.3),(1496.5,-397.7),(1463.3,-435.0),(1430.0,-472.4),(1396.8,-509.8),(1363.6,-547.1),(1330.3,-584.4),(1296.9,-621.6),(1263.4,-658.8),(1229.8,-695.8),(1196.0,-732.6),(1161.9,-769.2),(1127.5,-805.5),(1092.7,-841.4),(1057.5,-876.9),(1021.7,-911.8),(985.2,-945.9),(947.8,-979.2),(909.5,-1011.3),(870.0,-1042.0),(829.2,-1070.9),(787.1,-1097.8),(743.4,-1122.1),(698.2,-1143.5),(651.7,-1161.7),(603.9,-1176.5),(555.2,-1187.9),(505.9,-1195.9),(456.2,-1200.9),(406.2,-1203.1),(358.0,-1203.0)]],[1,'intakeFlex',-1,12.0],[2,[('set_pneumatic',{'pneumatic':'doinker_left','state':False},[-763,-1119])],(('fwd_volt',5.0),('speed_ramp_time',400),('slowdown_distance',250),('backwards',True)),[(268.0,-1192.0),(318.0,-1193.0),(368.0,-1192.8),(418.0,-1191.3),(467.9,-1188.3),(517.6,-1183.6),(567.2,-1176.8),(616.4,-1167.8),(665.0,-1156.3),(713.0,-1142.1),(760.0,-1125.1),(805.8,-1105.1),(843.0,-1086.0)]],[4,100],[5,False,'doinker_left'],[7,['RIGHT',160,100]],[4,200],[2,[],(('fwd_volt',5.0),('backwards',True)),[(856.0,-1090.0),(806.0,-1090.2),(756.0,-1090.9),(706.0,-1091.8),(656.0,-1092.8),(606.0,-1094.1),(556.1,-1095.4),(506.1,-1096.7),(456.1,-1098.1),(406.1,-1099.4),(356.1,-1100.6),(306.1,-1101.5),(256.1,-1102.0),(254.0,-1102.0)]],[5,True,'mogo'],[1,'intakeChain',1,12.0],[1,'intakeFlex',1,12.0],[2,[('set_flag',{'intake_auto_halt':True},[-597,-1238]),('set_pneumatic',{'pneumatic':'mogo','state':False},[-601,-1236])],(('fwd_volt',6.0),('speed_ramp_time',400),('timeout',2400),('look_ahead_dist',300)),[(250.0,-1098.0),(289.6,-1128.5),(331.4,-1155.9),(375.3,-1179.7),(421.3,-1199.4),(469.0,-1214.4),(517.9,-1224.7),(567.5,-1230.7),(617.5,-1232.8),(667.4,-1232.0),(717.4,-1229.1),(767.2,-1225.3),(817.1,-1221.4),(867.0,-1218.2),(916.9,-1216.6),(966.9,-1217.3),(1016.8,-1220.7),(1066.3,-1227.5),(1115.3,-1237.7),(1163.3,-1251.4),(1210.3,-1268.6),(1256.0,-1288.8),(1300.3,-1312.0),(1343.2,-1337.6),(1384.7,-1365.5),(1424.9,-1395.3),(1463.7,-1426.7),(1501.4,-1459.6),(1537.9,-1493.7),(1573.4,-1529.0),(1607.9,-1565.1),(1641.6,-1602.1),(1674.3,-1639.9),(1706.4,-1678.3),(1737.7,-1717.3),(1768.3,-1756.8),(1770.0,-1759.0)]],[2,[],(('fwd_volt',5.0),('backwards',True),('speed_ramp_time',600),('min_start_voltage',2.0),('slowdown_distance',250)),[(1768.0,-1787.0),(1728.8,-1756.0),(1688.2,-1726.8),(1646.7,-1698.9),(1604.6,-1672.0),(1562.1,-1645.7),(1519.3,-1619.8),(1476.3,-1594.2),(1433.4,-1568.6),(1390.4,-1543.0),(1347.6,-1517.1),(1305.0,-1491.0),(1262.6,-1464.5),(1220.5,-1437.5),(1178.8,-1410.0),(1137.5,-1381.8),(1096.7,-1352.8),(1056.5,-1323.1),(1017.0,-1292.5),(978.2,-1261.0),(940.2,-1228.4),(903.2,-1194.9),(867.1,-1160.2),(832.3,-1124.4),(798.6,-1087.4),(766.4,-1049.2),(735.6,-1009.8),(706.4,-969.2),(678.9,-927.5),(653.2,-884.5),(629.5,-840.6),(607.7,-795.5),(588.0,-749.6),(570.4,-702.8),(555.0,-655.2),(541.8,-607.0),(530.6,-558.3),(521.6,-509.1),(514.7,-459.6),(509.8,-409.8),(506.9,-359.9),(506.0,-331.0)]],[5,True,'mogo'],[1,'intakeChain',1,12.0],[1,'intakeChain',1,12.0],[5,True,'intake'],[7,['RIGHT',75,0]],[2,[('set_pneumatic',{'pneumatic':'intake','state':False},[-938,254])],(('fwd_volt',5.0),('speed_ramp_time',300),('slowdown_distance',200),('timeout',4250)),[(491.0,-323.0),(541.0,-322.8),(591.0,-323.3),(641.0,-324.0),(691.0,-324.3),(741.0,-324.0),(791.0,-322.6),(840.9,-319.8),(890.6,-314.8),(940.0,-307.2),(988.8,-296.0),(1036.2,-280.2),(1081.2,-258.5),(1122.1,-229.8),(1156.5,-193.7),(1182.7,-151.2),(1199.9,-104.4),(1209.1,-55.3),(1211.7,-5.4),(1211.0,20.0),(1207.6,69.9),(1200.6,119.4),(1187.9,167.7),(1165.4,212.2),(1129.1,246.0),(1082.4,263.2),(1032.7,267.6),(982.8,264.9),(933.2,258.5),(883.9,250.2),(834.8,240.9),(785.7,231.2),(736.7,221.4),(687.6,211.8),(638.5,202.5),(589.3,193.7),(539.9,185.4),(490.5,177.7),(479.0,176.0)]],[1,'intakeChain',0,'COAST'],[1,'intakeFlex',0,'COAST']]
    
    red_ring_side = [[0,{'pos':[-1346,920],'angle':90}],[3,(('intake_auto_halt',True),)],[1,'intakeChain',1,12.0],[1,'intakeFlex',1,12.0],[2,[],(('fwd_volt',6.0),('speed_ramp_time',500),('slowdown_distance',300),('finish_margin',150)),[(-1353.0,920.0),(-1303.0,917.9),(-1253.1,915.8),(-1203.1,913.7),(-1153.2,911.7),(-1103.2,909.7),(-1053.2,907.8),(-1003.3,906.1),(-953.3,904.4),(-903.3,903.0),(-853.3,901.9),(-803.3,901.2),(-753.3,900.9),(-703.3,901.2),(-653.4,902.4),(-603.4,904.7),(-553.6,908.6),(-503.9,914.6),(-454.8,923.9),(-406.9,937.9),(-361.7,959.1),(-322.8,990.3),(-294.7,1031.5),(-278.4,1078.6),(-270.8,1128.0),(-268.7,1177.9),(-270.0,1227.9),(-273.5,1277.8),(-278.2,1327.6),(-279.0,1335.0)]],[2,[],(('fwd_volt',5.5),('slowdown_distance',350),('backwards',True),('heading_authority',2.5)),[(-249.0,1340.0),(-253.0,1290.2),(-258.1,1240.4),(-264.3,1190.8),(-272.1,1141.4),(-281.6,1092.3),(-293.3,1043.7),(-307.6,995.8),(-324.9,948.9),(-345.7,903.5),(-369.9,859.7),(-397.5,818.1),(-428.1,778.5),(-461.1,740.9),(-495.9,705.0),(-531.9,670.4),(-568.9,636.8),(-606.4,603.7),(-644.2,570.9),(-682.1,538.3),(-701.0,522.0)]],[5,True,'mogo'],[3,(('intake_auto_halt',False),)],[4,200],[1,'intakeChain',1,12.0]]
    blue_ring_side = [[0,{'pos':[-1346,920],'angle':90}]]

    red_goal_rush = [[0,{'pos':[-1282,-1498],'angle':70}],[5,True,'doinker'],[1,'intakeFlex',1,12.0],[2,[],(('fwd_volt',12.0),('speed_ramp_time',600),('slowdown_distance',1000)),[(-1283.0,-1499.0),(-1235.7,-1482.9),(-1188.4,-1466.5),(-1141.2,-1450.0),(-1094.1,-1433.4),(-1047.0,-1416.6),(-999.9,-1399.8),(-952.8,-1383.0),(-905.7,-1366.2),(-858.6,-1349.3),(-811.6,-1332.4),(-764.5,-1315.6),(-717.4,-1298.8),(-670.3,-1282.0),(-623.2,-1265.3),(-576.0,-1248.6),(-528.8,-1232.1),(-481.6,-1215.7),(-434.3,-1199.5),(-386.9,-1183.6),(-385.0,-1183.0)]],[2,[],(('fwd_volt',8.0),('backwards',True),('speed_ramp_time',1000),('slowdown_distance',400)),[(-485.0,-1243.0),(-531.6,-1261.1),(-578.5,-1278.4),(-625.7,-1294.9),(-673.1,-1310.7),(-720.8,-1325.9),(-768.6,-1340.6),(-816.5,-1354.9),(-864.5,-1368.9),(-912.6,-1382.6),(-960.7,-1396.3),(-1005.0,-1409.0)]],[5,False,'doinker'],[7,['RIGHT',110,0]],[4,300],[2,[],(('fwd_volt',5.5),('slowdown_distance',350),('backwards',True)),[(-1014.0,-1411.0),(-990.0,-1367.1),(-964.9,-1323.9),(-939.1,-1281.1),(-912.7,-1238.6),(-885.9,-1196.4),(-859.0,-1154.2),(-832.1,-1112.1),(-805.3,-1069.9),(-778.6,-1027.6),(-752.3,-985.1),(-726.5,-942.2),(-701.4,-899.0),(-677.1,-855.4),(-653.8,-811.1),(-631.9,-766.2),(-611.7,-720.4),(-593.7,-673.8),(-578.7,-626.1),(-567.7,-577.3),(-562.1,-527.7),(-562.0,-501.0)]],[5,True,'mogo'],[4,200],[1,'intakeChain',1,12.0],[4,200],[2,[('set_pneumatic',{'pneumatic':'mogo','state':False},[-788,-903])],(('fwd_volt',5.5),('timeout',2250),('slowdown_distance',800),('speed_ramp_time',700)),[(-540.0,-445.0),(-558.8,-491.3),(-578.4,-537.3),(-598.8,-583.0),(-620.1,-628.2),(-642.4,-672.9),(-665.7,-717.2),(-690.0,-760.9),(-715.4,-804.0),(-741.8,-846.4),(-769.4,-888.1),(-798.2,-929.0),(-828.2,-969.0),(-859.3,-1008.1),(-891.7,-1046.2),(-925.2,-1083.3),(-959.9,-1119.4),(-995.6,-1154.3),(-1032.4,-1188.1),(-1070.2,-1220.9),(-1108.9,-1252.6),(-1148.3,-1283.3),(-1188.4,-1313.2),(-1229.1,-1342.3),(-1270.1,-1370.9),(-1311.4,-1399.0),(-1352.9,-1426.9),(-1394.4,-1454.8),(-1435.7,-1483.0),(-1476.7,-1511.6),(-1517.3,-1540.8),(-1557.2,-1570.9),(-1596.3,-1602.1),(-1634.4,-1634.5),(-1671.3,-1668.2),(-1707.0,-1703.2),(-1741.2,-1739.7),(-1773.8,-1777.6),(-1804.8,-1816.8),(-1805.0,-1817.0)]],[3,(('intake_auto_halt',True),)],[1,'intakeChain',0,'COAST'],[3,(('intake_auto_halt',False),)],[2,[],(('fwd_volt',5.5),('backwards',True),('slowdown_distance',350),('speed_ramp_time',250),('heading_authority',1.75)),[(-1774.0,-1808.0),(-1762.1,-1759.5),(-1743.9,-1713.0),(-1718.9,-1669.7),(-1687.6,-1630.8),(-1650.8,-1597.0),(-1609.9,-1568.3),(-1565.9,-1544.6),(-1519.8,-1525.2),(-1472.3,-1509.5),(-1424.0,-1496.9),(-1375.0,-1486.8),(-1325.6,-1478.9),(-1276.0,-1472.7),(-1226.2,-1468.1),(-1176.4,-1464.7),(-1126.4,-1462.3),(-1076.4,-1460.9),(-1026.4,-1460.1),(-976.4,-1460.0),(-926.4,-1460.3),(-876.4,-1461.0),(-826.5,-1462.0),(-776.5,-1463.2),(-726.5,-1464.5),(-676.5,-1465.7),(-626.5,-1467.0),(-576.5,-1468.1),(-526.5,-1469.0),(-476.5,-1469.5),(-426.5,-1469.7),(-376.5,-1469.3),(-354.0,-1469.0)]],[5,True,'mogo'],[1,'intakeChain',1,12.0],[5,True,'doinker_left'],[2,[],(('fwd_volt',4.0),('timeout',1600),('slowdown_distance',300)),[(-534.0,-1460.0),(-582.9,-1449.8),(-632.4,-1442.5),(-682.2,-1438.1),(-732.2,-1436.6),(-782.2,-1437.7),(-832.0,-1441.4),(-881.7,-1447.4),(-931.0,-1455.2),(-980.1,-1464.7),(-1029.0,-1475.4),(-1077.6,-1487.1),(-1126.0,-1499.5),(-1174.4,-1512.2),(-1222.7,-1525.1),(-1271.0,-1537.8),(-1319.5,-1550.2),(-1368.0,-1562.1),(-1416.8,-1573.3),(-1465.7,-1583.7),(-1514.0,-1593.0)]],[7,['RIGHT',180,1.35]],[5,False,'doinker_left'],[1,'intakeChain',0,'COAST'],[1,'intakeFlex',0,'COAST'],[2,[],(),[(-1184.0,-1460.0),(-1134.0,-1460.2),(-1084.0,-1460.1),(-1034.0,-1459.5),(-984.0,-1458.5),(-934.1,-1456.6),(-884.1,-1453.8),(-834.3,-1449.8),(-784.5,-1445.4),(-734.6,-1441.7),(-684.7,-1439.1),(-634.7,-1437.5),(-584.7,-1436.6),(-534.7,-1436.3),(-484.7,-1436.3),(-434.7,-1436.6),(-400.0,-1437.0)]]]
    blue_goal_rush = [[0,{'pos':[1282,-1498],'angle':290}],[5,True,'doinker_left'],[1,'intakeFlex',1,12.0],[2,[],(('fwd_volt',12.0),('speed_ramp_time',600),('slowdown_distance',1000)),[(1283.0,-1499.0),(1235.5,-1483.4),(1187.9,-1468.2),(1140.1,-1453.3),(1092.3,-1438.7),(1044.5,-1424.2),(996.6,-1409.8),(948.7,-1395.5),(900.7,-1381.3),(852.8,-1367.1),(804.8,-1352.9),(756.9,-1338.8),(708.9,-1324.6),(661.0,-1310.3),(613.1,-1296.0),(565.2,-1281.5),(517.4,-1266.9),(469.7,-1252.1),(422.0,-1237.0),(385.0,-1225.0)]],[2,[],(('fwd_volt',8.0),('backwards',True),('speed_ramp_time',1000),('slowdown_distance',400)),[(485.0,-1243.0),(531.6,-1261.1),(578.5,-1278.4),(625.7,-1294.9),(673.1,-1310.7),(720.8,-1325.9),(768.6,-1340.6),(816.5,-1354.9),(864.5,-1368.9),(912.6,-1382.6),(960.7,-1396.3),(1005.0,-1409.0)]],[5,False,'doinker_left'],[7,['LEFT',120,0]],[4,300],[2,[],(('fwd_volt',5.5),('slowdown_distance',350),('backwards',True)),[(1014.0,-1411.0),(992.9,-1365.7),(971.5,-1320.5),(949.8,-1275.4),(927.9,-1230.5),(905.8,-1185.6),(883.6,-1140.8),(861.3,-1096.1),(838.8,-1051.4),(816.2,-1006.8),(793.5,-962.3),(770.7,-917.8),(747.8,-873.3),(724.9,-828.9),(701.9,-784.5),(678.8,-740.2),(655.6,-695.8),(632.4,-651.5),(609.2,-607.3),(585.9,-563.0),(577.0,-546.0)]],[5,True,'mogo'],[4,200],[1,'intakeChain',1,12.0],[4,200],[2,[('set_pneumatic',{'pneumatic':'mogo','state':False},[789,-854])],(('fwd_volt',5.5),('timeout',2250),('slowdown_distance',800),('speed_ramp_time',700)),[(540.0,-445.0),(560.4,-490.6),(581.6,-535.9),(603.6,-580.8),(626.6,-625.2),(650.6,-669.1),(675.6,-712.4),(701.8,-755.0),(729.2,-796.8),(757.9,-837.8),(787.9,-877.7),(819.3,-916.6),(852.1,-954.4),(886.3,-990.9),(921.8,-1026.0),(958.7,-1059.8),(996.6,-1092.4),(1035.6,-1123.7),(1075.4,-1154.0),(1115.8,-1183.4),(1156.6,-1212.2),(1197.7,-1240.8),(1238.8,-1269.2),(1279.8,-1297.9),(1320.4,-1327.0),(1360.6,-1356.8),(1400.2,-1387.3),(1439.1,-1418.7),(1477.3,-1451.0),(1514.5,-1484.3),(1550.9,-1518.6),(1586.4,-1553.9),(1620.9,-1590.0),(1654.5,-1627.0),(1687.2,-1664.9),(1719.0,-1703.4),(1750.0,-1742.7),(1780.1,-1782.6),(1805.0,-1817.0)]],[3,(('intake_auto_halt',True),)],[1,'intakeChain',0,'COAST'],[3,(('intake_auto_halt',False),)],[2,[],(('fwd_volt',5.5),('backwards',True),('slowdown_distance',350),('speed_ramp_time',250),('heading_authority',1.75)),[(1774.0,-1808.0),(1762.1,-1759.5),(1743.9,-1713.0),(1718.9,-1669.7),(1687.6,-1630.8),(1650.8,-1597.0),(1609.9,-1568.3),(1565.9,-1544.6),(1519.8,-1525.2),(1472.3,-1509.5),(1424.0,-1496.9),(1375.0,-1486.8),(1325.6,-1478.9),(1276.0,-1472.7),(1226.2,-1468.1),(1176.4,-1464.7),(1126.4,-1462.3),(1076.4,-1460.9),(1026.4,-1460.1),(976.4,-1460.0),(926.4,-1460.3),(876.4,-1461.0),(826.5,-1462.0),(776.5,-1463.2),(726.5,-1464.5),(676.5,-1465.7),(626.5,-1467.0),(576.5,-1468.1),(526.5,-1469.0),(476.5,-1469.5),(426.5,-1469.7),(376.5,-1469.3),(354.0,-1469.0)]],[5,True,'mogo'],[1,'intakeChain',1,12.0],[5,True,'doinker'],[2,[],(('fwd_volt',4.0),('timeout',1600),('slowdown_distance',300)),[(534.0,-1460.0),(582.9,-1449.8),(632.4,-1442.5),(682.2,-1438.1),(732.2,-1436.6),(782.2,-1437.7),(832.0,-1441.4),(881.7,-1447.4),(931.0,-1455.2),(980.1,-1464.7),(1029.0,-1475.4),(1077.6,-1487.1),(1126.0,-1499.5),(1174.4,-1512.2),(1222.7,-1525.1),(1271.0,-1537.8),(1319.5,-1550.2),(1368.0,-1562.1),(1416.8,-1573.3),(1465.7,-1583.7),(1514.0,-1593.0)]],[7,['LEFT',180,1.35]],[5,False,'doinker'],[1,'intakeChain',0,'COAST'],[1,'intakeFlex',0,'COAST'],[2,[],(('fwd_volt',5.0),),[(1210.0,-1552.0),(1160.0,-1550.6),(1110.0,-1548.9),(1060.1,-1546.8),(1010.1,-1544.4),(960.2,-1541.7),(910.3,-1538.6),(860.4,-1535.1),(810.6,-1531.2),(760.8,-1527.1),(710.9,-1522.9),(661.1,-1518.7),(611.3,-1514.9),(561.4,-1511.9),(511.4,-1509.7),(461.4,-1508.6),(411.4,-1508.6),(361.4,-1509.5),(311.5,-1511.2),(293.0,-1512.0)]]]

class Autonomous():
    def __init__(self, parent: Robot) -> None:
        """
        Setup autonomous. Runs at start of program!
        """
        self.robot = parent

        self.positioning_algorithm = DeltaPositioning(sensor.groundEncoder, sensor.imu)
        self.path_controller = PurePursuit

        self.mcl_controller = MCL_Handler()
        self.run_mcl = False
        self.run_mcl_lasers = True

        self.intake_jam_windup = 0
        self.last_intake_turn = 0

        class c_drivetrain:
            @staticmethod
            def turn_for(direction, deg: int, vel: float = 1.35, thresh: int = 5):
                turned_deg = 0
                if direction == TurnType.LEFT:
                    deg *= -1

                last_heading = sensor.imu.rotation()
                pid = MultipurposePID(0.09, 0.035, 0.0, 0)

                while abs(turned_deg - deg) > thresh:
                    d_h = sensor.imu.rotation() - last_heading
                    turned_deg += d_h
                    output = pid.calculate(deg, turned_deg) * -vel
                                        
                    motor.leftA.spin(FORWARD, output, VOLT)
                    motor.leftB.spin(FORWARD, output, VOLT)
                    motor.leftC.spin(FORWARD, output, VOLT)

                    motor.rightA.spin(REVERSE, output, VOLT)
                    motor.rightB.spin(REVERSE, output, VOLT)
                    motor.rightC.spin(REVERSE, output, VOLT)
                    
                    last_heading = sensor.imu.rotation()
                    sleep(20, MSEC)
                
                sleep(20, MSEC)
                motor.leftA.stop(BRAKE)
                motor.leftB.stop(BRAKE)
                motor.leftC.stop(BRAKE)
                motor.rightA.stop(BRAKE)
                motor.rightB.stop(BRAKE)
                motor.rightC.stop(BRAKE)

        self.drivetrain = c_drivetrain

        self.fwd_speed = 8

        drivetrain.set_timeout(1.5, TimeUnits.SECONDS)
        drivetrain.set_turn_threshold(50)
        drivetrain.set_turn_constant(0.1)

        log("Autonomous object setup")
    
    def autonomous_setup(self) -> None:
        """
        Call at start of auton. Sets up the coordinates n stuff for the loaded path.
        """
        try:
            sensor.imu.set_heading(self.initial_pose["angle"])
            self.robot.heading = (self.initial_pose["angle"])
            self.robot.pos = list(self.initial_pose["pos"])
            log("Set initial heading and position to ({}, {}, {}).".format(self.robot.pos[0], self.robot.pos[1], self.robot.heading))
        except:
            log("Couldn't set initial heading / pos! Likely because load_path hasn't been run yet.", LogLevel.WARNING)
        
        log("Starting autonomous threads.")
        self.pos_thread = Thread(self.position_thread)
        self.background_thread = Thread(self.background)
        log("Autonomous threads started.")

        log("Disabling Lady Brown PID", LogLevel.WARNING)
        self.robot.LB_PID.enabled = False
        motor.ladyBrown.stop()

        self.start_time = brain.timer.system()

    def load_path(self, sequence: str | list) -> None:
        """
        Loads autonomous data into object for later use.

        Args:
            sequence: The name or sequenced to be used to load. 
        """
        if type(sequence) == str:
            if hasattr(AutonomousRoutines, sequence):
                self.sequence = getattr(AutonomousRoutines, sequence)
                log("Loaded autonomous {}.".format(sequence))
            else:
                log("Autonomous {} not found in AutonomousRoutines wrapper".format(sequence))
        elif type(sequence) == list:
            self.sequence = sequence

        self.initial_pose: dict = self.sequence[0][1]
    
    def autonomous_cleanup(self) -> None:
        """
        Runs at end of autonomous. Do not modify.
        """
        self.end_time = brain.timer.system()
        elapsed_time = self.end_time - self.start_time
        log("Auton complete. Time taken: {}".format(elapsed_time), LogLevel.INFO)
    
    def position_thread(self):
        while True:
            self.robot.heading = sensor.imu.heading()
            dx, dy = self.positioning_algorithm.update()

            if self.run_mcl:
                self.mcl_controller.mcl.motion_offset((dx, dy))
            else:
                self.robot.pos[0] += dx
                self.robot.pos[1] += dy
            
            if self.run_mcl_lasers:
                self.mcl_controller.filter_lasers()

            sleep(20, TimeUnits.MSEC)
            # print(self.robot.pos)
            # sleep(100, TimeUnits.MSEC)
    
    def run(self) -> None:
        """
        Runs autonomous. Do not modify.
        """
        log("Running autonomous")
        self.autonomous_setup()
        motor.ladyBrown.stop()

        self.read_sequence()
        
        self.autonomous_cleanup()
        # self.test()

    def test(self) -> None:
        """
        Run a test version of autonomous. This is NOT run in competition!
        """
        log("Running autonomous TEST", LogLevel.WARNING)
        # self.aut-dyBrown.stop()
        self.run()
        log("Finished test at {}".format(robot.pos))
        # while True:
        #     log(str((robot.pos, robot.heading)))
        #     sleep(35)

    def read_sequence(self) -> None:
        """
        Runs commands within a sequence.
        """
        """
        POSE = 0
        MOTOR = 1
        PATH = 2
        FLAG = 3
        WAIT = 4
        PNEUMATIC = 5
        """
        i = -1

        for function in self.sequence:
            i += 1
            ID = function[0]

            if ID == 1: 
                # spin or stop motor
                # spin format: [1, 'intakeChain', 1,                    12.0]
                #                   name,         direction (-1 or 1)   voltage
                # stop format: [1, 'intakeChain', 0,    'COAST']
                #                   name          stop  braketype
                if hasattr(motor, function[1]): # if motor name is in motor class
                    if function[2] == 0:
                        getattr(motor, function[1]).stop(getattr(BrakeType, function[3]))
                        log("[{}] Stop motor {}, mode {}".format(i, function[1], function[3]))
                    else:
                        volt = function[3]
                        if function[2] == -1: volt *= -1
                        getattr(motor, function[1]).spin(DirectionType.FORWARD, volt, VOLT)
                        log("[{}] Spin motor {} at {} volts".format(i, function[1], volt))
                else:
                    log("[{}] No motor named {}".format(i, function[1]), LogLevel.WARNING)
            elif ID == 2:
                # run path
                if function[1] is None: 
                    events = []
                else:
                    events = function[1]
                log("[{}] Running path.".format(i))
                self.path(function[3], events, {k: v for k, v in function[2]})
            elif ID == 3:
                # set flag
                for flag_name, value in function[1]:
                    if hasattr(flags, flag_name):
                        if type(getattr(flags, flag_name)) == type(value):
                            if do_logging: old_value = getattr(flags, flag_name)
                            setattr(flags, flag_name, value)
                            if do_logging: log("[{}] Set {} flag to {} from {}".format(i, flag_name, value, old_value))
                        else:
                            log("[{}] Mismatch flag type {} type {} cannot override {}".format(
                                i, flag_name, type(value), type(getattr(flags, flag_name))), LogLevel.WARNING)
                    elif hasattr(AutonomousFlags, flag_name):
                        if type(getattr(AutonomousFlags, flag_name)) == type(value):
                            if do_logging: old_value = getattr(AutonomousFlags, flag_name)
                            setattr(AutonomousFlags, flag_name, value)
                            if do_logging: log("[{}] Set {} flag to {} from {}".format(i, flag_name, value, old_value))
                        else:
                            log("[{}] Mismatch flag type {} type {} cannot override {}".format(
                                i, flag_name, type(value), type(getattr(AutonomousFlags, flag_name))), LogLevel.WARNING)
                    else:
                        log("[{}] Flag {} not found.".format(i, flag_name), LogLevel.WARNING)

            elif ID == 4:
                # wait time
                log("[{}] Sleeping {}msec".format(i, function[1]))
                sleep(function[1])
            elif ID == 5:
                # pneumatic
                if hasattr(pneumatic, function[2]):
                    getattr(pneumatic, function[2]).set(function[1])
                    log("[{}] Set pneumatic {} to {}".format(i, function[2], function[1]))
                else:
                    log("[{}] No pneumatic named {}".format(i, function[2]), LogLevel.WARNING)
            elif ID == 6:
                # custom event
                if function[1] == "alliance_wall_stake":
                    log("[{}] Alliance wall stake macro".format(i))
                    motor.ladyBrown.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
                    end_timeout = brain.timer.system() + 700
                    while sensor.wallEncoder.angle() < 190:
                        if brain.timer.system() > end_timeout:
                            log("[{}] Lady brown movement timed out!".format(i), LogLevel.WARNING)
                            break
                        sleep(20, TimeUnits.MSEC)
                    log("[{}] Lady brown finished or timed out. Encoder at {} degrees.".format(i, sensor.wallEncoder.angle()))
                    motor.ladyBrown.stop(BrakeType.COAST)
                elif function[1] == "pos_override_red_positive_solo":
                    self.robot.pos = [-1370.0, -1545.0]
            elif ID == 7:
                # [7,['RIGHT',90,80]]
                args = function[1]
                if hasattr(TurnType, args[0]):
                    if len(args) == 3:
                        if args[2] != 0: velocity = args[2]
                        else: velocity = 1.35
                        self.drivetrain.turn_for(getattr(TurnType, args[0]), args[1], vel=velocity)
                        log("[{}] Turn {} for {} degrees.".format(i, args[0], args[1]))
                else:
                    log("[{}] No turntype {}".format(i, args[0]), LogLevel.WARNING)
            # print("{}, seq {}".format(brain.timer.system(), i))
            sleep(20)
    def background(self) -> None:
        """
        Starts at beginning of auton. Stops at end.
        """
        log("Starting background thread.")

        while True:
            self.listeners()
            self.robot.color_sort_controller.sense()

            sleep(35, MSEC)

    def listeners(self) -> None:
        auto_flags = AutonomousFlags

        # Auto halt intake when we get a ring
        if auto_flags.intake_auto_halt:
            if sensor.intakeDistance.object_distance() < auto_flags.intake_halt_tolerance:
                motor.intakeChain.stop(AutonomousFlags.intake_auto_stop_type)
                auto_flags.intake_auto_halt = False 
                log("Auto halted intake")

                if auto_flags.drop_after_auto_halt:
                    pneumatic.intake.set(False)
                if auto_flags.raise_after_auto_halt:
                    pneumatic.intake.set(True)

        if auto_flags.intake_flex_auto_halt:
            if sensor.intakeDistance.object_distance() < auto_flags.intake_halt_tolerance:
                auto_flags.intake_flex_auto_halt = False
                motor.intakeFlex.stop(BrakeType.COAST)

        if auto_flags.intake_anti_jam:
            if motor.intakeChain.command():
                intake_pos = motor.intakeChain.position()
                intake_change = intake_pos - self.last_intake_turn
                self.last_intake_turn = intake_pos
                # print(intake_pos, intake_change)
                if abs(intake_change) < 5:
                    self.intake_jam_windup += 1
                else:
                    self.intake_jam_windup = 0
                
                if self.intake_jam_windup > 45:
                    log("Intake jam!", LogLevel.WARNING)
                    command = motor.intakeChain.command(VelocityUnits.PERCENT)
                    brain.timer.event(motor.intakeChain.spin, 500, (DirectionType.FORWARD, command, VelocityUnits.PERCENT))

                    motor.intakeChain.stop()
                    motor.intakeChain.spin(DirectionType.REVERSE, 40, VelocityUnits.PERCENT)
                    self.intake_jam_windup = 0
        
        if auto_flags.lady_brown_autostop:
            if sensor.wallEncoder.angle() < 20:
                log("Auto stopped lady brown retract")
                auto_flags.lady_brown_autostop = False
                motor.ladyBrown.stop(BrakeType.COAST)

        if auto_flags.intake_color_kill is not None:
            if auto_flags.intake_color_kill == Color.RED:
                if sensor.intakeColor.hue() < ColorSortController.RED:
                    motor.intakeChain.stop(BrakeType.COAST)
            else:
                if sensor.intakeColor.hue() > ColorSortController.BLUE:
                    motor.intakeChain.stop(BrakeType.COAST)
    
    def path(self, path, events=[], custom_args: dict = {}) -> None:
        """
        Runs a path. Halts program execution until finished or times out.
        
        Arguments:
            path (Iterable): List of points that will be followed.
            Events (Iterable): List of position based events
            custom_args (Dict): Dictionary of custom arguments. These override their default counterpart found in params IF:
                argument in params
                AND
                custom_args[argument] != params[argument]
        """
        params = {
            # Runs path in reverse if true
            "backwards": False,
            # mm distance to look ahead using pure pursuit
            "look_ahead_dist": 350,
            # Distance from final coordinate within path that we will hald
            "finish_margin": 100,
            # Triggers any events if this far away
            "event_look_ahead_dist": 75,
            # Will finish early if the elapsed time has surpassed this if not None
            "timeout": None,
            # How much the heading PID output is multiplied by. Arbitrary.
            "heading_authority": 2.0,
            # 
            "max_turn_volts": 8,
            # Heading PID P value
            "hPID_KP": 0.1,
            # Heading PID D value
            "hPID_KD": 0.01,
            # Heading PID I value
            "hPID_KI": 0,
            # Heading PID I max value
            "hPID_KI_MAX": 0,
            # Heading PID min out value
            "hPID_MIN_OUT": None,
            # Constant that defines the weight that the path curvature will be multiplied by (usually more than 1, curvature is often small).
            "K_curvature_speed": 0.0,
            # Constant the defines the weight that the look ahead distance will shrink based on path curvature
            "K_curvature_look_ahead": 0.0,
            "a_curvature_exp": 0.1,
            # Most volts that the curvature will be allowed to change on the motors.
            "max_curvature_speed": 3.0,
            # Minimum look ahead that we will look on the path (curvature based look ahead)
            "min_look_ahead": 300,
            "a_curvature_speed_exp": 0.1,
            "speed_ramp_time": 1,
            "min_start_voltage": 4,
            "slowdown_distance": 0,
            "min_slow_voltage": 4,
            "checkpoints": [],
            "fwd_volt": 5.0,
        }
        # merge params with a dictionary of key/value pairs if the key is in params
        params.update({k: v for k, v in custom_args.items() if k in params})
        
        log("Running path")
        if params['timeout'] is not None:
            # determine when we will stop trying to drive the path and just complete it
            time_end = brain.timer.system() + params['timeout']

        # intialize the path controller object for this path only
        path_handler = self.path_controller(params['look_ahead_dist'], params['finish_margin'], path, params['checkpoints'])
        # create the heading PID with this path's settings
        heading_pid = MultipurposePID(params['hPID_KP'], params['hPID_KD'], params['hPID_KI'], params['hPID_KI_MAX'], params['hPID_MIN_OUT'])
        # reference the robot from the controller
        robot = self.robot

        # setup local path variables
        done = path_handler.path_complete
        waiting = False
        wait_stop = 0

        prev_curvature = 0
        prev_speed_curvature = 0

        prev_time = brain.timer.system()
        start_time = prev_time
        completed_events = set()

        # start driving this path
        while not done:
            current_time = brain.timer.system()
            dt = current_time - prev_time
            prev_time = current_time

            elapsed_time = current_time - start_time

            target_point, curvature = path_handler.goal_search(robot.pos)
            dx, dy = target_point[0] - robot.pos[0], target_point[1] - robot.pos[1] # type: ignore
            heading_to_target = math.degrees(math.atan2(dx, dy))
            distance_to_end = dist(self.robot.pos, path[-1][:2])

            if dx < 0:
                heading_to_target = 360 + heading_to_target

            heading_error = self.robot.heading - heading_to_target
            rollover = False

            if params['backwards']:
                if heading_error > 180:
                    heading_error -= 180
                else:
                    heading_error += 180

            # logic to handle the edge case of the target heading rolling over
            if heading_error > 180:
                heading_error = 360 - heading_error
                rollover = True
            if heading_error < -180:
                heading_error = -360 - heading_error
                rollover = True

            heading_output = heading_pid.calculate(0, heading_error)

            # Curvature exponential smoothing ((x * 1-a) + (y * a))
            if curvature < 0.05: curvature = 0
            curvature = (prev_curvature * (1 - params["a_curvature_exp"])) + (curvature * params["a_curvature_exp"])

            # avoid unnecessary calculations if these are off (0.0)
            if params["K_curvature_speed"] != 0.0:
                dynamic_forwards_speed = min(params["max_curvature_speed"], curvature * params["K_curvature_speed"])
                dynamic_forwards_speed = (
                    prev_speed_curvature * (1 - params["a_curvature_speed_exp"]) +
                    (dynamic_forwards_speed * params["a_curvature_speed_exp"])
                )
                prev_speed_curvature = dynamic_forwards_speed
            else:
                dynamic_forwards_speed = 0

            if params["K_curvature_look_ahead"] != 0.0:
                path_handler.look_dist = (
                    params["look_ahead_dist"] -
                    min(curvature * params["K_curvature_look_ahead"], params["min_look_ahead"])
                )

            # forwards speed linear acceleration
            if elapsed_time < params["speed_ramp_time"]:
                forwards_speed = params["min_start_voltage"] + (
                    params["fwd_volt"] - params["min_start_voltage"]
                ) * (elapsed_time / params["speed_ramp_time"])
            else:
                forwards_speed = params["fwd_volt"]

            # if close to end, slow down
            if distance_to_end < params["slowdown_distance"]:
                forwards_speed = params["min_slow_voltage"] + (
                    params["fwd_volt"] - params["min_slow_voltage"]
                ) * (distance_to_end / params["slowdown_distance"])

            if params["backwards"]:
                forwards_speed *= -1

            # Do some stuff that lowers the authority of turning, no idea if this is reasonable
            heading_output *= params["heading_authority"]
            if heading_output > params["max_turn_volts"]: heading_output = params["max_turn_volts"]
            if heading_output < -params["max_turn_volts"]: heading_output = -params["max_turn_volts"]

            # if we rollover, fix it
            if rollover:
                heading_output *= -1

            if not waiting:
                left_speed = -forwards_speed - dynamic_forwards_speed - heading_output
                right_speed = -forwards_speed - dynamic_forwards_speed + heading_output

                motor.leftA.spin(FORWARD, left_speed, VOLT)
                motor.leftB.spin(FORWARD, left_speed, VOLT)
                motor.leftC.spin(FORWARD, left_speed, VOLT)

                motor.rightA.spin(FORWARD, right_speed, VOLT)
                motor.rightB.spin(FORWARD, right_speed, VOLT)
                motor.rightC.spin(FORWARD, right_speed, VOLT)
            else:
                if brain.timer.system() >= wait_stop:
                    waiting = False

            if events:
                for i in range(len(events)):
                    event = events[i]
                    if i not in completed_events:
                        # event = ('set_flag', {'intake_auto_halt': True}, [-1229, -1455])
                        if dist(robot.pos, event[2]) < params["event_look_ahead_dist"]:
                            # [('set_flag', {'intake_auto_halt': True}, [-1229, -1455])]
                            if event[0] == 'set_flag':
                                for key in event[1]:
                                    if hasattr(AutonomousFlags, key):
                                        setattr(AutonomousFlags, key, event[1][key])
                                        log("[Event] Set AutonomousFlag {} to {}".format(key, event[1][key]))
                                    elif hasattr(flags, key):
                                        setattr(flags, key, event[1][key])
                                        log("[Event] Set flag {} to {}".format(key, event[1][key]))
                                    else:
                                        log("[Event] No flag found for key {}".format(key))
                            # ('set_pneumatic', {'pneumatic': 'mogo', 'state': False}, [0, 0])
                            elif event[0] == 'set_pneumatic':
                                if hasattr(pneumatic, event[1]['pneumatic']):
                                    getattr(pneumatic, event[1]['pneumatic']).set(event[1]['state'])
                                    log("[Event] Set {} pneumatic to {}".format(event[1]['pneumatic'], event[1]['state']))
                                else:
                                    log("[Event] No {} pneumatic found".format(event[1]['pneumatic']))
                            
                            completed_events.add(i)

            done = path_handler.path_complete
            if done:
                log("Path complete")

            if params["timeout"] is not None:
                if brain.timer.system() > time_end:
                    done = True
                    motor.leftA.stop()
                    motor.leftB.stop()
                    motor.leftC.stop()
                    motor.rightA.stop()
                    motor.rightB.stop()
                    motor.rightC.stop()
                    log("Path timed out", LogLevel.WARNING)


            if not done:
                sleep(20, MSEC)
            else:
                AutonomousCommands.kill_motors(BRAKE)

class ControllerFunctions():
    @staticmethod
    def switch_mogo():
        log("Switched mogo pneumatic")
        pneumatic.mogo.set(not pneumatic.mogo.value())

    @staticmethod
    def switch_intake_height():
        if not flags.elevating:
            log("Switched intake height")
            pneumatic.intake.set(not pneumatic.intake.value())

    @staticmethod
    def switch_doinker():
        log("Switched doinker")
        # pneumatic.doinker.set(not pneumatic.doinker.value())
        # pneumatic.doinker_left.set(pneumatic.doinker_left.value())
        pneumatic.doinker.set(False)
        pneumatic.doinker_left.set(False)

    @staticmethod
    def manual_elevation():
        log("Elevation not on robot!", LogLevel.WARNING)
        # if pneumatic.elevation_bar_lift_left.value(): # elevation is up
        #     pneumatic.elevation_bar_lift_left.set(False)
        #     pneumatic.elevation_bar_lift_right.set(False)

        #     pneumatic.elevation_bar_lower_left.set(True)
        #     pneumatic.elevation_bar_lower_right.set(True)
        # else:
        #     pneumatic.elevation_bar_lift_left.set(True)
        #     pneumatic.elevation_bar_lift_right.set(True)

        #     pneumatic.elevation_bar_lower_left.set(False)
        #     pneumatic.elevation_bar_lower_right.set(False)
    
    @staticmethod
    def toggle_PTO():
        log("Toggled PTO pneumatics")
        log("PTOs not on robot!", LogLevel.WARNING)
        # pneumatic.PTO.set(not pneumatic.PTO.value())
    
    @staticmethod
    def zero_lady_brown():
        log("Zeroed wall stake rotation sensor")
        sensor.wallEncoder.set_position(0)

    @staticmethod
    def wallstake_macro():
        log("Driving off wall stake")
        flags.disable_drive = True
        drivetrain.drive_for(DirectionType.FORWARD, 1.6, DistanceUnits.IN, 80, VelocityUnits.PERCENT)
        flags.disable_drive = False

class Driver():
    def __init__(self, parent: Robot) -> None:
        """
        Setup driver. Runs at start of program!
        """
        self.robot = parent
        self.elevation_hold_duration = 0
        self.elevation_hold_duration_reset = 7
        self.LB_braketype = BrakeType.HOLD

        log("Driver object setup")
    
    def driver_setup(self) -> None:
        """
        Setup driver runtime things
        """
        sensor.intakeColor.set_light_power(100, PERCENT)
        pneumatic.doinker.set(False)
        pneumatic.doinker.set(False)

        # bind controller functions
        control.DOINKER.pressed(ControllerFunctions.switch_doinker)
        control.MOGO_GRABBER_TOGGLE.pressed(ControllerFunctions.switch_mogo)
        control.INTAKE_HEIGHT_TOGGLE.pressed(ControllerFunctions.switch_intake_height)
        control.LB_MACRO_HOME.pressed(self.robot.LB_PID.home)
        control.LB_MACRO_DESCORE.pressed(self.robot.LB_PID.descore)
        control.BACK_OFF_ALLIANCE_STAKE.pressed(ControllerFunctions.wallstake_macro)

    def run(self) -> None:
        """
        Runs driver control loop.
        """
        self.driver_setup()
        
        while True:
            self.loop()

    def loop(self) -> None:
        """
        Runs every loop cycle
        """
        # if flags.elevating:
        #     self.elevation_loop()
        # else:
        self.driver_loop()

    def drive_controls(self) -> None:
        turnVolts = -(control.DRIVE_TURN_AXIS.position() * 0.12) * 0.9
        forwardVolts = -control.DRIVE_FORWARD_AXIS.position() * 0.12

        # Spin motors and combine controller axes
        motor.leftA.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motor.leftB.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motor.leftC.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        
        motor.rightA.spin(FORWARD, forwardVolts - turnVolts, VOLT)
        motor.rightB.spin(FORWARD, forwardVolts - turnVolts, VOLT)
        motor.rightC.spin(FORWARD, forwardVolts - turnVolts, VOLT)
    
    def intake_controls(self) -> None:
        if (control.INTAKE_IN_HOLD.pressing()):
            motor.intakeFlex.spin(FORWARD, 12, VOLT)

            if flags.allow_intake_input:
                motor.intakeChain.spin(FORWARD, 100, PERCENT)
        elif (control.INTAKE_OUT_HOLD.pressing()):
            motor.intakeFlex.spin(REVERSE, 100, PERCENT)
            motor.intakeChain.spin(REVERSE, 100, PERCENT)
        elif not flags.elevating:
            motor.intakeFlex.stop()
            motor.intakeChain.stop()
    
    def lady_brown_controls(self) -> None:
        # WALL STAKES MOTORS
        if control.LB_MANUAL_UP.pressing():
            if not 250 < sensor.wallEncoder.angle() < 355:
                motor.ladyBrown.spin(FORWARD, 100, PERCENT)
                self.robot.LB_PID.enabled = False
            else:
                motor.ladyBrown.stop(BRAKE)
        elif control.LB_MANUAL_DOWN.pressing():
            if not (sensor.wallEncoder.angle() < 5 or sensor.wallEncoder.angle() > 350): # arm is lowered
                motor.ladyBrown.spin(REVERSE, 90, PERCENT)
                self.robot.LB_PID.enabled = False
            else:
                motor.ladyBrown.stop(BRAKE)
        elif not self.robot.LB_PID.enabled and not flags.elevating:
            motor.ladyBrown.stop(BRAKE)

    def driver_loop(self) -> None:
        if not flags.disable_drive:
            self.drive_controls()
        self.intake_controls()
        self.lady_brown_controls()

        if flags.elevating and self.robot.LB_PID.enabled:
            self.robot.LB_PID.enabled = False
            print("stopped PID")

        self.robot.color_sort_controller.sense()

# control objects
class LadyBrown():
    POS_LOAD = 28
    POS_DESCORE = 155
    def __init__(self, wall_motor) -> None:
        # self.pid = MultipurposePID(0.3, 0.5, 0.01, 2, None)
        # self.pid = MultipurposePID(0.035, 0.05, 0.004, 2, None)
        self.pid = MultipurposePID(0.25, 0.1, 0, 2, None)
        self.enabled = False
        self.autostop = False
        self.threshold = 5
        self.motor: Motor = wall_motor
        self.target_rotation = 0
        self.thread: Thread
        self.in_range = False

        self.sensor: Rotation | Encoder = sensor.wallEncoder
        self.last_enabled = self.enabled
    
    def home(self):
        """
        Homes PID to ready to load position
        """
        self.target_rotation = LadyBrown.POS_LOAD
        self.enabled = True
        self.in_range = True
        log("Homing lady brown")
    
    def descore(self):
        """
        Run PID to try and unwinch the winches and prep for next ladder
        """
        self.target_rotation = LadyBrown.POS_DESCORE
        self.enabled = True
        self.in_range = True
        log("Descore lady brown")

    def run(self):
        """
        Run once to start PID thread.
        """
        log("Starting lady brown thread")
        while True:
            if self.enabled and self.in_range:
                self.loop()

            if (not self.enabled) and self.last_enabled:
                self.pid.integral = 0
            
            self.last_enabled = self.enabled
            # motor.ladyBrown.stop(BrakeType.COAST)
            
            sleep(20, MSEC)

    def loop(self):
        """
        Runs once every loop.
        """
        angle = self.sensor.position()
        if angle > 270:
            angle = 360 - angle
        
        output = self.pid.calculate(self.target_rotation, angle)
        # if abs(self.pid.error) > 40: 
        #     self.pid.integral = 0
        #     output *= 4
        # print(sensor.wallEncoder.angle(), output)

        # # debug data
        data = {
            "o": str(round(output, 2)),
            "error": str(round(self.target_rotation-angle, 2)),
            "integral": str(round(self.pid.integral, 2)),
            "integralp": str(round(self.pid.integral_processed, 2)),
            "d": str(round(self.pid.error * self.pid.kP, 2)),
            "p": str(round(self.pid.derivative * self.pid.kD))
        }
        packet_mgr.add_packet("lb", data)

        # limit output
        if output < -8:
            output = -8
        elif output > 8:
            output = 8

        self.motor.spin(FORWARD, output, VOLT)

        if abs(self.pid.error) < self.threshold and self.autostop:
            self.autostop = False
            self.enabled = False
            self.motor.stop(BrakeType.HOLD)
            log("LB PID Autostop")
            self.pid.integral = 0
            self.threshold = 5

class ColorSort:
    NONE = 0
    EJECT_BLUE = 1
    EJECT_RED = 2

class ColorSortController():
    BLUE = 195
    RED = 20
    def __init__(self, parent: Robot) -> None:
        self.robot = parent
        
        self.ejecting = False
        self.eject_next_ring = False
        self.give_up_threshold = 20

        self.blue_hue = ColorSortController.BLUE
        self.red_hue = ColorSortController.RED

        self.wait_time_ms = 210
        self.hold_time_ms = 150
    
    def sense(self) -> None:
        """
        Run to detect if we have an incorrectly colored disk.
        If so, queue an eject of the next dist to the top of the intake.
        """
        if not flags.color_setting == ColorSort.NONE and not self.ejecting:
            # Color flagging
            if sensor.intakeColor.is_near_object() and not self.eject_next_ring:
                if sensor.intakeColor.hue() < self.red_hue:
                    if flags.color_setting == ColorSort.EJECT_RED:
                        self.eject_next_ring = True
                        log("Queued [red] ring to eject. Hue: {}".format(sensor.intakeColor.hue()))
                
                if sensor.intakeColor.hue() > self.blue_hue:
                    if flags.color_setting == ColorSort.EJECT_BLUE:
                        self.eject_next_ring = True
                        log("Queued [blue] ring to eject. Hue: {}".format(sensor.intakeColor.hue()))

            if self.eject_next_ring:
                self.give_up_threshold -= 1
                
                if self.give_up_threshold <= 0:
                    self.eject_next_ring = False
                    log("Color sort timed out!")
            else:
                self.give_up_threshold = 20

            # color flag is on, now ring is up intake
            if ((sensor.intakeDistance.object_distance() < 70) and (self.eject_next_ring)):
                log("Ejecting ring!")

                self.allow_intake_input = False
                self.eject_next_ring = False
                self.ejecting = True
                command = motor.intakeChain.command(VelocityUnits.PERCENT)

                sleep(60, TimeUnits.MSEC)
                motor.intakeChain.spin(REVERSE, 70, PERCENT)
                brain.timer.event(self.stop_eject, 650, (command,))

    def stop_eject(self, speed):
        log("Done ejecting ring!")
        
        if speed > 0: motor.intakeChain.spin(FORWARD, speed, VelocityUnits.PERCENT)
        else: motor.intakeChain.spin(REVERSE, speed, VelocityUnits.PERCENT)
        
        self.allow_intake_input = True
        self.ejecting = False

class flags():
    """
    'global' booleans enum for various states.
    """
    color_setting = ColorSort.NONE
    allow_intake_input = True
    elevating = False
    wall_setpoint = 1
    disable_drive = False

def pull_data(data: dict, robot: Robot):
    """
    Assigns data variables to different objects.
    """
    pass

def null_function():
    pass

def odom_packets(robot: Robot):
    if not foxglove_log:
        return
    while True:
        data = {
            "x": round(robot.pos[0] / 25.4, 2),
            "y": round(robot.pos[1] / 25.4, 2),
            "theta": round(math.radians(sensor.imu.heading()), 2)
        }
        packet_mgr.add_packet("odometry", data)
        sleep(100, MSEC)

def start_odom_packets(robot: Robot):
    Thread(odom_packets, (robot,))

# run file
foxglove_log = False
do_logging = True
robot: Robot
log("Battery at {}".format(brain.battery.capacity()))
def main():
    global robot, do_logging
    sd_fail = False
    try:
        with open("config.json", 'r') as f:
            data = load(f)
        log("SUCCESS LOADING SD CARD")
    except:
        sd_fail = True
        log("ERROR LOADING SD CARD DATA", LogLevel.FATAL)

    if not sd_fail:
        robot = Robot()
        robot.LB_PID.thread = Thread(robot.LB_PID.run)

        comp = Competition(null_function, null_function)

        if not (comp.is_field_control() or comp.is_competition_switch()):
            brain.timer.event(start_odom_packets, 2000, (robot,))

        # Load autonomous into robot
        robot.autonomous_controller.load_path(data["autons"]["selected"])
        # robot.autonomous_controller.load_path("blue_positive_solo_AWP")

        con.screen.print(data["autons"]["selected"])

        pull_data(data, robot) # send data from SD data (local scope) into robot object & subobjects
        # when connected to the field, do everything normally
        if comp.is_field_control() or comp.is_competition_switch():
            log("Connected to field or comp switch.")
            
            comp = Competition(robot.driver, robot.autonomous)

            calibrate_imu()

            brain.screen.clear_screen(Color.GREEN)
            if brain.battery.capacity() < 25:
                brain.screen.clear_screen(Color.RED)

            log("Ready for match! Disabling log messages.")
            do_logging = False
        # not connected to field, auton test is on
        elif data["config"]["auton_test"]:
            calibrate_imu()

            brain.screen.clear_screen(Color.YELLOW)

            robot.autonomous_controller.test()
        # not connected to field, auton test is not on
        else:
            log("Default to driver (no auton test, no field control)", LogLevel.WARNING)

            # calibrate_imu()

            brain.screen.clear_screen(Color.CYAN)

            robot.driver_controller.run()

    else:
        log("Robot object not created. (SD Load Error)", LogLevel.FATAL)
        raise ImportError("SD Card not inserted")

main()
