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
    log("calibrating IMU. Minimum calibration time 2 seconds.")
    min_duration = brain.timer.system() + 2000
    calibrating = True

    sensor.imu.calibrate()
    while calibrating:
        calibrating = sensor.imu.is_calibrating() and brain.timer.system() < 2000
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

# robot states
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

        self.fwd_speed = 8

        drivetrain.set_timeout(1.5, TimeUnits.SECONDS)
        drivetrain.set_turn_threshold(5)
        drivetrain.set_turn_constant(0.6)

        log("Autonomous object setup")
    
    def autonomous_setup(self) -> None:
        """
        Call at start of auton. Sets up the coordinates n stuff for the loaded path.
        """
        # if pos_override is not None:
        try:
            sensor.imu.set_heading(self.autonomous_data["start_heading"])
            self.robot.heading = (self.autonomous_data["start_heading"])
            self.robot.pos = list(self.autonomous_data["start_pos"])
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

    def load_path(self, module_filename: str) -> None:
        """
        Loads autonomous data into object variables for later use.

        Args:
            module_filename: The filename of the imported auton, without any file extensions.
        """
        log("LOAD [{}]".format(module_filename))
        try:
            log("Importing module...")
            auton_module = __import__("{}".format(module_filename), None, None, ["run"])

            log("Importing sequence...")
            original_function = getattr(auton_module, "run")  # Reference

            # Create a new function that copies run
            def backup_run():
                return original_function(globals())

            self.sequence = backup_run  # Now it's independent of auton_module

            log("Importing data...")
            gen_data = getattr(auton_module, "gen_data")
            self.autonomous_data = gen_data()

            brain.screen.set_cursor(1,1)
            brain.screen.set_font(FontType.MONO30)
            brain.screen.print("Loaded autonomous off of SD card")

            log("Loaded data into Autonomous")
        except:
            log("Auton not recognized!", LogLevel.ERROR)
            raise ImportError("Can't find auton file")
    
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

        self.robot.pos = [-1545.0, 0.0]
        sensor.imu.set_heading(90)

        # controller.mcl_controller.start()

        # place temporary / testing code here
        AutonomousFlags.intake_anti_jam = True

        #* Skills
        controller = self
        drivetrain.set_turn_threshold(2)
        drivetrain.set_turn_constant(0.28)
        paths = [[(-1593.76, 6.33, 0), (-1523.77, 5.76, 0.02), (-1453.78, 4.66, 0.01), (-1383.79, 3.36, 0.0), (-1313.8, 2.02, 0.0), (-1243.81, 0.76, 0.01), (-1173.82, -0.25, 0.04)], [(-1201.9, -9.15, 0), (-1201.64, -79.15, 0.04), (-1202.38, -149.14, 0.05), (-1204.43, -219.11, 0.05), (-1207.76, -289.03, 0.02), (-1211.54, -358.93, 0.03), (-1214.66, -428.86, 0.05), (-1216.65, -498.83, 0.04), (-1217.63, -568.82, 0.03), (-1217.77, -638.82, 0.03), (-1217.26, -708.82, 0.03)], [(-1244.4, -726.06, 0), (-1239.23, -656.36, 0.62), (-1219.12, -589.65, 0.93), (-1178.74, -533.1, 1.11), (-1119.66, -496.47, 0.91), (-1051.8, -480.43, 0.63), (-981.94, -479.64, 0.43), (-912.71, -489.49, 0.32), (-844.99, -507.01, 0.25), (-779.06, -530.46, 0.23), (-715.23, -559.17, 0.21), (-653.69, -592.48, 0.21), (-594.82, -630.29, 0.25), (-539.42, -673.02, 0.32), (-489.23, -721.71, 0.35), (-434.68, -763.2, 0.52), (-372.14, -794.63, 0.08), (-310.47, -827.75, 0.04), (-249.22, -861.62, 0.08), (-188.9, -897.14, 0.07), (-129.43, -934.04, 0.05), (-70.67, -972.07, 0.08), (-13.05, -1011.82, 0.09), (43.27, -1053.38, 0.08), (98.37, -1096.53, 0.1), (151.98, -1141.51, 0.11), (203.77, -1188.58, 0.14), (253.28, -1238.05, 0.16), (299.87, -1290.27, 0.2), (342.6, -1345.66, 0.26), (380.07, -1404.72, 0.46), (407.56, -1468.92, 0.81), (415.8, -1537.77, 1.83), (380.8, -1595.35, 1.94), (314.74, -1615.29, 1.01), (245.1, -1610.63, 0.44), (176.88, -1595.25, 0.29), (110.55, -1572.94, 0.2), (45.92, -1546.07, 0.15), (-17.22, -1515.85, 0.34), (-75.55, -1478.9, 0.72), (-123.29, -1427.86, 0.42), (-178.05, -1384.38, 0.33), (-237.49, -1347.49, 0.24), (-299.82, -1315.7, 0.2), (-364.24, -1288.36, 0.18), (-430.24, -1265.06, 0.13), (-497.23, -1244.79, 0.11), (-564.96, -1227.14, 0.1), (-633.24, -1211.78, 0.08), (-701.96, -1198.45, 0.07), (-771.0, -1186.92, 0.07), (-840.29, -1177.0, 0.06), (-909.79, -1168.65, 0.06), (-979.44, -1161.68, 0.05), (-1049.2, -1155.95, 0.05), (-1119.05, -1151.37, 0.05), (-1188.96, -1147.93, 0.05), (-1258.93, -1145.72, 0.05), (-1328.92, -1144.63, 0.06), (-1398.92, -1144.91, 0.01), (-1468.88, -1145.47, 0.06), (-1538.84, -1144.64, 0.3), (-1608.45, -1151.12, 0.6), (-1674.88, -1171.93, 1.69), (-1716.31, -1224.59, 2.41), (-1695.65, -1290.05, 1.13), (-1650.04, -1342.89, 0.48), (-1596.09, -1387.38, 0.28), (-1537.96, -1426.33, 0.23), (-1476.84, -1460.42, 0.16), (-1413.94, -1491.11, 0.15), (-1349.51, -1518.45, 0.14), (-1283.8, -1542.55, 0.12), (-1217.13, -1563.82, 0.12), (-1149.63, -1582.32, 0.12), (-1081.43, -1598.06, 0.12), (-1012.66, -1611.04, 0.11)], [(-1006.48, -1634.53, 0), (-1054.61, -1583.7, 0.08), (-1104.11, -1534.22, 0.16), (-1156.29, -1487.59, 0.25), (-1212.29, -1445.65, 0.36), (-1273.12, -1411.15, 0.55), (-1339.34, -1388.98, 0.74), (-1408.88, -1384.53, 0.85), (-1476.57, -1400.85, 0.78), (-1537.32, -1435.07, 0.57), (-1590.08, -1480.89, 0.45), (-1635.0, -1534.5, 0.28), (-1674.48, -1592.24, 0.22), (-1709.44, -1652.86, 0.17), (-1740.71, -1715.46, 0.13), (-1769.03, -1779.46, 0.1), (-1795.0, -1844.44, 0.06)], [(-1587.87, -1546.77, 0), (-1557.23, -1483.83, 0.14), (-1529.8, -1419.43, 0.12), (-1505.12, -1353.93, 0.1), (-1482.74, -1287.61, 0.08), (-1462.21, -1220.69, 0.06), (-1443.15, -1153.34, 0.05), (-1425.18, -1085.68, 0.03), (-1408.0, -1017.83, 0.02), (-1391.31, -949.84, 0.01), (-1374.87, -881.8, 0.0), (-1358.45, -813.76, 0.01), (-1341.77, -745.77, 0.02), (-1324.59, -677.91, 0.03), (-1306.78, -610.22, 0.03), (-1288.2, -542.73, 0.04), (-1268.72, -475.5, 0.05), (-1247.99, -408.64, 0.06), (-1225.86, -342.23, 0.06), (-1202.43, -276.28, 0.06), (-1177.64, -210.82, 0.07), (-1151.21, -146.0, 0.09), (-1122.74, -82.06, 0.07), (-1092.71, -18.84, 0.07), (-1061.14, 43.63, 0.1), (-1027.39, 104.95, 0.09), (-991.64, 165.12, 0.07), (-954.38, 224.36, 0.09), (-915.26, 282.41, 0.11), (-873.85, 338.83, 0.07), (-831.1, 394.24, 0.09), (-786.58, 448.26, 0.11), (-740.0, 500.49, 0.06), (-692.39, 551.79, 0.11), (-642.75, 601.14, 0.08), (-591.83, 649.15, 0.07), (-539.81, 695.99, 0.08)], [(-468.96, 756.87, 0), (-508.51, 699.12, 0.12), (-550.5, 643.13, 0.19), (-596.08, 590.04, 0.28), (-646.59, 541.67, 0.4), (-703.28, 500.76, 0.54), (-766.61, 471.28, 0.62), (-834.8, 456.3, 0.59), (-904.65, 455.66, 0.49), (-973.62, 467.09, 0.37), (-1040.56, 487.35, 0.26), (-1105.39, 513.69, 0.22), (-1168.0, 544.96, 0.14), (-1228.95, 579.36, 0.12), (-1288.45, 616.22, 0.1), (-1346.68, 655.07, 0.08), (-1403.81, 695.52, 0.09)], [(-1305.43, 655.02, 0), (-1242.98, 623.4, 0.12), (-1179.28, 594.42, 0.15), (-1114.2, 568.76, 0.18), (-1047.65, 547.21, 0.21), (-979.67, 530.71, 0.25), (-910.52, 520.27, 0.29), (-840.67, 516.89, 0.34), (-770.92, 521.85, 0.41), (-702.58, 536.66, 0.36), (-636.66, 559.98, 0.42), (-574.86, 592.7, 0.32), (-517.13, 632.2, 0.33), (-464.3, 678.08, 0.22), (-415.19, 727.92, 0.2), (-369.65, 781.07, 0.12), (-326.36, 836.07, 0.07), (-284.39, 892.09, 0.01), (-242.31, 948.04, 0.1), (-198.26, 1002.42, 0.27), (-149.4, 1052.44, 0.85), (-88.67, 1085.53, 0.1), (-26.12, 1116.89, 0.28), (33.0, 1154.32, 0.24), (88.82, 1196.5, 0.23), (141.1, 1243.01, 0.25), (189.05, 1293.97, 0.31), (231.2, 1349.81, 0.43), (264.47, 1411.27, 0.76), (280.14, 1479.05, 1.49), (259.38, 1544.14, 1.81), (201.98, 1582.32, 1.04), (133.83, 1597.25, 0.51), (63.94, 1599.71, 0.28), (-5.89, 1595.38, 0.2), (-75.28, 1586.18, 0.36), (-142.61, 1568.44, 0.59), (-205.11, 1537.08, 0.5), (-261.11, 1495.31, 0.59), (-307.39, 1442.98, 0.4), (-346.02, 1384.63, 0.12), (-386.98, 1327.94, 0.55), (-437.97, 1280.23, 0.6), (-497.93, 1244.35, 0.44), (-562.73, 1218.06, 0.29), (-629.9, 1198.48, 0.21), (-698.33, 1183.79, 0.14), (-767.38, 1172.35, 0.11), (-836.82, 1163.53, 0.01), (-906.25, 1154.91, 0.11), (-975.34, 1143.6, 0.1), (-1044.77, 1134.79, 0.08), (-1114.41, 1127.9, 0.09), (-1184.25, 1123.3, 0.11), (-1254.22, 1121.46, 0.14), (-1324.2, 1123.01, 0.16), (-1394.0, 1128.36, 0.19), (-1463.27, 1138.3, 0.23), (-1531.48, 1153.86, 0.28), (-1597.83, 1175.99, 0.41), (-1660.25, 1207.34, 0.61), (-1714.39, 1251.21, 1.04), (-1748.95, 1311.21, 1.73), (-1741.11, 1379.41, 1.67), (-1694.78, 1430.87, 1.01), (-1632.96, 1463.04, 0.6), (-1565.58, 1481.47, 0.42), (-1496.13, 1489.79, 0.29), (-1426.18, 1491.12, 0.18), (-1357.15, 1496.83, 0.18), (-1287.96, 1507.06, 0.34), (-1218.02, 1508.99, 0.28), (-1148.23, 1504.0, 0.21), (-1078.98, 1493.83, 0.14), (-1010.31, 1480.27, 0.07), (-942.01, 1464.94, 0.02)], [(-862.7, 1442.86, 0), (-931.01, 1458.07, 0.19), (-1000.17, 1468.76, 0.21), (-1069.92, 1474.31, 0.2), (-1139.9, 1474.95, 0.17), (-1209.8, 1471.31, 0.06), (-1279.6, 1466.11, 0.2), (-1349.53, 1465.72, 0.83), (-1416.05, 1485.35, 1.33), (-1465.99, 1533.43, 0.9), (-1498.48, 1595.26, 0.41), (-1521.89, 1661.18, 0.23), (-1540.02, 1728.78, 0.12), (-1555.39, 1797.07, 0.08), (-1568.85, 1865.76, 0.05), (-1580.99, 1934.69, 0.03)], [(-1775.99, 1786.6, 0), (-1728.56, 1735.11, 0.31), (-1676.04, 1689.09, 0.27), (-1619.37, 1648.23, 0.23), (-1559.43, 1612.1, 0.31), (-1495.95, 1582.6, 0.15), (-1431.04, 1556.58, 0.13), (-1364.99, 1533.44, 0.19), (-1297.53, 1514.75, 0.09), (-1229.54, 1498.26, 0.08), (-1161.07, 1483.67, 0.13), (-1092.04, 1472.13, 0.04), (-1022.86, 1461.49, 0.09), (-953.38, 1452.96, 0.04), (-883.79, 1445.47, 0.05), (-814.08, 1439.08, 0.05), (-744.28, 1433.88, 0.02), (-674.44, 1429.16, 0.05), (-604.54, 1425.55, 0.01), (-534.62, 1422.14, 0.03), (-464.67, 1419.46, 0.01), (-394.72, 1416.91, 0.01), (-324.75, 1414.68, 0.0), (-254.79, 1412.48, 0.0), (-184.82, 1410.31, 0.01), (-114.86, 1407.91, 0.0), (-44.91, 1405.41, 0.03), (25.02, 1402.2, 0.01), (94.93, 1398.65, 0.04), (164.78, 1394.15, 0.05), (234.55, 1388.55, 0.03), (304.25, 1382.12, 0.1), (373.7, 1373.3, 0.08), (442.84, 1362.52, 0.1), (511.57, 1349.39, 0.15), (579.55, 1332.67, 0.24), (645.89, 1310.4, 0.28), (709.69, 1281.81, 0.4), (768.8, 1244.66, 0.56), (819.39, 1196.68, 0.73), (856.05, 1137.38, 0.84), (873.81, 1069.82, 0.76), (873.12, 1000.04, 0.11), (869.81, 930.2, 0.3), (873.73, 860.4, 0.33), (885.72, 791.53, 0.36), (906.36, 724.72, 0.46), (937.37, 662.06, 0.41), (977.11, 604.57, 0.44), (1025.12, 553.8, 0.46), (1080.77, 511.48, 0.42), (1142.0, 477.79, 0.44), (1207.75, 454.01, 0.45), (1276.38, 440.91, 0.47), (1346.22, 439.27, 0.56), (1415.01, 451.38, 0.72), (1478.47, 480.38, 0.94), (1528.75, 528.44, 1.1), (1556.86, 591.92, 1.0), (1561.21, 661.45, 0.71), (1548.27, 729.95, 1.05), (1561.38, 798.71, 0.09), (1572.4, 867.83, 0.09), (1581.19, 937.27, 0.09), (1587.71, 1006.96, 0.11), (1591.57, 1076.85, 0.08), (1593.58, 1146.82, 0.03), (1594.8, 1216.81, 0.06), (1597.61, 1286.75, 0.2), (1605.35, 1356.28, 0.32), (1620.88, 1424.49, 0.36), (1644.95, 1490.18, 0.31), (1676.05, 1552.86, 0.22), (1711.91, 1612.95, 0.17), (1751.23, 1670.85, 0.11), (1792.82, 1727.14, 0.09), (1836.15, 1782.12, 0.07), (1880.78, 1836.05, 0.05), (1926.39, 1889.14, 0.04)], [(1816.33, 2050.0, 0), (1782.62, 1988.65, 0.06), (1750.1, 1926.66, 0.05), (1718.7, 1864.1, 0.05), (1688.39, 1801.01, 0.05), (1659.1, 1737.43, 0.05), (1630.89, 1673.37, 0.05), (1603.88, 1608.79, 0.04), (1577.8, 1543.84, 0.04), (1552.61, 1478.53, 0.04), (1528.29, 1412.89, 0.05), (1505.03, 1346.87, 0.04), (1482.73, 1280.52, 0.03), (1461.23, 1213.9, 0.03), (1440.51, 1147.04, 0.04), (1420.73, 1079.9, 0.04), (1401.91, 1012.48, 0.03), (1383.82, 944.85, 0.03), (1366.47, 877.04, 0.04), (1350.06, 808.99, 0.04), (1334.53, 740.74, 0.03), (1319.72, 672.32, 0.03), (1305.6, 603.76, 0.04), (1292.52, 535.0, 0.03), (1280.21, 466.09, 0.03), (1268.61, 397.06, 0.03), (1257.75, 327.91, 0.04), (1247.97, 258.59, 0.03), (1238.91, 189.18, 0.03), (1230.56, 119.69, 0.04), (1223.11, 50.08, 0.04), (1216.66, -19.62, 0.03), (1210.95, -89.38, 0.03), (1206.0, -159.21, 0.04), (1202.11, -229.1, 0.04), (1199.16, -299.03, 0.03), (1197.02, -369.0, 0.03)], [(1123.34, -482.5, 0), (1133.58, -413.26, 0.05), (1142.59, -343.84, 0.06), (1150.1, -274.25, 0.06), (1156.06, -204.51, 0.07), (1160.43, -134.65, 0.07), (1163.03, -64.7, 0.08), (1163.66, 5.29, 0.09), (1162.06, 75.27, 0.11), (1157.77, 145.13, 0.13), (1150.36, 214.73, 0.15), (1139.43, 283.85, 0.17), (1124.35, 352.18, 0.21), (1104.37, 419.23, 0.27), (1078.2, 484.1, 0.36), (1044.06, 545.13, 0.39), (1001.87, 600.84, 0.53), (950.21, 647.79, 0.58), (890.13, 683.33, 0.54), (824.39, 706.93, 0.57), (755.21, 716.89, 0.42), (685.31, 716.54, 0.34), (615.93, 707.92, 0.25), (547.54, 693.17, 3.21), (530.56, 642.3, 0.81), (527.94, 572.71, 0.76), (506.86, 506.23, 0.57), (472.96, 445.17, 0.45), (429.79, 390.16, 0.33), (380.61, 340.39, 0.21), (327.98, 294.27, 0.14), (273.11, 250.83, 0.09), (216.82, 209.22, 0.05), (159.79, 168.64, 0.01), (102.59, 128.28, 0.04), (45.99, 87.1, 0.12), (-8.88, 43.66, 0.22), (-60.29, -3.78, 0.55), (-101.53, -60.0, 1.22), (-115.17, -127.19, 1.82), (-83.78, -188.31, 1.08), (-30.89, -233.68, 0.52), (29.6, -268.79, 0.25), (92.95, -298.53, 0.18), (158.09, -324.14, 0.12), (224.23, -347.04, 0.05), (290.77, -368.73, 0.04), (356.81, -391.22, 0.55), (417.5, -426.0, 0.3), (474.13, -467.05, 0.35), (525.23, -514.74, 0.39), (569.38, -568.9, 0.4), (605.44, -628.79, 0.45), (631.63, -693.6, 0.38), (649.03, -761.33, 0.31), (659.06, -830.56, 0.29), (662.03, -900.47, 0.19), (660.28, -970.43, 0.14), (655.16, -1040.23, 0.1), (647.53, -1109.81, 0.04), (639.03, -1179.3, 0.02), (630.97, -1248.83, 0.09), (625.19, -1318.58, 0.19), (624.14, -1388.53, 0.35), (631.62, -1458.04, 0.64), (654.44, -1524.02, 0.95), (697.62, -1578.68, 1.07), (757.53, -1613.19, 1.17), (827.17, -1620.3, 0.01), (896.78, -1627.66, 0.02), (966.45, -1634.49, 0.05), (1036.21, -1640.19, 0.08), (1106.11, -1643.92, 0.11), (1176.09, -1644.86, 0.16), (1246.01, -1641.99, 0.22), (1315.49, -1633.75, 0.32), (1383.61, -1617.87, 0.46), (1448.24, -1591.33, 0.64), (1505.26, -1551.17, 0.79), (1549.06, -1496.99, 0.8), (1575.98, -1432.66, 0.66), (1587.39, -1363.76, 0.39)], [(1578.64, -1280.07, 0), (1576.95, -1350.04, 0.2), (1580.09, -1419.95, 0.3), (1590.45, -1489.12, 0.43), (1610.97, -1555.95, 0.56), (1644.06, -1617.45, 0.63), (1689.88, -1670.14, 0.6), (1745.74, -1712.11, 0.49), (1808.01, -1743.92, 0.37), (1873.92, -1767.38, 0.28), (1941.82, -1784.31, 0.23)], [(1975.49, -1926.9, 0), (1915.22, -1891.3, 0.0), (1854.95, -1855.7, 0.11), (1796.1, -1817.81, 0.02), (1737.56, -1779.43, 0.12), (1680.68, -1738.67, 0.03), (1624.19, -1697.32, 0.15), (1569.95, -1653.08, 0.04), (1516.41, -1608.02, 0.14), (1465.17, -1560.33, 0.1), (1415.74, -1510.82, 0.09), (1367.86, -1459.76, 0.19), (1323.52, -1405.59, 0.12), (1281.59, -1349.6, 0.13), (1242.3, -1291.7, 0.16), (1206.35, -1231.64, 0.2), (1174.61, -1169.26, 0.15), (1146.3, -1105.29, 0.16), (1121.53, -1039.86, 0.15), (1100.33, -973.18, 0.15), (1082.61, -905.47, 0.14), (1068.28, -836.95, 0.13), (1057.19, -767.84, 0.1), (1048.53, -698.39, 0.08), (1041.89, -628.71, 0.07), (1036.87, -558.89, 0.05), (1033.03, -489.0, 0.03), (1029.95, -419.07, 0.01), (1027.2, -349.12, 0.01), (1024.3, -279.18, 0.03), (1020.74, -209.27, 0.04), (1016.13, -139.43, 0.06), (1010.14, -69.69, 0.07), (1002.42, -0.13, 0.08), (992.68, 69.18, 0.1), (980.49, 138.11, 0.13), (965.14, 206.4, 0.12), (946.95, 273.98, 0.09)], [(906.26, 358.94, 0), (925.99, 291.89, 0.63), (959.98, 230.85, 0.66), (1007.08, 179.24, 0.56), (1063.42, 137.84, 0.44), (1125.55, 105.69, 0.33), (1190.97, 80.91, 0.23), (1258.19, 61.44, 0.19), (1326.53, 46.37, 0.13), (1395.49, 34.36, 0.11), (1464.86, 25.07, 0.07), (1534.46, 17.58, 0.06), (1604.2, 11.57, 0.05), (1674.03, 6.75, 0.04), (1743.92, 2.88, 0.03), (1813.85, -0.33, 0.02), (1883.8, -3.06, 0.01), (1953.76, -5.45, 0.01), (2023.72, -7.64, 0.0), (2093.69, -9.74, 0.0)], [(2046.79, -45.01, 0), (1976.83, -47.4, 0.1), (1906.84, -47.26, 0.13), (1836.93, -43.92, 0.16), (1767.32, -36.73, 0.19), (1698.34, -24.94, 0.27), (1630.85, -6.59, 0.33), (1565.91, 19.35, 0.45), (1505.87, 55.21, 0.54), (1453.71, 101.74, 0.59), (1412.19, 157.95, 0.53), (1381.79, 220.89, 0.43), (1361.12, 287.7, 0.31), (1347.92, 356.42, 0.22), (1340.04, 425.96, 0.16), (1335.99, 495.84, 0.09), (1334.15, 565.81, 0.05), (1333.58, 635.81, 0.05), (1334.25, 705.8, 0.13), (1338.09, 775.69, 0.1), (1344.3, 845.41, 0.04), (1351.47, 915.04, 0.07), (1356.83, 984.83, 0.31), (1354.5, 1054.67, 0.81), (1332.71, 1120.58, 1.3), (1283.67, 1169.46, 1.02), (1219.99, 1197.91, 0.53), (1151.97, 1214.11, 0.27), (1082.67, 1223.84, 0.18), (1012.87, 1229.08, 0.1), (942.93, 1231.79, 0.07), (872.94, 1232.68, 0.05), (802.94, 1232.23, 0.04), (732.96, 1230.82, 0.06), (663.03, 1227.82, 0.06), (593.06, 1226.36, 0.27), (523.28, 1231.41, 0.28), (454.34, 1243.19, 0.36), (387.46, 1263.65, 0.45), (324.63, 1294.37, 0.51), (268.33, 1335.81, 0.53), (220.64, 1386.9, 0.5), (182.52, 1445.48, 0.43), (153.65, 1509.16, 0.36), (132.92, 1575.97, 0.29), (118.97, 1644.53, 0.23), (110.49, 1714.0, 0.18), (106.42, 1783.86, 0.14), (105.86, 1853.85, 0.14)], [(-7.83, 1891.55, 0), (-14.8, 1821.9, 0.17), (-17.66, 1752.0, 0.19), (-15.76, 1682.09, 0.22), (-8.46, 1612.55, 0.25), (4.82, 1543.89, 0.27), (24.52, 1476.79, 0.29), (50.88, 1412.03, 0.3), (83.92, 1350.41, 0.31), (123.4, 1292.7, 0.31), (168.86, 1239.58, 0.3), (219.7, 1191.56, 0.29), (275.22, 1149.0, 0.3), (334.91, 1112.5, 0.28), (397.91, 1082.07, 0.26), (463.37, 1057.41, 0.25), (530.71, 1038.49, 0.24), (599.41, 1025.27, 0.24), (668.98, 1017.76, 0.27), (738.95, 1016.8, 0.25), (808.72, 1021.85, 0.25), (877.77, 1033.05, 0.27), (945.47, 1050.69, 0.35), (1010.51, 1076.42, 0.34), (1072.03, 1109.64, 0.35)], [(1133.88, 1148.65, 0), (1086.49, 1097.13, 0.03), (1039.67, 1045.1, 0.01), (993.07, 992.86, 0.01), (946.59, 940.53, 0.0), (900.19, 888.11, 0.0), (853.86, 835.64, 0.0), (807.58, 783.12, 0.0), (761.36, 730.55, 0.0), (715.17, 677.95, 0.0), (669.03, 625.31, 0.0), (622.93, 572.63, 0.0), (576.86, 519.93, 0.0), (530.84, 467.19, 0.0), (484.85, 414.41, 0.0), (438.91, 361.6, 0.0), (393.01, 308.74, 0.0), (347.17, 255.85, 0.0), (301.38, 202.89, 0.0), (255.67, 149.88, 0.01), (210.06, 96.78, 0.01), (164.57, 43.58, 0.01), (119.29, -9.81, 0.02), (74.42, -63.53, 0.08)]]

        controller.robot.pos[0] = -1545.0

        motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        sleep(350, TimeUnits.MSEC)        
        motor.intakeChain.stop(BrakeType.COAST)

        log(controller.robot.pos)

        # drive away from wall
        controller.fwd_speed = 7
        controller.path(paths[0], 
                backwards=False,
                heading_authority=2.5,
                finish_margin=200, slowdown_distance=0)
            
        # turn to face close mogo with rear of robot
        drivetrain.set_turn_threshold(10)
        drivetrain.set_turn_constant(0.8)
        drivetrain.turn_for(TurnType.LEFT, 65, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
        sleep(100, TimeUnits.MSEC)

        # grab mogo path
        controller.fwd_speed = 6
        controller.path(paths[1], backwards=True, heading_authority=1, slowdown_distance=0, speed_ramp_time=300)
        
        pneumatic.mogo.set(True)
        sleep(300, TimeUnits.MSEC)

        motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

        controller.fwd_speed = 5
        # drive across 1st ring and into 2nd ring and wall stake
        controller.path(paths[2], backwards=False, 
                heading_authority=1.3, look_ahead_dist=350, event_look_ahead_dist=300, 
                finish_margin=100, checkpoints=[15, 20, 30, 50])

        # drop mogo in corner
        controller.path(paths[3], backwards=True, timeout=1300, heading_authority=2)
        pneumatic.mogo.set(False)

        # recalibrate x and y since we're in the corner
        sleep(200, TimeUnits.MSEC)
        h = math.radians(sensor.imu.heading())
        d1 = sensor.wallLeftDistance.object_distance()
        if d1 < 900:
            x = d1 * math.cos(h)
            left_offset = 172.6 * math.cos(math.radians(-10) + h)
            x = -1785 + x + left_offset

            diff = x - controller.robot.pos[0]
            log("Recalibrate x pos from {} to {}. Diff: {}".format(controller.robot.pos[0], x, diff))
            controller.robot.pos[0] = x
        else:
            log("Couldn't calibrate left sensor!")

        d2 = sensor.wallRightDistance.object_distance()
        if d2 < 900:
            y = d2*math.sin(h)
            right_offset = 172.6 * math.sin(math.radians(-10) - h)
            y = -1785 + y - right_offset
            diff = y - controller.robot.pos[1]
            log("Recalibrate y pos from {} to {}. Diff: {}".format(controller.robot.pos[1], y, diff))
            controller.robot.pos[1] = y
        else:
            log("Couldn't calibrate right distance!")
        log("L: {} R: {}".format(d1, d2))   

        motor.intakeChain.spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
        sleep(250, TimeUnits.MSEC)
        motor.intakeChain.stop(BrakeType.COAST)

        # cross field from bottom left goal after drop
        controller.fwd_speed = 7
        controller.path(paths[4], slowdown_distance=1000)
        # grab top left mogo
        controller.fwd_speed = 6
        controller.path(paths[5], backwards=True, speed_ramp_time=400)
        pneumatic.mogo.set(True)
        sleep(300, TimeUnits.MSEC)
        # Intake and grab top left rings
        motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        controller.fwd_speed = 6
        controller.path(paths[6], speed_ramp_time=400, checkpoints=[15, 30, 40, 60, 70], slowdown_distance=700, min_slow_voltage=5)
        # drop mogo in corner
        controller.path(paths[7], backwards=True, timeout=1000)
        pneumatic.mogo.set(False)

        sleep(200, TimeUnits.MSEC)

        #* Top left corner recalibration
        # h = sensor.imu.heading()
        # theta_1 = h%90
        # print(theta_1)
        # h = math.radians(h)
        # d1 = sensor.wallRightDistance.object_distance()
        # if d1 < 900:
        #     x = abs(d1 * math.sin(math.radians(theta_1)))
        #     left_offset = 172.6 * abs(math.sin(math.radians(theta_1 + 10)))
        #     print("Right offset: {}, x: {}".format(left_offset, x))
        #     x = -1785 + x + left_offset

        #     diff = x - controller.robot.pos[0]
        #     log("Recalibrate x pos from {} to {}. Diff: {}".format(controller.robot.pos[0], x, diff))
        #     controller.robot.pos[0] = x
        # else:
        #     log("Couldn't calibrate left sensor!")

        # d2 = sensor.wallLeftDistance.object_distance()
        # if d2 < 900:
        #     y = abs(d2*math.cos(math.radians(theta_1)))
        #     right_offset = 172.6 * abs(math.cos(math.radians(theta_1-10)))
        #     print("Left offset: {}, y: {}".format(right_offset, y))
        #     y = 1785 - y - right_offset
        #     diff = y - controller.robot.pos[1]
        #     log("Recalibrate y pos from {} to {}. Diff: {}".format(controller.robot.pos[1], y, diff))
        #     controller.robot.pos[1] = y
        # else:
        #     log("Couldn't calibrate right distance!")

        motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        # extake to ensure we aren't caught on the goal
        motor.intakeChain.spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
        sleep(350, TimeUnits.MSEC)
        motor.intakeChain.stop(BrakeType.COAST)

        controller.fwd_speed = 7
        motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        AutonomousFlags.intake_auto_halt = True
        AutonomousFlags.intake_flex_auto_halt = True
        # top blue goal corner 
        controller.path(paths[8], speed_ramp_time=1000, min_start_voltage=2.5, timeout=7000, slowdown_distance=1000, look_ahead_dist=400, events=[["pres intake", (920.0, 689.8), pneumatic.intake.set, (True,)]])
        motor.intakeChain.stop(BrakeType.COAST)

        #* Top right distance recalibration
        # h = math.radians(sensor.imu.heading())
        # d1 = sensor.wallRightDistance.object_distance()
        # if d1 < 900:
        #     x = d1 * math.cos(h)
        #     left_offset = 172.6 * math.cos(math.radians(10) + h)
        #     x = 1785 - x - left_offset

        #     diff = x - controller.robot.pos[0]
        #     log("Recalibrate x pos from {} to {}. Diff: {}".format(controller.robot.pos[0], x, diff))
        #     controller.robot.pos[0] = x
        # else:
        #     log("Couldn't calibrate left sensor!")

        # d2 = sensor.wallFrontDistance.object_distance()
        # if d2 < 900:
        #     y = d2*math.cos(h)
        #     offset = 210 * math.sin(math.radians(38) + h)
        #     y = 1785 - y - offset
        #     diff = y - controller.robot.pos[1]
        #     log("Recalibrate y pos from {} to {}. Diff: {}".format(controller.robot.pos[1], y, diff))
        #     controller.robot.pos[1] = y
        # else:
        #     log("Couldn't calibrate right distance!")
        # log("R: {} F: {}".format(d1, d2))   
        # log("Recalibrated to {}".format(controller.robot.pos))

        AutonomousFlags.intake_auto_halt = False
        AutonomousFlags.intake_flex_auto_halt = False
        controller.fwd_speed = 6
        # back out of corner
        controller.path(paths[9], backwards=True, speed_ramp_time=300, heading_authority=1.8)
        pneumatic.intake.set(False)

        pneumatic.mogo.set(True)
        sleep(300, TimeUnits.MSEC)

        motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        controller.path(paths[10], speed_ramp_time=400, checkpoints=[30, 50], event_look_ahead_dist=150,
                    events=[
                            ["reverse", (1016.0, 508.0), motor.intakeChain.spin, (DirectionType.REVERSE, 100, VelocityUnits.PERCENT)],
                            ["stop", (510.0, 510.0), motor.intakeChain.stop, (BrakeType.COAST,)],
                            ["spin", (264.0, 299.7), motor.intakeChain.spin, (DirectionType.FORWARD, 100, VelocityUnits.PERCENT)],
                            ["reverse", (150.0, -170.0), motor.intakeChain.spin, (DirectionType.REVERSE, 100, VelocityUnits.PERCENT)],
                            ["spin", (630.0, -630.0), motor.intakeChain.spin, (DirectionType.FORWARD, 100, VelocityUnits.PERCENT)]
                        ])
        
        motor.intakeChain.stop()
        controller.path(paths[11], backwards=True, timeout=1200)
        pneumatic.mogo.set(False)

        # sleep(200, TimeUnits.MSEC)
        # #* Bottom right corner calibration
        # h = math.radians((sensor.imu.heading()))
        # d1 = sensor.wallRightDistance.object_distance()
        # if d1 < 900:
        #     x = d1 * abs(math.cos(h))
        #     left_offset = 172.6 * abs(math.cos(math.radians(-10) - h))
        #     # print("left offset: {}".format(left_offset))
        #     x = 1785 - x - left_offset

        #     diff = x - controller.robot.pos[0]
        #     log("Recalibrate x pos from {} to {}. Diff: {}".format(controller.robot.pos[0], x, diff))
        #     controller.robot.pos[0] = x
        # else:
        #     log("Couldn't calibrate right sensor!")

        # d2 = sensor.wallLeftDistance.object_distance()
        # if d2 < 900:
        #     y = abs(d2*math.sin(h))
        #     right_offset = 172.6 * abs(math.sin(math.radians(-10) + h)) #! - h???
        #     y = -1785 + y + right_offset
        #     diff = y - controller.robot.pos[1]
        #     log("Recalibrate y pos from {} to {}. Diff: {}".format(controller.robot.pos[1], y, diff))
        #     controller.robot.pos[1] = y
        # else:
        #     log("Couldn't calibrate left distance!")
        # log(controller.robot.pos)

        motor.intakeChain.spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
        motor.intakeFlex.spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
        sleep(300, TimeUnits.MSEC)
        motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        brain.timer.event(motor.intakeChain.spin, 350, (DirectionType.FORWARD, 100, VelocityUnits.PERCENT))
        AutonomousFlags.intake_auto_halt = True
        # motor.intakeFlex.stop(BrakeType.BRAKE)

        controller.path(paths[12], speed_ramp_time=800, slowdown_distance=800)
        motor.intakeChain.stop()
        controller.fwd_speed = 7
        controller.path(paths[13], backwards=True, speed_ramp_time=400, timeout=1500)
        drivetrain.drive_for(DirectionType.FORWARD, 3, DistanceUnits.IN, 50, VelocityUnits.PERCENT)
        motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        sleep(1000, TimeUnits.MSEC)
        motor.intakeChain.spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)
        sleep(200, TimeUnits.MSEC)
        motor.intakeChain.stop()

        ## flip lady brown guide off
        # motor.ladyBrown.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        # sleep(450, TimeUnits.MSEC)
        controller.robot.LB_PID.home()

        # position off the wall
        x = sensor.wallBackDistance.object_distance()
        x += 140
        controller.robot.pos[0] = 1785 - x
        log("Recalibrated x to {}".format(controller.robot.pos[0]))

        AutonomousFlags.intake_anti_jam = False
        AutonomousFlags.intake_auto_halt = False
        ### start intake and drive to grab last ring
        motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

        controller.fwd_speed = 8
        controller.path(paths[14], speed_ramp_time=600, slowdown_distance=800, timeout=3500, heading_authority=1.5)

        # extake slightly to make sure we aren't caught on the ring
        motor.intakeChain.spin(DirectionType.REVERSE, 50, VelocityUnits.PERCENT)
        sleep(100, TimeUnits.MSEC)
        motor.intakeChain.stop(BrakeType.COAST)

        ## spin lady brown to put ring on wall stake
        controller.robot.LB_PID.enabled = False
        motor.ladyBrown.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        sleep(800)
        motor.ladyBrown.stop(BrakeType.HOLD)
        # motor.ladyBrown.spin_for(DirectionType.REVERSE, 500, RotationUnits.DEG, False)

        controller.fwd_speed = 8
        controller.path(paths[15], backwards=True, speed_ramp_time=500, slowdown_distance=600)
        Thread(ControllerFunctions.elevation_bar)
        sleep(500, TimeUnits.MSEC)

        AutonomousFlags.intake_anti_jam = True
        motor.intakeChain.spin(DirectionType.FORWARD, 20, VelocityUnits.PERCENT)
        controller.fwd_speed = 11
        # # controller.robot.pos = [650.0, 758.0]
        # controller.path(paths[16], timeout=2200, speed_ramp_time=400)
        # pneumatic.doinker.set(True)
        # pneumatic.elevatoinBarLift.set(False)

        # motor.intakeChain.stop()
        # motor.intakeFlex.stop()

        # motor.ladyBrown.spin(DirectionType.FORWARD, 20, VelocityUnits.PERCENT)
        # sleep(300, TimeUnits.MSEC)
        # motor.ladyBrown.stop(BrakeType.COAST)
        
        self.autonomous_cleanup()
        # self.test()

    def test(self) -> None:
        """
        Run a test version of autonomous. This is NOT run in competition!
        """
        log("Running autonomous TEST", LogLevel.WARNING)
        self.run()
    
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
    
    def path(self, path, events=[], checkpoints=[], backwards = False,
             look_ahead_dist=350, finish_margin=100, event_look_ahead_dist=75, timeout=None,
             heading_authority=2.0, max_turn_volts = 8,
             hPID_KP = 0.1, hPID_KD = 0.01, hPID_KI = 0, hPID_KI_MAX = 0, hPID_MIN_OUT = None,
             K_curvature_speed = 0.0, K_curvature_look_ahead = 0.0, a_curvature_exp = 0.1,
             max_curvature_speed = 3.0, min_look_ahead = 300, a_curvature_speed_exp = 0.1,
             speed_ramp_time = 1, min_start_voltage: int | float = 4, 
             slowdown_distance = 0, min_slow_voltage: int | float = 4) -> None:
        """
        Runs a path. Halts program execution until finished.
        
        Arguments:
            path (Iterable): List of points that will be followed.
            K_curvature_speed (float): Constant that defines the weight 
                that the path curvature will be multiplied by (usually more than 1, curvature is often small).
            K_curvature_look_ahead (float): Constant the defines the weight that the look
                ahead distance will shrink based on path curvature
            max_curvature_speed (float): Most volts that the curvature will be allowed to change on the motors.
            min_look_ahead (int): Minimum look ahead that we will look on the path (curvature based look ahead)
        """
        log("Running path")
        if timeout is not None:
            # determine when we will stop trying to drive the path and just complete it
            time_end = brain.timer.system() + timeout

        # intialize the path controller object for this path only
        path_handler = self.path_controller(look_ahead_dist, finish_margin, path, checkpoints)
        # create the heading PID with this path's settings
        heading_pid = MultipurposePID(hPID_KP, hPID_KD, hPID_KI, hPID_KI_MAX, hPID_MIN_OUT)
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

            # logic to handle the edge case of the target heading rolling over
            heading_error = self.robot.heading - heading_to_target
            rollover = False

            if backwards:
                if heading_error > 180:
                    heading_error -= 180
                else:
                    heading_error += 180

            if heading_error > 180:
                heading_error = 360 - heading_error
                rollover = True
            if heading_error < -180:
                heading_error = -360 - heading_error
                rollover = True

            heading_output = heading_pid.calculate(0, heading_error)

            # Curvature exponential smoothing ((x * 1-a) + (y * a))
            if curvature < 0.05: curvature = 0
            curvature = (prev_curvature * (1-a_curvature_exp)) + (curvature * a_curvature_exp)
            # avoid unnecessary calculations if these are off (0.0)
            if K_curvature_speed != 0.0:
                dynamic_forwards_speed = min(max_curvature_speed, curvature * K_curvature_speed)
                dynamic_forwards_speed = (prev_speed_curvature * (1-a_curvature_speed_exp) + (dynamic_forwards_speed * a_curvature_speed_exp))
                prev_speed_curvature = dynamic_forwards_speed
            else: dynamic_forwards_speed = 0
            
            if K_curvature_look_ahead != 0.0:
                path_handler.look_dist = look_ahead_dist - min(curvature * K_curvature_look_ahead, min_look_ahead)

            # forwards speed linear acceleration
            if elapsed_time < speed_ramp_time:
                forwards_speed = min_start_voltage + (self.fwd_speed - min_start_voltage) * (elapsed_time / speed_ramp_time)
            else:
                forwards_speed = self.fwd_speed

            # if close to end, slow down
            if distance_to_end < slowdown_distance:
                forwards_speed = min_slow_voltage + (self.fwd_speed - min_slow_voltage) * (distance_to_end / slowdown_distance)

            if backwards:
                forwards_speed *= -1

            # Do some stuff that lowers the authority of turning, no idea if this is reasonable
            heading_output *= heading_authority
            if heading_output > max_turn_volts: heading_output = max_turn_volts
            if heading_output < -max_turn_volts: heading_output = -max_turn_volts
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

            for event in events:
                if dist(robot.pos, event[1]) < event_look_ahead_dist:
                    # "tag", (x, y), ""
                    log(str(event))
                    if type(event[2]) == str:
                        if event[2] == "wait_function":
                            if not event[4]:
                                # This tells us to wait
                                # format: ["description", (x, y), EventWaitType(), duration, completed]
                                waiting = True
                                wait_stop = brain.timer.system() + event[3]

                                # make sure we dont get stuck in a waiting loop
                                event[4] = True

                                AutonomousCommands.kill_motors()
                        elif event[2] == "flags":
                            if hasattr(flags, event[3]):
                                setattr(flags, event[3], event[4])
                                log("Changed flags {} to {}".format(event[3], event[4]))
                            else:
                                log("Flags does not have attribute {}".format(event[3]), LogLevel.WARNING)
                        elif event[2] == "AutonomousFlags":
                            if hasattr(AutonomousFlags, event[3]):
                                setattr(AutonomousFlags, event[3], event[4])
                                log("Changed AutonomousFlags {} to {}".format(event[3], event[4]))
                            else:
                                log("AutonomousFlags does not have attribute {}".format(event[3]), LogLevel.WARNING)
                        else:
                            # this is a variable change
                            # format: ["speed down", (0, 1130), "speed", 3.5]
                            if hasattr(self, event[2]):
                                setattr(self, event[2], event[3])
                                log("Event updated {} to {}".format(event[2], event[3]))
                            else:
                                log("Event couldn't find attribute {}".format(event[2]), LogLevel.FATAL)
                                raise AttributeError("No attribute found {}".format(event[2]))
                    elif callable(event[2]):
                        # Call the function (at index 2) with the unpacked (*) args (at index 3)
                        try:
                            event[2](*event[3])
                            log("Event ran {}({})".format(event[2], event[3]))
                        except:
                            log("Event couldn't find function {}".format(event[2]), LogLevel.FATAL)
                            raise NameError("Function not defined")

            done = path_handler.path_complete
            if done:
                log("Path complete")

            if timeout is not None:
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
        pneumatic.doinker.set(not pneumatic.doinker.value())
        pneumatic.doinker_left.set(not pneumatic.doinker_left.value())

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
    def elevation_bar():
        log("Toggled elevation bar pneumatics")
        flags.elevating = True
        motor.ladyBrown.spin(DirectionType.FORWARD, 90, VelocityUnits.PERCENT)
        sleep(200, TimeUnits.MSEC)
        pneumatic.mogo.set(False)
        pneumatic.doinker.set(True)
        sleep(600, TimeUnits.MSEC)
        motor.ladyBrown.stop(BrakeType.HOLD)
        sleep(1000, TimeUnits.MSEC)
        flags.elevating = False

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

        # bind controller functions
        control.DOINKER.pressed(ControllerFunctions.switch_doinker)
        control.MOGO_GRABBER_TOGGLE.pressed(ControllerFunctions.switch_mogo)
        control.INTAKE_HEIGHT_TOGGLE.pressed(ControllerFunctions.switch_intake_height)
        control.LB_MACRO_HOME.pressed(self.robot.LB_PID.home)
        # control.MANUAL_ELEVATION_PNEUMATICS.pressed(ControllerFunctions.elevation_bar)
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
        forwardVolts = control.DRIVE_FORWARD_AXIS.position() * 0.12

        # Spin motors and combine controller axes
        motor.leftA.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motor.leftB.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        motor.leftC.spin(FORWARD, forwardVolts + turnVolts, VOLT)
        
        motor.rightA.spin(FORWARD, forwardVolts - turnVolts, VOLT)
        motor.rightB.spin(FORWARD, forwardVolts - turnVolts, VOLT)
        motor.rightC.spin(FORWARD, forwardVolts - turnVolts, VOLT)
    
    def intake_controls(self) -> None:
        if (control.INTAKE_IN_HOLD.pressing()):
            motor.intakeFlex.spin(FORWARD, 100, PERCENT)

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
            if not 200 < sensor.wallEncoder.angle() < 355:
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

        # elevation hold button
        # if con.buttonUp.pressing() and not flags.elevating:
        #     self.elevation_hold_duration -= 1
        #     if self.elevation_hold_duration <= 0:
        #         self.start_elevation()
        # else:
        #     self.elevation_hold_duration = self.elevation_hold_duration_reset
    
    def elevation_drive_controls(self) -> None:
        if abs(con_2.axis2.position()) > 1 or abs(con_2.axis3.position()) > 1:
            motor.leftA.spin(FORWARD, (con_2.axis3.position() / 100) * 12, VOLT)
            motor.leftB.spin(FORWARD, (con_2.axis3.position() / 100) * 12, VOLT)
            motor.leftC.spin(FORWARD, (con_2.axis3.position() / 100) * 12, VOLT)

            motor.rightA.spin(FORWARD, (con_2.axis2.position() / 100) * 12, VOLT)
            motor.rightB.spin(FORWARD, (con_2.axis2.position() / 100) * 12, VOLT)
            motor.rightC.spin(FORWARD, (con_2.axis2.position() / 100) * 12, VOLT)

            self.LB_braketype = BrakeType.COAST
        else:
            motor.leftA.stop(BrakeType.COAST)
            motor.leftB.stop(BrakeType.COAST)
            motor.leftC.stop(BrakeType.COAST)

            motor.rightA.stop(BrakeType.COAST)
            motor.rightB.stop(BrakeType.COAST)
            motor.rightC.stop(BrakeType.COAST)

        # lady brown controls
        if con.buttonL1.pressing() or control_2.LB_MANUAL_DOWN.pressing():

            motor.ladyBrown.spin(REVERSE, 100, PERCENT)
            self.robot.LB_PID.enabled = False
            self.LB_braketype = BrakeType.HOLD
        elif con.buttonL2.pressing() or control_2.LB_MANUAL_UP.pressing():
            motor.ladyBrown.spin(FORWARD, 100, PERCENT)
            self.robot.LB_PID.enabled = False
            self.LB_braketype = BrakeType.HOLD
        elif not self.robot.LB_PID.enabled:
            motor.ladyBrown.stop(self.LB_braketype)

        # intake controls
        if con.buttonR1.pressing() or control_2.INTAKE_IN.pressing():
            motor.intakeFlex.spin(FORWARD, 100, PERCENT)
            motor.intakeChain.spin(FORWARD, 100, PERCENT)
        elif con.buttonR2.pressing():# or controls2["INTAKE_OUT"].pressing():
            motor.intakeFlex.spin(REVERSE, 100, PERCENT)
            motor.intakeChain.spin(REVERSE, 100, PERCENT)
        else:
            motor.intakeFlex.stop()
            motor.intakeChain.stop(BrakeType.BRAKE)

        # roll = imu.orientation(OrientationType.PITCH)
        # pid_output = round(roll_pid.calculate(0, roll), 3)
        # data = {
        #     "roll": round(roll, 2),
        #     "output": round(pid_output, 2),
        #     "height": round(elevationDistance.object_distance(), 2)
        # }
        # payload_manager.send_data("elevation", data)

        # scr = con_2.screen
        # scr.set_cursor(1,1)
        # if pneumatic.elevation_bar_lift.value():
        #     scr.print("UP__")
        # else:
        #     scr.print("DOWN")

        sleep(35, MSEC)

    def start_elevation(self) -> None:
        log("Starting elevation")
        self.robot.LB_PID.enabled = False
        
        pneumatic.mogo.set(True)

        roll_pid = MultipurposePID(0.1, 0, 0, 0)

        # wait for matics
        sleep(100, MSEC)
        # motors["misc"]["wall_stake"].spin_for(REVERSE, 1200, MSEC, 100, PERCENT)
        sleep(200, MSEC)
        # elevation_bar_lift.set(False)
        # sleep(100, MSEC)

        control_2.DOINKER.pressed(ControllerFunctions.switch_doinker)
        control_2.PTO_TOGGLE.pressed(ControllerFunctions.toggle_PTO)
        control_2.ELEVATION_PRIMARY_PNEUMATICS.pressed(ControllerFunctions.manual_elevation)
        self.LB_braketype = BrakeType.COAST

        self.elevation_loop()

    def elevation_loop(self) -> None:
        while True:
            self.elevation_drive_controls()

# control objects
class LadyBrown():
    POS_LOAD = 25
    POS_ELEVATION = 190
    def __init__(self, wall_motor) -> None:
        self.pid = MultipurposePID(0.3, 0.5, 0.01, 2, None)
        self.enabled = False
        self.autostop = False
        self.threshold = 5
        self.motor = wall_motor
        self.target_rotation = 0
        self.thread: Thread

        self.sensor = 0
        self.last_enabled = self.enabled
    
    def home(self):
        """
        Homes PID to ready to load position
        """
        self.target_rotation = LadyBrown.POS_LOAD
        self.enabled = True
        log("Homing lady brown")
    
    def elevation(self):
        """
        Run PID to try and unwinch the winches and prep for next ladder
        """
        self.target_rotation = LadyBrown.POS_ELEVATION
        self.autostop = True
        self.enabled = False

    def run(self):
        """
        Run once to start PID thread.
        """
        log("Starting lady brown thread")
        while True:
            if self.enabled:
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
        self.sensor = sensor.wallEncoder.angle()

        if self.sensor > 270:
            self.sensor = 360 - self.sensor
        
        output = self.pid.calculate(self.target_rotation, self.sensor)
        if abs(self.pid.error) > 40: self.pid.integral = 0
        # print(sensor.wallEncoder.angle(), output)

        # # debug data
        # data = {
        #     "o": str(round(output, 2)),
        #     "error": str(round(self.target_rotation-self.sensor, 2)),
        #     "integral": str(round(self.pid.integral, 2)),
        #     "integralp": str(round(self.pid.integral_processed, 2)),
        #     "d": str(round(self.pid.error * self.pid.kP, 2)),
        #     "p": str(round(self.pid.derivative * self.pid.kD))
        # }
        # packet_mgr.add_packet("lb", data)

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
    # try:
    #     with open("cfg/config.json", 'r') as f:
    #         data = load(f)
    #     log("SUCCESS LOADING SD CARD")
    # except:
    #     sd_fail = True
    #     log("ERROR LOADING SD CARD DATA", LogLevel.FATAL)

    if not sd_fail:
        robot = Robot()
        robot.LB_PID.thread = Thread(robot.LB_PID.run)

        comp = Competition(null_function, null_function)

        # if not (comp.is_field_control() or comp.is_competition_switch()):
        #     brain.timer.event(start_odom_packets, 2000, (robot,))

        # Load autonomous into robot
        # robot.autonomous_controller.load_path("skills")

        # con.screen.print(data["autons"]["selected"])

        calibrate_imu()

        brain.screen.clear_screen(Color.GREEN)
        comp = Competition(null_function, robot.autonomous_controller.run)
        if not comp.is_field_control() or comp.is_competition_switch():
            robot.autonomous_controller.run()
        
    else:
        log("Robot object not created. (SD Load Error)", LogLevel.FATAL)
        raise ImportError("SD Card not inserted")

main()
