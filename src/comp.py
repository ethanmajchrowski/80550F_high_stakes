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
    
    time_ms = brain.timer.system()
    time_s = time_ms % 60000 // 1000
    time_min = time_ms // 60000
    time_ms -= (time_s * 1000) + (time_min * 60000)

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
        packet_mgr.add_packet("foxglove.Log", {"timestamp": time_ms, "message": msg, "level": foxglove_logLevel, "name": "main", "file": "main", "line": 0})
    else:
        print("[{}] {}: {}".format(tag, time_ms + (time_s + time_min*60)*1000, msg))

def log(msg: Any, level = LogLevel.INFO):
    if do_logging:
        Thread(threaded_log, (msg, level))

class PacketTiming:
    CONTROLLER = 40
    CONTROLLER_2 = 50
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
packet_mgr = PacketManager(PacketTiming.CONTROLLER_2)
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
    MANUAL_ELEVATION_PNEUMATICS =   con.buttonUp
    LB_MACRO_HOME =                 con.buttonDown

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
    leftA = Motor( Ports.PORT3, GearSetting.RATIO_6_1, True) # stacked top
    leftB = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True) # rear
    leftC = Motor( Ports.PORT4, GearSetting.RATIO_6_1, True) # front

    rightA = Motor(Ports.PORT18, GearSetting.RATIO_6_1, False) # stacked top
    rightB = Motor(Ports.PORT16, GearSetting.RATIO_6_1, False) # rear
    rightC = Motor(Ports.PORT15, GearSetting.RATIO_6_1, False) # front

    intakeChain = Motor(Ports.PORT10, GearSetting.RATIO_6_1, False)
    intakeFlex = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)
    ladyBrown = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)

# PNEUMATICS
class pneumatic():
    mogo = DigitalOut(brain.three_wire_port.a)
    # elevation_hook_release = DigitalOut(brain.three_wire_port.b)
    elevation_bar_lift_left = DigitalOut(brain.three_wire_port.c)
    PTO = DigitalOut(brain.three_wire_port.d)

    elevation_bar_lower_right = DigitalOut(brain.three_wire_port.f)
    intake = DigitalOut(brain.three_wire_port.g)
    intake = DigitalOut(brain.three_wire_port.h)

#### SENSORS
# ENCODERS
class sensor():
    leftEncoder = Rotation(Ports.PORT2)
    rightEncoder = Rotation(Ports.PORT17)
    driftEncoder = Rotation(Ports.PORT21)

    wallEncoder = Rotation(Ports.PORT1)
    # DISTANCE SENSORS
    intakeDistance = Distance(Ports.PORT9)

    wallLeftDistance = Distance(Ports.PORT6)
    wallBackDistance = Distance(Ports.PORT20)
    wallFrontDistance = Distance(Ports.PORT9)
    wallRightDistance = Distance(Ports.PORT13)

    # elevationDistance = Distance(Ports.PORT20) # unplugged

    # MISC SENSORS
    intakeColor = Optical(Ports.PORT10)
    imu = Inertial(Ports.PORT11)

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
    log("calibrating IMU")

    sensor.imu.calibrate()
    while sensor.imu.is_calibrating():
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
                            curvature = (self.path[i][2] * (1 - percent) + self.path[i+1][2] * percent)
                            
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
                        if (self.last_found_point + 2 >= self.current_checkpoint):
                            # If we are done with our checkpoints,
                            if (self.checkpoints.index(self.current_checkpoint)+1) >= len(self.checkpoints):
                                self.checkpoints_complete = True
                            else:
                                # Set the current checkpoint to the next checkpoint in the checkpoints list
                                self.current_checkpoint = self.checkpoints[self.checkpoints.index(self.current_checkpoint) + 1]
        except ZeroDivisionError:
            raise NameError
    
        return goal[:2], curvature

class DeltaPositioning():
    def __init__(self, leftEnc: Rotation | Motor, rightEnc: Rotation | Motor, 
                 driftEnc: Rotation | Motor, imu: Inertial) -> None:
        """Odometry based on tracking wheels."""
        self.last_time = brain.timer.time()
        self.leftEnc = leftEnc
        self.rightEnc = rightEnc
        self.driftEnc = driftEnc

        self.leftEnc.reset_position()
        self.rightEnc.reset_position()
        self.driftEnc.reset_position()

        self.imu = imu

        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.last_drift_encoder = 0
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
        dl = self.leftEnc.position() - self.last_left_encoder
        dr = self.rightEnc.position() - self.last_right_encoder
        d_drift = self.driftEnc.position() - self.last_drift_encoder
        # change in heading
        dh = h - self.last_heading

        # get change in encode wheel in distance
        dl = (dl / 360) * self.circumference
        dr = (dr / 360) * self.circumference
        d_drift = (d_drift / 360) * self.circumference

        # average the position of left & right to get the center of the robot
        dNet = (dl + dr) / 2 

        # To determine if this is actually drift or just IMU error, we compare the angle error
        # of the drift wheel to the IMU
        drift_angle = (d_drift / self.drift_circumference) * 360
        angle_error = round(abs(dh - drift_angle), 3)
        # print(round(angle_error, 2))
        threshold = 2 + 0.5*abs(dh)

        encoder_sign = sign(dl) * sign(dr)

        if encoder_sign == 1:
            # possibly drifting
            if angle_error > threshold:
                self.drift_accum += abs(d_drift)
        else:
            self.drift_accum *= 0.95

        # print(self.drift_accum > 10, encoder_sign,angle_error > threshold, round(self.drift_accum, 2), round(dh, 2), threshold)
        if abs(self.drift_accum) > 1:
            print("difting!")
            # we are drifting
            drift_x = d_drift * math.sin(math.radians(h + 90))
            drift_y = d_drift * math.cos(math.radians(h + 90))
        else:
            # print("not difting!")
            drift_x, drift_y = 0, 0

        dx = (dNet * math.sin(h_rad)) + drift_x
        dy = (dNet * math.cos(h_rad)) + drift_y

        # print(round(dh, 2), round(dl, 2), round(dr, 2), round(d_drift, 2))

        self.last_time = brain.timer.time()
        self.last_heading = self.imu.heading()
        self.last_right_encoder = self.rightEnc.position()
        self.last_left_encoder = self.leftEnc.position()
        self.last_drift_encoder = self.driftEnc.position()
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
    drop_after_auto_halt = False
    raise_after_auto_halt = False
    last_intake_command = None

# robot states
class Autonomous():
    def __init__(self, parent: Robot) -> None:
        """
        Setup autonomous. Runs at start of program!
        """
        self.robot = parent

        self.positioning_algorithm = DeltaPositioning(sensor.leftEncoder, sensor.rightEncoder, sensor.driftEncoder, sensor.imu)
        self.path_controller = PurePursuit

        self.mcl_controller = MCL_Handler()
        self.run_mcl = False
        self.run_mcl_lasers = True

        self.fwd_speed = 8

        drivetrain.set_timeout(1.5, TimeUnits.SECONDS)
        drivetrain.set_turn_threshold(5)
        drivetrain.set_turn_constant(0.6)

        log("Autonomous object setup")
    
    def autonomous_setup(self, pos_override: None | list[int] | list[tuple] = None, heading_override: None | int | float = None) -> None:
        """
        Call at start of auton. Sets up the coordinates n stuff for the loaded path.
        """
        # if pos_override is not None:
        try:
            sensor.imu.set_heading(self.autonomous_data["start_heading"])
            self.robot.heading = (self.autonomous_data["start_heading"])
            self.robot.pos = list(self.autonomous_data["start_pos"])
            log("Set initial heading and position.")
        except:
            log("Couldn't set initial heading / pos! Likely because load_path hasn't been run yet.", LogLevel.WARNING)
        
        log("Starting autonomous threads.")
        self.pos_thread = Thread(self.position_thread)
        self.background_thread = Thread(self.background)
        log("Autonomous threads started.")

        self.start_time = brain.timer.system()

    def load_path(self, module_filename: str) -> None:
        """
        Loads autonomous data into object variables for later use.

        Args:
            module_filename: The filename of the imported auton, without any file extensions.
        """
        log("Loading module {}".format(module_filename))
        try:
            log("Importing module...")
            auton_module = __import__("module.{}".format(module_filename), None, None, ["run"])

            log("Importing sequence...")
            self.sequence = getattr(auton_module, "run")
            
            log("Importing data...")
            gen_data = getattr(auton_module, "gen_data")
            self.autonomous_data = gen_data()

            log("Loaded data into {} autonomous object".format(self))
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

        self.sequence(globals())
        
        self.autonomous_cleanup()
    
    # def laser_calibrate(self, direction: int, calibrate_axis: int ):
    #     """Reset x/y based on laser distance.
    #     direction: 0-3 forwards, left, back, rght as index
    #     calibrate_axis: 0 or 1, x or y"""
    #     laser = self.mcl_controller.all_directions[direction]
    #     # print(math.cos(math.radians((laser.angle + self.robot.heading - 90) % 360)))
    #     # print(self.robot.heading, (laser.angle + self.robot.heading - 90) % 360)
    #     if calibrate_axis:
    #         flip = 1
    #     else:
    #         flip = -1
    #     # start_x = self.robot.pos[1] + (laser.distance*math.cos(math.radians((laser.angle + self.robot.heading - 90) % 360))) * flip
    #     # start_y = self.robot.pos[0] - (laser.distance*math.sin(math.radians((laser.angle + self.robot.heading - 90) % 360))) * flip
    #     temp = start_x

    #     start_x = start_y
    #     start_y = temp

    #     # distance = laser.get_distance()
    #     # distance_x = distance*math.cos(math.radians(self.robot.heading)) + start_y
    #     # distance_y = distance*math.sin(math.radians(self.robot.heading)) + start_x
    #     robot_x = start_x + 170 * flip
    #     robot_y = start_y - 60 * flip
    #     # -1630
    #     # return (-1800+distance_x)
    #     print("start pos: {}".format((start_x, start_y)))
    #     return("rx: {}, ry: {}".format(robot_x, robot_y))

    def test(self) -> None:
        """
        Run a test version of autonomous. This is NOT run in competition!
        """
        log("Running autonomous TEST", LogLevel.WARNING)
        # self.run()
        self.autonomous_setup()

        self.robot.pos = [0.0, 0.0]
        sensor.imu.set_heading(90)

        self.mcl_controller.start()

        # place temporary / testing code here

        #* Skills
        controller = self
        drivetrain.set_turn_threshold(2)
        drivetrain.set_turn_constant(0.28)
        paths = [[(-1593.76, 6.33, 0), (-1553.77, 5.83, 0.07), (-1513.78, 4.74, 0.04), (-1473.81, 3.36, 0.02), (-1433.84, 1.8, 0.01), (-1393.87, 0.15, 0.01), (-1353.91, -1.55, 0.0), (-1313.94, -3.25, 0.0), (-1273.98, -4.92, 0.01), (-1234.01, -6.49, 0.02), (-1194.03, -7.9, 0.03), (-1154.05, -9.03, 0.07)], [(-1176.67, -24.08, 0), (-1179.01, -64.01, 0.05), (-1181.73, -103.92, 0.05), (-1184.83, -143.8, 0.05), (-1188.34, -183.65, 0.05), (-1192.23, -223.46, 0.04), (-1196.44, -263.23, 0.02), (-1200.84, -302.99, 0.0), (-1205.26, -342.75, 0.03), (-1209.47, -382.52, 0.05), (-1213.26, -422.34, 0.07), (-1216.48, -462.21, 0.08), (-1219.05, -502.13, 0.08), (-1220.96, -542.08, 0.08), (-1222.27, -582.06, 0.07), (-1223.0, -622.06, 0.06), (-1223.25, -662.05, 0.05), (-1223.07, -702.05, 0.05), (-1222.52, -742.05, 0.05)], [(-1195.33, -666.9, 0), (-1158.53, -651.2, 0.01), (-1121.77, -635.44, 0.01), (-1084.97, -619.75, 0.03), (-1048.08, -604.31, 0.06), (-1010.99, -589.33, 0.1), (-973.61, -575.08, 0.15), (-935.83, -561.94, 0.21), (-897.53, -550.43, 0.29), (-858.63, -541.17, 0.39), (-819.13, -534.98, 0.51), (-779.21, -532.81, 0.62), (-739.34, -535.59, 0.7), (-700.26, -543.9, 0.72), (-662.77, -557.73, 0.67), (-627.45, -576.44, 0.57), (-594.49, -599.06, 0.49), (-563.87, -624.78, 0.37), (-535.26, -652.72, 0.31), (-508.43, -682.38, 0.25), (-483.11, -713.34, 0.02), (-457.66, -744.18, 0.48), (-429.36, -772.41, 0.43), (-398.75, -798.14, 0.36), (-366.35, -821.58, 0.27), (-332.73, -843.24, 0.25), (-298.06, -863.17, 0.2), (-262.62, -881.71, 0.17), (-226.58, -899.07, 0.17), (-189.98, -915.19, 0.13), (-152.98, -930.38, 0.11), (-115.63, -944.72, 0.11), (-77.98, -958.22, 0.1), (-40.06, -970.95, 0.09), (-1.93, -983.02, 0.08), (36.39, -994.48, 0.07), (74.88, -1005.37, 0.08), (113.53, -1015.67, 0.07), (152.32, -1025.44, 0.06), (191.22, -1034.74, 0.06), (230.23, -1043.57, 0.06), (269.34, -1051.97, 0.06), (308.55, -1059.87, 0.05), (347.85, -1067.37, 0.05), (387.21, -1074.45, 0.05), (426.66, -1081.1, 0.06), (466.17, -1087.3, 0.05), (505.75, -1093.1, 0.07), (545.4, -1098.37, 0.06), (585.11, -1103.18, 0.07), (624.89, -1107.4, 0.09), (664.73, -1110.92, 0.11), (704.64, -1113.56, 0.19), (744.62, -1114.69, 0.51)], [(719.37, -1102.13, 0), (682.63, -1086.31, 0.2), (645.29, -1071.98, 0.22), (607.35, -1059.33, 0.25), (568.83, -1048.6, 0.34), (529.66, -1040.5, 0.32), (490.07, -1034.9, 0.45), (450.13, -1032.89, 0.45), (410.2, -1034.51, 0.53), (370.66, -1040.38, 0.6), (332.11, -1050.96, 0.65), (295.26, -1066.45, 0.67), (260.79, -1086.71, 0.65), (229.25, -1111.27, 0.61), (200.93, -1139.5, 0.55), (175.89, -1170.67, 0.48), (153.98, -1204.11, 0.42), (134.95, -1239.27, 0.36), (118.49, -1275.71, 0.32), (104.38, -1313.14, 0.29), (92.45, -1351.31, 0.23), (82.26, -1389.99, 0.23), (73.87, -1429.09, 0.18), (66.86, -1468.47, 0.19), (61.33, -1508.09, 0.14), (56.93, -1547.84, 0.15), (53.73, -1587.71, 0.13), (51.54, -1627.65, 0.11), (50.25, -1667.63, 0.13), (49.96, -1707.62, 0.1), (50.46, -1747.62, 0.09), (51.7, -1787.6, 0.1)], [(36.85, -1839.17, 0), (37.02, -1799.17, 0.03), (37.42, -1759.18, 0.04), (38.11, -1719.18, 0.06), (39.24, -1679.2, 0.06), (40.81, -1639.23, 0.06), (42.84, -1599.28, 0.06), (45.38, -1559.37, 0.08), (48.53, -1519.49, 0.09), (52.41, -1479.68, 0.11), (57.13, -1439.96, 0.13), (62.85, -1400.38, 0.15), (69.74, -1360.98, 0.18), (78.02, -1321.85, 0.21), (87.94, -1283.1, 0.31), (100.27, -1245.06, 0.34), (115.13, -1207.94, 0.49), (133.55, -1172.46, 0.64), (156.32, -1139.61, 0.83), (184.21, -1111.02, 0.96), (217.04, -1088.31, 0.99), (253.7, -1072.5, 0.89), (292.62, -1063.48, 0.65), (332.41, -1059.62, 0.47), (372.4, -1059.54, 0.32), (412.32, -1061.99, 0.19), (452.12, -1065.97, 0.1), (491.83, -1070.77, 0.04), (531.51, -1075.85, 0.02), (571.2, -1080.79, 0.06), (610.95, -1085.26, 0.1), (650.78, -1088.94, 0.13), (690.69, -1091.58, 0.14)], [(367.75, -1185.65, 0), (327.76, -1184.88, 0.06), (287.76, -1184.6, 0.08), (247.76, -1184.95, 0.07), (207.78, -1185.88, 0.08), (167.81, -1187.42, 0.08), (127.87, -1189.62, 0.08), (87.97, -1192.44, 0.08), (48.11, -1195.87, 0.07), (8.32, -1199.88, 0.06), (-31.43, -1204.4, 0.05), (-71.12, -1209.35, 0.04), (-110.77, -1214.63, 0.03), (-150.39, -1220.11, 0.01), (-190.01, -1225.65, 0.01), (-229.63, -1231.1, 0.03), (-269.29, -1236.3, 0.05), (-309.0, -1241.13, 0.06), (-348.76, -1245.48, 0.07), (-388.58, -1249.26, 0.08), (-428.46, -1252.41, 0.09), (-468.38, -1254.88, 0.09), (-508.34, -1256.67, 0.09), (-548.33, -1257.76, 0.09), (-588.32, -1258.15, 0.08), (-628.32, -1257.92, 0.08), (-668.31, -1257.04, 0.08), (-708.28, -1255.54, 0.1), (-748.22, -1253.26, 0.08), (-788.11, -1250.35, 0.08), (-827.95, -1246.77, 0.1), (-867.7, -1242.36, 0.12), (-907.34, -1236.99, 0.15), (-946.8, -1230.42, 0.17), (-986.01, -1222.54, 0.19), (-1024.9, -1213.2, 0.18), (-1063.42, -1202.43, 0.13), (-1101.66, -1190.7, 0.01), (-1139.89, -1178.92, 0.14), (-1178.43, -1168.23, 0.24), (-1217.43, -1159.38, 0.27), (-1256.85, -1152.61, 0.23), (-1296.54, -1147.7, 0.19), (-1336.4, -1144.33, 0.16), (-1376.34, -1142.25, 0.83), (-1415.15, -1133.7, 0.08), (-1454.32, -1125.74, 0.76), (-1494.21, -1123.79, 1.19), (-1533.31, -1131.31, 2.24), (-1564.71, -1155.06, 3.19), (-1574.33, -1193.08, 2.49), (-1563.7, -1231.39, 1.34), (-1543.14, -1265.61, 0.81), (-1517.31, -1296.11, 0.55), (-1488.3, -1323.62, 0.41), (-1457.11, -1348.64, 0.31), (-1424.4, -1371.65, 0.27), (-1390.51, -1392.88, 0.23), (-1355.69, -1412.55, 0.2), (-1320.11, -1430.82, 0.16), (-1283.95, -1447.92, 0.15), (-1247.28, -1463.91, 0.15), (-1210.16, -1478.81, 0.13), (-1172.68, -1492.76, 0.12), (-1134.88, -1505.84, 0.11), (-1096.81, -1518.12, 0.1), (-1058.51, -1529.65, 0.09), (-1020.0, -1540.48, 0.1), (-981.3, -1550.56, 0.08), (-942.42, -1560.0, 0.08), (-903.42, -1568.85, 0.05)], [(-831.22, -1575.91, 0), (-855.17, -1543.88, 0.3), (-881.0, -1513.36, 0.38), (-909.06, -1484.87, 0.45), (-939.58, -1459.03, 0.51), (-972.54, -1436.42, 0.58), (-1007.91, -1417.8, 0.65), (-1045.4, -1403.97, 0.73), (-1084.52, -1395.79, 0.72), (-1124.4, -1393.36, 0.71), (-1164.23, -1396.59, 0.66), (-1203.29, -1405.08, 0.61), (-1241.02, -1418.31, 0.52), (-1277.17, -1435.38, 0.45), (-1311.63, -1455.66, 0.4), (-1344.38, -1478.6, 0.34), (-1375.5, -1503.72, 0.32), (-1404.97, -1530.76, 0.26), (-1432.97, -1559.31, 0.23), (-1459.62, -1589.13, 0.22), (-1484.92, -1620.11, 0.2), (-1508.98, -1652.06, 0.18), (-1531.91, -1684.84, 0.17), (-1553.68, -1718.39, 0.15), (-1574.43, -1752.58, 0.15), (-1594.17, -1787.37, 0.14), (-1612.92, -1822.7, 0.12), (-1630.81, -1858.48, 0.15)]]        

        x = -1800 + sensor.wallBackDistance.object_distance() + 130
        dev = x - self.robot.pos[0]
        log("Reset x position to {}. Deviation: {}".format(x, dev))
        self.robot.pos[0] = x

        motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        sleep(350, TimeUnits.MSEC)        
        motor.intakeChain.stop(BrakeType.COAST)

        motor.ladyBrown.spin(DirectionType.FORWARD, 70, VelocityUnits.PERCENT)
        brain.timer.event(motor.ladyBrown.spin, 200, (DirectionType.REVERSE, 50, VelocityUnits.PERCENT))

        self.fwd_speed = 8
        self.path(paths[0], 
                backwards=False,
                heading_authority=3,
                finish_margin=200, slowdown_distance=0)

        motor.ladyBrown.stop(BrakeType.COAST)
        
        drivetrain.set_turn_threshold(10)
        drivetrain.set_turn_constant(0.8)
        drivetrain.turn_for(TurnType.LEFT, 65, RotationUnits.DEG, 100, VelocityUnits.PERCENT)

        # x = -1800 + (sensor.wallLeftDistance.object_distance() + 170 - abs(60*math.sin(math.radians(90+sensor.imu.heading())))) * math.cos(math.radians(sensor.imu.heading()))
        # x = ((170 + sensor.wallLeftDistance.object_distance()) * math.cos(math.radians(sensor.imu.heading()))) - (60 * math.sin(math.radians(sensor.imu.heading())))
        # x = -1800 + x
        # x = round(x, 1)
        # dev = x - self.robot.pos[0]
        # print(sensor.wallLeftDistance.object_distance())
        # log("Reset x position from {} to {}. Deviation: {}".format(self.robot.pos[0], x, dev))
        # self.robot.pos[0] = x

        self.fwd_speed = 6
        # grab mogo
        self.path(paths[1], backwards=True, heading_authority=2, slowdown_distance=0)
        
        pneumatic.mogo.set(True)
        sleep(100, TimeUnits.MSEC)

        drivetrain.turn_for(TurnType.RIGHT, 60, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
        motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

        self.fwd_speed = 12
        # drive across field to get 2 rings and raise LB for second ring
        self.path(paths[2], backwards=False, 
                events=[["raise LB", [500, -1000], self.robot.LB_PID.home, []]],
                heading_authority=1.4, look_ahead_dist=400, event_look_ahead_dist=300, finish_margin=200,
                K_curvature_speed=2400,
                max_curvature_speed=4.5,
                a_curvature_exp=0.025,
                K_curvature_look_ahead=100,
                a_curvature_speed_exp=0.1)

        drivetrain.turn_for(TurnType.LEFT, 105, RotationUnits.DEG, 80, VelocityUnits.PERCENT)
        drivetrain.stop()
        motor.intakeChain.stop()
        
        self.fwd_speed = 7
        # line up for wall stake
        self.path(paths[3], backwards=False, heading_authority=2, timeout=2000, look_ahead_dist=250)
        # disable PID to allow us to spin it and score
        self.robot.LB_PID.enabled = False
        motor.ladyBrown.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        sleep(300, TimeUnits.MSEC)
        motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

        sleep(200, TimeUnits.MSEC)
        motor.ladyBrown.spin(DirectionType.REVERSE, 70, VelocityUnits.PERCENT)
        brain.timer.event(motor.ladyBrown.stop, 700, (BrakeType.COAST,))

        # Recalibrate position
        y = -1800 + sensor.wallFrontDistance.object_distance() + 120
        diff = y - self.robot.pos[1]
        log("Recalibrate y pos from {} to {}. Diff: {}".format(self.robot.pos[1], y, diff))
        self.robot.pos[1] = y

        # x = -1800 + (sensor.wallRightDistance.object_distance() * math.cos(math.radians(sensor.imu.heading() - 180))) + (170 * math.cos(math.radians(sensor.imu.heading() - 180)))
        # print(sensor.wallRightDistance.object_distance())
        # diff = x - self.robot.pos[0]
        # print("Recalibrate x pos from {} to {}. Diff: {}".format(self.robot.pos[0], x, diff))
        # self.robot.pos[0] = x

        # backup away from wall stake
        self.path(paths[4], backwards=True, heading_authority=2)
        motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        # pickup bottom left rings
        self.fwd_speed = 9
        self.path(paths[5], look_ahead_dist=280, checkpoints=[47], heading_authority=1.8,
                  K_curvature_speed=3000,
                  max_curvature_speed=5,
                  a_curvature_exp=0.025, 
                  K_curvature_look_ahead=50,
                  a_curvature_speed_exp=0.1)

        # drop mogo in corner
        self.path(paths[6], backwards=True, timeout=1300, heading_authority=2)
        pneumatic.mogo.set(False)

        # recalibrate x and y since we're in the corner
        sleep(200, TimeUnits.MSEC)
        sensor.imu.set_heading(45)
        h = math.radians(sensor.imu.heading())
        d1 = sensor.wallLeftDistance.object_distance()
        x = d1 * math.cos(h)
        left_offset = 180.3 * math.cos(math.radians(19.4) + h)
        x = -1800 + x + left_offset

        diff = x - self.robot.pos[0]
        print("Recalibrate x pos from {} to {}. Diff: {}".format(self.robot.pos[0], x, diff))
        self.robot.pos[0] = x

        d2 = sensor.wallRightDistance.object_distance()
        y = d2*math.sin(h)
        right_offset = 180.3 * math.sin(math.radians(19.4) - h)
        y = -1800 + y - right_offset
        diff = y - self.robot.pos[1]
        print("Recalibrate y pos from {} to {}. Diff: {}".format(self.robot.pos[1], y, diff))
        self.robot.pos[1] = y
        print(d1, d2)        

        motor.intakeFlex.stop()
        
        log("Done!")

        # sleep(500, TimeUnits.MSEC)
        # paths = [[(0.0, 0.0, 0), (1.72, 59.98, 0.0), (3.44, 119.95, 0.05), (4.35, 179.93, 0.11), (3.33, 239.92, 0.0), (2.3, 299.91, 0.16), (-1.63, 359.78, 0.02), (-5.94, 419.62, 0.16), (-13.19, 479.16, 0.05), (-21.4, 538.59, 0.23), (-33.69, 597.3, 0.1), (-47.79, 655.57, 0.24), (-66.07, 712.72, 0.29), (-89.17, 768.02, 0.29), (-116.99, 821.05, 0.38), (-150.62, 870.57, 0.46), (-190.73, 915.01, 0.51), (-237.12, 952.83, 0.51), (-288.77, 983.1, 0.47), (-344.18, 1005.85, 0.39), (-401.9, 1021.98, 0.3), (-460.87, 1032.82, 0.21), (-520.44, 1039.87, 0.13), (-580.25, 1044.58, 0.05), (-640.13, 1048.31, 0.02), (-699.99, 1052.4, 0.11), (-759.69, 1058.4, 0.2), (-818.9, 1067.94, 0.3), (-877.0, 1082.69, 0.4), (-932.85, 1104.32, 0.5), (-984.78, 1134.07, 0.57), (-1030.9, 1172.17, 0.57), (-1069.84, 1217.62, 0.52), (-1101.31, 1268.56, 0.42), (-1126.1, 1323.06, 0.32), (-1145.59, 1379.71, 0.23), (-1161.16, 1437.64, 0.31), (-1171.36, 1496.77, 0.12), (-1179.47, 1556.17, 0.14), (-1185.0, 1615.91, 0.14), (-1188.08, 1675.81, 0.08), (-1189.74, 1735.78, 0.06)]]
        # while True:
        #     print(self.laser_calibrate(1, 0), round(self.mcl_controller.all_directions[1].get_distance(), 1))

        # controller.fwd_speed = 12
        # controller.path(paths[0], 
        #                 K_curvature_speed=1800,
        #                 max_curvature_speed=3.5,
        #                 a_curvature_exp=0.025, 
        #                 K_curvature_look_ahead=10,
        #                 a_curvature_speed_exp=0.1)
        # controller.path(paths[0], 
        #                 K_curvature_speed=3000,
        #                 max_curvature_speed=5,
        #                 a_curvature_exp=0.025, 
        #                 K_curvature_look_ahead=10,
        #                 a_curvature_speed_exp=0.1)
        sleep(1000, SECONDS)

        self.autonomous_cleanup()
    
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
                motor.intakeChain.stop(BrakeType.BRAKE)

                if auto_flags.drop_after_auto_halt:
                    pneumatic.intake.set(False)
                if auto_flags.raise_after_auto_halt:
                    pneumatic.intake.set(True)
    
    def path(self, path, *, events=[], checkpoints=[], backwards = False,
             look_ahead_dist=350, finish_margin=100, event_look_ahead_dist=75, timeout=None,
             heading_authority=2.0, max_turn_volts = 8,
             hPID_KP = 0.1, hPID_KD = 0.01, hPID_KI = 0, hPID_KI_MAX = 0, hPID_MIN_OUT = None,
             K_curvature_speed = 0.0, K_curvature_look_ahead = 0.0, a_curvature_exp = 0.1,
             max_curvature_speed = 3.0, min_look_ahead = 300, a_curvature_speed_exp = 0.1,
             speed_ramp_time = 1, min_start_voltage = 4, 
             slowdown_distance = 0, min_slow_voltage = 4) -> None:
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

            print(forwards_speed)

            # Do some stuff that lowers the authority of turning, no idea if this is reasonable
            heading_output *= heading_authority
            if heading_output > max_turn_volts: heading_output = max_turn_volts
            if heading_output < -max_turn_volts: heading_output = -max_turn_volts
            # if we rollover, fix it
            if rollover:
                heading_output *= -1
            if not waiting:
                left_speed = forwards_speed - dynamic_forwards_speed + heading_output
                right_speed = forwards_speed - dynamic_forwards_speed - heading_output
    
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
                        else:
                            # this is a variable change
                            # format: ["speed down", (0, 1130), "speed", 3.5]
                            if hasattr(self, event[2]):
                                setattr(self, event[2], event[3])
                                log("Updated {} to {}".format(event[2], event[3]))
                            else:
                                raise AttributeError("No attribute found {}".format(event[2]))
                    elif callable(event[2]):
                        # Call the function (at index 2) with the unpacked (*) args (at index 3)
                        try:
                            event[2](*event[3])
                            log("Ran {}({})".format(event[2], event[3]))
                        except:
                            raise NameError("Function not defined")

            data = {
                "curvature": curvature,
                "fwd_adjust": dynamic_forwards_speed,
                # "look_ahead": path_handler.look_dist
            }
            # packet_mgr.add_packet("pd", data)

            done = path_handler.path_complete
            if done:
                log("Path complete")

            if timeout is not None:
                if brain.timer.system() > time_end:
                    done = True
                    log("Path timed out", LogLevel.WARNING)

            if not done:
                sleep(20, MSEC)
            else:
                AutonomousCommands.kill_motors(BRAKE)

class ControllerFunctions():
    @staticmethod
    def switch_mogo_engaged():
        flags.mogo_pneu_engaged = not flags.mogo_pneu_engaged

    @staticmethod
    def switch_mogo():
        log("Switched mogo pneumatic")
        if pneumatic.mogo.value() == 0 and flags.mogo_pneu_engaged:
            flags.mogo_pneu_engaged = False

        pneumatic.mogo.set(not pneumatic.mogo.value())

    @staticmethod
    def switch_intake_height():
        if not flags.elevating:
            log("Switched intake height")
            pneumatic.intake.set(not pneumatic.intake.value())

    @staticmethod
    def switch_doinker():
        log("Switched doinker")
        log("Fix doinker pneumatics!", LogLevel.WARNING)
        # pneumatic.doinker.set(not pneumatic.doinker.value())

    @staticmethod
    def manual_elevation():
        log("Manual elevation pneumatics")
        log("Fix elevation pneumatics!", LogLevel.WARNING)
        # pneumatic.elevation_bar_lift.set(not pneumatic.elevation_bar_lift.value())
        # pneumatic.elevation_bar_lower.set(pneumatic.elevation_bar_lift.value())
    
    @staticmethod
    def toggle_PTO():
        log("Toggled PTO pneumatics")
        pneumatic.PTO.set(not pneumatic.PTO.value())
    
    @staticmethod
    def zero_lady_brown():
        log("Zeroed wall stake rotation sensor")
        sensor.wallEncoder.set_position(0)

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
        control_2.ZERO_ROT_SENSOR.pressed(ControllerFunctions.zero_lady_brown)

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
        if flags.elevating:
            self.elevation_loop()
        else:
            self.driver_loop()

    def drive_controls(self) -> None:
        turnVolts = (control.DRIVE_TURN_AXIS.position() * 0.12) * 0.9
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
        else:
            motor.intakeFlex.stop()
            motor.intakeChain.stop()
    
    def lady_brown_controls(self) -> None:
        # WALL STAKES MOTORS
        if control.LB_MANUAL_UP.pressing():
            motor.ladyBrown.spin(FORWARD, 100, PERCENT)
            self.robot.LB_PID.enabled = False
        elif control.LB_MANUAL_DOWN.pressing():
            motor.ladyBrown.spin(REVERSE, 40, PERCENT)
            self.robot.LB_PID.enabled = False
        elif not self.robot.LB_PID.enabled:
            motor.ladyBrown.stop(HOLD)

    def driver_loop(self) -> None:
        self.drive_controls()
        self.intake_controls()
        self.lady_brown_controls()

        self.robot.color_sort_controller.sense()

        # elevation hold button
        if con.buttonUp.pressing() and not flags.elevating:
            self.elevation_hold_duration -= 1
            if self.elevation_hold_duration <= 0:
                self.start_elevation()
        else:
            self.elevation_hold_duration = self.elevation_hold_duration_reset
    
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
    POS_LOAD = 12
    POS_ELEVATION_UNWIND = 110
    def __init__(self, wall_motor) -> None:
        # self.pid = MultipurposePID(0.03, 0, 0.005, 3, None)
        # self.pid = MultipurposePID(0.06, 0.08, 0.01, 5, None)
        self.pid = MultipurposePID(0.3, 0.55, 0.03, 2, None)
        # self.pid = MultipurposePID(0.1, 0.015, 0.02, 5, None)
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
        self.target_rotation = LadyBrown.POS_ELEVATION_UNWIND
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
        # print(sensor.wallEncoder.angle(), output)

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
    def __init__(self, parent: Robot) -> None:
        self.robot = parent
        
        self.queued_sort = False
        self.eject_next_ring = False

        self.blue_hue = 100

        self.wait_time_ms = 210
        self.hold_time_ms = 150
    
    def sense(self) -> None:
        """
        Run to detect if we have an incorrectly colored disk.
        If so, queue an eject of the next dist to the top of the intake.
        """
        # If we eject blue, 
        if ((flags.color_setting == "eject_blue") and
        sensor.intakeColor.hue() > self.blue_hue and 
        sensor.intakeColor.is_near_object() and 
        not self.eject_next_ring):

            self.eject_next_ring = True
            log("Found [blue] ring to eject.")
        
        if (flags.color_setting == "eject_red" and 
        sensor.intakeColor.hue() < 18 and 
        sensor.intakeColor.is_near_object()and 
        not self.eject_next_ring):
            
            self.eject_next_ring = True
            log("Found [red] ring to eject.")

        if ((sensor.intakeDistance.object_distance() < 70) and 
        (not self.queued_sort) and 
        (self.eject_next_ring)):

            self.allow_intake_input = False
            motor.intakeChain.spin(FORWARD, 100, PERCENT)

            brain.timer.event(self.intake_sorter, self.wait_time_ms)

            self.queued_sort = True
    
    def intake_sorter(self) -> None:
        # check if we are moving from manual --> auto or auto --> manual
        if flags.allow_intake_input:
            motor.intakeChain.stop(BRAKE)
            # stop chain and don't let driver loop re-enable it until we're done with it
            flags.allow_intake_input = False

            # At this point, allow_intake_input will be false
            # so the current code won't run in a loop
            # schedule this again to re-enable to intake in ms
            brain.timer.event(self.intake_sorter, self.hold_time_ms)
            log("Ejecting ring.")
        else:
            # Re-enable intake
            flags.allow_intake_input = True
            # reset timing flags
            self.queued_sort = False
            self.eject_next_ring = False

class PayloadManager():
    def __init__(self) -> None:
        self.queue = []
    
    def add_packet(self, topic, payload):
        pass

class flags():
    """
    'global' booleans enum for various states.
    """
    mogo_pneu_engaged = False
    mogo_pneu_status = False
    color_setting = ColorSort.NONE
    allow_intake_input = True
    elevating = False
    wall_setpoint = 1

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
        sleep(35, MSEC)

def start_odom_packets(robot: Robot):
    Thread(odom_packets, (robot,))

# run file
foxglove_log = True
log("Battery at {}".format(brain.battery.capacity()))
def main():
    sd_fail = False
    try:
        with open("cfg/config.json", 'r') as f:
            data = load(f)
        log("SUCCESS LOADING SD CARD")
    except:
        sd_fail = True
        log("ERROR LOADING SD CARD DATA", LogLevel.FATAL)

    if not sd_fail:
        robot = Robot()
        robot.LB_PID.thread = Thread(robot.LB_PID.run)
        comp = Competition(null_function, null_function)

        # brain.timer.event(start_odom_packets, 2000, (robot,))

        # Load autonomous into robot
        robot.autonomous_controller.load_path(data["autons"]["selected"])

        pull_data(data, robot) # send data from SD data (local scope) into robot object & subobjects
        # when connected to the field, do everything normally
        if comp.is_field_control() or comp.is_competition_switch():
            log("Connected to field or comp switch.")
            comp = Competition(robot.driver, robot.autonomous)

            calibrate_imu()
            calibrate_lady_brown()

            brain.screen.clear_screen(Color.GREEN)
            brain.screen.render()

            log("Ready for match!")

        elif data["config"]["auton_test"]:
            calibrate_imu()

            brain.screen.clear_screen(Color.YELLOW)
            brain.screen.render()
            log("Not auto-calibrating LB (auton test)", LogLevel.WARNING)
            robot.autonomous_controller.test()

        else:
            log("Default to driver (no auton test, no field control)", LogLevel.WARNING)

            calibrate_lady_brown()
            calibrate_imu()

            brain.screen.clear_screen(Color.YELLOW)
            brain.screen.render()

            robot.driver_controller.run()

    else:
        log("Robot object not created. (SD Load Error)", LogLevel.FATAL)
        raise ImportError("SD Card not inserted")

main()
