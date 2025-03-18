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
        packet_mgr.add_packet("foxglove.Log", {"timestamp": {"sec": time_s, "nsec": time_ms * 1000000}, "message": msg, "level": foxglove_logLevel, "name": "main", "file": "main", "line": 0})
    else:
        print("[{}] {}: {}".format(tag, time_ms + (time_s + time_min*60)*1000, msg))

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
    MANUAL_ELEVATION_PNEUMATICS =   con.buttonUp
    LB_MACRO_HOME =                 con.buttonDown
    BACK_OFF_ALLIANCE_STAKE =       con.buttonX

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
    doinker = DigitalOut(brain.three_wire_port.h)
    elevatoinBarLift = DigitalOut(brain.three_wire_port.d)
    intake = DigitalOut(brain.three_wire_port.g)

#### SENSORS
# ENCODERS
class sensor():
    leftEncoder = Rotation(Ports.PORT2)
    rightEncoder = Rotation(Ports.PORT17)
    driftEncoder = Rotation(Ports.PORT13)

    wallEncoder = Rotation(Ports.PORT20)
    # DISTANCE SENSORS
    intakeDistance = Distance(Ports.PORT21)

    wallLeftDistance = Distance(Ports.PORT6)
    wallBackDistance = Distance(Ports.PORT1)
    wallFrontDistance = Distance(Ports.PORT9)
    wallRightDistance = Distance(Ports.PORT19)

    # elevationDistance = Distance(Ports.PORT20) # unplugged

    # MISC SENSORS
    intakeColor = Optical(Ports.PORT14)
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
            # print("difting!")
            # we are drifting
            drift_x = d_drift * math.sin(math.radians(h + 90))
            drift_y = d_drift * math.cos(math.radians(h + 90))
        else:
            # print("not difting!")
            drift_x, drift_y = 0, 0

        dx = (dNet * math.sin(h_rad))# + drift_x
        dy = (dNet * math.cos(h_rad))# + drift_y

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

        self.positioning_algorithm = DeltaPositioning(sensor.leftEncoder, sensor.rightEncoder, sensor.driftEncoder, sensor.imu)
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
        motor.ladyBrown.stop()

        self.sequence(globals())
        
        self.autonomous_cleanup()

    def test(self) -> None:
        """
        Run a test version of autonomous. This is NOT run in competition!
        """
        log("Running autonomous TEST", LogLevel.WARNING)
        # self.run()
        self.autonomous_setup()


        paths = [[(-1510.0, 330.0, 0), (-1474.09, 364.72, 0.54), (-1433.79, 394.26, 0.37), (-1390.89, 419.92, 0.12), (-1347.25, 444.33, 0.25), (-1305.25, 471.39, 0.84), (-1269.92, 506.51, 1.45), (-1249.67, 551.84, 1.39), (-1246.36, 601.54, 0.88), (-1254.08, 650.88, 0.44)], [(-1259.67, 710.7, 0), (-1243.86, 663.27, 0.16), (-1229.95, 615.24, 0.16), (-1218.0, 566.69, 0.16), (-1208.04, 517.7, 0.16), (-1200.05, 468.35, 0.16), (-1194.02, 418.72, 0.15), (-1189.83, 368.9, 0.13), (-1187.29, 318.96, 0.12), (-1186.2, 268.98, 0.09), (-1186.27, 218.98, 0.06), (-1187.15, 168.99, 0.03), (-1188.45, 119.0, 0.0), (-1189.71, 69.02, 0.04), (-1190.45, 19.02, 0.08), (-1190.13, -30.97, 0.13)], [(-1184.28, -76.7, 0), (-1182.92, -26.73, 0.4), (-1176.63, 22.85, 0.38), (-1165.61, 71.6, 0.37), (-1150.17, 119.14, 0.35), (-1130.68, 165.16, 0.33), (-1107.51, 209.45, 0.3), (-1081.05, 251.85, 0.29), (-1051.61, 292.25, 0.27), (-1019.51, 330.57, 0.26), (-985.01, 366.74, 0.25), (-948.32, 400.69, 0.24), (-909.64, 432.36, 0.24), (-869.13, 461.65, 0.24), (-826.93, 488.45, 0.25), (-783.17, 512.6, 0.25), (-737.95, 533.92, 0.27), (-691.4, 552.13, 0.29), (-643.65, 566.92, 0.32), (-594.89, 577.88, 0.33)], [(-543.48, 582.95, 0), (-543.72, 632.95, 0.05), (-544.54, 682.94, 0.06), (-546.13, 732.92, 0.07), (-548.62, 782.86, 0.07), (-552.03, 832.74, 0.05), (-556.11, 882.57, 0.03), (-560.54, 932.38, 0.0), (-565.02, 982.17, 0.01), (-569.41, 1031.98, 0.01), (-573.63, 1081.8, 0.07), (-577.02, 1131.67, 0.05), (-579.75, 1181.5, 0.87), (-593.18, 1229.38, 1.37), (-621.94, 1269.69, 1.55), (-664.09, 1295.86, 1.14), (-712.17, 1309.03, 0.68), (-761.89, 1313.91, 0.4), (-811.87, 1313.82, 0.21), (-861.79, 1311.1, 0.12), (-911.61, 1306.87, 0.07), (-961.36, 1301.82, 0.01), (-1011.09, 1296.67, 0.03), (-1060.86, 1291.85, 0.06), (-1110.69, 1287.75, 0.09), (-1160.59, 1284.72, 0.12), (-1210.56, 1283.13, 0.14), (-1260.56, 1283.33, 0.18), (-1310.49, 1285.84, 0.21), (-1360.22, 1290.99, 0.24), (-1409.53, 1299.15, 0.27), (-1458.16, 1310.66, 0.31), (-1505.76, 1325.86, 0.34), (-1551.91, 1345.0, 0.36), (-1596.15, 1368.19, 0.37), (-1638.05, 1395.38, 0.38), (-1677.22, 1426.38, 0.37), (-1713.38, 1460.83, 0.35), (-1746.44, 1498.28, 0.32), (-1776.42, 1538.23, 0.28), (-1803.51, 1580.22, 0.25), (-1827.89, 1623.87, 0.3), (-1848.93, 1669.23, 0.19), (-1867.82, 1715.49, 0.15), (-1884.92, 1762.46, 0.15)], [(-1785.64, 1764.06, 0), (-1752.28, 1726.82, 0.02), (-1718.72, 1689.75, 0.02), (-1684.94, 1652.89, 0.03), (-1650.92, 1616.25, 0.03), (-1616.61, 1579.88, 0.03), (-1582.0, 1543.79, 0.04), (-1547.05, 1508.04, 0.04), (-1511.72, 1472.66, 0.05), (-1475.96, 1437.71, 0.05), (-1439.77, 1403.21, 0.05), (-1403.12, 1369.21, 0.05), (-1366.0, 1335.71, 0.05), (-1328.42, 1302.72, 0.05), (-1290.44, 1270.21, 0.04)], [(-1260.43, 1242.9, 0), (-1213.73, 1225.04, 0.03), (-1166.91, 1207.48, 0.04), (-1119.94, 1190.36, 0.06), (-1072.72, 1173.9, 0.08), (-1025.2, 1158.37, 0.1), (-977.28, 1144.09, 0.13), (-928.92, 1131.42, 0.17), (-880.06, 1120.82, 0.18), (-830.78, 1112.39, 0.18), (-781.17, 1106.22, 0.16), (-731.35, 1102.0, 0.13), (-681.42, 1099.43, 0.1), (-631.44, 1098.05, 0.07), (-581.44, 1097.54, 0.05), (-531.44, 1097.63, 0.03), (-481.44, 1098.1, 0.02), (-431.45, 1098.83, 0.01), (-381.46, 1099.69, 0.0), (-331.46, 1100.61, 0.0)], [(-115.12, 1096.69, 0), (-165.07, 1094.64, 0.0), (-215.03, 1092.55, 0.01), (-264.98, 1090.39, 0.01), (-314.93, 1088.13, 0.01), (-364.88, 1085.79, 0.01), (-414.82, 1083.35, 0.01), (-464.75, 1080.81, 0.01), (-514.68, 1078.14, 0.01), (-564.6, 1075.31, 0.01), (-614.51, 1072.31, 0.02), (-664.41, 1069.1, 0.02), (-714.29, 1065.64, 0.02), (-764.15, 1061.88, 0.04), (-813.97, 1057.68, 0.05), (-863.74, 1052.9, 0.07), (-913.42, 1047.25, 0.08), (-962.97, 1040.56, 0.25), (-1012.79, 1036.95, 0.43), (-1062.75, 1038.74, 0.07)], [(-1153.21, 1052.83, 0), (-1104.09, 1043.65, 0.41), (-1056.17, 1029.5, 0.48), (-1010.31, 1009.7, 0.42), (-966.74, 985.22, 0.43), (-926.1, 956.16, 0.37), (-888.29, 923.48, 0.35), (-853.49, 887.62, 0.31), (-821.56, 849.16, 0.26), (-792.17, 808.72, 0.27), (-765.56, 766.4, 0.2), (-741.12, 722.79, 0.19), (-718.73, 678.09, 0.19), (-698.53, 632.36, 0.15), (-680.05, 585.91, 0.14), (-663.19, 538.84, 0.14), (-647.94, 491.22, 0.13), (-634.25, 443.14, 0.11), (-621.88, 394.7, 0.11), (-610.79, 345.94, 0.1), (-600.98, 296.92, 0.11), (-592.47, 247.65, 0.09), (-585.09, 198.2, 0.09), (-578.83, 148.59, 0.1), (-573.81, 98.85, 0.09), (-569.97, 49.0, 0.09), (-567.27, -0.93, 0.11), (-565.93, -50.91, 0.1), (-565.89, -100.9, 0.1)]]
        self.robot.pos = [-1510.0, 330.0]
        sensor.imu.set_heading(210)
        flags.color_setting = ColorSort.EJECT_BLUE

        motor.ladyBrown.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        end_timeout = brain.timer.system() + 700
        while sensor.wallEncoder.angle() < 190:
            if brain.timer.system() > end_timeout:
                log("Lady brown movement timed out!", LogLevel.WARNING)
                break
            sleep(20, TimeUnits.MSEC)
        log("Lady brown finished or timed out. Encoder at {} degrees.".format(sensor.wallEncoder.angle()))
        motor.ladyBrown.stop(BrakeType.COAST)

        self.fwd_speed = 7
        self.path(paths[0], backwards=True, speed_ramp_time=400)

        AutonomousFlags.intake_auto_halt = True
        pneumatic.intake.set(True)
        motor.intakeFlex.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)

        AutonomousFlags.lady_brown_autostop = True
        motor.ladyBrown.spin(DirectionType.REVERSE, 100, VelocityUnits.PERCENT)

        self.path(paths[1], speed_ramp_time=400, slowdown_distance=800)

        # grab mobile goal
        self.path(paths[2], backwards=True, speed_ramp_time=400, slowdown_distance=800)
        pneumatic.intake.set(False)
        pneumatic.mogo.set(True)
        sleep(150, TimeUnits.MSEC)

        # turn towards ring
        drivetrain.turn_for(TurnType.RIGHT, 90, RotationUnits.DEG, 80, VelocityUnits.PERCENT)
        
        motor.intakeChain.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
        self.fwd_speed = 8
        # drive into corner
        self.path(paths[3], speed_ramp_time=400, slowdown_distance=700, timeout=3000)

        self.fwd_speed = 6
        self.path(paths[4], backwards=True)

        drivetrain.turn_for(TurnType.RIGHT, 170, RotationUnits.DEG, 100, VelocityUnits.PERCENT)
        self.fwd_speed = 8
        AutonomousFlags.intake_color_kill = Color.BLUE
        self.path(paths[5], slowdown_distance=800, speed_ramp_time=400)
        self.path(paths[6], backwards=True, speed_ramp_time=400, slowdown_distance=600)
        self.path(paths[7], speed_ramp_time=400, slowdown_distance=600, timeout=2500)

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
        pneumatic.doinker.set(not pneumatic.doinker.value())

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
        pneumatic.elevatoinBarLift.set(not pneumatic.elevatoinBarLift.value())
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
        control.MANUAL_ELEVATION_PNEUMATICS.pressed(ControllerFunctions.elevation_bar)
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
    BLUE = 100
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
    mogo_pneu_engaged = False
    mogo_pneu_status = False
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

        con.screen.print(data["autons"]["selected"])

        pull_data(data, robot) # send data from SD data (local scope) into robot object & subobjects
        # when connected to the field, do everything normally
        if comp.is_field_control() or comp.is_competition_switch():
            log("Connected to field or comp switch.")
            
            comp = Competition(robot.driver, robot.autonomous)

            calibrate_imu()

            brain.screen.clear_screen(Color.GREEN)

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

            calibrate_imu()

            brain.screen.clear_screen(Color.CYAN)

            robot.driver_controller.run()

    else:
        log("Robot object not created. (SD Load Error)", LogLevel.FATAL)
        raise ImportError("SD Card not inserted")

main()
