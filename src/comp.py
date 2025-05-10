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
    """
    Handle the display of a log with format.
    Arguments:
        msg: Any to convert to string
        level: LogLevel with serverity
    """
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
        # thread the log so that it doesn't clog the main process
        Thread(threaded_log, (msg, level))

class PacketTiming:
    """
    Namespace class with msec timings to not drop packets.
    """
    CONTROLLER = 50
    BRAIN = 20

class PacketManager():
    """
    Class to handle queued messages for foxglove.
    """
    def __init__(self, timing: PacketTiming | int) -> None:
        """
        Create a packet manager.
        Sets timing for all messages sent through this object.
        """
        self.thread: Thread
        self.queue = []
        self.delay = timing
        self.allow_sending = True

    def start(self) -> None:
        """
        Initalize the packet sender.
        """
        #print newline to ensure we don't get a decode error from previous print
        print("") 
        self.thread = Thread(self.loop)
    
    def loop(self) -> None:
        while True:
            if self.allow_sending:
                if len(self.queue) != 0:
                    print(self.queue.pop(0)[1])
            
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

# setup logging functionality
do_logging = True
packet_mgr = PacketManager(PacketTiming.CONTROLLER)
packet_mgr.start()

brain = Brain()
con = Controller(PRIMARY)
con_2 = Controller(PARTNER)

class control():
    """
    Bindings for controller inputs so they can be reassigned here rather than in the code.
    """
    DRIVE_FORWARD_AXIS =            con.axis3
    DRIVE_TURN_AXIS =               con.axis1
    INTAKE_IN_HOLD =                con.buttonR1
    INTAKE_OUT_HOLD =               con.buttonR2
    MOGO_GRABBER_TOGGLE =           con.buttonA
    DOINKER =                       con.buttonRight
    LB_MANUAL_UP =                  con.buttonL1
    LB_MANUAL_DOWN =                con.buttonL2
    LB_MACRO_HOME =                 con.buttonDown
    LB_MACRO_DESCORE =              con.buttonUp

class motor():
    leftA = Motor(Ports.PORT14, GearSetting.RATIO_6_1, False) # stacked top
    leftB = Motor(Ports.PORT15, GearSetting.RATIO_6_1, False) # rear
    leftC = Motor(Ports.PORT16, GearSetting.RATIO_6_1, False) # front

    rightA = Motor(Ports.PORT11, GearSetting.RATIO_6_1, True) # stacked top
    rightB = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True) # rear
    rightC = Motor(Ports.PORT13, GearSetting.RATIO_6_1, True) # front

    intakeChain = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
    intakeFlex = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
    ladyBrown = Motor(Ports.PORT18, GearSetting.RATIO_36_1,  False)

class pneumatic():
    mogo = DigitalOut(brain.three_wire_port.e)
    doinker_left = DigitalOut(brain.three_wire_port.f)
    doinker = DigitalOut(brain.three_wire_port.h)
    intake = DigitalOut(brain.three_wire_port.g)

class sensor():
    groundEncoder = Rotation(Ports.PORT2)
    wallEncoder = Rotation(Ports.PORT17)

    intakeDistance = Distance(Ports.PORT21)
    # wallLeftDistance = Distance(Ports.PORT6)
    # wallBackDistance = Distance(Ports.PORT1)
    # wallFrontDistance = Distance(Ports.PORT9)
    # wallRightDistance = Distance(Ports.PORT19)

    intakeColor = Optical(Ports.PORT4)
    imu = Inertial(Ports.PORT3)

lmg = MotorGroup(motor.leftA, motor.leftB, motor.leftC)
rmg = MotorGroup(motor.rightA, motor.rightB, motor.rightC)

# drivetrain for stuff that we want to use the generic drive/turn functions
drivetrain = SmartDrive(lmg, rmg, sensor.imu)

# Turn on intake color sensor LED for consistency
sensor.intakeColor.set_light_power(100, PERCENT)

# global functions (no I/O)
def calibrate_imu():
    log("calibrating IMU. Minimum calibration time 2.5 seconds.")
    min_duration = brain.timer.system() + 2500

    sensor.imu.calibrate()
    # we use time here instead of sensor.imu.is_calibrating() because from our experience
    # is_calibrating can return false positives. Only really an issue when practicing
    # because in a match the robot doesn't start moving before the imu is done calibrating
    # on it's own.
    while brain.timer.system() < min_duration:
        sleep(10, TimeUnits.MSEC)

    log("IMU calibration complete")

# helper math functions
def dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

def gauss(mu = 0, sigma = 1):
    u1 = random()
    u2 = random()

    z = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
    return mu + sigma * z

class PurePursuit():
    """
    Class to handle all math involved with path calculations. Create a new instance for each path.
    """
    def __init__(self, look_ahead_dist: int, finish_margin: int, 
                 path: list[tuple[float, float, float]] | list[tuple[float, float]], 
                 checkpoints: list[int]) -> None:
        """
        Init variables. Robot position is able to be passed in from another source, like a GPS sensor or odometry algorithm.
        Curvature in the path is optional -- if given, goal_search will also return a value linearly interpolated
            between the points before and after the found goal point and their curvatures.
        Arguments:
            look_ahead_dist: distance in mm
            finish_margin: distance in mm from last point in path to return true on self.path_complete
            path: list of tuples with three floats x, y, and curvature.
            checkpoints: list of point indeces to break the path
        """
        self.path = path
        self.look_dist = look_ahead_dist
        self.finish_margin = finish_margin
        self.last_found_point: int = 0

        self.path_complete: bool = False

        self.checkpoints = checkpoints
        if len(self.checkpoints) != 0:
            log("Setup path with checkpoints")
            self.checkpoint_index = 0
            self.current_checkpoint = checkpoints[self.checkpoint_index]
            self.checkpoints_complete = False
        else:
            self.checkpoints_complete = True

    def goal_search(self, current_pos: list[float] | tuple[float, float]
                    ) -> tuple[tuple[float, float], float]:
        """
        Run every time you want to update your goal position. 
        Returns the point along our path that is look_dist away from the robot
            and that is closest to the end of the path.

        Arguments
            current_pos: an iterable of floats representing x, y position of our robot
        Returns
            goal_pos: x, y floats of the targeted goal
            curvature: lerp between the third value in each point along the path
        """
        goal = self.path[self.last_found_point+1][:2]
        curvature = 0.0

        # Iterate over every un-crossed point in our path.
        # if we are close to the finish point, regardless of what has happened, finish the path
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
                    
                    # this is where divide by zero shows up, never bothered to fix though
                    m = (by - ay) / (bx - ax) # slope of line between point a and b
                    b = m*(-ax) + ay # y-intercept of line
                    r = self.look_dist
                    # quadratic terms
                    # using math from https://mathworld.wolfram.com/Circle-LineIntersection.html
                    # see graphs at https://www.desmos.com/calculator/zb3qakxeev 
                    A = (m*m) + 1
                    B = (2*m*(b-k)-(2*h))
                    C = ((h*h) + ((b-k)*(b-k)) - (r*r))

                    discriminant = (B*B) - (4*A*C)
                    if discriminant >= 0:
                        # there is at least one possible solution
                        sol1_x = (-B + math.sqrt(discriminant)) / (2*A)
                        sol1_y = m*sol1_x + b

                        sol2_x = (-B - math.sqrt(discriminant)) / (2*A)
                        sol2_y = m*sol2_x + b

                        sol1 = (sol1_x, sol1_y)
                        sol2 = (sol2_x, sol2_y)

                        minX, minY = min(ax, bx), min(ay, by)
                        maxX, maxY = max(ax, bx), max(ay, by)

                        # general check to see if either point is on the line 
                        # based on the limits of the lines
                        in_bounds_sol1: bool = (minX < sol1_x < maxX) and (minY < sol1_y < maxY)
                        in_bounds_sol2: bool = (minX < sol2_x < maxX) and (minY < sol2_y < maxY)
                        if in_bounds_sol1 or in_bounds_sol2:
                            sol1_distance = dist(sol1, point2)
                            sol2_distance = dist(sol2, point2)

                            if in_bounds_sol1 and in_bounds_sol2:
                                # both solutions are within bounds, so we need to compare and decide which is better

                                # choose based on distance to pt2
                                if sol1_distance < sol2_distance:
                                    goal = sol1
                                else:
                                    goal = sol2
                            else:
                                # only one solution is in bounds
                                if in_bounds_sol1:
                                    goal = sol1
                                else:
                                    goal = sol2
                            
                            # we have a goal solution at this point in time
                            # we can now get that solutions completion of the line segment
                            # this lets us linearly interpolate between the curvature of point[i] and point[i+1]
                            if len(self.path[i]) == 3: # lerp for curvature at this point
                                segment_length = dist(self.path[i][:2], self.path[i+1][:2])
                                if goal == sol1: completed_length = sol1_distance
                                else: completed_length = sol2_distance

                                percent = (1 - (completed_length/segment_length))
                                curvature = (self.path[i][2] * (1 - percent) + self.path[i+1][2] * percent) #type: ignore
                            
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

        return goal[:2], curvature

class DeltaPositioning():
    """Odometry based on tracking wheels."""
    def __init__(self, enc: Rotation | Motor, imu: Inertial) -> None:
        """
        This class handled forwards and backwards odom in mm based on changes in tracking wheel
        angle and inertial change in heading.
        Arguments:
            enc: desired encoder(s) to be used for locomotion
            imu: desired inertial for heading data
        """
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

        # drift stuff is unused because I didn't do it right
        # drift_dist = 80
        # self.drift_circumference = 2 * 3.14159 * drift_dist
        # self.drift_accum = 0

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
        # or use the single tracking wheel
        dNet = d

        dx = (dNet * math.sin(h_rad))# + drift_x
        dy = (dNet * math.cos(h_rad))# + drift_y

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
    def calculate(self, TARGET, INPUT) -> int:
        '''
        Calculates the output based on the PID's tuning and the provided target & input

        Args:
            TARGET (int): a number that the PID will try to reach
            INPUT (int): the current sensor or other input

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

        return output

class Robot():
    """Small class that allows driver and auton to share data."""
    # probably not the best way to do this but its better than storing everything globally (?)
    def __init__(self) -> None:
        self.color_sort_controller = ColorSortController(parent=self)

        self.autonomous_controller = Autonomous(parent=self)
        self.driver_controller = Driver(parent=self)

        self.LB_PID = LadyBrown(motor.ladyBrown)

        self.pos = [0.0, 0.0]
        self.heading = 0.0
        self.last_intake_color = 0.0

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
    # honestly basically a duplicate of flags but only used in auton so i suppose it gets its own class
    intake_halt_tolerance = 60
    intake_auto_halt = False
    intake_flex_auto_halt = False
    drop_after_auto_halt = False
    raise_after_auto_halt = False
    last_intake_command = 0.0
    intake_anti_jam = False
    intake_auto_stop_type = BrakeType.BRAKE
    intake_color_kill: None | Color.DefinedColor = None
    intake_flex_extake_on_ring = 0 # valid types 0 (none), 1 (extake when see blue), 2 (extake when see red)

    lady_brown_autostop = False

# robot states
class AutonomousRoutines:
    # this is from the output of our GUI program
    red_solo_AWP = [[0,{'pos':[-1550,-350],'angle':330}],[6,'alliance_wall_stake'],[2,[],(('fwd_volt',5.0),('backwards',True),('speed_ramp_time',500),('slowdown_distance',300)),[(-1551.0,-351.0),(-1520.7,-390.8),(-1489.3,-429.6),(-1457.0,-467.9),(-1424.3,-505.7),(-1391.3,-543.2),(-1358.2,-580.7),(-1325.2,-618.2),(-1292.5,-656.1),(-1260.5,-694.5)]],[1,'ladyBrown',-1,12.0],[7,['RIGHT',40,0.0]],[1,'ladyBrown',0,'BRAKE'],[1,'intakeChain',1,12.0],[1,'intakeFlex',1,12.0],[5,True,'intake'],[2,[],(('fwd_volt',5.0),('finish_margin',500)),[(-1240.0,-732.0),(-1239.9,-662.0),(-1241.2,-592.0),(-1242.7,-522.0),(-1244.3,-452.1),(-1245.8,-382.1),(-1247.2,-312.1),(-1248.3,-242.1),(-1249.0,-172.1),(-1249.1,-102.1),(-1249.0,-80.0),(-1247.8,-10.0),(-1247.5,60.0),(-1247.7,130.0),(-1248.0,200.0),(-1248.1,270.0),(-1247.5,340.0)]],[3,(('intake_auto_halt',True),)],[4,125],[5,False,'intake'],[4,75],[1,'intakeFlex',-1,12.0],[2,[],(('fwd_volt',5.0),('min_start_voltage',2.0),('speed_ramp_time',500),('backwards',True),('slowdown_distance',400)),[(-1228.0,214.0),(-1224.6,164.1),(-1216.8,114.8),(-1205.2,66.1),(-1190.3,18.4),(-1172.6,-28.4),(-1152.5,-74.1),(-1130.0,-118.8),(-1105.5,-162.4),(-1079.2,-204.8),(-1051.2,-246.2),(-1021.6,-286.6),(-990.5,-325.8),(-958.2,-363.9),(-924.6,-400.9),(-889.9,-436.9),(-854.1,-471.8),(-817.3,-505.6),(-779.5,-538.4),(-740.9,-570.2),(-701.6,-601.0),(-661.4,-630.8),(-620.5,-659.6),(-579.0,-687.5)]],[5,True,'mogo'],[3,(('intake_auto_halt',False),)],[1,'intakeChain',1,12.0],[1,'intakeFlex',1,12.0],[7,['LEFT',110,0.0]],[4,350],[5,False,'mogo'],[2,[],(('fwd_volt',5.0),),[(-592.0,-733.0),(-591.5,-783.0),(-590.9,-833.0),(-590.3,-883.0),(-589.5,-933.0),(-588.7,-983.0),(-587.7,-1033.0),(-586.6,-1083.0),(-585.6,-1132.9),(-585.0,-1182.9),(-584.6,-1232.9),(-584.4,-1282.9),(-584.2,-1332.9),(-584.1,-1382.9),(-584.1,-1432.9),(-584.0,-1482.9),(-584.0,-1532.9)]],[3,(('intake_auto_halt',True),)],[4,100],[2,[],(('fwd_volt',3.7),('backwards',True),('slowdown_distance',900),('look_ahead_dist',250),('finish_margin',270)),[(-555.0,-1632.0),(-561.8,-1582.5),(-567.1,-1532.8),(-570.3,-1482.9),(-570.8,-1432.9),(-567.5,-1383.0),(-558.6,-1333.8),(-541.8,-1286.8),(-514.3,-1245.3),(-475.8,-1213.6),(-430.1,-1193.6),(-381.3,-1183.1),(-331.5,-1179.1),(-281.5,-1179.3),(-231.6,-1182.3),(-181.8,-1187.2),(-132.2,-1193.4),(-82.7,-1200.4),(-33.3,-1208.1),(16.1,-1216.1)]],[5,True,'mogo'],[2,[('motor_spin',{'motor':'intakeChain','velocity':100,'direction':1},[-634,-1277])],(('fwd_volt',5.0),('timeout',2300)),[(18.0,-1193.0),(-31.2,-1201.7),(-80.6,-1209.8),(-130.0,-1217.2),(-179.5,-1224.2),(-229.1,-1230.9),(-278.7,-1237.3),(-328.3,-1243.5),(-377.9,-1249.5),(-427.6,-1255.5),(-477.2,-1261.4),(-526.8,-1267.4),(-576.5,-1273.5),(-626.1,-1279.6),(-675.7,-1286.0),(-725.3,-1292.6),(-774.8,-1299.4),(-824.3,-1306.6),(-873.7,-1314.1),(-923.1,-1322.0),(-972.4,-1330.5),(-1021.5,-1339.5),(-1070.6,-1349.1),(-1119.5,-1359.4),(-1168.3,-1370.6),(-1216.8,-1382.6),(-1265.0,-1395.7),(-1313.0,-1410.0),(-1360.5,-1425.6),(-1407.5,-1442.7),(-1453.8,-1461.5),(-1499.2,-1482.3),(-1543.6,-1505.4),(-1586.4,-1531.2),(-1627.3,-1559.9),(-1665.6,-1592.0),(-1700.5,-1627.8),(-1730.9,-1667.4),(-1756.0,-1710.7),(-1774.7,-1757.0)]],[1,'intakeChain',1,12.0],[3,(('intake_auto_halt',False),)],[4,200],[2,[],(('fwd_volt',5.0),('backwards',True)),[(-1696.0,-1730.0),(-1664.3,-1691.4),(-1630.3,-1654.7),(-1594.1,-1620.2),(-1555.8,-1588.1),(-1515.5,-1558.5),(-1473.5,-1531.4),(-1430.0,-1506.7),(-1385.5,-1484.0),(-1340.3,-1462.6),(-1294.9,-1441.6),(-1249.8,-1420.1),(-1205.7,-1396.5),(-1163.8,-1369.3),(-1125.6,-1337.1),(-1093.0,-1299.2),(-1067.2,-1256.5),(-1048.2,-1210.2),(-1035.2,-1162.0),(-1026.9,-1112.7),(-1022.3,-1062.9),(-1020.7,-1013.0),(-1021.4,-963.0),(-1023.9,-913.0),(-1028.0,-863.0),(-1028.5,-813.0),(-1032.7,-763.2),(-1044.2,-714.6),(-1069.9,-672.2),(-1112.1,-646.2),(-1160.9,-635.9),(-1210.8,-632.9),(-1260.8,-632.9),(-1310.8,-633.9),(-1360.8,-635.1)]],[2,[],(('fwd_volt',5.0),('speed_ramp_time',500),('slowdown_distance',600),('timeout',1600)),[(-1409.0,-636.0),(-1359.0,-634.5),(-1309.3,-629.8),(-1260.1,-620.7),(-1212.4,-605.8),(-1167.8,-583.4),(-1128.8,-552.4),(-1098.0,-513.1),(-1076.0,-468.3),(-1061.2,-420.5),(-1050.8,-371.6),(-1042.4,-322.4),(-1033.5,-273.1),(-1022.1,-224.5),(-1005.5,-177.4),(-981.6,-133.5),(-949.2,-95.5),(-909.5,-65.3),(-864.9,-43.0),(-817.4,-27.5),(-768.4,-17.3),(-718.8,-11.3),(-668.9,-8.4)]],[1,'intakeChain',0,'COAST'],[1,'intakeFlex',0,'COAST']]
    blue_solo_AWP = [[0,{'pos':[1550,-350],'angle':30}],[6,'alliance_wall_stake'],[2,[],(('fwd_volt',5.0),('backwards',True),('speed_ramp_time',500),('slowdown_distance',300)),[(1551.0,-351.0),(1520.7,-390.8),(1489.3,-429.6),(1457.0,-467.9),(1424.3,-505.7),(1391.3,-543.2),(1358.2,-580.7),(1325.2,-618.2),(1292.5,-656.1),(1260.5,-694.5)]],[1,'ladyBrown',-1,12.0],[7,['LEFT',40,0.0]],[1,'ladyBrown',0,'BRAKE'],[1,'intakeChain',1,12.0],[1,'intakeFlex',1,12.0],[5,True,'intake'],[3,(('intake_flex_extake_on_ring',2),)],[2,[],(('fwd_volt',5.0),('finish_margin',500)),[(1240.0,-732.0),(1239.9,-662.0),(1241.2,-592.0),(1242.7,-522.0),(1244.3,-452.1),(1245.8,-382.1),(1247.2,-312.1),(1248.3,-242.1),(1249.0,-172.1),(1249.1,-102.1),(1249.0,-80.0),(1247.7,-10.0),(1247.6,60.0),(1247.9,130.0),(1248.1,200.0),(1247.4,270.0)]],[3,(('intake_auto_halt',True),)],[4,125],[5,False,'intake'],[4,75],[3,(('intake_flex_extake_on_ring',0),)],[2,[],(('fwd_volt',5.0),('min_start_voltage',2.0),('speed_ramp_time',500),('backwards',True),('slowdown_distance',400)),[(1228.0,214.0),(1224.6,164.1),(1216.8,114.8),(1205.2,66.1),(1190.3,18.4),(1172.6,-28.4),(1152.5,-74.1),(1130.0,-118.8),(1105.5,-162.4),(1079.2,-204.8),(1051.2,-246.2),(1021.6,-286.6),(990.5,-325.8),(958.2,-363.9),(924.6,-400.9),(889.9,-436.9),(854.1,-471.8),(817.3,-505.6),(779.5,-538.4),(740.9,-570.2),(701.6,-601.0),(661.4,-630.8),(620.5,-659.6),(579.0,-687.5)]],[5,True,'mogo'],[3,(('intake_auto_halt',False),)],[1,'intakeChain',1,12.0],[1,'intakeFlex',1,12.0],[7,['RIGHT',105,0.0]],[4,350],[5,False,'mogo'],[2,[],(('fwd_volt',5.0),),[(633.0,-689.0),(632.6,-739.0),(632.5,-789.0),(632.8,-839.0),(633.6,-889.0),(635.1,-939.0),(637.7,-988.9),(641.6,-1038.7),(646.5,-1088.5),(651.2,-1138.3),(654.8,-1188.2),(657.5,-1238.1),(659.5,-1288.0),(660.8,-1338.0),(661.8,-1388.0),(662.4,-1438.0),(662.8,-1488.0),(663.0,-1538.0)]],[3,(('intake_auto_halt',True),)],[4,100],[1,'intakeFlex',-1,12.0],[2,[],(('fwd_volt',3.7),('backwards',True),('slowdown_distance',900),('look_ahead_dist',250),('finish_margin',270)),[(630.0,-1601.0),(637.6,-1551.6),(642.9,-1501.9),(645.0,-1451.9),(642.5,-1402.0),(632.9,-1353.0),(612.9,-1307.3),(579.8,-1270.3),(536.1,-1246.4),(487.6,-1234.3),(437.8,-1230.3),(387.8,-1231.4),(338.0,-1235.7),(288.5,-1242.1),(239.1,-1250.0),(189.9,-1258.9),(140.8,-1268.4),(91.8,-1278.3),(42.8,-1288.5)]],[5,True,'mogo'],[1,'intakeFlex',1,12.0],[2,[('motor_spin',{'motor':'intakeChain','velocity':100,'direction':1},[697,-1254])],(('fwd_volt',5.0),('timeout',2300)),[(-6.0,-1269.0),(44.0,-1270.8),(93.9,-1272.2),(143.9,-1273.4),(193.9,-1274.6),(243.9,-1276.1),(293.9,-1277.8),(343.8,-1279.9),(393.8,-1282.3),(443.7,-1285.1),(493.6,-1288.4),(543.4,-1292.1),(593.3,-1296.3),(643.0,-1301.0),(692.8,-1306.3),(742.4,-1312.2),(792.0,-1318.6),(841.5,-1325.7),(890.9,-1333.4),(940.2,-1341.8),(989.4,-1350.9),(1038.4,-1360.7),(1087.2,-1371.2),(1135.9,-1382.6),(1184.4,-1394.8),(1232.7,-1407.8),(1280.7,-1421.7),(1328.5,-1436.5),(1375.9,-1452.3),(1423.0,-1469.1),(1469.7,-1486.9),(1516.1,-1505.8),(1561.9,-1525.7),(1607.2,-1546.8),(1652.0,-1569.1),(1696.2,-1592.5),(1739.6,-1617.2),(1782.4,-1643.2),(1824.3,-1670.4)]],[4,200],[2,[],(('fwd_volt',5.0),('backwards',True)),[(1696.0,-1730.0),(1664.3,-1691.4),(1630.3,-1654.7),(1594.1,-1620.2),(1555.8,-1588.1),(1515.5,-1558.5),(1473.5,-1531.4),(1430.0,-1506.7),(1385.5,-1484.0),(1340.3,-1462.6),(1294.9,-1441.6),(1249.8,-1420.1),(1205.7,-1396.5),(1163.8,-1369.3),(1125.6,-1337.1),(1093.0,-1299.2),(1067.2,-1256.5),(1048.2,-1210.2),(1035.2,-1162.0),(1026.9,-1112.7),(1022.3,-1062.9),(1020.7,-1013.0),(1021.4,-963.0),(1023.9,-913.0),(1028.0,-863.0),(1030.5,-813.1),(1039.7,-764.0),(1055.6,-716.6),(1077.8,-671.9),(1105.8,-630.5),(1138.6,-592.8),(1175.6,-559.2),(1215.9,-529.7),(1259.0,-504.3),(1304.3,-483.2),(1351.3,-466.3),(1399.7,-453.8),(1449.1,-445.8),(1498.9,-442.7)]],[2,[],(('fwd_volt',5.0),('speed_ramp_time',500),('slowdown_distance',600),('timeout',1600)),[(1489.0,-440.0),(1439.7,-448.0),(1389.7,-448.5),(1340.3,-441.5),(1292.1,-428.1),(1245.8,-409.5),(1201.2,-386.8),(1158.4,-361.0),(1117.0,-333.0),(1076.7,-303.4),(1037.2,-272.7),(998.2,-241.4),(959.4,-209.8),(920.6,-178.4),(881.3,-147.4),(841.3,-117.5),(800.2,-89.0),(757.6,-62.8),(713.3,-39.7),(666.9,-21.0),(618.6,-8.2),(569.0,-2.9)]],[1,'intakeChain',0,'COAST'],[1,'intakeFlex',0,'COAST']]
    
    red_ring_side = [[0,{'pos':[-1500,920],'angle':90}],[3,(('intake_auto_halt',True),)],[1,'intakeChain',1,12.0],[1,'intakeFlex',1,12.0],[2,[],(('fwd_volt',5.0),('speed_ramp_time',500),('slowdown_distance',500),('finish_margin',150),('K_curvature_speed',2.0),('look_ahead_dist',400),('timeout',3000)),[(-1479.0,925.0),(-1429.1,928.9),(-1379.3,932.2),(-1329.3,934.9),(-1279.4,937.0),(-1229.4,938.6),(-1179.4,939.6),(-1129.4,940.0),(-1079.4,939.9),(-1029.4,939.3),(-979.4,938.2),(-929.5,936.6),(-879.5,934.7),(-829.5,932.6),(-779.6,930.4),(-729.6,928.3),(-679.7,926.7),(-629.7,926.2),(-579.7,927.5),(-529.9,931.9),(-480.9,941.7),(-434.4,959.9),(-394.1,989.2),(-363.3,1028.4),(-341.7,1073.4),(-326.8,1121.1),(-316.4,1170.0),(-309.1,1219.4),(-303.9,1269.2),(-300.2,1319.0),(-297.7,1369.0),(-295.9,1418.9),(-294.8,1468.9),(-294.2,1518.9),(-294.0,1562.0),(-297.0,1611.9),(-305.1,1661.2),(-327.5,1705.4)]],[2,[],(('fwd_volt',5.5),('slowdown_distance',350),('backwards',True),('heading_authority',2.5),('speed_ramp_time',500)),[(-248.0,1660.0),(-250.1,1610.0),(-254.6,1560.3),(-261.3,1510.7),(-270.4,1461.6),(-281.7,1412.9),(-295.2,1364.7),(-310.9,1317.2),(-328.6,1270.5),(-348.3,1224.5),(-370.0,1179.5),(-393.6,1135.4),(-419.0,1092.3),(-446.1,1050.3),(-474.9,1009.5),(-505.4,969.9),(-537.5,931.5),(-571.2,894.5),(-606.3,859.0),(-642.9,824.9),(-680.9,792.4),(-720.3,761.6),(-761.0,732.7),(-803.1,705.6),(-846.4,680.7),(-891.0,658.1),(-936.7,637.9),(-983.6,620.5),(-1031.5,606.0)]],[5,True,'mogo'],[4,250],[3,(('intake_auto_halt',False),)],[1,'intakeChain',-1,9.6],[4,200],[1,'intakeChain',1,12.0],[3,(('intake_color_kill','BLUE'),)],[2,[],(('fwd_volt',4.5),('timeout',4500),('slowdown_distance',1200),('speed_ramp_time',500),('look_ahead_dist',400)),[(-978.0,473.0),(-932.8,494.4),(-889.4,519.2),(-848.1,547.4),(-809.2,578.7),(-773.0,613.2),(-739.6,650.4),(-709.3,690.2),(-682.2,732.2),(-658.4,776.1),(-637.8,821.7),(-620.4,868.5),(-606.2,916.5),(-595.3,965.3),(-587.7,1014.7),(-583.3,1064.5),(-582.5,1114.4),(-585.3,1164.4),(-592.3,1213.8),(-604.2,1262.4),(-622.6,1308.8),(-650.3,1350.3),(-691.6,1377.3),(-740.4,1372.9),(-777.0,1348.0),(-815.2,1315.7),(-853.1,1283.1),(-891.7,1251.4),(-931.5,1221.2),(-973.0,1193.2),(-1016.4,1168.4),(-1061.8,1147.5),(-1109.1,1131.5),(-1158.0,1121.2),(-1207.8,1117.1),(-1257.7,1119.4),(-1307.0,1127.9),(-1354.9,1141.9),(-1401.3,1160.7),(-1445.8,1183.4),(-1488.5,1209.4),(-1529.5,1238.1),(-1568.8,1268.9),(-1606.6,1301.7),(-1643.0,1336.0),(-1678.1,1371.5),(-1712.1,1408.2),(-1745.0,1445.8),(-1777.0,1484.3),(-1808.0,1523.4),(-1838.3,1563.2),(-1867.8,1603.6),(-1896.6,1644.5),(-1924.8,1685.8),(-1952.4,1727.5)]],[4,200],[2,[],(('fwd_volt',5.0),('backwards',True),('speed_ramp_time',400),('slowdown_distance',400)),[(-1981.0,1721.0),(-1950.8,1681.1),(-1919.9,1641.8),(-1888.2,1603.2),(-1855.5,1565.3),(-1822.0,1528.3),(-1787.3,1492.2),(-1751.7,1457.2),(-1714.8,1423.4),(-1676.8,1390.9),(-1637.6,1359.9),(-1597.1,1330.6),(-1555.3,1303.1),(-1512.3,1277.6),(-1468.2,1254.2),(-1422.8,1233.1),(-1376.5,1214.4),(-1329.2,1198.2),(-1281.1,1184.5),(-1232.3,1173.5),(-1183.1,1165.0),(-1133.4,1159.0),(-1083.5,1155.5),(-1033.6,1154.3)]],[2,[('set_pneumatic',{'pneumatic':'intake','state':True},[-1465,497])],(('fwd_volt',4.0),('speed_ramp_time',400),('slowdown_distance',1200),('look_ahead_dist',450),('min_start_voltage',3.0)),[(-978.0,1143.0),(-1027.9,1146.3),(-1077.9,1148.1),(-1127.9,1148.0),(-1177.8,1145.8),(-1227.6,1141.1),(-1276.9,1133.0),(-1325.4,1120.9),(-1372.3,1103.7),(-1416.2,1080.0),(-1455.0,1048.5),(-1485.6,1009.2),(-1505.8,963.6),(-1515.4,914.6),(-1516.1,864.6),(-1509.8,815.1),(-1498.4,766.4),(-1486.0,727.0),(-1475.4,678.2),(-1468.7,628.6),(-1465.1,578.7),(-1463.8,528.8),(-1464.1,478.8),(-1465.5,428.8),(-1467.6,378.8),(-1470.1,328.9),(-1472.8,279.0),(-1475.3,229.0),(-1477.5,179.1),(-1479.4,129.1),(-1480.6,79.1),(-1481.2,29.1),(-1481.0,-20.9),(-1479.9,-70.9)]],[5,False,'intake'],[4,2000],[1,'intakeFlex',-1,12.0],[4,1000],[1,'intakeChain',0,'COAST'],[1,'intakeFlex',0,'COAST']]
    blue_ring_side = [[0,{'pos':[1500,920],'angle':270}],[3,(('intake_auto_halt',True),)],[1,'intakeChain',1,12.0],[1,'intakeFlex',1,12.0],[2,[],(('fwd_volt',5.0),('speed_ramp_time',500),('slowdown_distance',500),('finish_margin',150),('K_curvature_speed',2.0),('look_ahead_dist',400),('timeout',3000)),[(1473.0,914.0),(1423.0,915.6),(1373.0,916.9),(1323.0,917.7),(1273.1,918.2),(1223.1,918.3),(1173.1,918.1),(1123.1,917.4),(1073.1,916.5),(1023.1,915.1),(973.1,913.5),(923.1,911.6),(873.2,909.4),(823.2,907.1),(773.3,904.7),(723.4,902.3),(673.4,900.0),(623.4,898.2),(573.5,897.2),(523.5,897.4),(473.5,899.9),(423.9,906.0),(375.5,918.4),(331.1,940.9),(296.0,976.2),(273.7,1020.7),(261.6,1069.2),(256.0,1118.8),(254.4,1168.8),(255.4,1218.8),(258.2,1268.7),(261.0,1305.0),(259.7,1355.0),(260.1,1405.0),(263.0,1454.9),(269.5,1504.4),(280.0,1553.3),(293.1,1601.6),(306.5,1649.7)]],[2,[],(('fwd_volt',5.5),('slowdown_distance',350),('backwards',True),('heading_authority',2.5),('speed_ramp_time',500)),[(343.0,1674.0),(332.0,1625.3),(327.6,1575.5),(328.4,1525.5),(333.3,1475.8),(341.5,1426.5),(352.6,1377.7),(366.1,1329.6),(381.6,1282.1),(399.1,1235.2),(418.2,1189.0),(438.8,1143.5),(460.9,1098.6),(484.4,1054.5),(509.2,1011.1),(535.3,968.4),(562.7,926.6),(591.3,885.6),(621.2,845.5),(652.4,806.5),(685.0,768.5),(719.0,731.8),(754.4,696.5),(791.3,662.9),(830.0,631.1),(870.4,601.7),(912.6,575.0),(956.9,551.7),(1003.1,532.7)]],[5,True,'mogo'],[4,250],[3,(('intake_auto_halt',False),)],[1,'intakeChain',-1,9.6],[4,200],[1,'intakeChain',1,12.0],[2,[],(('fwd_volt',4.5),('timeout',4500),('slowdown_distance',1200),('speed_ramp_time',500),('look_ahead_dist',400)),[(911.0,422.0),(866.9,445.5),(825.5,473.5),(787.1,505.5),(752.0,541.1),(720.4,579.8),(692.3,621.1),(667.6,664.6),(646.2,709.8),(628.1,756.4),(612.9,804.0),(600.7,852.5),(591.3,901.6),(584.5,951.1),(580.5,1000.9),(579.3,1050.9),(580.8,1100.9),(585.4,1150.7),(593.4,1200.0),(605.5,1248.5),(622.9,1295.3),(648.0,1338.5),(685.3,1371.1),(733.9,1374.7),(777.0,1348.0),(815.2,1315.7),(853.1,1283.2),(891.9,1251.6),(932.1,1221.9),(974.4,1195.2),(1019.0,1172.7),(1066.0,1155.7),(1114.8,1145.3),(1164.7,1142.3),(1214.5,1146.7),(1263.2,1157.9),(1310.2,1174.8),(1355.2,1196.4),(1398.3,1221.7),(1439.5,1250.1),(1479.0,1280.8),(1516.9,1313.4),(1553.4,1347.6),(1588.6,1383.1),(1622.6,1419.6),(1655.7,1457.2),(1687.8,1495.5),(1719.1,1534.5),(1749.6,1574.1),(1779.4,1614.2),(1808.6,1654.8),(1837.2,1695.8),(1865.2,1737.2),(1892.7,1779.0)]],[4,200],[2,[],(('fwd_volt',5.0),('backwards',True),('speed_ramp_time',400),('slowdown_distance',400)),[(1927.0,1766.0),(1897.1,1725.9),(1866.9,1686.1),(1836.2,1646.6),(1805.1,1607.4),(1773.4,1568.8),(1741.0,1530.7),(1707.7,1493.4),(1673.5,1456.9),(1638.2,1421.5),(1601.7,1387.3),(1563.9,1354.6),(1524.8,1323.5),(1484.1,1294.4),(1442.0,1267.5),(1398.4,1243.1),(1353.3,1221.4),(1307.0,1202.6),(1259.6,1186.9),(1211.2,1174.3),(1162.1,1164.8),(1112.5,1158.5),(1062.6,1155.1),(1012.6,1154.5)]],[2,[('set_pneumatic',{'pneumatic':'intake','state':True},[1475,515])],(('fwd_volt',4.0),('speed_ramp_time',400),('slowdown_distance',1200),('look_ahead_dist',450),('min_start_voltage',3.0)),[(978.0,1143.0),(1027.9,1146.3),(1077.9,1148.1),(1127.9,1148.0),(1177.8,1145.8),(1227.6,1141.1),(1276.9,1133.0),(1325.4,1120.9),(1372.3,1103.7),(1416.2,1080.0),(1455.0,1048.5),(1485.6,1009.2),(1505.8,963.6),(1515.4,914.6),(1516.1,864.6),(1509.8,815.1),(1498.4,766.4),(1486.0,727.0),(1475.4,678.2),(1468.7,628.6),(1465.1,578.7),(1463.8,528.8),(1464.1,478.8),(1465.5,428.8),(1467.6,378.8),(1470.1,328.9),(1472.8,279.0),(1475.3,229.0),(1477.5,179.1),(1479.4,129.1),(1480.6,79.1),(1481.2,29.1),(1481.0,-20.9),(1479.9,-70.9)]],[5,False,'intake'],[4,2000],[1,'intakeFlex',-1,12.0],[1,'intakeFlex',0,'COAST'],[4,1000],[1,'intakeChain',0,'COAST']]

    red_goal_rush = [[0,{'pos':[-1282,-1498],'angle':70}],[5,True,'doinker'],[1,'intakeFlex',1,12.0],[2,[],(('fwd_volt',12.0),('speed_ramp_time',600),('slowdown_distance',1000)),[(-1283.0,-1499.0),(-1235.7,-1482.9),(-1188.4,-1466.5),(-1141.2,-1450.0),(-1094.1,-1433.4),(-1047.0,-1416.6),(-999.9,-1399.8),(-952.8,-1383.0),(-905.7,-1366.2),(-858.6,-1349.3),(-811.6,-1332.4),(-764.5,-1315.6),(-717.4,-1298.8),(-670.3,-1282.0),(-623.2,-1265.3),(-576.0,-1248.6),(-528.8,-1232.1),(-481.6,-1215.7),(-434.3,-1199.5)]],[2,[],(('fwd_volt',8.0),('backwards',True),('speed_ramp_time',1000),('slowdown_distance',400)),[(-485.0,-1243.0),(-531.6,-1261.1),(-578.5,-1278.4),(-625.7,-1294.9),(-673.1,-1310.7),(-720.8,-1325.9),(-768.6,-1340.6),(-816.5,-1354.9),(-864.5,-1368.9),(-912.6,-1382.6),(-960.7,-1396.3)]],[5,False,'doinker'],[7,['RIGHT',110,0]],[4,300],[2,[],(('fwd_volt',5.5),('slowdown_distance',350),('backwards',True)),[(-1014.0,-1411.0),(-990.0,-1367.1),(-964.9,-1323.9),(-939.1,-1281.1),(-912.7,-1238.6),(-885.9,-1196.4),(-859.0,-1154.2),(-832.1,-1112.1),(-805.3,-1069.9),(-778.6,-1027.6),(-752.3,-985.1),(-726.5,-942.2),(-701.4,-899.0),(-677.1,-855.4),(-653.8,-811.1),(-631.9,-766.2),(-611.7,-720.4),(-593.7,-673.8),(-578.7,-626.1),(-567.7,-577.3),(-562.1,-527.7)]],[5,True,'mogo'],[4,200],[1,'intakeChain',1,12.0],[4,200],[2,[('set_pneumatic',{'pneumatic':'mogo','state':False},[-788,-903])],(('fwd_volt',5.5),('timeout',2250),('slowdown_distance',800),('speed_ramp_time',700)),[(-540.0,-445.0),(-558.8,-491.3),(-578.4,-537.3),(-598.8,-583.0),(-620.1,-628.2),(-642.4,-672.9),(-665.7,-717.2),(-690.0,-760.9),(-715.4,-804.0),(-741.8,-846.4),(-769.4,-888.1),(-798.2,-929.0),(-828.2,-969.0),(-859.3,-1008.1),(-891.7,-1046.2),(-925.2,-1083.3),(-959.9,-1119.4),(-995.6,-1154.3),(-1032.4,-1188.1),(-1070.2,-1220.9),(-1108.9,-1252.6),(-1148.3,-1283.3),(-1188.4,-1313.2),(-1229.1,-1342.3),(-1270.1,-1370.9),(-1311.4,-1399.0),(-1352.9,-1426.9),(-1394.4,-1454.8),(-1435.7,-1483.0),(-1476.7,-1511.6),(-1517.3,-1540.8),(-1557.2,-1570.9),(-1596.3,-1602.1),(-1634.4,-1634.5),(-1671.3,-1668.2),(-1707.0,-1703.2),(-1741.2,-1739.7),(-1773.8,-1777.6)]],[3,(('intake_auto_halt',True),)],[1,'intakeChain',0,'COAST'],[3,(('intake_auto_halt',False),)],[2,[],(('fwd_volt',5.5),('backwards',True),('slowdown_distance',350),('speed_ramp_time',250),('heading_authority',1.75)),[(-1774.0,-1808.0),(-1762.1,-1759.5),(-1743.9,-1713.0),(-1718.9,-1669.7),(-1687.6,-1630.8),(-1650.8,-1597.0),(-1609.9,-1568.3),(-1565.9,-1544.6),(-1519.8,-1525.2),(-1472.3,-1509.5),(-1424.0,-1496.9),(-1375.0,-1486.8),(-1325.6,-1478.9),(-1276.0,-1472.7),(-1226.2,-1468.1),(-1176.4,-1464.7),(-1126.4,-1462.3),(-1076.4,-1460.9),(-1026.4,-1460.1),(-976.4,-1460.0),(-926.4,-1460.3),(-876.4,-1461.0),(-826.5,-1462.0),(-776.5,-1463.2),(-726.5,-1464.5),(-676.5,-1465.7),(-626.5,-1467.0),(-576.5,-1468.1),(-526.5,-1469.0),(-476.5,-1469.5),(-426.5,-1469.7),(-376.5,-1469.3)]],[5,True,'mogo'],[1,'intakeChain',1,12.0]]
    blue_goal_rush = [[0,{'pos':[1282,-1498],'angle':290}],[5,True,'doinker_left'],[1,'intakeFlex',1,12.0],[2,[],(('fwd_volt',12.0),('speed_ramp_time',600),('slowdown_distance',1000)),[(1283.0,-1499.0),(1235.5,-1483.4),(1187.9,-1468.2),(1140.1,-1453.3),(1092.3,-1438.7),(1044.5,-1424.2),(996.6,-1409.8),(948.7,-1395.5),(900.7,-1381.3),(852.8,-1367.1),(804.8,-1352.9),(756.9,-1338.8),(708.9,-1324.6),(661.0,-1310.3),(613.1,-1296.0),(565.2,-1281.5),(517.4,-1266.9),(469.7,-1252.1),(422.0,-1237.0)]],[2,[],(('fwd_volt',8.0),('backwards',True),('speed_ramp_time',1000),('slowdown_distance',400)),[(485.0,-1243.0),(531.6,-1261.1),(578.5,-1278.4),(625.7,-1294.9),(673.1,-1310.7),(720.8,-1325.9),(768.6,-1340.6),(816.5,-1354.9),(864.5,-1368.9),(912.6,-1382.6),(960.7,-1396.3)]],[5,False,'doinker_left'],[7,['LEFT',120,0]],[4,300],[2,[],(('fwd_volt',5.5),('slowdown_distance',350),('backwards',True)),[(1014.0,-1411.0),(992.9,-1365.7),(971.5,-1320.5),(949.8,-1275.4),(927.9,-1230.5),(905.8,-1185.6),(883.6,-1140.8),(861.3,-1096.1),(838.8,-1051.4),(816.2,-1006.8),(793.5,-962.3),(770.7,-917.8),(747.8,-873.3),(724.9,-828.9),(701.9,-784.5),(678.8,-740.2),(655.6,-695.8),(632.4,-651.5),(609.2,-607.3),(585.9,-563.0)]],[5,True,'mogo'],[4,200],[1,'intakeChain',1,12.0],[4,200],[2,[('set_pneumatic',{'pneumatic':'mogo','state':False},[789,-854])],(('fwd_volt',5.5),('timeout',2250),('slowdown_distance',800),('speed_ramp_time',700)),[(540.0,-445.0),(560.4,-490.6),(581.6,-535.9),(603.6,-580.8),(626.6,-625.2),(650.6,-669.1),(675.6,-712.4),(701.8,-755.0),(729.2,-796.8),(757.9,-837.8),(787.9,-877.7),(819.3,-916.6),(852.1,-954.4),(886.3,-990.9),(921.8,-1026.0),(958.7,-1059.8),(996.6,-1092.4),(1035.6,-1123.7),(1075.4,-1154.0),(1115.8,-1183.4),(1156.6,-1212.2),(1197.7,-1240.8),(1238.8,-1269.2),(1279.8,-1297.9),(1320.4,-1327.0),(1360.6,-1356.8),(1400.2,-1387.3),(1439.1,-1418.7),(1477.3,-1451.0),(1514.5,-1484.3),(1550.9,-1518.6),(1586.4,-1553.9),(1620.9,-1590.0),(1654.5,-1627.0),(1687.2,-1664.9),(1719.0,-1703.4),(1750.0,-1742.7),(1780.1,-1782.6)]],[3,(('intake_auto_halt',True),)],[1,'intakeChain',0,'COAST'],[3,(('intake_auto_halt',False),)],[2,[],(('fwd_volt',5.5),('backwards',True),('slowdown_distance',350),('speed_ramp_time',250),('heading_authority',1.75)),[(1774.0,-1808.0),(1762.1,-1759.5),(1743.9,-1713.0),(1718.9,-1669.7),(1687.6,-1630.8),(1650.8,-1597.0),(1609.9,-1568.3),(1565.9,-1544.6),(1519.8,-1525.2),(1472.3,-1509.5),(1424.0,-1496.9),(1375.0,-1486.8),(1325.6,-1478.9),(1276.0,-1472.7),(1226.2,-1468.1),(1176.4,-1464.7),(1126.4,-1462.3),(1076.4,-1460.9),(1026.4,-1460.1),(976.4,-1460.0),(926.4,-1460.3),(876.4,-1461.0),(826.5,-1462.0),(776.5,-1463.2),(726.5,-1464.5),(676.5,-1465.7),(626.5,-1467.0),(576.5,-1468.1),(526.5,-1469.0),(476.5,-1469.5),(426.5,-1469.7),(376.5,-1469.3)]],[5,True,'mogo'],[1,'intakeChain',1,12.0]]

    blue_ring_side_ladder = [[0,{'pos':[1500,920],'angle':270}],[3,(('intake_auto_halt',True),)],[1,'intakeChain',1,12.0],[1,'intakeFlex',1,12.0],[2,[],(('fwd_volt',5.0),('speed_ramp_time',500),('slowdown_distance',500),('finish_margin',150),('K_curvature_speed',2.0),('look_ahead_dist',400),('timeout',3000)),[(1473.0,914.0),(1423.0,915.6),(1373.0,916.9),(1323.0,917.7),(1273.1,918.2),(1223.1,918.3),(1173.1,918.1),(1123.1,917.4),(1073.1,916.5),(1023.1,915.1),(973.1,913.5),(923.1,911.6),(873.2,909.4),(823.2,907.1),(773.3,904.7),(723.4,902.3),(673.4,900.0),(623.4,898.2),(573.5,897.2),(523.5,897.4),(473.5,899.9),(423.9,906.0),(375.5,918.4),(331.1,940.9),(296.0,976.2),(273.7,1020.7),(261.6,1069.2),(256.0,1118.8),(254.4,1168.8),(255.4,1218.8),(258.2,1268.7),(261.0,1305.0),(259.7,1355.0),(260.1,1405.0),(263.0,1454.9),(269.5,1504.4),(280.0,1553.3),(293.1,1601.6),(306.5,1649.7)]],[2,[],(('fwd_volt',5.5),('slowdown_distance',350),('backwards',True),('heading_authority',2.5),('speed_ramp_time',500)),[(343.0,1674.0),(332.0,1625.3),(327.6,1575.5),(328.4,1525.5),(333.3,1475.8),(341.5,1426.5),(352.6,1377.7),(366.1,1329.6),(381.6,1282.1),(399.1,1235.2),(418.2,1189.0),(438.8,1143.5),(460.9,1098.6),(484.4,1054.5),(509.2,1011.1),(535.3,968.4),(562.7,926.6),(591.3,885.6),(621.2,845.5),(652.4,806.5),(685.0,768.5),(719.0,731.8),(754.4,696.5),(791.3,662.9),(830.0,631.1),(870.4,601.7),(912.6,575.0),(956.9,551.7),(1003.1,532.7)]],[5,True,'mogo'],[4,250],[3,(('intake_auto_halt',False),)],[1,'intakeChain',-1,9.6],[4,200],[1,'intakeChain',1,12.0],[2,[],(('fwd_volt',4.5),('timeout',4500),('slowdown_distance',1200),('speed_ramp_time',500),('look_ahead_dist',400)),[(911.0,422.0),(866.9,445.5),(825.5,473.5),(787.1,505.5),(752.0,541.1),(720.4,579.8),(692.3,621.1),(667.6,664.6),(646.2,709.8),(628.1,756.4),(612.9,804.0),(600.7,852.5),(591.3,901.6),(584.5,951.1),(580.5,1000.9),(579.3,1050.9),(580.8,1100.9),(585.4,1150.7),(593.4,1200.0),(605.5,1248.5),(622.9,1295.3),(648.0,1338.5),(685.3,1371.1),(733.9,1374.7),(777.0,1348.0),(815.2,1315.7),(853.1,1283.2),(891.9,1251.6),(932.1,1221.9),(974.4,1195.2),(1019.0,1172.7),(1066.0,1155.7),(1114.8,1145.3),(1164.7,1142.3),(1214.5,1146.7),(1263.2,1157.9),(1310.2,1174.8),(1355.2,1196.4),(1398.3,1221.7),(1439.5,1250.1),(1479.0,1280.8),(1516.9,1313.4),(1553.4,1347.6),(1588.6,1383.1),(1622.6,1419.6),(1655.7,1457.2),(1687.8,1495.5),(1719.1,1534.5),(1749.6,1574.1),(1779.4,1614.2),(1808.6,1654.8),(1837.2,1695.8),(1865.2,1737.2),(1892.7,1779.0)]],[4,200],[2,[],(('fwd_volt',5.0),('backwards',True),('speed_ramp_time',400),('slowdown_distance',400)),[(1927.0,1766.0),(1897.1,1725.9),(1866.9,1686.1),(1836.2,1646.6),(1805.1,1607.4),(1773.4,1568.8),(1741.0,1530.7),(1707.7,1493.4),(1673.5,1456.9),(1638.2,1421.5),(1601.7,1387.3),(1563.9,1354.6),(1524.8,1323.5),(1484.1,1294.4),(1442.0,1267.5),(1398.4,1243.1),(1353.3,1221.4),(1307.0,1202.6),(1259.6,1186.9),(1211.2,1174.3),(1162.1,1164.8),(1112.5,1158.5),(1062.6,1155.1),(1012.6,1154.5)]],[2,[],(('fwd_volt',4.0),('speed_ramp_time',400),('slowdown_distance',1200),('look_ahead_dist',450),('min_start_voltage',3.0)),[(978.0,1143.0),(1027.9,1146.3),(1077.9,1148.1),(1127.9,1148.0),(1177.8,1145.8),(1227.6,1141.1),(1276.9,1133.0),(1325.4,1120.9),(1372.3,1103.7),(1416.2,1080.0),(1455.0,1048.5),(1485.6,1009.2),(1505.8,963.6),(1515.4,914.6),(1516.1,864.6),(1509.8,815.1),(1498.4,766.4),(1486.0,727.0),(1473.5,678.6),(1460.6,630.3),(1446.2,582.4),(1429.9,535.1),(1411.3,488.7),(1390.3,443.4),(1366.6,399.3),(1340.2,356.9),(1311.1,316.3),(1279.3,277.7),(1244.9,241.4),(1208.0,207.7),(1168.9,176.5),(1127.9,148.0),(1085.0,122.2),(1040.7,99.0),(995.1,78.5),(948.5,60.4),(901.1,44.8),(852.9,31.3),(804.2,19.9),(755.1,10.5),(705.7,2.9),(656.0,-3.0),(606.2,-7.3),(556.3,-10.2),(506.3,-11.7)]]]

    none = []
    drive_forward = []

class Autonomous():
    def __init__(self, parent: Robot) -> None:
        """
        Setup autonomous. Runs at start of program!
        """
        self.robot = parent

        self.positioning_algorithm = DeltaPositioning(sensor.groundEncoder, sensor.imu)
        self.path_controller = PurePursuit

        self.intake_jam_windup = 0
        self.last_intake_turn = 0

        class c_drivetrain: # custom_drivetrain
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
        Calls at start of auton. Sets up the coordinates n stuff for the loaded path.
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
                raise ModuleNotFoundError("Invalid auton {}".format(sequence))

        elif type(sequence) == list:
            self.sequence = sequence
        
        self.initial_pose: dict = self.sequence[0][1]
    
    def autonomous_cleanup(self) -> None:
        """
        Runs at end of autonomous. Do not modify.
        """
        # actually doesn't do anything important lol
        self.end_time = brain.timer.system()
        elapsed_time = self.end_time - self.start_time
        log("Auton complete. Time taken: {}".format(elapsed_time), LogLevel.INFO)
    
    def position_thread(self):
        """Handle robot pose updates in the background so that is happens regardless of what we
        are doing in the foreground."""
        while True:
            self.robot.heading = sensor.imu.heading()
            dx, dy = self.positioning_algorithm.update()

            self.robot.pos[0] += dx
            self.robot.pos[1] += dy

            sleep(20, TimeUnits.MSEC)
    
    def run(self) -> None:
        """
        Runs autonomous. Do not modify.
        """
        log("Running autonomous")
        self.autonomous_setup()
        motor.ladyBrown.stop()

        self.read_sequence()
        
        self.autonomous_cleanup()

    def test(self) -> None:
        """
        Run a test version of autonomous. This is NOT run in competition!
        """
        log("Running autonomous TEST", LogLevel.WARNING)
        # self.autonomous_setup()
        self.run()
        # # log("Finished test at {}".format(robot.pos))
        # # flags.color_setting = ColorSort.EJECT_RED
        # AutonomousFlags.intake_flex_extake_on_ring = 2
        # motor.intakeFlex.spin(FORWARD, 100, PERCENT)
        # motor.intakeChain.spin(FORWARD, 100, PERCENT)
        # while True:
        #     # self.robot.color_sort_controller.sense()
        #     # print(sensor.intakeColor.hue())
        #     sleep(15, MSEC)

    def read_sequence(self) -> None:
        """
        Runs commands within a sequence. This is setup to handle format of our GUI program which spits out 
        autons in a python list that gets pasted right into this source code. The list is 2D where each
        item contains an ID at index 0 then subsequent places contain data for that action.
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
                        if flag_name == "intake_color_kill":
                            if value == "RED": value = Color.RED
                            elif value == "BLUE": value = Color.BLUE
                            else: value = None
                        if type(getattr(AutonomousFlags, flag_name)) == type(value) or flag_name == "intake_color_kill":
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
                    self.robot.LB_PID.enabled = False
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
                elif function[1] == "home_lady_brown":
                    self.robot.LB_PID.home()
                else:
                    log("[{}] No custom function found for {}".format(i, function[1]))
            elif ID == 7:
                # Turn command
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
            # handle background things
            self.listeners()
            # color sort
            self.robot.color_sort_controller.sense()
            self.robot.last_intake_color = sensor.intakeColor.hue()

            sleep(35, MSEC)

    def listeners(self) -> None:
        """
        Collection of background things we might want to run during auton. Includes things like
        automatically stopping the intake when it gets a ring, reversing the intake on a certain
        color of ring, and trying to unjam the intake. All of these default to off/not doing anything.
        """
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
        
        if auto_flags.intake_flex_extake_on_ring != 0:
            if auto_flags.intake_flex_extake_on_ring == 1: # extake when see blue
                if sensor.intakeColor.hue() > ColorSortController.BLUE and self.robot.last_intake_color < ColorSortController.BLUE:
                    motor.intakeFlex.spin(REVERSE, 100, VelocityUnits.PERCENT)
                elif sensor.intakeColor.hue() < ColorSortController.BLUE and self.robot.last_intake_color > ColorSortController.BLUE:
                    brain.timer.event(motor.intakeFlex.spin, 200, (FORWARD, 100, VelocityUnits.PERCENT))
                
            elif auto_flags.intake_flex_extake_on_ring == 2: # extake when see red
                # print(sensor.intakeColor.hue())
                if sensor.intakeColor.hue() < ColorSortController.RED and self.robot.last_intake_color > ColorSortController.RED:
                    motor.intakeFlex.spin(REVERSE, 100, VelocityUnits.PERCENT)
                elif sensor.intakeColor.hue() > ColorSortController.RED and self.robot.last_intake_color < ColorSortController.RED:
                    brain.timer.event(motor.intakeFlex.spin, 200, (FORWARD, 100, VelocityUnits.PERCENT))
    
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
            # curvature things to refine that behavior
            "a_curvature_speed_exp": 0.1,
            # msec of time until we are at max speed
            "speed_ramp_time": 1,
            # minimum speed we start at if speed ramp time is non negligible
            "min_start_voltage": 4,
            # mm distance from end of path to begin slowing down, linearly
            "slowdown_distance": 0,
            "min_slow_voltage": 4,
            "checkpoints": [],
            # constant forwards speed to drive the path
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

            # get target point from pure pursuit
            target_point, curvature = path_handler.goal_search(robot.pos)
            # get heading to target from our current pos
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

            # check position-based events
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
                            elif event[0] == "motor_spin":
                                if hasattr(motor, event[1]['motor']):
                                    if event[1]['direction'] == 1: direction = FORWARD
                                    else: direction = REVERSE
                                    getattr(motor, event[1]['motor']).spin(direction, event[1]['velocity'], VelocityUnits.PERCENT)
                                    log("[Event] Spin motor {} {} at %{}".format(event[1]['motor'], direction, event[1]['velocity']))
                                else:
                                    log("[Event] No {} motor found".format(event[1]['motor']))
                                
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
    def toggle_PTO():
        log("Toggled PTO pneumatics")
        log("PTOs not on robot!", LogLevel.WARNING)
        # pneumatic.PTO.set(not pneumatic.PTO.value())
    
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
        pneumatic.doinker.set(False)
        pneumatic.doinker.set(False)

        # bind controller functions
        control.DOINKER.pressed(ControllerFunctions.switch_doinker)
        control.MOGO_GRABBER_TOGGLE.pressed(ControllerFunctions.switch_mogo)
        control.LB_MACRO_HOME.pressed(self.robot.LB_PID.home)
        control.LB_MACRO_DESCORE.pressed(self.robot.LB_PID.descore)

        # just in case
        flags.color_setting = ColorSort.NONE

    def run(self) -> None:
        """
        Runs driver control.
        """
        # probably too much unnecessary nesting here
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
        if foxglove_log:
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
                self.give_up_threshold = 100

            # color flag is on, now ring is up intake
            if ((sensor.intakeDistance.object_distance() < 70) and (self.eject_next_ring)):
                log("Ejecting ring!")

                self.allow_intake_input = False
                self.eject_next_ring = False
                self.ejecting = True
                command = motor.intakeChain.command(VelocityUnits.PERCENT)

                sleep(160, TimeUnits.MSEC)
                motor.intakeChain.stop(HOLD)
                brain.timer.event(self.stop_eject, 200, (command,))

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

def pull_data(data: dict, robot: Robot):
    """
    Assigns data variables to different objects.
    """
    pass

# blank function so that we can define a competition variable without binding anything to it
def null_function():
    pass

def odom_packets(robot: Robot):
    # send odometry data if connected to foxglove
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

        # when connected to the field, do everything normally (no tests)
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

            brain.screen.clear_screen(Color.CYAN)

            robot.driver_controller.run()

    else:
        log("Robot object not created. (SD Load Error)", LogLevel.FATAL)
        raise ImportError("SD Card not inserted")

main()
