from ev3dev import ev3
from enum import Enum
pi = 3.141

wheel_diameter = 5.6
robot_width = 11.6

stage_width = 90

wheel_circum = pi * wheel_diameter
robot_turn_circle = pi * robot_width


class Color(Enum):
    RED = 'red'
    GREEN = 'green'
    BLUE = 'blue'
    GREY = 'grey'
    BLACK = 'black'
    WHITE = 'white'


class Direction(Enum):
        RIGHT = 'right'
        LEFT = 'left'

        def __invert__(self):
            if self.value == Direction.LEFT.value:
                return Direction.RIGHT
            return Direction.LEFT

        def get_arm_sign(self):
            if self.value == Direction.RIGHT.value:
                return 1
            return -1


def get_dir(num):
    if num < 0:
        return Direction.RIGHT
    return Direction.LEFT


def dist(list1, list2):
    return sum([(vi - vj) ** 2.0 for vi, vj in zip(list1, list2)])


def get_closest_color(color_measure):
    colors = {Color.RED: [251, 31, 25],
              Color.GREEN: [81, 209, 70],
              Color.BLUE: [53, 61, 196],
              Color.GREY: [228, 230, 235],
              Color.BLACK: [20, 20, 20],
              Color.WHITE: [255, 255, 255]}
    distances = [(dist(color, color_measure), name) for name, color in colors.items()]
    color = min(distances)[1]
    return color


class RobotHandler:
    def __init__(self, m1='outA', m2='outD', ar='outB'):
        self.m1 = ev3.LargeMotor(m1)
        self.m2 = ev3.LargeMotor(m2)
        self.ar = ev3.MediumMotor(ar)
        self.cl = ev3.ColorSensor()
        self.cl.mode = 'RGB-RAW'
        self.gy = ev3.GyroSensor()
        self.gy.mode = 'GYRO-CAL'
        self.gy.mode = 'GYRO-ANG'
        self.us = ev3.UltrasonicSensor()
        self.us.mode = 'US-DIST-CM'

    def get_orientation(self):
        return self.gy.value()

    def stop_running(self):
        self.m1.stop(stop_action="brake")
        self.m2.stop(stop_action="brake")

    def return_colors(self):
        scale = [345.0, 324.0, 214.0]
        return [255.0 / s * self.cl.value(i) for i, s in enumerate(scale)]

    def search_for_diff_to_line(self, target_color, target_angle=0):
        angle_of_sensor = target_angle
        self.ar.run_to_abs_pos(position_sp=angle_of_sensor, speed_sp=300)
        self.ar.wait_until('running', timeout=angle_of_sensor/300.0*1000)
        if get_closest_color(self.return_colors()) == target_color:
            self.ar.stop(stop_action="hold")
            return target_angle - self.ar.position

        for i in range(max(target_angle-20, -90), min(target_angle+21, 90), 4):
            angle_of_sensor = i
            self.ar.run_to_abs_pos(position_sp=angle_of_sensor, speed_sp=600)
            self.ar.wait_until('running', timeout=10)  # TODO: should dthis be wait_while
            if get_closest_color(self.return_colors()) == target_color:
                self.ar.stop(stop_action="hold")
                return target_angle - self.ar.position

        return None

    def check_outside_line(self):
        pass

    # find_color_to_turn
    # this checks whether the wanted color is to the left or to the right of the desired color.
    # this can be used dto straddle a line if angle is 0 or to drive to the side if angle is 85
    # if the wanted color is not found, return None
    # if the wanted color is found, return the direction that the robot needs to turn to
    #
    # examples:
    #          rob.find_color_to_turn(Color.BLUE, 0)
    #           --- world ---  | W W W W B W W |0| W W W W W W W |    --->  (Direction.Left, 20)
    #           --- world ---  | W W W W W W W |0| B W W W W W W |    --->  (Direction.Right, -5)
    #          rob.find_color_to_turn(Color.GREEN, 60)
    #           --- world ---  | W W W W W W W |-60| G W W W W W W |    --->  (Direction.Right, -5)
    #           --- world ---  | W W G W W W W |-60| W W W W W W W |    --->  (Direction.Right, 20)

    def find_color_to_turn(self, color_to_find=Color.BLACK, target_angle=0):  # -> returns (direction, angle_turn_to_)
        angle_diff = self.search_for_diff_to_line(color_to_find, target_angle)
        if angle_diff is None:
            return None, None
        return ~get_dir(angle_diff), angle_diff

    # follow_line_at_angle
    # assume the robot starts on a line.
    # drive along line.
    # keep scanning the line, when you find you need to turn, stop the motors, then turn, then restart the motors.

    def follow_line_at_angle(self, color=Color.BLACK, angle=0, exit_col=None, look_for_exit=None):
        go_speed = 100
        self.m1.run_forever(speed_sp=go_speed)
        self.m2.run_forever(speed_sp=go_speed)
        self.ar.run_to_abs_pos(position_sp=0, speed_sp=200)
        while self.m1.is_running:
            if get_closest_color(self.return_colors()) != color:
                print('follow line at angle - no orig color', color)
                direc, turn = self.find_color_to_turn(color, angle)

                if direc is None:
                    print('found none - in follow line at angle ', exit_col, look_for_exit)
                    self.stop_running()
                    return None

                print(direc, turn, 'in follow - found color')
                if turn != 0:
                    self.drive(0, abs(turn), direc, 200, True)
                self.ar.run_to_abs_pos(position_sp=0, speed_sp=200)
                self.m1.run_forever(speed_sp=go_speed)
                self.m2.run_forever(speed_sp=go_speed)

            elif exit_col is not None and look_for_exit is not None:
                if self.scan_for_other_color(main_color=color,  look_for=exit_col, direction=look_for_exit):
                    self.stop_running()
                    self.drive(forward=3, turn_deg=15, turn_dir=look_for_exit)
                    return exit_col

    def drive(self, forward, turn_deg=0, turn_dir=None, speed=200, wwr=True):
        self.stop_running()

        wheel_travel_distance = robot_turn_circle * turn_deg / 360.0

        rotation_deg = wheel_travel_distance / wheel_circum * 360.0

        move_rot = float(forward) / wheel_circum * 360.0

        if turn_dir == Direction.RIGHT:
            r_rot = rotation_deg + move_rot
            l_rot = -rotation_deg + move_rot

        elif turn_dir == Direction.LEFT:
            r_rot = -rotation_deg + move_rot
            l_rot = rotation_deg + move_rot

        else:
            r_rot = move_rot
            l_rot = move_rot

        if abs(l_rot) > abs(r_rot):
            l_speed = speed * l_rot / r_rot
            r_speed = speed
        else:
            l_speed = speed
            r_speed = speed * r_rot / l_rot

        try:
            if r_rot != 0:
                self.m1.run_to_rel_pos(position_sp=r_rot, speed_sp=r_speed)
            if l_rot != 0:
                self.m2.run_to_rel_pos(position_sp=l_rot, speed_sp=l_speed)
        except:
            print('exeption')
            pass

        comp = 50.0
        if wwr:
            self.m1.wait_while('running', timeout=max(1.0, r_rot / r_speed * 1000.0 - comp))
            self.m2.wait_while('running', timeout=max(1.0, l_rot / l_speed * 1000.0 - comp))

    def d_to_wall(self):
        return self.us.value() / 10.0

    # look_for_other_color -
    # while following color look for other color to change behaviour .
    # e.g. when on blue look for black green or red
    # when on green or red, look for blue.
    #
    def scan_for_other_color(self, main_color=Color.GREEN, look_for=Color.BLUE, direction=Direction.RIGHT):
        start_pos = self.ar.position
        scan_angle_sign = - direction.get_arm_sign()
        scan_pos = start_pos + scan_angle_sign *  20
        self.ar.run_to_abs_pos(position_sp=scan_pos, speed_sp=400)
        self.ar.wait_while('running', timeout=50)

        if get_closest_color(self.return_colors()) == look_for:
            return True
        self.ar.run_to_abs_pos(position_sp=start_pos, speed_sp=400)
        return False

    # choose_circle_direction
    # selects the direction to follow the circle
    # if facing north +- 30  - > face north exactly, then take distance to left wall and decide
    # if facing eastish  turn left before nav circle
    # westish - turn  right before circle
    def choose_circle_direction(self):
        orien = self.get_orientation()
        to_wall = None
        if -30 < orien < 30:
            to_wall = self.straiten(orien)
        go_dir = get_dir(orien)

        if to_wall is not None:
            if to_wall < stage_width/3:
                go_dir = Direction.RIGHT
            else:
                go_dir = Direction.LEFT
        return go_dir

    def straiten(self, orientation):
        dire = get_dir(orientation)
        self.drive(0, abs(orientation), dire)
        dist_wall = self.d_to_wall()
        self.drive(0, abs(orientation), ~dire)
        return dist_wall

    # takes a color
    # if redd charge, retreat then follow the circle.
    # if green follow circle
    def circle_process(self, color):
        if color == Color.RED:
            print('found red - charge ')
            self.ar.run_to_abs_pos(position_sp=80, speed_sp=400)

            self.ar.wait_while('running', timeout=500)
            self.drive(20, speed=500)
            self.ar.run_to_abs_pos(position_sp=-80, speed_sp=400)
            self.ar.wait_while('running', timeout=500)
            self.ar.run_to_abs_pos(position_sp=0, speed_sp=200)

            self.drive(-20)
            print('have retreated')
            self.ar.wait_while('running', timeout=500)

        where_to = self.choose_circle_direction()
        self.drive(-4)
        self.drive(3, 20, where_to)
        angle = 30 * where_to.get_arm_sign()
        if self.follow_line_at_angle(color, angle, exit_col=Color.BLUE, look_for_exit=where_to) is None:
            print('I lost the circle ' , color)
            return None
        self.drive(.5, 20, where_to)
        return color

    def look_for_circle_color(self):  # returns green red or black or none
        for r in [20, 30, 40]:
            self.ar.run_to_abs_pos(position_sp=-r, speed_sp=200)
            self.ar.wait_while('running', timeout=r*5)
            result = []
            for angle in range(-r, r + 1, 4):
                self.ar.run_to_abs_pos(position_sp=angle, speed_sp=200)
                self.ar.wait_while('running', timeout=10)   # Wait until ?
                result.append(get_closest_color(self.return_colors()))
                for c in [Color.BLACK, Color.GREEN, Color.RED]:
                    if c in result:
                        return c
        return None

    # navigate chooses the actions.
    # follow blue, until lost
    # look for change
    # process_circle
        # look for _ blue
    # loop until black

    def navigate(self):
        self.follow_line_at_angle()
        pass
