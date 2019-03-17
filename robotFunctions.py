from ev3dev import ev3
from enum import Enum


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

        def get_arm(self):
            if self.value == Direction.RIGHT.value:
                return -90
            return 90

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
        self.ar.wait_until('running', timeout=10)
        if get_closest_color(self.return_colors()) == target_color:
            self.ar.stop(stop_action="hold")
            return target_angle - self.ar.position

        for i in range(max(target_angle-20, -90), min(target_angle+21, 90), 4):
            angle_of_sensor = i
            self.ar.run_to_abs_pos(position_sp=angle_of_sensor, speed_sp=300)
            self.ar.wait_until('running', timeout=10)
            if get_closest_color(self.return_colors()) == target_color:
                self.ar.stop(stop_action="hold")
                return target_angle - self.ar.position

        return None

    def check_outside_line(self):
        pass

    def distance_to_wall(self):
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

    def find_color_to_turn(self, color_to_find=Color.BLACK, target_angle=0):  #-> returns (direction, angle_turn_to_)
        angle_diff = self.search_for_diff_to_line(color_to_find, target_angle)
        if angle_diff is None:
            return None
        if angle_diff < 0:
            return Direction.RIGHT, angle_diff
        else:
            return Direction.LEFT, angle_diff

    # follow_line_at_angle
    # assume the robot starts on a line.
    # drive along line.
    # keep scanning the line, when you find you need to turn, stop the motors, then turn, then restart the motors.

    def follow_line_at_angle(self, color=Color.BLACK, angle=0):
        pass
