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
