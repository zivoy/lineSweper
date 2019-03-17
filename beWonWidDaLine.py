import robotFunctions
from robotFunctions import Color, Direction
from ev3dev.ev3 import Sound

robot = robotFunctions.RobotHandler('outA', 'outD', 'outB')

robot.ar.position = 0

while True:
    print('while TRUE')
    robot.follow_line_at_angle(Color.BLUE, angle=0)
    print('exit_ follow____ ++ _ ++ _ + _+ _ ++_ _')
    c = robot.look_for_circle_color()
    if c is  None:
        print('sorry, oops no expected color')
        break
    if c == Color.BLACK:
        robot.drive(15)
        print('tada')
        Sound.beep()
        break
    robot.circle_process(c)

robot.ar.stop(stop_action='coast')
