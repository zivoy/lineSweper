import robotFunctions
from robotFunctions import Color, Direction

robot = robotFunctions.RobotHandler('outA', 'outD', 'outB')

while True:
    robot.follow_line_at_angle(Color.BLUE, angle=0)
    c = robot.look_for_circle_color()
    if c is  None:
        print('sorry, oops no expected color')
        break
    if c == Color.BLACK:
        print('tada')
        break
    robot.circle_process(c)

robot.ar.stop(stop_action='coast')