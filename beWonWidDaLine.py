import robotFunctions
from robotFunctions import Color

robot = robotFunctions.RobotHandler('outA', 'outD', 'outB')

robot.m1.run_forever(speed_sp=200)
robot.m2.run_forever(speed_sp=200)
while robot.m1.is_running:
    if robotFunctions.get_closest_color(robot.return_colors()) != Color.BLUE:
        robot.stop_running()
        break
