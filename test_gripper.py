from numpy.lib.type_check import isreal
import openravepy as orpy
from openravepy import PlanningError
import numpy as np
import time
import glob

if __name__ == "__main__":

    '''Generate Enviroment and Robot'''
    env = orpy.Environment()
    env.SetViewer('qtosg')
    viewer = env.GetViewer()
    env.Load('worlds/test-gripper.xml')
    robot = env.GetRobot('inspection_gripper')
    # manip = robot.SetActiveManipulator('inspection_gripper')
    # manip.SetLocalToolDirection([0,0,1])
    manipprob = orpy.interfaces.BaseManipulation(robot)
    raw_input("Enter")
    manipprob.MoveManipulator(goal=[np.pi], jitter = 0.04)
    robot.WaitForController(0)
    time.sleep(1)
    manipprob.MoveManipulator(goal=[0], jitter = 0.04)
    #robot.SetActiveDOFValues([np.pi])
    raw_input("Enter")

    # import IPython
    # IPython.embed()
