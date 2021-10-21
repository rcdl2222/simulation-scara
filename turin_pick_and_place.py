from numpy.lib.type_check import isreal
import openravepy as orpy
from openravepy import PlanningError
import numpy as np
import time
import random
from copy import deepcopy
from functions.ik_utils import find_lens_solution, find_general_solution, find_general_solutions
from functions.object_utils import create_20_lenses
from functions.robot_utils import execute_3_trajs, run_lens_inspection, path_planning, execute_path

ROBOT_HOME = [0,0,0,0]
ROBOT_WAIT = [-0.5,-0.5,0.5,0]
INSPECTION_HOME = [0.207, -0.455, -0.184]
CAMERA_TRANSFORM = [[ 0.90549821,  0.18271801, -0.38299755,  0.7626754 ],
                    [ 0.4240237 , -0.3542006 ,  0.83351415, -1.11235273],
                    [ 0.01664009, -0.91714561, -0.39820478,  0.4262858 ],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]
LENS_STARTING_COORD = [0.3925, -0.069 , -0.114]
FAIL_LENS_STARTING_COORD = [0.3925, 0.1805, -0.108]

if __name__ == "__main__":

    '''Generate Enviroment and Robot'''
    env = orpy.Environment()
    env.SetViewer('qtosg')
    viewer = env.GetViewer()
    viewer.SetCamera(CAMERA_TRANSFORM)
    env.Load('worlds/turin-world.xml')
    robot = env.GetRobot('robot')
    manip = robot.SetActiveManipulator('gripper')
    manip.SetLocalToolDirection([1,0,0])
    robot.SetActiveDOFs(manip.GetArmIndices())

    gripper = env.GetRobot('inspection_gripper')
    gripper_manip = gripper.SetActiveManipulator('inspection_gripper')
    gripper_manip.SetLocalToolDirection([0,1,0])

    ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(
        robot, iktype=orpy.IkParameterization.Type.TranslationXAxisAngleZNorm4D
        )
    if not ikmodel.load():
        print("Loading IK model")
        ikmodel.autogenerate()

    lmodel = orpy.databases.linkstatistics.LinkStatisticsModel(robot)
    if not lmodel.load():
        lmodel.autogenerate()
    lmodel.setRobotResolutions(0.01) # set resolution given smallest object is 0.01m
    lmodel.setRobotWeights() # set the weights for planning

    robot.SetActiveDOFValues(ROBOT_HOME)
    lenses = create_20_lenses(LENS_STARTING_COORD, env)
    fail_lens_count = 0
    current_fail_lens_coord = deepcopy(FAIL_LENS_STARTING_COORD)
    random.seed(42)

    raw_input("Press Enter to execute pipeline.")
    time.sleep(3)
    for lens_index in range(20):
        ik_list = []
        ik_list.append(find_lens_solution(lenses, manip, lens_index))
        ik_list.append(find_general_solution(INSPECTION_HOME, manip))
        ik_list.append(ROBOT_WAIT)
        execute_3_trajs(robot, ik_list, lenses[lens_index])
        run_lens_inspection(gripper, lenses[lens_index])
        inspection_result = random.choice([True, False])
        if inspection_result:
            return_ik_list = []
            return_ik_list.append(ik_list[1])
            return_ik_list.append(ik_list[0])
            return_ik_list.append(ROBOT_HOME)
            execute_3_trajs(robot, return_ik_list, lenses[lens_index])
        else:
            return_ik_list = []
            return_ik_list.append(ik_list[1])
            return_ik_list.append(find_general_solution(current_fail_lens_coord, manip))
            return_ik_list.append(ROBOT_HOME)
            execute_3_trajs(robot, return_ik_list, lenses[lens_index])
            # Move next failed lens coordinate
            fail_lens_count += 1
            current_fail_lens_coord[1] -= 0.05
            if fail_lens_count % 5 == 0:
                current_fail_lens_coord[1] = FAIL_LENS_STARTING_COORD[1]
                current_fail_lens_coord[0] -= 0.035


        

    raw_input("Press Enter to finish")
    
    # Uncomment below to enter python shell
    # import IPython
    # IPython.embed()