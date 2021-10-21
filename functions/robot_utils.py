"""Utility functions for robots in OpenRAVE"""
import numpy as np
import openravepy as orpy
import time
from openravepy import PlanningError

def path_planning(robot, ik, env):
    """
    Given an IK solution and a robot, 
    plan a path with the robot's current joint values
    
    Returns:
        Trajectory for the robot to follow.
    """
    planner = orpy.RaveCreatePlanner(env, 'birrt') # Using bidirectional RRT
    params = orpy.Planner.PlannerParameters()
    params.SetRobotActiveJoints(robot)
    params.SetGoalConfig(ik)
    params.SetPostProcessing('ParabolicSmoother', '<_nmaxiterations>40</_nmaxiterations>')
    planner.InitPlan(robot, params)
    # Plan a trajectory
    traj = orpy.RaveCreateTrajectory(env, '')
    planner.PlanPath(traj)
    return traj

def execute_path(robot, traj):
    """Executes a trajectory for a given robot."""
    controller = robot.GetController()
    controller.SetPath(traj)

def execute_3_trajs(robot, trajs, lens):
    '''
    Executes 3 trajectories in order; 2nd trajectory should be inspection
    '''
    manipprob = orpy.interfaces.BaseManipulation(robot)
    manipprob.MoveManipulator(goal=trajs[0], jitter = 0.04)
    robot.WaitForController(0)
    robot.Grab(lens)
    manipprob.MoveManipulator(goal=trajs[1], jitter = 0.04)
    robot.WaitForController(0)
    robot.Release(lens)
    manipprob.MoveManipulator(goal=trajs[2], jitter = 0.04)
    robot.WaitForController(0)

def rotate_lens(gripper, degree):
    """Rotate a gripper in degrees"""
    rad = degree / 180 * np.pi
    manipprob = orpy.interfaces.BaseManipulation(gripper)
    manipprob.MoveManipulator(goal=[rad], jitter = 0.04)
    gripper.WaitForController(0)

def run_lens_inspection(gripper, lens):
    """
    Simulate an inspection routine
    Flips inspection gripper 180deg, then back to 0deg
    """
    gripper.Grab(lens)
    rotate_lens(gripper, 180)
    time.sleep(0.5)
    rotate_lens(gripper, 0)
    gripper.Release(lens)