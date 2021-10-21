"""Inverse kinematic functions"""
import numpy as np
import openravepy as orpy
from .object_utils import rotX, rotY, rotZ

alpha = 5. * np.pi / 180.

def find_lens_solution(lenses, manip, index):
    '''
    Given an array of lenses, find the solution for a certain
    lens at an index
    Args: 
        lenses: list of lenses
        index: index of lens

    Returns: 
        IKSolution of robot
    '''
    lens = lenses[index]
    Tlens = lens.GetTransform()
    Tlens[:3, 3] = lens.ComputeAABB().pos()
    a = manip.FindIKSolution(Tlens, orpy.IkFilterOptions.CheckEnvCollisions)
    if a is not None: 
        return a
    for i in range(80):
        Tlens = rotZ(Tlens, alpha)
        a = manip.FindIKSolution(Tlens, orpy.IkFilterOptions.CheckEnvCollisions)
        if a is not None:
            return a

def find_general_solution(coord, manip):
    """Gets the IK solution given some coordinates"""
    T = np.eye(4)
    T[0, 3] = coord[0]
    T[1, 3] = coord[1]
    T[2, 3] = coord[2]
    ik = manip.FindIKSolution(T, orpy.IkFilterOptions.CheckEnvCollisions)
    if ik is not None:
        return ik
    for i in range(80):
        T = rotZ(T, alpha)
        ik = manip.FindIKSolution(T, orpy.IkFilterOptions.CheckEnvCollisions)
        if ik is not None:
            return ik

def find_general_solutions(coord, manip):
    """
    Gets the IK solution given some coordinates
    Use this if 1 solution has PlanningError
    """
    T = np.eye(4)
    T[0, 3] = coord[0]
    T[1, 3] = coord[1]
    T[2, 3] = coord[2]
    ik = manip.FindIKSolutions(T, orpy.IkFilterOptions.CheckEnvCollisions)
    if ik != []:
        return ik
    for i in range(80):
        print('here')
        T = rotZ(T, alpha)
        ik = manip.FindIKSolutions(T, orpy.IkFilterOptions.CheckEnvCollisions)
        if ik != []:
            return ik
    print('fail to find ik')
