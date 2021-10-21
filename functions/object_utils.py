"""Utility functions for various objects in OpenRAVE"""
import numpy as np
import openravepy as orpy

def rotX(T, theta):
    rot = np.eye(4)
    rot[1, 1] = np.cos(theta)
    rot[1, 2] = -np.sin(theta)
    rot[2, 1] = np.sin(theta)
    rot[2, 2] = np.cos(theta)
    return np.dot(T, rot)

def rotY(T, theta):
    rot = np.eye(4)
    rot[0, 0] = np.cos(theta)
    rot[0, 2] = np.sin(theta)
    rot[2, 0] = -np.sin(theta)
    rot[2, 2] = np.cos(theta)
    return np.dot(T, rot)

def rotZ(T, theta):
    rot = np.eye(4)
    rot[0, 0] = np.cos(theta)
    rot[0, 1] = -np.sin(theta)
    rot[1, 0] = np.sin(theta)
    rot[1, 1] = np.cos(theta)
    return np.dot(T, rot)

def createLens(T, env):
    print("Create new lens")
    lens = env.ReadKinBodyXMLFile('objects/lens.kinbody.xml')
    lens.SetName('lens')
    lens.SetTransform(T)
    env.Add(lens,  True)
    return lens

def create_20_lenses(starting_coord, env):
    lenses = []
    T = np.eye(4)
    x, y, z = starting_coord[0], starting_coord[1], starting_coord[2]
    for i in range(4):
        for j in range(5):
            T[0, 3] = x
            T[1, 3] = y
            T[2, 3] = z
            lens = createLens(T, env)
            lenses.append(lens)
            y -= 0.05
        x -= 0.035
        y = -0.07
    return lenses