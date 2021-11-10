#! /usr/bin/python

import math
import numpy as np
from numpy import linalg
from numpy.core.fromnumeric import shape
from sympy import sin, cos, pi
#turtlebot3_ik_solver_exercise.
from turtle_arm import turtle_arm_dh

def normalize_angles(angles):
  '''
  Normalized angles in passed array to interval [-pi, pi]

  INPUT:
    angles              (List) Array of angles in [rad]

  OUTPUT
    angles_normalized   (List) Array of normalized angles
  '''
  
  angles_normalized = angles

  if isinstance(angles, list):
    for i in range(len(angles_normalized)):
      angles_normalized[i] = np.mod(np.mod(angles_normalized[i], 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi)  
      for angle_normalized in np.nditer(angles_normalized[i], op_flags=['readwrite']):
        if angle_normalized > math.pi:
          angle_normalized -= 2.0 * math.pi
  else:
    angles_normalized = np.mod(np.mod(angles_normalized, 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi)
    for angle_normalized in np.nditer(angles_normalized, op_flags=['readwrite']):
      if angle_normalized > math.pi:
        angle_normalized -= 2.0 * math.pi
      
  return angles_normalized


def get_tcp_position(TM):
  '''
    The method get_tcp_position extracts the endeffector position
    with regard to the base from the data structure of the
    manipulators transformations resulting form forwardKinematicsDH.

    INPUT
      TM:      [N]    List containing all N transformation matrices
      TM[i]:   [4x4]  Transformation matrix (2D python array)
                      of the (i+1)-th robot link with regard to the base
    OUTPUT
      p:       [3x1]  Endeffector possition with regard to the base
  '''
  # initialize return value
  p = np.zeros((3, 1))

  # | Implement your code here |
  # v                          v
  p= np.reshape(TM[len(TM) - 1 ][:3,3],(3,1))
  # ^                          ^
  # | -------- End ----------- |
  return p

# compute  homogen rotaion matrix for given x or z axis
def computr_rotaion_matric(q, axis):
    return  np.array([[cos(q),-1 * sin(q),0,0],[sin(q),cos(q),0,0],[0,0,1,0],[0,0,0,1]] if axis == 'z'
    else [[1,0,0,0],[0,cos(q),-1 * sin(q),0],[0,sin(q),cos(q),0],[0,0,0,1]])

#compute ahomogen Translation matrix for given x or z axis
def compute_translation_matric(q, axis):
    return  np.array([[1,0,0,0],[0,1,0,0],[0,0,1,q],[0,0,0,1]] if axis == 'z'
    else [[1,0,0,q],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

def compute_transformation_dh(dh, q):
  '''
  The function compute_transformation_dh computes, given the list
  dh containing the DH-Parameter as well as the joint value q, the
  local transformation matrix of one robot link.

  INPUT
    dh:            Dictionary containing four DH-Parameter and one joint type\n
    dh["theta"]:   [1x1]  DH-Parameter theta\n
    dh["d"]:       [1x1]  DH-Parameter d\n
    dh["a"]:       [1x1]  DH-Parameter a\n
    dh["alpha"]:   [1x1]  DH-Parameter alpha\n
    dh["rho"]:     [1x1]  Joint type (0 = prismatic, 1 = revolute)\n
    q:             [1x1]  Joint variable in [rad] or [m] according to the joint type

  OUTPUT
    T:             [4x4]  Transformation matrix (2D python array) of the robot link
  '''
  # initialize return value
  T = np.eye(4)
  # | Implement your code here |
  # v                          v
  rotZ = []
  traZ = []
  if dh["rho"]:
        rotZ = computr_rotaion_matric(dh["theta"] + q,'z')
        traZ = compute_translation_matric(dh["d"],'z')
  else :
        rotZ = computr_rotaion_matric(dh["theta"], 'z')
        traZ = compute_translation_matric(dh["d"] + q,'z')

  traX = compute_translation_matric(dh['a'],'x')
  rotX = computr_rotaion_matric(dh["alpha"],'x')
  
  T = rotZ @ traZ @ traX @ rotX
  # ^                          ^
  # | -------- End ----------- |
  
  return T