#! /usr/bin/python

import math
import numpy as np

def inverse_kinematics_tb(request_position, py, l):
  '''
  The method inverse_kinematics_tb computes the joint configurations
  suitable to reach the requested position and orientation in
  both elbow configurations.

  Input values are the endeffector position, the endeffector
  orientation with the yaw rotation about the z_0-axis and
  pitch angle between the z_0-axis and the endeffector direction
  described by x_n-axis.

  INPUT
    request_position       [3x1]  Requested endeffector position as [x, y, z]'
    py                            Dictionary containing the requested pitch and yaw
    py["pitch"]                   Requestet pitch
    py["yaw"]                     Requestet yaw
    l                             List containing N elements according to the link lengths

  OUTPUT
    qs                            List containing two joint configuration arrays of shape [1xN]
                                  or empty [] if no solution can be found
  '''
  # initialize return value
  qs = []

  # | Implement your code here |
  # v                          v

  

  # ^                          ^
  # | -------- End ----------- |

  return qs
