#!/usr/bin/env python
import math
import numpy as np

def slerp(v0, v1, t_array):
    # >>> slerp([1,0,0,0],[0,0,0,1],np.arange(0,1,0.001))
    t_array = np.array(t_array)
    v0 = np.array(v0)
    v1 = np.array(v1)
    dot = np.sum(v0*v1)

    if (dot < 0.0):
        v1 = -v1
        dot = -dot
    
    DOT_THRESHOLD = 0.9995
    if (dot > DOT_THRESHOLD):
        result = v0[np.newaxis,:] + t_array[:,np.newaxis]*(v1 - v0)[np.newaxis,:]
        return (result.T / np.linalg.norm(result, axis=1)).T
    
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)

    theta = theta_0*t_array
    sin_theta = np.sin(theta)
    
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0[:,np.newaxis] * v0[np.newaxis,:]) + (s1[:,np.newaxis] * v1[np.newaxis,:])

def axis_angle2quaternion(axis, angle):
    '''Convert axis angle representation to quaternion (assumes axis is already normalised).

    Parameters:
        axis (list): 3-D normalised vector of rotation axis (right-hand rule)
        angle (double): Magnitude of tilt angle in degrees
    Returns:
        quaternion (list): quaternion representation in order of qx, qy, qz, qw
    
    '''
    s = math.sin(math.radians(angle)/2)
    qx = axis[0] * s
    qy = axis[1] * s
    qz = axis[2] * s
    qw = math.cos(math.radians(angle)/2)
    return [qx, qy, qz, qw]