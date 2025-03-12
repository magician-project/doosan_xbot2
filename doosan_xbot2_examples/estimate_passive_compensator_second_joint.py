#!/bin/python

from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as opt

import rospy
import numpy as np

import scipy.io as sio


def save_data_to_mat(file_name, data1, data2, data3):
    # Check if the file exists
    try:
        # Try to load the existing data
        mat_data = sio.loadmat(file_name)
    except FileNotFoundError:
        # If the file does not exist, start a new dictionary
        mat_data = {}

    # Append new data
    if 'pos' not in mat_data:
        mat_data['pos'] = []
    if 'tor' not in mat_data:
        mat_data['tor'] = []
    if 'gcomp' not in mat_data:
        mat_data['gcomp'] = []

    # Append new data to existing lists
    mat_data['pos'] = np.append(mat_data['pos'], data1)
    mat_data['tor'] = np.append(mat_data['tor'], data2)
    mat_data['gcomp'] = np.append(mat_data['gcomp'], data3)

    # Save the updated data to the mat file
    sio.savemat(file_name, mat_data)




rospy.init_node('estimate_passive_compensator')


def get_xbot_cfg(is_fb):
    """
    A function to construct the xbotinterface config object from ros
    """
    urdf = rospy.get_param('/xbotcore/robot_description')
    srdf = rospy.get_param('/xbotcore/robot_description_semantic')
    
    cfg = opt.ConfigOptions()
    cfg.set_urdf(urdf)
    cfg.set_srdf(srdf)
    cfg.generate_jidmap()
    cfg.set_bool_parameter('is_model_floating_base', is_fb)
    cfg.set_string_parameter('model_type', 'RBDL')
    cfg.set_string_parameter('framework', 'ROS')
    
    return cfg

cfg = get_xbot_cfg(is_fb=False)  # get config object
robot = xbot.RobotInterface(cfg)  # create robot (note: xbot2 should be up and running)
robot.sense()

jnames = robot.getEnabledJointNames()   # get list of joint names
j_pos = robot.getJointPosition()  # get actual joint position
j_torque = robot.getJointEffort()  # get actual position reference
g_comp = robot.model().computeGravityCompensation()

for n, q, tau, gcomp in zip(jnames, j_pos, j_torque, g_comp):
    if n == "joint2":
        print('{}: {} -> {} -> {}'.format(n, q, tau, gcomp))
        #save_data_to_mat("estimate_passive.mat", q, tau, gcomp)


