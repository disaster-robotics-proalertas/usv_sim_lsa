#!/usr/bin/env python

import xacro, sys
from subprocess import check_output
from roslaunch import substitution_args
import os
from lxml import etree
import yaml
from numpy import array

if __name__ == '__main__':
    
    p_default = 2
    i_default = 0
    d_default = 0
    
    if len(sys.argv) < 2:
        print 'Input a .xacro or .urdf file'
        print 'syntax : gen_pid.py <package> <urdf file>'
        sys.exit(0)
        
    robot_package = substitution_args.resolve_args('$(find %s)' % sys.argv[1])        
    robot_file = sys.argv[2]
    
    # find file
    robot_abs_file = ''
    for urdf_dir in ['urdf', 'sdf']:
        if os.path.lexists('%s/%s/%s' % (robot_package, urdf_dir, robot_file)):
            robot_abs_file = '%s/%s/%s' % (robot_package, urdf_dir, robot_file)
            
    # create config directory
    if not os.path.lexists('%s/config' % robot_package):
        os.mkdir('%s/config' % robot_package)
    config_file = '%s/config/%s_pid.yaml' % (robot_package, robot_file.split('.')[0])
    
    # load description
    robot_description = etree.fromstring(check_output(['rosrun', 'xacro', 'xacro', robot_abs_file]))
    
    # init config dictionary
    pid = {'config': {}}
    
    # parse joints
    joints = [joint for joint in robot_description.findall('joint') if joint.get('type') != "fixed"]
    if len(joints) != 0:
        pid['config']['joints'] = {}
        pid['config']['joints']['state'] = 'joint_states'
        pid['config']['joints']['setpoint'] = 'joint_setpoint'
        pid['config']['joints']['command'] = 'joint_command'
        pid['config']['joints']['cascaded_position'] = False
        pid['config']['joints']['dynamic_reconfigure'] = True    
        for joint in joints:
            name = joint.get('name')
            pid[name] = {}
            for controller in ['position', 'velocity']:
                pid[name][controller] = {'p': p_default, 'i': i_default, 'd': d_default}
                
    # parse thrusters
    thrusters = []
    for gazebo in robot_description.findall('gazebo'):
        for plugin in gazebo.findall('plugin'):
            thrusters += plugin.findall('thruster')
    if len(thrusters) != 0:
        pid['config']['body'] = {}
        pid['config']['body']['state'] = 'body_state'
        pid['config']['body']['setpoint'] = 'body_setpoint'
        pid['config']['body']['command'] = 'body_command'
        pid['config']['body']['cascaded_position'] = False
        pid['config']['body']['dynamic_reconfigure'] = True
        body_axis_candidates = {0: 'x', 1: 'y', 2: 'z', 3: 'roll', 4: 'pitch', 5: 'yaw'}
        body_axis = {}
        for thruster in thrusters:
            thruster_map = [float(v) for v in thruster.find('map').text.split(' ')]
            for x in body_axis_candidates:
                if thruster_map[x] != 0:
                    body_axis[x] = body_axis_candidates[x]
        for x in body_axis.values():
            pid[x] = {}
            for controller in ['position', 'velocity']:
                pid[x][controller] = {'p': p_default, 'i': i_default, 'd': d_default}
        
    # write config
    if len(joints) + len(thrusters) != 0:
        with open(config_file, 'w') as f:
            yaml.dump({'controllers': pid}, f)
        print 'controller written in', config_file
        