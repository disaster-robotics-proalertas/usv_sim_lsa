#!/usr/bin/env python

import xacro, sys
from subprocess import check_output
from roslaunch import substitution_args
import os
from lxml import etree
import yaml
import numpy as np

    
# write default values
def write(d, key, val):
    if '/' in key:
        keys = key.split('/')
        if keys[0] not in d:
            d[keys[0]] = {}
        write(d[keys[0]], '/'.join(keys[1:]), val)
    elif key not in d:
        d[key] = val
        
def parse(pkg, model):
    
    gains = {'p': 2, 'i': 0, 'd': 0}
    
       
    robot_package = substitution_args.resolve_args('$(find %s)' % pkg)        
    robot_file = os.path.splitext(model)[0]
    
    # find file
    robot_abs_file = ''
    for urdf_dir in ['urdf', 'sdf']:
        for ext in ('urdf','xacro'):
            candidate = '{}/{}/{}.{}'.format(robot_package, urdf_dir, robot_file, ext)
            if os.path.lexists(candidate):
                robot_abs_file = candidate
    print('Using model ' + robot_abs_file)
            
    # create config directory
    if not os.path.lexists('%s/config' % robot_package):
        os.mkdir('%s/config' % robot_package)
    # look for existing config file
    config_file = ''
    for cf in os.listdir('{}/config'.format(robot_package)):
        if robot_file in cf:
            with open('{}/config/{}'.format(robot_package,cf)) as f:
                content = yaml.load(f)
            if 'controllers' in content:
                config_file = '{}/config/{}'.format(robot_package,cf)
    if config_file == '':       
        config_file = '{}/config/{}_pid.yaml'.format(robot_package, robot_file)
    print('Using config ' + config_file)
    # load description
    robot_description = etree.fromstring(check_output(['rosrun', 'xacro', 'xacro', '--inorder', robot_abs_file]))
    
    # init config dictionary
    if os.path.lexists(config_file):
        with open(config_file) as f:
            pid = yaml.load(f)['controllers']
    else:
        pid = {'config': {}}
    
    # parse joints
    joints = [joint for joint in robot_description.findall('joint') if joint.get('type') != "fixed"]
    if len(joints) != 0:
        write(pid, 'config/joints/state', 'joint_states')
        write(pid, 'config/joints/setpoint', 'joint_setpoint')
        write(pid, 'config/joints/command', 'joint_command')
        write(pid, 'config/joints/cascaded_position', False)
        write(pid, 'config/joints/dynamic_reconfigure', True)  
        for joint in joints:
            for mode in ('position','velocity'):
                for g in gains:
                    write(pid, '{}/{}/{}'.format(joint.get('name'), mode, g), gains[g])
                
    # parse thrusters
    thrusters = []
    base_link = 'base_link'
    damping = [100. for i in range(6)]
    m = 5
    
    for gazebo in robot_description.findall('gazebo'):
        for plugin in gazebo.findall('plugin'):
            if plugin.get('name') == 'freefloating_gazebo_control':
                thrusters += plugin.findall('thruster')
                if plugin.find('link') != None:
                    base_link = plugin.find('link').text
            
    if len(thrusters) != 0:        
        write(pid, 'config/body/state', 'body_state')
        write(pid, 'config/body/setpoint', 'body_setpoint')
        write(pid, 'config/body/command', 'body_command')
        write(pid, 'config/body/cascaded_position', False)
        write(pid, 'config/body/dynamic_reconfigure', True)
        
        # get thruster characteristics
        T = []
        Umax = []
        for thruster in thrusters:
            thruster_map = [float(v) for v in thruster.find('map').text.split(' ')]
            T.append(thruster_map)
            Umax.append(float(thruster.find('effort').text))

        T = np.matrix(T).transpose()
        n = len(Umax)
        Umax = np.matrix(Umax).reshape(n,1)  
        Fmax = np.abs(T)*Umax
        Fmax = [Fmax[i,0] for i in range(6)]
        
        body_axis = {0: 'x', 1: 'y', 2: 'z', 3: 'roll', 4: 'pitch', 5: 'yaw'}
        for i in range(6):
            if Fmax[i]:
                for mode in ('position','velocity'):
                    for g in gains:
                        write(pid, '{}/{}/{}'.format(body_axis[i], mode, g), gains[g])
        
        # get damping
        for link in robot_description.findall('link'):
            if link.get('name') == base_link:
                damp = link.find('buoyancy').find('damping')
                damping = [float(v) for v in damp.get('xyz').split(' ') + damp.get('rpy').split(' ')]        
                m = float(link.find('inertial').find('mass').get('value'))
                mass = [m,m,m]
                for key in 'ixx','iyy','izz':
                    mass.append(float(link.find('inertial').find('inertia').get('ixx')))
                    
    with open(config_file, 'w') as f:
        yaml.dump({'controllers': pid}, f, default_flow_style=False)
                    
    return pid, [Umax[i,0] for i in range(n)], Fmax, mass, damping, config_file
