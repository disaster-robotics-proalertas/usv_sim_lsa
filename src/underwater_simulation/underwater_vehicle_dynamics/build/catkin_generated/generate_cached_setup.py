# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/kinetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/kinetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/paravisi/usv_sim_lsa/devel_isolated/visualization_osg;/home/paravisi/usv_sim_lsa/devel_isolated/uwsim;/home/paravisi/usv_sim_lsa/devel_isolated/usv_tf;/home/paravisi/usv_sim_lsa/devel_isolated/usv_sim_test;/home/paravisi/usv_sim_lsa/devel_isolated/usv_sim_rviz;/home/paravisi/usv_sim_lsa/devel_isolated/usv_sim;/home/paravisi/usv_sim_lsa/devel_isolated/usv_navigation;/home/paravisi/usv_sim_lsa/devel_isolated/usv_base_ctrl;/home/paravisi/usv_sim_lsa/devel_isolated/underwater_vehicle_dynamics;/home/paravisi/usv_sim_lsa/devel_isolated/underwater_sensor_msgs;/home/paravisi/usv_sim_lsa/devel_isolated/sail_plugin;/home/paravisi/usv_sim_lsa/devel_isolated/rudder_plugin;/home/paravisi/usv_sim_lsa/devel_isolated/osg_interactive_markers;/home/paravisi/usv_sim_lsa/devel_isolated/osg_utils;/home/paravisi/usv_sim_lsa/devel_isolated/osg_markers;/home/paravisi/usv_sim_lsa/devel_isolated/liftdragww;/home/paravisi/usv_sim_lsa/devel_isolated/freefloating_gazebo;/opt/ros/kinetic".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/paravisi/usv_sim_lsa/src/underwater_simulation/underwater_vehicle_dynamics/build/devel/env.sh')

output_filename = '/home/paravisi/usv_sim_lsa/src/underwater_simulation/underwater_vehicle_dynamics/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
