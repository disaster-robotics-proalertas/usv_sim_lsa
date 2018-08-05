#!/usr/bin/env python

import rospy
import argparse
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

args="-x 100 -y 100 -g 0.2 -b 1 -a 0.2"


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.description = 'Spawns a water surface with given color, position and size'

    parser.add_argument('-x', type=float, help='x-size', default=10)
    parser.add_argument('-y', type=float, help='y-size', default=10)    
    parser.add_argument('-r', type=float, help='Red color (0-1)', default=0)
    parser.add_argument('-g', type=float, help='Green color (0-1)', default=0.2)
    parser.add_argument('-b', type=float, help='Blue color (0-1)', default=0.8)
    parser.add_argument('-a', type=float, help='Alpha color (0-1)', default=0.2)
    parser.add_argument('-X', type=float, help='X position', default=0)
    parser.add_argument('-Y', type=float, help='Y position', default=0)
    parser.add_argument('-Z', type=float, help='Z position', default=0)
    
    args = parser.parse_known_args()[0]
    
    rospy.init_node('rpy_setpoint_bridge')  
    
    # wait for Gazebo to run
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn.wait_for_service()
    
    # build model XML
    surface_xml = """<?xml version="1.0"?>
<sdf version="1.4">

<model name="surface">
        <static>true</static>
        <link name="water">
        <visual name="water">
        <geometry>
          <box>
            <size>$x $y 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>$r $g $b $a</ambient>
          <specular>$r $g $b $a</specular>
          <diffuse>$r $g $b $a</diffuse>
        </material>
      </visual>
  </link>
</model>
</sdf>"""

    for arg in ['x','y','r','g','b','a']:
        surface_xml = surface_xml.replace('$'+arg, str(getattr(args, arg)))
        
    pose = Pose()
    pose.orientation.w = 1
    pose.position.x = args.X
    pose.position.y = args.Y
    pose.position.z = args.Z    
        
    spawn('surface', surface_xml, 'surface', pose, 'world')    
