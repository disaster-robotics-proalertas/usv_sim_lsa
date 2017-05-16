#!/usr/bin/env sh
# generated from catkin.builder module

# remember type of shell if not already set
if [ -z "$CATKIN_SHELL" ]; then
  CATKIN_SHELL=sh
fi
. "/home/lsa/ocean2/devel_isolated/underwater_vehicle_dynamics/setup.$CATKIN_SHELL"

# detect if running on Darwin platform
_UNAME=`uname -s`
IS_DARWIN=0
if [ "$_UNAME" = "Darwin" ]; then
  IS_DARWIN=1
fi

# Prepend to the environment
export CMAKE_PREFIX_PATH="/home/lsa/ocean2/devel_isolated/uwsim_bullet:$CMAKE_PREFIX_PATH"
if [ $IS_DARWIN -eq 0 ]; then
  export LD_LIBRARY_PATH="/home/lsa/ocean2/devel_isolated/uwsim_bullet/lib:/home/lsa/ocean2/devel_isolated/uwsim_bullet/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
else
  export DYLD_LIBRARY_PATH="/home/lsa/ocean2/devel_isolated/uwsim_bullet/lib:/home/lsa/ocean2/devel_isolated/uwsim_bullet/lib/x86_64-linux-gnu:$DYLD_LIBRARY_PATH"
fi
export PATH="/home/lsa/ocean2/devel_isolated/uwsim_bullet/bin:$PATH"
export PKG_CONFIG_PATH="/home/lsa/ocean2/devel_isolated/uwsim_bullet/lib/pkgconfig:/home/lsa/ocean2/devel_isolated/uwsim_bullet/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH"
export PYTHONPATH="/home/lsa/ocean2/devel_isolated/uwsim_bullet/lib/python2.7/dist-packages:$PYTHONPATH"
