# - Config file for the osgBullet package
# It defines the following variables
#  OSGBULLET_INCLUDE_DIRS - include directories for osgBullet
#  OSGBULLET_LIBRARIES    - libraries to link against
#  OSGBULLET_EXECUTABLE   - the bar executable


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was osgBulletConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

set(OSGBULLET_PREFIX "/home/lsa/ocean2/install_isolated")

set(OSGBULLET_LIBRARIES "${OSGBULLET_PREFIX}/lib/libosgbCollision.so;${OSGBULLET_PREFIX}/lib/libosgbDynamics.so;${OSGBULLET_PREFIX}/lib/libosgbInteraction.so" )

set_and_check(OSGBULLET_INCLUDE_DIR "${OSGBULLET_PREFIX}/include/uwsim_osgbullet")
