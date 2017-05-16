# - Config file for the bullet package
# It defines the following variables
#  BULLET_INCLUDE_DIRS - include directories for osgOcean
#  BULLET_LIBRARIES    - libraries to link against


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was bulletConfig.cmake.in                            ########

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

set(BULLET_PREFIX "/home/lsa/ocean2/install_isolated")
set(BULLET_LIBRARIES "${BULLET_PREFIX}/lib/uwsim_bullet/libBulletCollision.a;${BULLET_PREFIX}/lib/uwsim_bullet/libBulletDynamics.a;${BULLET_PREFIX}/lib/uwsim_bullet/libBulletSoftBody.a;${BULLET_PREFIX}/lib/uwsim_bullet/libLinearMath.a" )

# Compute paths
set_and_check(BULLET_INCLUDE_DIRS "${BULLET_PREFIX}/include/uwsim_bullet")
