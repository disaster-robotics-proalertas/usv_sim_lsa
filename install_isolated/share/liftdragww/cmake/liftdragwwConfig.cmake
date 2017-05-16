# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(liftdragww_CONFIG_INCLUDED)
  return()
endif()
set(liftdragww_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(liftdragww_SOURCE_PREFIX /home/lsa/ocean2/src/liftdragww)
  set(liftdragww_DEVEL_PREFIX /home/lsa/ocean2/devel_isolated/liftdragww)
  set(liftdragww_INSTALL_PREFIX "")
  set(liftdragww_PREFIX ${liftdragww_DEVEL_PREFIX})
else()
  set(liftdragww_SOURCE_PREFIX "")
  set(liftdragww_DEVEL_PREFIX "")
  set(liftdragww_INSTALL_PREFIX /home/lsa/ocean2/install_isolated)
  set(liftdragww_PREFIX ${liftdragww_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'liftdragww' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(liftdragww_FOUND_CATKIN_PROJECT TRUE)

if(NOT "include " STREQUAL " ")
  set(liftdragww_INCLUDE_DIRS "")
  set(_include_dirs "include")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${liftdragww_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'liftdragww' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'lsa <lsa@todo.todo>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'liftdragww' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/lsa/ocean2/install_isolated/${idir}'.  Ask the maintainer 'lsa <lsa@todo.todo>' to fix it.")
    endif()
    _list_append_unique(liftdragww_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "liftdragww")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND liftdragww_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND liftdragww_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND liftdragww_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/lsa/ocean2/install_isolated/lib;/home/lsa/ocean2/devel_isolated/freefloating_gazebo_demo/lib;/home/lsa/ocean2/devel_isolated/freefloating_gazebo/lib;/home/lsa/ocean2/devel_isolated/first_control/lib;/opt/ros/kinetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(liftdragww_LIBRARY_DIRS ${lib_path})
      list(APPEND liftdragww_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'liftdragww'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND liftdragww_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(liftdragww_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${liftdragww_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "roscpp;std_msgs")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 liftdragww_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${liftdragww_dep}_FOUND)
      find_package(${liftdragww_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${liftdragww_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(liftdragww_INCLUDE_DIRS ${${liftdragww_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(liftdragww_LIBRARIES ${liftdragww_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${liftdragww_dep}_LIBRARIES})
  _list_append_deduplicate(liftdragww_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(liftdragww_LIBRARIES ${liftdragww_LIBRARIES})

  _list_append_unique(liftdragww_LIBRARY_DIRS ${${liftdragww_dep}_LIBRARY_DIRS})
  list(APPEND liftdragww_EXPORTED_TARGETS ${${liftdragww_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${liftdragww_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
