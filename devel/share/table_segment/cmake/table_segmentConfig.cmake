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


if(table_segment_CONFIG_INCLUDED)
  return()
endif()
set(table_segment_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(table_segment_SOURCE_PREFIX /home/thomas/practice_ws/src/table_segment)
  set(table_segment_DEVEL_PREFIX /home/thomas/practice_ws/devel)
  set(table_segment_INSTALL_PREFIX "")
  set(table_segment_PREFIX ${table_segment_DEVEL_PREFIX})
else()
  set(table_segment_SOURCE_PREFIX "")
  set(table_segment_DEVEL_PREFIX "")
  set(table_segment_INSTALL_PREFIX /home/thomas/practice_ws/install)
  set(table_segment_PREFIX ${table_segment_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'table_segment' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(table_segment_FOUND_CATKIN_PROJECT TRUE)

if(NOT " " STREQUAL " ")
  set(table_segment_INCLUDE_DIRS "")
  set(_include_dirs "")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'thomas <thomas@todo.todo>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${table_segment_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'table_segment' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'table_segment' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/thomas/practice_ws/src/table_segment/${idir}'.  ${_report}")
    endif()
    _list_append_unique(table_segment_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND table_segment_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND table_segment_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND table_segment_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/thomas/practice_ws/devel/lib;/opt/ros/kinetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(table_segment_LIBRARY_DIRS ${lib_path})
      list(APPEND table_segment_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'table_segment'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND table_segment_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(table_segment_EXPORTED_TARGETS "table_segment_generate_messages_cpp;table_segment_generate_messages_eus;table_segment_generate_messages_lisp;table_segment_generate_messages_nodejs;table_segment_generate_messages_py")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${table_segment_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 table_segment_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${table_segment_dep}_FOUND)
      find_package(${table_segment_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${table_segment_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(table_segment_INCLUDE_DIRS ${${table_segment_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(table_segment_LIBRARIES ${table_segment_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${table_segment_dep}_LIBRARIES})
  _list_append_deduplicate(table_segment_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(table_segment_LIBRARIES ${table_segment_LIBRARIES})

  _list_append_unique(table_segment_LIBRARY_DIRS ${${table_segment_dep}_LIBRARY_DIRS})
  list(APPEND table_segment_EXPORTED_TARGETS ${${table_segment_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "table_segment-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${table_segment_DIR}/${extra})
  endif()
  include(${extra})
endforeach()