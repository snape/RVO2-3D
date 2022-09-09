# -*- mode: cmake; -*-
# vi: set ft=cmake:

#
# src/CMakeLists.txt
# RVO2-3D Library
#
# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Please send all bug reports to <geom@cs.unc.edu>.
#
# The authors may be contacted via:
#
# Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
# Dept. of Computer Science
# 201 S. Columbia St.
# Frederick P. Brooks, Jr. Computer Science Bldg.
# Chapel Hill, N.C. 27599-3175
# United States of America
#
# <https://gamma.cs.unc.edu/RVO2/>
#

set(RVO3D_HEADERS
  Plane.h
  RVO.h
  RVOSimulator.h
  Vector3.h)

set(RVO3D_SOURCES
  Agent.cc
  Agent.h
  Export.cc
  KdTree.cc
  KdTree.h
  Plane.cc
  RVOSimulator.cc
  Vector3.cc)

add_library(${RVO3D_LIBRARY} ${RVO3D_HEADERS} ${RVO3D_SOURCES})

include(GenerateExportHeader)

generate_export_header(${RVO3D_LIBRARY}
  EXPORT_FILE_NAME Export.h)

set_target_properties(${RVO3D_LIBRARY} PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  PUBLIC_HEADER "${RVO3D_HEADERS}"
  SOVERSION ${PROJECT_VERSION_MAJOR}
  VERSION ${PROJECT_VERSION}
  VISIBILITY_INLINES_HIDDEN ON)

if(ENABLE_INTERPROCEDURAL_OPTIMIZATION
    AND RVO3D_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED)
  set_target_properties(${RVO3D_LIBRARY} PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION ON)
endif()

target_include_directories(${RVO3D_LIBRARY} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

if(WIN32)
  target_compile_definitions(${RVO3D_LIBRARY} PUBLIC NOMINMAX)
endif()

if(ENABLE_OPENMP AND OpenMP_FOUND)
  set_target_properties(${RVO3D_LIBRARY} PROPERTIES
    LINK_FLAGS "${OpenMP_CXX_FLAGS}")
  target_compile_options(${RVO3D_LIBRARY} PRIVATE ${OpenMP_CXX_FLAGS})
endif()

export(TARGETS ${RVO3D_LIBRARY} NAMESPACE ${PROJECT_NAME}::
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.12)
  set(RVO3D_NAMELINK_COMPONENT_ARGUMENT NAMELINK_COMPONENT development)
else()
  set(RVO3D_NAMELINK_COMPONENT_ARGUMENT)
endif()

install(TARGETS ${RVO3D_LIBRARY}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE
    COMPONENT development
    DESTINATION ${RVO3D_LIBRARY_DIR}
  LIBRARY
    COMPONENT runtime
    DESTINATION ${RVO3D_LIBRARY_DIR}
    ${RVO3D_NAMELINK_COMPONENT_ARGUMENT}
  PUBLIC_HEADER
    COMPONENT development
    DESTINATION ${RVO3D_INCLUDE_DIR}
  RUNTIME
    COMPONENT runtime
    DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${RVO3D_INCLUDE_DIR})

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/Export.h"
  COMPONENT development
  DESTINATION ${RVO3D_INCLUDE_DIR})

install(EXPORT ${PROJECT_NAME}Targets
  COMPONENT development
  DESTINATION ${RVO3D_DIR}
  NAMESPACE ${PROJECT_NAME}::)
