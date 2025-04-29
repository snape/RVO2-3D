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

add_library(${RVO3D_LIBRARY})

include(GenerateExportHeader)

generate_export_header(${RVO3D_LIBRARY}
  EXPORT_FILE_NAME Export.h)

target_sources(${RVO3D_LIBRARY}
  PUBLIC
    FILE_SET HEADERS
    BASE_DIRS
      "${CMAKE_CURRENT_BINARY_DIR}"
      "${CMAKE_CURRENT_SOURCE_DIR}"
    FILES
      "${CMAKE_CURRENT_BINARY_DIR}/Export.h"
      Plane.h
      RVO.h
      RVOSimulator.h
      Vector3.h
    PRIVATE
      Agent.cc
      Agent.h
      Export.cc
      KdTree.cc
      KdTree.h
      Plane.cc
      RVOSimulator.cc
      Vector3.cc)

set_target_properties(${RVO3D_LIBRARY} PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  SOVERSION ${PROJECT_VERSION_MAJOR}
  VERSION ${PROJECT_VERSION}
  VISIBILITY_INLINES_HIDDEN ON)

if(ENABLE_INTERPROCEDURAL_OPTIMIZATION
    AND RVO3D_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED)
  set_target_properties(${RVO3D_LIBRARY} PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION ON)
endif()

if(WIN32)
  target_compile_definitions(${RVO3D_LIBRARY} PUBLIC NOMINMAX)
endif()

if(ENABLE_OPENMP AND OpenMP_FOUND)
  target_link_libraries(${RVO3D_LIBRARY} PRIVATE OpenMP::OpenMP_CXX)
endif()

export(TARGETS ${RVO3D_LIBRARY} NAMESPACE ${PROJECT_NAME}::
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

install(TARGETS ${RVO3D_LIBRARY}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE
    COMPONENT development
    DESTINATION ${RVO3D_LIBRARY_DIR}
    FILE_SET HEADERS
    COMPONENT development
    DESTINATION ${RVO3D_INCLUDE_DIR}
  LIBRARY
    COMPONENT runtime
    DESTINATION ${RVO3D_LIBRARY_DIR}
    NAMELINK_COMPONENT development
  RUNTIME
    COMPONENT runtime
    DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${RVO3D_INCLUDE_DIR})

install(EXPORT ${PROJECT_NAME}Targets
  COMPONENT development
  DESTINATION ${RVO3D_DIR}
  NAMESPACE ${PROJECT_NAME}::)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.31
    AND ENABLE_CMAKE_EXPERIMENTAL_FEATURES)
  set(CMAKE_EXPERIMENTAL_EXPORT_PACKAGE_INFO
    b80be207-778e-46ba-8080-b23bba22639e)

  install(PACKAGE_INFO ${RVO3D_LIBRARY}
    EXPORT ${PROJECT_NAME}Targets
    COMPONENT development
    DESTINATION ${RVO3D_CPS_DIR}
    VERSION ${PROJECT_VERSION}
    COMPAT_VERSION ${PROJECT_VERSION_MAJOR}
    VERSION_SCHEMA simple)
endif()
