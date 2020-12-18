# -*- mode: cmake -*-
# vi: set ft=cmake :

#
# examples/CMakeLists.txt
# RVO2-3D Library
#
# Copyright 2008 University of North Carolina at Chapel Hill
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

if(BUILD_TESTING)
  option(OUTPUT_TIME_AND_POSITIONS "Output time and positions" ON)

  if(OUTPUT_TIME_AND_POSITIONS)
    set(RVO3D_EXAMPLES_COMPILE_DEFINITIONS
      RVO3D_OUTPUT_TIME_AND_POSITIONS=1)
  else()
    set(RVO3D_EXAMPLES_COMPILE_DEFINITIONS
      RVO3D_OUTPUT_TIME_AND_POSITIONS=0)
  endif()

  option(SEED_RANDOM_NUMBER_GENERATOR "Seed random number generator" ON)

  if(SEED_RANDOM_NUMBER_GENERATOR)
    list(APPEND RVO3D_EXAMPLES_COMPILE_DEFINITIONS
      RVO3D_SEED_RANDOM_NUMBER_GENERATOR=1)
  else()
    list(APPEND RVO3D_EXAMPLES_COMPILE_DEFINITIONS
      RVO3D_SEED_RANDOM_NUMBER_GENERATOR=0)
  endif()

  if(ENABLE_OPENMP AND OpenMP_FOUND)
    set(RVO3D_EXAMPLES_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    set(RVO3D_EXAMPLES_LINK_FLAGS "${OpenMP_CXX_FLAGS}")
  else()
    set(RVO3D_EXAMPLES_COMPILE_OPTIONS)
    set(RVO3D_EXAMPLES_LINK_FLAGS)
  endif()

  if(ENABLE_INTERPROCEDURAL_OPTIMIZATION
      AND RVO3D_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED)
    set(RVO3D_EXAMPLES_INTERPROCEDURAL_OPTIMIZATION ON)
  else()
    set(RVO3D_EXAMPLES_INTERPROCEDURAL_OPTIMIZATION OFF)
  endif()

  add_executable(Sphere Sphere.cpp)
  target_compile_definitions(Sphere PRIVATE
    ${RVO3D_EXAMPLES_COMPILE_DEFINITIONS})
  target_link_libraries(Sphere PRIVATE ${RVO3D_LIBRARY})
  target_compile_options(Sphere PRIVATE ${RVO3D_EXAMPLES_COMPILE_OPTIONS})
  set_target_properties(Sphere PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION ${RVO3D_EXAMPLES_INTERPROCEDURAL_OPTIMIZATION}
    LINK_FLAGS "${RVO3D_EXAMPLES_LINK_FLAGS}")
  add_test(NAME Sphere COMMAND Sphere)
endif()
