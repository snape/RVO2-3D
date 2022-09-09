/*
 * Sphere.cc
 * RVO2-3D Library
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/RVO2/>
 */

/**
 * @file  Sphere.cc
 * @brief Example file showing a demo with 812 agents initially positioned
 *        evenly distributed on a sphere attempting to move to the antipodal
 *        position on the sphere.
 */

#ifndef RVO3D_OUTPUT_TIME_AND_POSITIONS
#define RVO3D_OUTPUT_TIME_AND_POSITIONS 1
#endif

#include <cmath>
#include <cstddef>
#include <vector>

#if RVO3D_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif /* RVO3D_OUTPUT_TIME_AND_POSITIONS */

#include <RVO.h>

namespace {
const float RVO3D_TWO_PI = 6.28318530717958647692F;

void setupScenario(
    RVO::RVOSimulator *sim,
    std::vector<RVO::Vector3> &goals) { /* NOLINT(runtime/references) */
  /* Specify the global time step of the simulation. */
  sim->setTimeStep(0.125F);

  /* Specify the default parameters for agents that are subsequently added. */
  sim->setAgentDefaults(15.0F, 10U, 10.0F, 1.5F, 2.0F);

  /* Add agents, specifying their start position, and store their goals on the
   * opposite side of the environment.
   */
  for (std::size_t i = 0U; i < 5.0F * RVO3D_TWO_PI; ++i) {
    const float z = 100.0F * std::cos(i / 10.0F);
    const float r = 100.0F * std::sin(i / 10.0F);

    for (std::size_t j = 0U; j < r / 2.5F; ++j) {
      const float x = r * std::cos(j * RVO3D_TWO_PI / (r / 2.5F));
      const float y = r * std::sin(j * RVO3D_TWO_PI / (r / 2.5F));

      sim->addAgent(RVO::Vector3(x, y, z));
      goals.push_back(-sim->getAgentPosition(sim->getNumAgents() - 1U));
    }
  }
}

#if RVO3D_OUTPUT_TIME_AND_POSITIONS
void updateVisualization(RVO::RVOSimulator *sim) {
  /* Output the current global time. */
  std::cout << sim->getGlobalTime();

  /* Output the position for all the agents. */
  for (std::size_t i = 0U; i < sim->getNumAgents(); ++i) {
    std::cout << " " << sim->getAgentPosition(i);
  }

  std::cout << std::endl;
}
#endif /* RVO3D_OUTPUT_TIME_AND_POSITIONS */
} /* namespace */

void setPreferredVelocities(RVO::RVOSimulator *sim,
                            const std::vector<RVO::Vector3> &goals) {
  /* Set the preferred velocity to be a vector of unit magnitude (speed) in the
   * direction of the goal.
   */
  for (std::size_t i = 0U; i < sim->getNumAgents(); ++i) {
    RVO::Vector3 goalVector = goals[i] - sim->getAgentPosition(i);

    if (RVO::absSq(goalVector) > 1.0F) {
      goalVector = RVO::normalize(goalVector);
    }

    sim->setAgentPrefVelocity(i, goalVector);
  }
}

bool reachedGoal(RVO::RVOSimulator *sim,
                 const std::vector<RVO::Vector3> &goals) {
  /* Check if all agents have reached their goals. */
  for (std::size_t i = 0U; i < sim->getNumAgents(); ++i) {
    if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) >
        4.0F * sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
      return false;
    }
  }

  return true;
}

int main() {
  /* Create a new simulator instance. */
  RVO::RVOSimulator *sim = new RVO::RVOSimulator();

  /* Store the goals of the agents. */
  std::vector<RVO::Vector3> goals;

  /* Set up the scenario. */
  setupScenario(sim, goals);

  /* Perform (and manipulate) the simulation. */
  do {
#if RVO3D_OUTPUT_TIME_AND_POSITIONS
    updateVisualization(sim);
#endif /* RVO3D_OUTPUT_TIME_AND_POSITIONS */
    setPreferredVelocities(sim, goals);
    sim->doStep();
  } while (!reachedGoal(sim, goals));

  delete sim;

  return 0;
}
