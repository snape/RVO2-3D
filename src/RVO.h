/*
 * RVO.h
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

#ifndef RVO3D_RVO_H_
#define RVO3D_RVO_H_

/* IWYU pragma: begin_exports */
#include "Export.h"
#include "Plane.h"
#include "RVOSimulator.h"
#include "Vector3.h"
/* IWYU pragma: end_exports */

/**

@file  RVO.h
@brief Includes all public headers in the library.

@namespace RVO
@brief     Contains all classes, functions, and constants used in the library.

@mainpage  RVO2-3D Library
@author    Jur van den Berg
@author    Stephen J. Guy
@author    Jamie Snape
@author    Ming C. Lin
@author    Dinesh Manocha
@copyright 2008 University of North Carolina at Chapel Hill

<b>RVO2-3D Library</b> is an easy-to-use C++ implementation of the <a
href="https://gamma.cs.unc.edu/CA/">Optimal Reciprocal Collision Avoidance</a>
(ORCA) formulation for multi-agent simulation in three dimensions. <b>RVO2-3D
Library</b> automatically uses parallelism for computing the motion of the
agents if your machine has multiple processors and your compiler supports <a
href="http://www.openmp.org/">OpenMP</a>.

Please follow the following steps to install and use <b>RVO2-3D Library</b>.

@li @subpage whatsnew
@li @subpage building
@li @subpage using
@li @subpage params

See the documentation of the RVO::RVOSimulator class for an exhaustive list of
public functions of <b>RVO2-3D Library</b>.

<b>RVO2-3D Library</b>, accompanying example code, and this documentation is
released under the following @subpage terms "terms and conditions".

@page whatsnew What Is New in RVO2-3D Library

@section localca Three Dimensions

In contrast to RVO2 Library, <b>RVO2-3D Library</b> operates in
three-dimensional workspaces. It uses a three-dimensional implementation of <a
href="https://gamma.cs.unc.edu/CA/">Optimal Reciprocal Collision Avoidance</a>
(ORCA) for local collision avoidance. <b>RVO2-3D Library</b> does not replace
RVO2 Library; for two-dimensional applications, RVO2 Library should be used.

@section structure Structure of RVO2-3D Library

The structure of <b>RVO2-3D Library</b> is similar to that of RVO2 Library.
Users familiar with RVO2 Library should find little trouble in using <b>RVO2-3D
Library</b>. <b>RVO2-3D Library</b> currently does not support static obstacles.

@page building Building RVO2-3D Library

We assume that you have downloaded <b>RVO2-3D Library</b> and unpacked the ZIP
archive to a path <tt>$RVO3D_ROOT</tt>.

@section cmake CMake

Create and switch to your chosen build directory, e.g.,
<tt>$RVO3D_ROOT/build</tt>. Run <tt>cmake</tt> inside the build directory on the
source directory, e.g., <tt>cmake $RVO3D_ROOT/src</tt>. Build files for the
default generator for your platform will be generated in the build directory.

@page using Using RVO2-3D Library

@section structure Structure

A program performing an <b>RVO2-3D Library</b> simulation has the following
global structure.

@code{.cpp}
#include <RVO.h>

#include <vector>

int main() {
  // Create a new simulator instance.
  RVO::RVOSimulator *sim = new RVO::RVOSimulator();

  // Store the goals of the agents.
  std::vector<RVO::Vector3> goals;

  // Set up the scenario.
  setupScenario(sim, goals);

  // Perform (and manipulate) the simulation.
  do {
    updateVisualization(sim);
    setPreferredVelocities(sim, goals);
    sim->doStep();
  } while (!reachedGoal(sim, goals));

  delete sim;

  return 0;
}
@endcode

In order to use <b>RVO2-3D Library</b>, the user needs to include RVO.h. The
first step is then to create an instance of RVO::RVOSimulator. Then, the process
consists of two stages. The first stage is specifying the simulation scenario
and its parameters. In the above example program, this is done in the method
<tt>setupScenario(...)</tt>, which we will discuss below. The second stage is
the actual performing of the simulation.

In the above example program, simulation steps are taken until all the agents
have reached some predefined goals. Prior to each simulation step, we set the
preferred velocity for each agent, i.e., the velocity the agent would have taken
if there were no other agents around, in the method
<tt>setPreferredVelocities(...)</tt>. The simulator computes the actual
velocities of the agents and attempts to follow the preferred velocities as
closely as possible while guaranteeing collision avoidance at the same time.
During the simulation, the user may want to retrieve information from the
simulation for instance to visualize the simulation. In the above example
program, this is done in the method <tt>updateVisualization(...)</tt>, which we
will discuss below. It is also possible to manipulate the simulation during the
simulation, for instance by changing positions, radii, velocities, etc. of the
agents.

@section spec Setting up the Simulation Scenario

A scenario that is to be simulated can be set up as follows. A scenario consists
of a set of agents that can be manually specified. Agents may be added anytime
before or during the simulation. The user may also want to define goal positions
of the agents, or a roadmap to guide the agents around obstacles. This is not
done in <b>RVO2-3D Library</b>, but needs to be taken care of in the user's
external application.

The following example creates a scenario with eight agents exchanging positions.

@code{.cpp}
void setupScenario(RVO::RVOSimulator *sim, std::vector<RVO::Vector3> &goals) {
  // Specify global time step of the simulation.
  sim->setTimeStep(0.25F);

  // Specify default parameters for agents that are subsequently added.
  sim->setAgentDefaults(15.0F, 10U, 10.0F, 2.0F, 2.0F);

  // Add agents, specifying their start position.
  sim->addAgent(RVO::Vector3(-50.0F, -50.0F, -50.0F));
  sim->addAgent(RVO::Vector3(50.0F, -50.0F, -50.0F));
  sim->addAgent(RVO::Vector3(50.0F, 50.0F, -50.0F));
  sim->addAgent(RVO::Vector3(-50.0F, 50.0F, -50.0F));
  sim->addAgent(RVO::Vector3(-50.0F, -50.0F, 50.0F));
  sim->addAgent(RVO::Vector3(50.0F, -50.0F, 50.0F));
  sim->addAgent(RVO::Vector3(50.0F, 50.0F, 50.0F));
  sim->addAgent(RVO::Vector3(-50.0F, 50.0F, 50.0F));

  // Create goals (simulator is unaware of these).
  for (std::size_t i = 0U; i < sim->getNumAgents(); ++i) {
    goals.push_back(-sim->getAgentPosition(i));
  }
}
@endcode

See the documentation on RVO::RVOSimulator for a full overview of the
functionality to specify scenarios.

@section ret Retrieving Information from the Simulation

During the simulation, the user can extract information from the simulation for
instance for visualization purposes, or to determine termination conditions of
the simulation. In the example program above, visualization is done in the
<tt>updateVisualization(...)</tt> method. Below we give an example that simply
writes the positions of each agent in each time step to the standard output. The
termination condition is checked in the <tt>reachedGoal(...)</tt> method. Here
we give an example that returns true if all agents are within one radius of
their goals.

@code{.cpp}
#include <iostream>

void updateVisualization(RVO::RVOSimulator *sim) {
  // Output the current global time.
  std::cout << sim->getGlobalTime() << " ";

  // Output the position for all the agents.
  for (std::size_t i = 0U; i < sim->getNumAgents(); ++i) {
    std::cout << sim->getAgentPosition(i) << " ";
  }

  std::cout << std::endl;
}
@endcode

@code{.cpp}
bool reachedGoal(RVO::RVOSimulator *sim,
                 const std::vector<RVO::Vector3> &goals) {
  // Check whether all agents have arrived at their goals.
  for (std::size_t i = 0U; i < sim->getNumAgents(); ++i) {
    if (absSq(goals[i] - sim->getAgentPosition(i)) > sim->getAgentRadius(i) *
        sim->getAgentRadius(i)) {
      // Agent is further away from its goal than one radius.
      return false;
    }
  }

  return true;
}
@endcode

Using similar functions as the ones used in this example, the user can access
information about other parameters of the agents, as well as the global
parameters, and the obstacles. See the documentation of the class
RVO::RVOSimulator for an exhaustive list of public functions for retrieving
simulation information.

@section manip Manipulating the Simulation

During the simulation, the user can manipulate the simulation, for instance by
changing the global parameters, or changing the parameters of the agents
(potentially causing abrupt different behavior). It is also possible to give the
agents a new position, which make them jump through the scene. New agents can be
added to the simulation at any time.

See the documentation of the class RVO::RVOSimulator for an exhaustive list of
public functions for manipulating the simulation.

To provide global guidance to the agents, the preferred velocities of the agents
can be changed ahead of each simulation step. In the above example program, this
happens in the method <tt>setPreferredVelocities(...)</tt>. Here we give an
example that simply sets the preferred velocity to the unit vector towards the
agent's goal for each agent (i.e., the preferred speed is <tt>1.0</tt>).

@code{.cpp}
void setPreferredVelocities(RVO::RVOSimulator *sim,
                            const std::vector<RVO::Vector3> &goals) {
  // Set the preferred velocity for each agent.
  for (std::size_t i = 0U; i < sim->getNumAgents(); ++i) {
    if (absSq(goals[i] - sim->getAgentPosition(i)) < sim->getAgentRadius(i) *
        sim->getAgentRadius(i) ) {
      // Agent is within one radius of its goal, set preferred velocity to zero.
      sim->setAgentPrefVelocity(i, RVO::Vector3());
    } else {
      // Agent is far away from its goal, set preferred velocity as unit vector
      // towards agent's goal.
      sim->setAgentPrefVelocity(i, normalize(goals[i] -
      sim->getAgentPosition(i)));
    }
  }
}
@endcode

@section example Example Program

<b>RVO2-3D Library</b> is accompanied by one example program, which can be found
in the <tt>$RVO3D_ROOT/examples</tt> directory. The example is named Sphere, and
contains the following demonstration scenario:

<table border="0" cellpadding="3" width="100%">
<tr>
<td valign="top" width="100"><b>Sphere</b></td>
<td valign="top">A scenario in which 812 agents, initially positioned evenly
distributed on a sphere, move to the antipodal position on the sphere.</td>
</tr>
</table>

@page params Parameter Overview

@section globalp Global Parameters

<table border="0" cellpadding="3" width="100%">
<tr>
<td valign="top" width="150"><strong>Parameter</strong></td>
<td valign="top" width="150"><strong>Type (unit)</strong></td>
<td valign="top"><strong>Meaning</strong></td>
</tr>
<tr>
<td valign="top">timeStep</td>
<td valign="top">float (time)</td>
<td valign="top">The time step of the simulation. Must be positive.</td>
</tr>
</table>

@section agent Agent Parameters

<table border="0" cellpadding="3" width="100%">
<tr>
<td valign="top" width="150"><strong>Parameter</strong></td>
<td valign="top" width="150"><strong>Type (unit)</strong></td>
<td valign="top"><strong>Meaning</strong></td>
</tr>
<tr>
<td valign="top">maxNeighbors</td>
<td valign="top">std::size_t</td>
<td valign="top">The maximum number of other agents the agent takes into account
in the navigation. The larger this number, the longer the running time of the
simulation. If the number is too low, the simulation will not be safe.</td>
</tr>
<tr>
<td valign="top">maxSpeed</td>
<td valign="top">float (distance/time)</td>
<td valign="top">The maximum speed of the agent. Must be non-negative.</td>
</tr>
<tr>
<td valign="top">neighborDist</td>
<td valign="top">float (distance)</td>
<td valign="top">The maximum distance (center point to center point) to other
agents the agent takes into account in the navigation. The larger this number,
the longer the running time of the simulation. If the number is too low, the
simulation will not be safe. Must be non-negative.</td>
</tr>
<tr>
<td valign="top" width="150">position</td>
<td valign="top" width="150">RVO::Vector3 (distance, distance)</td>
<td valign="top">The current position of the agent.</td>
</tr>
<tr>
<td valign="top" width="150">prefVelocity</td>
<td valign="top" width="150">RVO::Vector3 (distance/time, distance/time)
</td>
<td valign="top">The current preferred velocity of the agent. This is the
velocity the agent would take if no other agents or obstacles were around. The
simulator computes an actual velocity for the agent that follows the preferred
velocity as closely as possible, but at the same time guarantees collision
avoidance.</td>
</tr>
<tr>
<td valign="top">radius</td>
<td valign="top">float (distance)</td>
<td valign="top">The radius of the agent. Must be non-negative.</td>
</tr>
<tr>
<td valign="top" width="150">timeHorizon</td>
<td valign="top" width="150">float (time)</td>
<td valign="top">The minimum amount of time for which the agent's velocities
that are computed by the simulation are safe with respect to other agents. The
larger this number, the sooner this agent will respond to the presence of other
agents, but the less freedom the agent has in choosing its velocities. Must be
positive.</td>
</tr>
<tr>
<td valign="top" width="150">velocity</td>
<td valign="top" width="150">RVO::Vector3 (distance/time, distance/time)</td>
<td valign="top">The (current) velocity of the agent.</td>
</tr>
</table>

@page terms Terms and Conditions

<b>RVO2-3D Library</b>

Copyright 2008 University of North Carolina at Chapel Hill

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

 */

#endif /* RVO3D_RVO_H_ */
