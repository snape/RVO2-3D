/*
 * Agent.h
 * RVO2-3D Library
 *
 * Copyright (c) 2008-2013 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the authors <geom@cs.unc.edu> or the Office of
 * Technology Development at the University of North Carolina at Chapel Hill
 * <otd@unc.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * North Carolina at Chapel Hill. The software program and documentation are
 * supplied "as is," without any accompanying services from the University of
 * North Carolina at Chapel Hill or the authors. The University of North
 * Carolina at Chapel Hill and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 * AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
 * CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 * DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 * STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
 * AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
 * AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
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
 * <http://gamma.cs.unc.edu/RVO2/>
 */

/**
 * \file    Agent.h
 * \brief   Contains the Agent class.
 */
#ifndef RVO_AGENT_H_
#define RVO_AGENT_H_

#include "API.h"

#include <cstddef>
#include <utility>
#include <vector>

#include "RVOSimulator.h"
#include "Vector3.h"

namespace RVO {
	/**
	 * \brief   Defines an agent in the simulation.
	 */
	class Agent {
	private:
		/**
		 * \brief   Constructs an agent instance.
		 * \param   sim  The simulator instance.
		 */
		explicit Agent(RVOSimulator *sim);

		/**
		 * \brief   Computes the neighbors of this agent.
		 */
		void computeNeighbors();

		/**
		 * \brief   Computes the new velocity of this agent.
		 */
		void computeNewVelocity();

		/**
		 * \brief   Inserts an agent neighbor into the set of neighbors of this agent.
		 * \param   agent    A pointer to the agent to be inserted.
		 * \param   rangeSq  The squared range around this agent.
		 */
		void insertAgentNeighbor(const Agent *agent, float &rangeSq);

		/**
		 * \brief   Updates the three-dimensional position and three-dimensional velocity of this agent.
		 */
		void update();

		Vector3 newVelocity_;
		Vector3 position_;
		Vector3 prefVelocity_;
		Vector3 velocity_;
		RVOSimulator *sim_;
		size_t id_;
		size_t maxNeighbors_;
		float maxSpeed_;
		float neighborDist_;
		float radius_;
		float timeHorizon_;
		std::vector<std::pair<float, const Agent *> > agentNeighbors_;
		std::vector<Plane> orcaPlanes_;

		friend class KdTree;
		friend class RVOSimulator;
	};
}

#endif /* RVO_AGENT_H_ */
