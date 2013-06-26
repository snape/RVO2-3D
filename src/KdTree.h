/*
 * KdTree.h
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
 * \file    KdTree.h
 * \brief   Contains the KdTree class.
 */
#ifndef RVO_KD_TREE_H_
#define RVO_KD_TREE_H_

#include "API.h"

#include <cstddef>
#include <vector>

#include "Vector3.h"

namespace RVO {
	class Agent;
	class RVOSimulator;

	/**
	 * \brief   Defines <i>k</i>d-trees for agents in the simulation.
	 */
	class KdTree {
	private:
		/**
		 * \brief   Defines an agent <i>k</i>d-tree node.
		 */
		class AgentTreeNode {
		public:
			/**
			 * \brief   The beginning node number.
			 */
			size_t begin;

			/**
			 * \brief   The ending node number.
			 */
			size_t end;

			/**
			 * \brief   The left node number.
			 */
			size_t left;

			/**
			 * \brief   The right node number.
			 */
			size_t right;

			/**
			 * \brief   The maximum coordinates.
			 */
			Vector3 maxCoord;

			/**
			 * \brief   The minimum coordinates.
			 */
			Vector3 minCoord;
		};

		/**
		 * \brief   Constructs a <i>k</i>d-tree instance.
		 * \param   sim  The simulator instance.
		 */
		explicit KdTree(RVOSimulator *sim);

		/**
		 * \brief   Builds an agent <i>k</i>d-tree.
		 */
		void buildAgentTree();

		void buildAgentTreeRecursive(size_t begin, size_t end, size_t node);

		/**
		 * \brief   Computes the agent neighbors of the specified agent.
		 * \param   agent    A pointer to the agent for which agent neighbors are to be computed.
		 * \param   rangeSq  The squared range around the agent.
		 */
		void computeAgentNeighbors(Agent *agent, float rangeSq) const;

		void queryAgentTreeRecursive(Agent *agent, float &rangeSq, size_t node) const;

		std::vector<Agent *> agents_;
		std::vector<AgentTreeNode> agentTree_;
		RVOSimulator *sim_;
		
		friend class Agent;
		friend class RVOSimulator;
	};
}

#endif /* RVO_KD_TREE_H_ */
