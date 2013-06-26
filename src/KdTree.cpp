/*
 * KdTree.cpp
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

#include "KdTree.h"

#include <algorithm>

#include "Agent.h"
#include "Definitions.h"
#include "RVOSimulator.h"

namespace RVO {
	const size_t RVO_MAX_LEAF_SIZE = 10;

	KdTree::KdTree(RVOSimulator *sim) : sim_(sim) { }

	void KdTree::buildAgentTree()
	{
		agents_ = sim_->agents_;

		if (!agents_.empty()) {
			agentTree_.resize(2 * agents_.size() - 1);
			buildAgentTreeRecursive(0, agents_.size(), 0);
		}
	}

	void KdTree::buildAgentTreeRecursive(size_t begin, size_t end, size_t node)
	{
		agentTree_[node].begin = begin;
		agentTree_[node].end = end;
		agentTree_[node].minCoord = agents_[begin]->position_;
		agentTree_[node].maxCoord = agents_[begin]->position_;

		for (size_t i = begin + 1; i < end; ++i) {
			agentTree_[node].maxCoord[0] = std::max(agentTree_[node].maxCoord[0], agents_[i]->position_.x());
			agentTree_[node].minCoord[0] = std::min(agentTree_[node].minCoord[0], agents_[i]->position_.x());
			agentTree_[node].maxCoord[1] = std::max(agentTree_[node].maxCoord[1], agents_[i]->position_.y());
			agentTree_[node].minCoord[1] = std::min(agentTree_[node].minCoord[1], agents_[i]->position_.y());
			agentTree_[node].maxCoord[2] = std::max(agentTree_[node].maxCoord[2], agents_[i]->position_.z());
			agentTree_[node].minCoord[2] = std::min(agentTree_[node].minCoord[2], agents_[i]->position_.z());
		}

		if (end - begin > RVO_MAX_LEAF_SIZE) {
			/* No leaf node. */
			size_t coord;

			if (agentTree_[node].maxCoord[0] - agentTree_[node].minCoord[0] > agentTree_[node].maxCoord[1] - agentTree_[node].minCoord[1] && agentTree_[node].maxCoord[0] - agentTree_[node].minCoord[0] > agentTree_[node].maxCoord[2] - agentTree_[node].minCoord[2]) {
				coord = 0;
			}
			else if (agentTree_[node].maxCoord[1] - agentTree_[node].minCoord[1] > agentTree_[node].maxCoord[2] - agentTree_[node].minCoord[2]) {
				coord = 1;
			}
			else {
				coord = 2;
			}

			const float splitValue = 0.5f * (agentTree_[node].maxCoord[coord] + agentTree_[node].minCoord[coord]);

			size_t left = begin;

			size_t right = end;

			while (left < right) {
				while (left < right && agents_[left]->position_[coord] < splitValue) {
					++left;
				}

				while (right > left && agents_[right - 1]->position_[coord] >= splitValue) {
					--right;
				}

				if (left < right) {
					std::swap(agents_[left], agents_[right - 1]);
					++left;
					--right;
				}
			}

			size_t leftSize = left - begin;

			if (leftSize == 0) {
				++leftSize;
				++left;
				++right;
			}

			agentTree_[node].left = node + 1;
			agentTree_[node].right = node + 2 * leftSize;

			buildAgentTreeRecursive(begin, left, agentTree_[node].left);
			buildAgentTreeRecursive(left, end, agentTree_[node].right);
		}
	}

	void KdTree::computeAgentNeighbors(Agent *agent, float rangeSq) const
	{
		queryAgentTreeRecursive(agent, rangeSq, 0);
	}

	void KdTree::queryAgentTreeRecursive(Agent *agent, float &rangeSq, size_t node) const
	{
		if (agentTree_[node].end - agentTree_[node].begin <= RVO_MAX_LEAF_SIZE) {
			for (size_t i = agentTree_[node].begin; i < agentTree_[node].end; ++i) {
				agent->insertAgentNeighbor(agents_[i], rangeSq);
			}
		}
		else {
			const float distSqLeft = sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minCoord[0] - agent->position_.x())) + sqr(std::max(0.0f, agent->position_.x() - agentTree_[agentTree_[node].left].maxCoord[0])) + sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minCoord[1] - agent->position_.y())) + sqr(std::max(0.0f, agent->position_.y() - agentTree_[agentTree_[node].left].maxCoord[1])) + sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minCoord[2] - agent->position_.z())) + sqr(std::max(0.0f, agent->position_.z() - agentTree_[agentTree_[node].left].maxCoord[2]));

			const float distSqRight = sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minCoord[0] - agent->position_.x())) + sqr(std::max(0.0f, agent->position_.x() - agentTree_[agentTree_[node].right].maxCoord[0])) + sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minCoord[1] - agent->position_.y())) + sqr(std::max(0.0f, agent->position_.y() - agentTree_[agentTree_[node].right].maxCoord[1])) + sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minCoord[2] - agent->position_.z())) + sqr(std::max(0.0f, agent->position_.z() - agentTree_[agentTree_[node].right].maxCoord[2]));

			if (distSqLeft < distSqRight) {
				if (distSqLeft < rangeSq) {
					queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);

					if (distSqRight < rangeSq) {
						queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);
					}
				}
			}
			else {
				if (distSqRight < rangeSq) {
					queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);
					
					if (distSqLeft < rangeSq) {
						queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);
					}
				}
			}
		}
	}
}
