/*
 * RVOSimulator.cpp
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

#include "RVOSimulator.h"

#ifdef _OPENMP
#include <omp.h>
#endif

#include "Agent.h"
#include "KdTree.h"

namespace RVO {
	RVOSimulator::RVOSimulator() : defaultAgent_(NULL), kdTree_(NULL), globalTime_(0.0f), timeStep_(0.0f)
	{
		kdTree_ = new KdTree(this);
	}

	RVOSimulator::RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, const Vector3 &velocity) : defaultAgent_(NULL), kdTree_(NULL), globalTime_(0.0f), timeStep_(timeStep)
	{
		kdTree_ = new KdTree(this);
		defaultAgent_ = new Agent(this);

		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->velocity_ = velocity;
	}

	RVOSimulator::~RVOSimulator()
	{
		if (defaultAgent_ != NULL) {
			delete defaultAgent_;
		}

		for (size_t i = 0; i < agents_.size(); ++i) {
			delete agents_[i];
		}

		if (kdTree_ != NULL) {
			delete kdTree_;
		}
	}

	size_t RVOSimulator::getAgentNumAgentNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->agentNeighbors_.size();
	}

	size_t RVOSimulator::getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
	}

	size_t RVOSimulator::getAgentNumORCAPlanes(size_t agentNo) const
	{
		return agents_[agentNo]->orcaPlanes_.size();
	}

	const Plane &RVOSimulator::getAgentORCAPlane(size_t agentNo, size_t planeNo) const
	{
		return agents_[agentNo]->orcaPlanes_[planeNo];
	}

	void RVOSimulator::removeAgent(size_t agentNo)
	{
		delete agents_[agentNo];
		agents_[agentNo] = agents_.back();
		agents_.pop_back();
	}

	size_t RVOSimulator::addAgent(const Vector3 &position)
	{
		if (defaultAgent_ == NULL) {
			return RVO_ERROR;
		}

		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
		agent->maxSpeed_ = defaultAgent_->maxSpeed_;
		agent->neighborDist_ = defaultAgent_->neighborDist_;
		agent->radius_ = defaultAgent_->radius_;
		agent->timeHorizon_ = defaultAgent_->timeHorizon_;
		agent->velocity_ = defaultAgent_->velocity_;

		agent->id_ = agents_.size();

		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	size_t RVOSimulator::addAgent(const Vector3 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, const Vector3 &velocity)
	{
		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = maxNeighbors;
		agent->maxSpeed_ = maxSpeed;
		agent->neighborDist_ = neighborDist;
		agent->radius_ = radius;
		agent->timeHorizon_ = timeHorizon;
		agent->velocity_ = velocity;

		agent->id_ = agents_.size();

		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	void RVOSimulator::doStep()
	{
		kdTree_->buildAgentTree();

#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->computeNeighbors();
			agents_[i]->computeNewVelocity();
		}

#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->update();
		}

		globalTime_ += timeStep_;
	}

	size_t RVOSimulator::getAgentMaxNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->maxNeighbors_;
	}

	float RVOSimulator::getAgentMaxSpeed(size_t agentNo) const
	{
		return agents_[agentNo]->maxSpeed_;
	}

	float RVOSimulator::getAgentNeighborDist(size_t agentNo) const
	{
		return agents_[agentNo]->neighborDist_;
	}

	const Vector3 &RVOSimulator::getAgentPosition(size_t agentNo) const
	{
		return agents_[agentNo]->position_;
	}

	const Vector3 &RVOSimulator::getAgentPrefVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->prefVelocity_;
	}

	float RVOSimulator::getAgentRadius(size_t agentNo) const
	{
		return agents_[agentNo]->radius_;
	}

	float RVOSimulator::getAgentTimeHorizon(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizon_;
	}

	const Vector3 &RVOSimulator::getAgentVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->velocity_;
	}

	float RVOSimulator::getGlobalTime() const
	{
		return globalTime_;
	}

	size_t RVOSimulator::getNumAgents() const
	{
		return agents_.size();
	}

	float RVOSimulator::getTimeStep() const
	{
		return timeStep_;
	}

	void RVOSimulator::setAgentDefaults(float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, const Vector3 &velocity)
	{
		if (defaultAgent_ == NULL) {
			defaultAgent_ = new Agent(this);
		}

		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->velocity_ = velocity;
	}

	void RVOSimulator::setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
	{
		agents_[agentNo]->maxNeighbors_ = maxNeighbors;
	}

	void RVOSimulator::setAgentMaxSpeed(size_t agentNo, float maxSpeed)
	{
		agents_[agentNo]->maxSpeed_ = maxSpeed;
	}

	void RVOSimulator::setAgentNeighborDist(size_t agentNo, float neighborDist)
	{
		agents_[agentNo]->neighborDist_ = neighborDist;
	}

	void RVOSimulator::setAgentPosition(size_t agentNo, const Vector3 &position)
	{
		agents_[agentNo]->position_ = position;
	}

	void RVOSimulator::setAgentPrefVelocity(size_t agentNo, const Vector3 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity_ = prefVelocity;
	}

	void RVOSimulator::setAgentRadius(size_t agentNo, float radius)
	{
		agents_[agentNo]->radius_ = radius;
	}

	void RVOSimulator::setAgentTimeHorizon(size_t agentNo, float timeHorizon)
	{
		agents_[agentNo]->timeHorizon_ = timeHorizon;
	}
	
	void RVOSimulator::setAgentVelocity(size_t agentNo, const Vector3 &velocity)
	{
		agents_[agentNo]->velocity_ = velocity;
	}
	
	void RVOSimulator::setTimeStep(float timeStep)
	{
		timeStep_ = timeStep;
	}
}
