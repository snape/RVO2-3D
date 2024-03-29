/*
 * RVOSimulator.cc
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

#include "RVOSimulator.h"

#include <utility>

#ifdef _OPENMP
#include <omp.h>
#endif /* _OPENMP */

#include "Agent.h"
#include "KdTree.h"
#include "Plane.h"

namespace RVO {
RVOSimulator::RVOSimulator()
    : defaultAgent_(NULL),
      kdTree_(new KdTree(this)),
      globalTime_(0.0F),
      timeStep_(0.0F) {}

RVOSimulator::RVOSimulator(float timeStep, float neighborDist,
                           std::size_t maxNeighbors, float timeHorizon,
                           float radius, float maxSpeed,
                           const Vector3 &velocity)
    : defaultAgent_(new Agent(this)),
      kdTree_(new KdTree(this)),
      globalTime_(0.0F),
      timeStep_(timeStep) {
  defaultAgent_->maxNeighbors_ = maxNeighbors;
  defaultAgent_->maxSpeed_ = maxSpeed;
  defaultAgent_->neighborDist_ = neighborDist;
  defaultAgent_->radius_ = radius;
  defaultAgent_->timeHorizon_ = timeHorizon;
  defaultAgent_->velocity_ = velocity;
}

RVOSimulator::~RVOSimulator() {
  delete defaultAgent_;
  delete kdTree_;

  for (std::size_t i = 0U; i < agents_.size(); ++i) {
    delete agents_[i];
  }
}

std::size_t RVOSimulator::getAgentNumAgentNeighbors(std::size_t agentNo) const {
  return agents_[agentNo]->agentNeighbors_.size();
}

std::size_t RVOSimulator::getAgentAgentNeighbor(std::size_t agentNo,
                                                std::size_t neighborNo) const {
  return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
}

std::size_t RVOSimulator::getAgentNumORCAPlanes(std::size_t agentNo) const {
  return agents_[agentNo]->orcaPlanes_.size();
}

const Plane &RVOSimulator::getAgentORCAPlane(std::size_t agentNo,
                                             std::size_t planeNo) const {
  return agents_[agentNo]->orcaPlanes_[planeNo];
}

void RVOSimulator::removeAgent(std::size_t agentNo) {
  delete agents_[agentNo];
  agents_[agentNo] = agents_.back();
  agents_.pop_back();
}

std::size_t RVOSimulator::addAgent(const Vector3 &position) {
  if (defaultAgent_ == NULL) {
    return RVO3D_ERROR;
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

  return agents_.size() - 1U;
}

std::size_t RVOSimulator::addAgent(const Vector3 &position, float neighborDist,
                                   std::size_t maxNeighbors, float timeHorizon,
                                   float radius, float maxSpeed,
                                   const Vector3 &velocity) {
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

  return agents_.size() - 1U;
}

void RVOSimulator::doStep() {
  kdTree_->buildAgentTree();

#ifdef _OPENMP
#pragma omp parallel for
#endif /* _OPENMP */
  for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
    agents_[i]->computeNeighbors();
    agents_[i]->computeNewVelocity();
  }

#ifdef _OPENMP
#pragma omp parallel for
#endif /* _OPENMP */
  for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
    agents_[i]->update();
  }

  globalTime_ += timeStep_;
}

std::size_t RVOSimulator::getAgentMaxNeighbors(std::size_t agentNo) const {
  return agents_[agentNo]->maxNeighbors_;
}

float RVOSimulator::getAgentMaxSpeed(std::size_t agentNo) const {
  return agents_[agentNo]->maxSpeed_;
}

float RVOSimulator::getAgentNeighborDist(std::size_t agentNo) const {
  return agents_[agentNo]->neighborDist_;
}

const Vector3 &RVOSimulator::getAgentPosition(std::size_t agentNo) const {
  return agents_[agentNo]->position_;
}

const Vector3 &RVOSimulator::getAgentPrefVelocity(std::size_t agentNo) const {
  return agents_[agentNo]->prefVelocity_;
}

float RVOSimulator::getAgentRadius(std::size_t agentNo) const {
  return agents_[agentNo]->radius_;
}

float RVOSimulator::getAgentTimeHorizon(std::size_t agentNo) const {
  return agents_[agentNo]->timeHorizon_;
}

const Vector3 &RVOSimulator::getAgentVelocity(std::size_t agentNo) const {
  return agents_[agentNo]->velocity_;
}

void RVOSimulator::setAgentDefaults(float neighborDist,
                                    std::size_t maxNeighbors, float timeHorizon,
                                    float radius, float maxSpeed,
                                    const Vector3 &velocity) {
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

void RVOSimulator::setAgentMaxNeighbors(std::size_t agentNo,
                                        std::size_t maxNeighbors) {
  agents_[agentNo]->maxNeighbors_ = maxNeighbors;
}

void RVOSimulator::setAgentMaxSpeed(std::size_t agentNo, float maxSpeed) {
  agents_[agentNo]->maxSpeed_ = maxSpeed;
}

void RVOSimulator::setAgentNeighborDist(std::size_t agentNo,
                                        float neighborDist) {
  agents_[agentNo]->neighborDist_ = neighborDist;
}

void RVOSimulator::setAgentPosition(std::size_t agentNo,
                                    const Vector3 &position) {
  agents_[agentNo]->position_ = position;
}

void RVOSimulator::setAgentPrefVelocity(std::size_t agentNo,
                                        const Vector3 &prefVelocity) {
  agents_[agentNo]->prefVelocity_ = prefVelocity;
}

void RVOSimulator::setAgentRadius(std::size_t agentNo, float radius) {
  agents_[agentNo]->radius_ = radius;
}

void RVOSimulator::setAgentTimeHorizon(std::size_t agentNo, float timeHorizon) {
  agents_[agentNo]->timeHorizon_ = timeHorizon;
}

void RVOSimulator::setAgentVelocity(std::size_t agentNo,
                                    const Vector3 &velocity) {
  agents_[agentNo]->velocity_ = velocity;
}
} /* namespace RVO */
