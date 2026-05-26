/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Queen's University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Queen's University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef OMPL_GEOMETRIC_PLANNERS_EST_AOXBIEST_
#define OMPL_GEOMETRIC_PLANNERS_EST_AOXBIEST_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/Path.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

namespace ompl
{
    namespace geometric
    {
        /** \brief Cost-augmented bidirectional EST inner planner for AOBiEST. */
        class AOXBiEST : public base::Planner
        {
        public:
            AOXBiEST(const base::SpaceInformationPtr &si);

            ~AOXBiEST() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void setup() override;

            void clear() override;

            void reset(bool solvedProblem);

            void setPathCost(double cost);

            void setFoundPath(const base::PathPtr &path)
            {
                foundPath_ = path;
            }

            base::PathPtr getFoundPath() const
            {
                return foundPath_;
            }

            bool internalResetCondition() const;

            void setRange(double distance)
            {
                maxDistance_ = distance;
                nbrhoodRadius_ = maxDistance_ / 3.0;
            }

            double getRange() const
            {
                return maxDistance_;
            }

            void setMaxInternalSamples(std::size_t value);

            void setMaxInternalVertices(std::size_t value);

            void getPlannerData(base::PlannerData &data) const override;

        protected:
            class Motion
            {
            public:
                Motion() = default;

                explicit Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                base::State *state{nullptr};
                Motion *parent{nullptr};
                PDF<Motion *>::Element *element{nullptr};
                const base::State *root{nullptr};
                double cost{0.0};
            };

            using TreeNN = std::shared_ptr<NearestNeighbors<Motion *>>;

            struct TreeData
            {
                TreeNN spatial;
                TreeNN costAugmented;
                std::vector<Motion *> motions;
                PDF<Motion *> pdf;
                bool start{true};
            };

            void freeMemory();

            void clearTree(TreeData &tree);

            void allocateNearestNeighbors();

            void configureDistanceFunctions();

            double spatialDistanceFunction(const Motion *a, const Motion *b) const;

            double costAugmentedDistanceFunction(const Motion *a, const Motion *b) const;

            bool hasFiniteCostBound() const;

            Motion *selectMotion(TreeData &tree);

            Motion *addMotion(TreeData &tree, Motion *motion, const std::vector<Motion *> &neighbors);

            Motion *tryAddSample(TreeData &tree, TreeData &otherTree, Motion *existing, const base::State *sample);

            Motion *selectCostAugmentedParent(TreeData &tree, TreeData &otherTree, const base::State *sample,
                                              double targetCost) const;

            bool isMotionValidFromTree(const TreeData &tree, const Motion *parent, const base::State *child) const;

            double lowerBoundToRoots(const TreeData &tree, const base::State *state) const;

            double pathCostThroughConnection(const Motion *startMotion, const Motion *goalMotion) const;

            base::PathPtr constructPath(Motion *startMotion, Motion *goalMotion) const;

            bool endpointsMatch(const PathGeometric &path) const;

            bool tryConnect(TreeData &tree, TreeData &otherTree, Motion *motion, const base::GoalSampleableRegion *goal);

            TreeData startTree_;
            TreeData goalTree_;

            base::ValidStateSamplerPtr sampler_;
            base::OptimizationObjectivePtr opt_;
            base::PathPtr foundPath_{nullptr};

            base::State *startState_{nullptr};
            base::State *goalState_{nullptr};

            double maxDistance_{0.0};
            double nbrhoodRadius_{0.0};
            base::Cost bestCost_{std::numeric_limits<double>::infinity()};

            std::size_t sampleAttempts_{0};
            std::size_t maxInternalSamples_{10000000};
            std::size_t maxInternalSamplesIncrement_{10000000};
            std::size_t maxInternalVertices_{10000};
            std::size_t maxInternalVerticesIncrement_{10000};

            bool startTreeTurn_{true};
            RNG rng_;
            std::pair<base::State *, base::State *> connectionPoint_{nullptr, nullptr};
        };
    }  // namespace geometric
}  // namespace ompl

#endif
