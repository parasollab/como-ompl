/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Queen's University
 *  All rights reserved.
 *********************************************************************/

#ifndef OMPL_GEOMETRIC_PLANNERS_EST_AOBIEST_
#define OMPL_GEOMETRIC_PLANNERS_EST_AOBIEST_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/geometric/planners/est/AOXBiEST.h"

#include <cstddef>
#include <limits>
#include <memory>

namespace ompl
{
    namespace geometric
    {
        /** \brief Anytime asymptotically improving bidirectional EST. */
        class AOBiEST : public base::Planner
        {
        public:
            AOBiEST(const base::SpaceInformationPtr &si);

            ~AOBiEST() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void setup() override;

            void reset(bool solvedProblem);

            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            double getRange() const
            {
                return maxDistance_;
            }

            const PathSimplifierPtr &getPathSimplifier() const
            {
                return pathSimplifier_;
            }

            PathSimplifierPtr &getPathSimplifier()
            {
                return pathSimplifier_;
            }

            void simplifySolution(const base::PathPtr &path, const base::PlannerTerminationCondition &ptc);

            base::Cost bestCost() const;

            void setMaxInternalSamples(std::size_t value);

            void setMaxInternalVertices(std::size_t value);

        protected:
            void freeMemory();

            void reportIntermediateSolution(const base::PathPtr &path) const;

            bool updateBestPathIfImproved(const base::PathPtr &path, const char *source);

            std::shared_ptr<AOXBiEST> aoxPlanner_;
            PathSimplifierPtr pathSimplifier_;
            base::PathPtr bestPath_{nullptr};
            base::Cost bestCost_{std::numeric_limits<double>::infinity()};
            base::OptimizationObjectivePtr opt_;

            double maxDistance_{0.0};
            std::size_t maxInternalSamples_{0};
            std::size_t maxInternalVertices_{0};
            base::PlannerStatus solveStatus_{base::PlannerStatus::TIMEOUT};
        };
    }  // namespace geometric
}  // namespace ompl

#endif
