/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, CoMotion contributors
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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_AST_RRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_AST_RRT_

#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace ompl
{
    namespace geometric
    {
        /** \brief Adaptive Space-Time RRT.

            AST-RRT first computes a geometric path in configuration space,
            parameterizes it at maximum velocity in a SpaceTimeStateSpace, then
            repeatedly asks user-provided callbacks to find dynamic-obstacle
            conflicts and repair a local space-time window.

            The callbacks keep this planner independent from any particular
            dynamic-obstacle representation while allowing downstream packages
            to reuse their ST-RRT/ST-RRT* validation machinery.
         */
        class ASTRRT : public base::Planner
        {
        public:
            struct Conflict
            {
                double time{0.0};
                unsigned int dynamicObstacleIndex{std::numeric_limits<unsigned int>::max()};
            };

            struct RepairRequest
            {
                const PathGeometric *currentPath{nullptr};
                Conflict conflict;
                double windowStart{0.0};
                double windowEnd{0.0};
                bool localRewire{true};
                unsigned int localBudget{0};
                bool cascadeDynamicObstacles{true};
                std::vector<unsigned int> dynamicObstacleIndices;
            };

            struct RepairResult
            {
                bool success{false};
                PathGeometricPtr path;
            };

            using ConflictCheckerFn = std::function<std::optional<Conflict>(const PathGeometric &)>;
            using LocalRepairFn =
                std::function<RepairResult(const RepairRequest &, const base::PlannerTerminationCondition &)>;
            using InitialPlannerAllocator = std::function<base::PlannerPtr(const base::SpaceInformationPtr &)>;

            explicit ASTRRT(const base::SpaceInformationPtr &si);

            ~ASTRRT() override = default;

            void clear() override;

            void setup() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void getPlannerData(base::PlannerData &data) const override;

            void setGeometricSpaceInformation(const base::SpaceInformationPtr &si);
            const base::SpaceInformationPtr &getGeometricSpaceInformation() const;

            void setInitialPlannerAllocator(InitialPlannerAllocator allocator);

            void setConflictChecker(ConflictCheckerFn checker);
            void setLocalRepairCallback(LocalRepairFn repair);

            void setSimplifyInitialPath(bool simplify);
            bool getSimplifyInitialPath() const;

            void setLocalRewire(bool rewire);
            bool getLocalRewire() const;

            void setLocalBudget(unsigned int budget);
            unsigned int getLocalBudget() const;

            void setInitialWindowSize(double size);
            double getInitialWindowSize() const;

            void setExpansionStep(double step);
            double getExpansionStep() const;

            void setCascadeDynamicObstacles(bool cascade);
            bool getCascadeDynamicObstacles() const;

            void setDynamicObstacleCount(unsigned int count);
            unsigned int getDynamicObstacleCount() const;

            unsigned int getNumConflicts() const;
            unsigned int getNumRepairAttempts() const;
            unsigned int getNumTemporalExpansions() const;
            const PathGeometricPtr &getLastSolutionPath() const;

        private:
            PathGeometricPtr planInitialGeometricPath(const base::PlannerTerminationCondition &ptc);
            PathGeometricPtr timeParameterizePath(const PathGeometric &geometricPath) const;
            std::vector<unsigned int> allDynamicObstacleIndices() const;
            double pathEndTime(const PathGeometric &path) const;

            base::SpaceInformationPtr geometricSi_;
            InitialPlannerAllocator initialPlannerAllocator_;
            ConflictCheckerFn conflictChecker_;
            LocalRepairFn localRepair_;

            bool simplifyInitialPath_{true};
            bool localRewire_{true};
            unsigned int localBudget_{1024};
            double initialWindowSize_{1.0};
            double expansionStep_{1.0};
            bool cascadeDynamicObstacles_{true};
            unsigned int dynamicObstacleCount_{0};

            unsigned int numConflicts_{0};
            unsigned int numRepairAttempts_{0};
            unsigned int numTemporalExpansions_{0};
            PathGeometricPtr lastSolutionPath_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif
