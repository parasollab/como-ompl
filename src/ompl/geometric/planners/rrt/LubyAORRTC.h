/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, MR-OMPL contributors
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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_LUBY_AORRTC_
#define OMPL_GEOMETRIC_PLANNERS_RRT_LUBY_AORRTC_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

#include <cstdint>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#if !defined(_WIN32)
#include <sys/types.h>
#endif

namespace ompl
{
    namespace geometric
    {
        /** \brief Process-parallel AO-RRT-Connect with Luby restart budgets. */
        class LubyAORRTC : public base::Planner
        {
        public:
            explicit LubyAORRTC(const base::SpaceInformationPtr &si);

            ~LubyAORRTC() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void setup() override;

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
                return psk_;
            }

            PathSimplifierPtr &getPathSimplifier()
            {
                return psk_;
            }

            void simplifySolution(const base::PathPtr &p, const base::PlannerTerminationCondition &ptc);

            base::Cost bestCost() const;

            void setWorkerProcesses(unsigned int n);

            unsigned int getWorkerProcesses() const
            {
                return workerProcesses_;
            }

            void setBaseInternalSamples(std::size_t value);

            std::size_t getBaseInternalSamples() const
            {
                return baseInternalSamples_;
            }

            void setBaseInternalVertices(std::size_t value);

            std::size_t getBaseInternalVertices() const
            {
                return baseInternalVertices_;
            }

            void setMaxInternalSamples(std::size_t value)
            {
                setBaseInternalSamples(value);
            }

            void setMaxInternalVertices(std::size_t value)
            {
                setBaseInternalVertices(value);
            }

            static std::uint64_t lubyMultiplier(std::uint64_t index);

        protected:
            struct SharedState;

            struct WorkerProcess
            {
                int workerIndex{-1};
#if !defined(_WIN32)
                pid_t pid{-1};
#endif
                int commandFd{-1};
                int resultFd{-1};
                bool ready{false};
            };

            void freeMemory();

            void initializeBestFromProblemDefinition();

            bool considerSolutionPath(const base::PathPtr &path, const base::PlannerTerminationCondition &ptc);

            base::PlannerStatus solveSerial(const base::PlannerTerminationCondition &ptc);

            base::PlannerStatus solveParallel(const base::PlannerTerminationCondition &ptc);

            void startWorkers();

            void stopWorkers();

            bool workersStarted() const
            {
                return !workers_.empty();
            }

            void resetSharedStateForSolve(double initialBestCost);

            void updateSharedBestCost(double cost);

            double sharedBestCost() const;

            std::uint64_t sharedGeneration() const;

            void setSharedStop(bool stop);

            bool sharedStopRequested() const;

            void workerLoop(int workerIndex, int commandFd, int resultFd);

            void workerSolveLoop(int workerIndex, int resultFd, std::uint64_t solveId, std::uint_fast32_t seed);

            bool deserializeSolutionPath(std::uint64_t stateCount, std::uint64_t stateLength,
                                         const std::vector<unsigned char> &payload, base::PathPtr &path) const;

            bool serializeSolutionPath(const PathGeometric &path, std::vector<unsigned char> &payload,
                                       std::uint64_t &stateCount, std::uint64_t &stateLength) const;

            double maxDistance_{0.0};

            unsigned int workerProcesses_{1};

            std::size_t baseInternalSamples_{10000};

            std::size_t baseInternalVertices_{10000};

            PathSimplifierPtr psk_;

            base::PathPtr bestPath_{nullptr};

            base::Cost bestCost_{std::numeric_limits<double>::infinity()};

            base::OptimizationObjectivePtr opt_;

            base::PlannerStatus lastStatus_{base::PlannerStatus::TIMEOUT};

            SharedState *sharedState_{nullptr};

            std::size_t sharedStateBytes_{0};

            std::vector<WorkerProcess> workers_;

            std::uint64_t nextSolveId_{1};
        };
    }  // namespace geometric
}  // namespace ompl

#endif
