#include "ompl/geometric/planners/rrt/ASTRRT.h"

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/spaces/SpaceTimeStateSpace.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/util/Exception.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace ob = ompl::base;
namespace og = ompl::geometric;

ompl::geometric::ASTRRT::ASTRRT(const ob::SpaceInformationPtr &si)
  : Planner(si, "AdaptiveSpaceTimeRRT")
{
    if (std::dynamic_pointer_cast<ob::SpaceTimeStateSpace>(si->getStateSpace()) == nullptr)
    {
        OMPL_ERROR("%s: State Space needs to be of type SpaceTimeStateSpace.", getName().c_str());
        throw ompl::Exception("Non-SpaceTimeStateSpace");
    }

    specs_.recognizedGoal = ob::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = false;

    Planner::declareParam<bool>("simplify_initial_path", this, &ASTRRT::setSimplifyInitialPath,
                                &ASTRRT::getSimplifyInitialPath, "0,1");
    Planner::declareParam<bool>("local_rewire", this, &ASTRRT::setLocalRewire, &ASTRRT::getLocalRewire, "0,1");
    Planner::declareParam<unsigned int>("local_budget", this, &ASTRRT::setLocalBudget, &ASTRRT::getLocalBudget,
                                        "0:1:1000000");
    Planner::declareParam<double>("initial_window_size", this, &ASTRRT::setInitialWindowSize,
                                  &ASTRRT::getInitialWindowSize, "0.:0.1:100000.");
    Planner::declareParam<double>("expansion_step", this, &ASTRRT::setExpansionStep, &ASTRRT::getExpansionStep,
                                  "0.:0.1:100000.");
    Planner::declareParam<double>("repair_horizon", this, &ASTRRT::setRepairHorizon, &ASTRRT::getRepairHorizon,
                                  "0.:0.1:100000.");
    Planner::declareParam<bool>("cascade_dynamic_obstacles", this, &ASTRRT::setCascadeDynamicObstacles,
                                &ASTRRT::getCascadeDynamicObstacles, "0,1");
}

void ompl::geometric::ASTRRT::clear()
{
    Planner::clear();
    numConflicts_ = 0;
    numRepairAttempts_ = 0;
    numTemporalExpansions_ = 0;
    astTotalTimeNs_ = 0;
    astInitialPathTimeNs_ = 0;
    astTimeParameterizationTimeNs_ = 0;
    astConflictCallbackTimeNs_ = 0;
    astRepairCallbackTimeNs_ = 0;
    astSolutionCommitTimeNs_ = 0;
    lastSolutionPath_.reset();
}

void ompl::geometric::ASTRRT::setup()
{
    Planner::setup();

    if (!geometricSi_)
    {
        auto *spaceTime = si_->getStateSpace()->as<ob::SpaceTimeStateSpace>();
        geometricSi_ = std::make_shared<ob::SpaceInformation>(spaceTime->getSpaceComponent());
        geometricSi_->setStateValidityChecker([](const ob::State *) { return true; });
        OMPL_WARN("%s: No geometric SpaceInformation was supplied; initial RRT-Connect ignores validity.",
                  getName().c_str());
    }
    if (!geometricSi_->isSetup())
        geometricSi_->setup();

    if (!initialPlannerAllocator_)
    {
        initialPlannerAllocator_ = [](const ob::SpaceInformationPtr &geomSi) {
            return std::make_shared<og::RRTConnect>(geomSi);
        };
    }
}

ompl::base::PlannerStatus ompl::geometric::ASTRRT::solve(const ob::PlannerTerminationCondition &ptc)
{
    const auto astSolveStart = std::chrono::steady_clock::now();
    auto finish = [this, astSolveStart](const ob::PlannerStatus &status) {
        astTotalTimeNs_ =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - astSolveStart)
                .count();
        return status;
    };

    checkValidity();
    numConflicts_ = 0;
    numRepairAttempts_ = 0;
    numTemporalExpansions_ = 0;
    astTotalTimeNs_ = 0;
    astInitialPathTimeNs_ = 0;
    astTimeParameterizationTimeNs_ = 0;
    astConflictCallbackTimeNs_ = 0;
    astRepairCallbackTimeNs_ = 0;
    astSolutionCommitTimeNs_ = 0;
    lastSolutionPath_.reset();

    auto sectionStart = std::chrono::steady_clock::now();
    auto geometricPath = planInitialGeometricPath(ptc);
    astInitialPathTimeNs_ +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - sectionStart)
            .count();
    if (!geometricPath)
        return finish(ptc ? ob::PlannerStatus::TIMEOUT : ob::PlannerStatus::UNKNOWN);

    sectionStart = std::chrono::steady_clock::now();
    auto timedPath = timeParameterizePath(*geometricPath);
    astTimeParameterizationTimeNs_ +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - sectionStart)
            .count();
    if (!timedPath)
        return finish(ptc ? ob::PlannerStatus::TIMEOUT : ob::PlannerStatus::UNKNOWN);

    if (!conflictChecker_ || !localRepair_)
    {
        lastSolutionPath_ = timedPath;
        sectionStart = std::chrono::steady_clock::now();
        pdef_->addSolutionPath(lastSolutionPath_, false, 0.0, getName());
        astSolutionCommitTimeNs_ +=
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - sectionStart)
                .count();
        return finish(ob::PlannerStatus::EXACT_SOLUTION);
    }

    double nextConflictCheckTime = 0.0;
    while (!ptc)
    {
        sectionStart = std::chrono::steady_clock::now();
        auto conflict = conflictChecker_(*timedPath, nextConflictCheckTime);
        astConflictCallbackTimeNs_ +=
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - sectionStart)
                .count();
        if (!conflict)
        {
            lastSolutionPath_ = timedPath;
            sectionStart = std::chrono::steady_clock::now();
            pdef_->addSolutionPath(lastSolutionPath_, false, 0.0, getName());
            astSolutionCommitTimeNs_ +=
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - sectionStart)
                    .count();
            return finish(ob::PlannerStatus::EXACT_SOLUTION);
        }

        ++numConflicts_;
        const double pathHorizon = pathEndTime(*timedPath);
        const double horizon = repairHorizon_ > 0.0 ? std::max(pathHorizon, repairHorizon_) : pathHorizon;
        double windowStart = std::max(0.0, conflict->time - initialWindowSize_);
        double windowEnd = std::min(std::max(windowStart, conflict->time + initialWindowSize_), horizon);
        if (windowEnd <= windowStart)
            windowEnd = std::max(windowStart, conflict->time);

        bool repaired = false;
        while (!ptc)
        {
            RepairRequest request;
            request.currentPath = timedPath.get();
            request.conflict = *conflict;
            request.windowStart = windowStart;
            request.windowEnd = windowEnd;
            request.localRewire = localRewire_;
            request.localBudget = localBudget_;
            request.cascadeDynamicObstacles = cascadeDynamicObstacles_;
            if (cascadeDynamicObstacles_ &&
                conflict->dynamicObstacleIndex != std::numeric_limits<unsigned int>::max())
            {
                request.dynamicObstacleIndices.push_back(conflict->dynamicObstacleIndex);
            }
            else
            {
                request.dynamicObstacleIndices = allDynamicObstacleIndices();
            }

            ++numRepairAttempts_;
            sectionStart = std::chrono::steady_clock::now();
            RepairResult result = localRepair_(request, ptc);
            astRepairCallbackTimeNs_ +=
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - sectionStart)
                    .count();
            if (result.success && result.path)
            {
                timedPath = result.path;
                nextConflictCheckTime = std::max(0.0, result.resumeConflictCheckTime - 1e-6);
                repaired = true;
                break;
            }

            if (windowStart <= 0.0 && windowEnd >= horizon)
                return finish(ob::PlannerStatus::TIMEOUT);

            const double previousStart = windowStart;
            const double previousEnd = windowEnd;
            windowStart = std::max(0.0, windowStart - expansionStep_);
            windowEnd = std::min(horizon, windowEnd + expansionStep_);
            if (windowStart == previousStart && windowEnd == previousEnd)
                return finish(ob::PlannerStatus::TIMEOUT);
            ++numTemporalExpansions_;
        }

        if (!repaired)
            return finish(ob::PlannerStatus::TIMEOUT);
    }

    return finish(ob::PlannerStatus::TIMEOUT);
}

void ompl::geometric::ASTRRT::getPlannerData(ob::PlannerData &data) const
{
    Planner::getPlannerData(data);
}

void ompl::geometric::ASTRRT::setGeometricSpaceInformation(const ob::SpaceInformationPtr &si)
{
    geometricSi_ = si;
    setup_ = false;
}

const ompl::base::SpaceInformationPtr &ompl::geometric::ASTRRT::getGeometricSpaceInformation() const
{
    return geometricSi_;
}

void ompl::geometric::ASTRRT::setInitialPlannerAllocator(InitialPlannerAllocator allocator)
{
    initialPlannerAllocator_ = std::move(allocator);
}

void ompl::geometric::ASTRRT::setConflictChecker(ConflictCheckerFn checker)
{
    conflictChecker_ = std::move(checker);
}

void ompl::geometric::ASTRRT::setLocalRepairCallback(LocalRepairFn repair)
{
    localRepair_ = std::move(repair);
}

void ompl::geometric::ASTRRT::setSimplifyInitialPath(bool simplify)
{
    simplifyInitialPath_ = simplify;
}

bool ompl::geometric::ASTRRT::getSimplifyInitialPath() const
{
    return simplifyInitialPath_;
}

void ompl::geometric::ASTRRT::setLocalRewire(bool rewire)
{
    localRewire_ = rewire;
}

bool ompl::geometric::ASTRRT::getLocalRewire() const
{
    return localRewire_;
}

void ompl::geometric::ASTRRT::setLocalBudget(unsigned int budget)
{
    localBudget_ = budget;
}

unsigned int ompl::geometric::ASTRRT::getLocalBudget() const
{
    return localBudget_;
}

void ompl::geometric::ASTRRT::setInitialWindowSize(double size)
{
    initialWindowSize_ = std::max(0.0, size);
}

double ompl::geometric::ASTRRT::getInitialWindowSize() const
{
    return initialWindowSize_;
}

void ompl::geometric::ASTRRT::setExpansionStep(double step)
{
    expansionStep_ = std::max(0.0, step);
}

double ompl::geometric::ASTRRT::getExpansionStep() const
{
    return expansionStep_;
}

void ompl::geometric::ASTRRT::setRepairHorizon(double horizon)
{
    repairHorizon_ = std::max(0.0, horizon);
}

double ompl::geometric::ASTRRT::getRepairHorizon() const
{
    return repairHorizon_;
}

void ompl::geometric::ASTRRT::setCascadeDynamicObstacles(bool cascade)
{
    cascadeDynamicObstacles_ = cascade;
}

bool ompl::geometric::ASTRRT::getCascadeDynamicObstacles() const
{
    return cascadeDynamicObstacles_;
}

void ompl::geometric::ASTRRT::setDynamicObstacleCount(unsigned int count)
{
    dynamicObstacleCount_ = count;
}

unsigned int ompl::geometric::ASTRRT::getDynamicObstacleCount() const
{
    return dynamicObstacleCount_;
}

unsigned int ompl::geometric::ASTRRT::getNumConflicts() const
{
    return numConflicts_;
}

unsigned int ompl::geometric::ASTRRT::getNumRepairAttempts() const
{
    return numRepairAttempts_;
}

unsigned int ompl::geometric::ASTRRT::getNumTemporalExpansions() const
{
    return numTemporalExpansions_;
}

std::uint64_t ompl::geometric::ASTRRT::getAstTotalTimeNs() const
{
    return astTotalTimeNs_;
}

std::uint64_t ompl::geometric::ASTRRT::getAstInitialPathTimeNs() const
{
    return astInitialPathTimeNs_;
}

std::uint64_t ompl::geometric::ASTRRT::getAstTimeParameterizationTimeNs() const
{
    return astTimeParameterizationTimeNs_;
}

std::uint64_t ompl::geometric::ASTRRT::getAstConflictCallbackTimeNs() const
{
    return astConflictCallbackTimeNs_;
}

std::uint64_t ompl::geometric::ASTRRT::getAstRepairCallbackTimeNs() const
{
    return astRepairCallbackTimeNs_;
}

std::uint64_t ompl::geometric::ASTRRT::getAstSolutionCommitTimeNs() const
{
    return astSolutionCommitTimeNs_;
}

std::uint64_t ompl::geometric::ASTRRT::getAstOtherTimeNs() const
{
    const std::uint64_t accounted =
        astInitialPathTimeNs_ +
        astTimeParameterizationTimeNs_ +
        astConflictCallbackTimeNs_ +
        astRepairCallbackTimeNs_ +
        astSolutionCommitTimeNs_;
    return astTotalTimeNs_ > accounted ? astTotalTimeNs_ - accounted : 0;
}

const ompl::geometric::PathGeometricPtr &ompl::geometric::ASTRRT::getLastSolutionPath() const
{
    return lastSolutionPath_;
}

ompl::geometric::PathGeometricPtr
ompl::geometric::ASTRRT::planInitialGeometricPath(const ob::PlannerTerminationCondition &ptc)
{
    auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return nullptr;
    }
    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return nullptr;
    }

    og::SimpleSetup setup(geometricSi_);
    setup.setPlanner(initialPlannerAllocator_(geometricSi_));

    auto *spaceTime = si_->getStateSpace()->as<ob::SpaceTimeStateSpace>();
    while (const ob::State *st = pis_.nextStart())
    {
        ob::ScopedState<> geometricStart(geometricSi_->getStateSpace());
        geometricSi_->getStateSpace()->copyState(geometricStart.get(), st->as<ob::CompoundState>()->components[0]);
        setup.addStartState(geometricStart);
    }

    ob::ScopedState<> sampledGoal(si_->getStateSpace());
    goal->sampleGoal(sampledGoal.get());
    ob::ScopedState<> geometricGoal(geometricSi_->getStateSpace());
    spaceTime->getSpaceComponent()->copyState(geometricGoal.get(),
                                              sampledGoal.get()->as<ob::CompoundState>()->components[0]);
    setup.setGoalState(geometricGoal);

    const auto status = setup.solve(ptc);
    if (status != ob::PlannerStatus::EXACT_SOLUTION)
        return nullptr;

    if (simplifyInitialPath_ && !ptc)
        setup.simplifySolution(ptc);

    return std::make_shared<og::PathGeometric>(setup.getSolutionPath());
}

ompl::geometric::PathGeometricPtr
ompl::geometric::ASTRRT::timeParameterizePath(const PathGeometric &geometricPath) const
{
    if (geometricPath.getStateCount() == 0)
        return nullptr;

    auto *spaceTime = si_->getStateSpace()->as<ob::SpaceTimeStateSpace>();
    const auto spaceComponent = spaceTime->getSpaceComponent();
    const double vMax = std::max(spaceTime->getVMax(), std::numeric_limits<double>::epsilon());

    double time = 0.0;
    if (pdef_->getStartStateCount() > 0)
        time = ob::SpaceTimeStateSpace::getStateTime(pdef_->getStartState(0));

    auto path = std::make_shared<og::PathGeometric>(si_);
    ob::State *state = si_->allocState();
    for (std::size_t i = 0; i < geometricPath.getStateCount(); ++i)
    {
        if (i > 0)
        {
            time += spaceComponent->distance(geometricPath.getState(static_cast<unsigned int>(i - 1)),
                                             geometricPath.getState(static_cast<unsigned int>(i))) /
                    vMax;
        }
        auto *compound = state->as<ob::CompoundState>();
        spaceComponent->copyState(compound->components[0], geometricPath.getState(static_cast<unsigned int>(i)));
        compound->as<ob::TimeStateSpace::StateType>(1)->position = time;
        path->append(state);
    }
    si_->freeState(state);

    return path;
}

std::vector<unsigned int> ompl::geometric::ASTRRT::allDynamicObstacleIndices() const
{
    std::vector<unsigned int> indices;
    indices.reserve(dynamicObstacleCount_);
    for (unsigned int i = 0; i < dynamicObstacleCount_; ++i)
        indices.push_back(i);
    return indices;
}

double ompl::geometric::ASTRRT::pathEndTime(const PathGeometric &path) const
{
    if (path.getStateCount() == 0)
        return 0.0;
    return ob::SpaceTimeStateSpace::getStateTime(path.getState(static_cast<unsigned int>(path.getStateCount() - 1)));
}
