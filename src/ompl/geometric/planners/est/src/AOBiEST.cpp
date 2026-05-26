/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Queen's University
 *  All rights reserved.
 *********************************************************************/

#include "ompl/geometric/planners/est/AOBiEST.h"

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"

#include <limits>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    constexpr double IMPROVEMENT_EPS = 1e-9;
}

og::AOBiEST::AOBiEST(const ob::SpaceInformationPtr &si) : ob::Planner(si, "AOBiEST")
{
    specs_.approximateSolutions = true;
    specs_.canReportIntermediateSolutions = true;
    specs_.directed = true;
}

og::AOBiEST::~AOBiEST()
{
    freeMemory();
}

void og::AOBiEST::clear()
{
    setup_ = false;
    Planner::clear();
    bestPath_.reset();
    bestCost_ = ob::Cost(std::numeric_limits<double>::infinity());
    solveStatus_ = ob::PlannerStatus::TIMEOUT;
    freeMemory();
}

void og::AOBiEST::setup()
{
    Planner::setup();

    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            opt_ = std::make_shared<ob::PathLengthOptimizationObjective>(si_);
            pdef_->setOptimizationObjective(opt_);
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
        return;
    }

    aoxPlanner_ = std::make_shared<AOXBiEST>(si_);
    aoxPlanner_->setProblemDefinition(pdef_);
    aoxPlanner_->setName("AOXBiEST");
    aoxPlanner_->setRange(getRange());
    if (maxInternalSamples_ > 0)
        aoxPlanner_->setMaxInternalSamples(maxInternalSamples_);
    if (maxInternalVertices_ > 0)
        aoxPlanner_->setMaxInternalVertices(maxInternalVertices_);
    aoxPlanner_->setup();

    pathSimplifier_ = std::make_shared<PathSimplifier>(si_);
    solveStatus_ = ob::PlannerStatus::TIMEOUT;
}

void og::AOBiEST::reset(bool solvedProblem)
{
    if (!aoxPlanner_)
        return;
    aoxPlanner_->clearQuery();
    aoxPlanner_->setFoundPath(nullptr);
    aoxPlanner_->reset(solvedProblem);
    solveStatus_ = ob::PlannerStatus::TIMEOUT;
}

void og::AOBiEST::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    if (aoxPlanner_)
        aoxPlanner_->setProblemDefinition(pdef);
}

void og::AOBiEST::freeMemory()
{
    if (aoxPlanner_)
        aoxPlanner_->clear();
}

ob::Cost og::AOBiEST::bestCost() const
{
    return bestCost_;
}

void og::AOBiEST::setMaxInternalSamples(std::size_t value)
{
    if (value == 0)
        return;
    maxInternalSamples_ = value;
    if (aoxPlanner_)
        aoxPlanner_->setMaxInternalSamples(value);
}

void og::AOBiEST::setMaxInternalVertices(std::size_t value)
{
    if (value == 0)
        return;
    maxInternalVertices_ = value;
    if (aoxPlanner_)
        aoxPlanner_->setMaxInternalVertices(value);
}

void og::AOBiEST::simplifySolution(const ob::PathPtr &path, const ob::PlannerTerminationCondition &ptc)
{
    if (!path)
    {
        OMPL_WARN("%s: No solution to simplify", getName().c_str());
        return;
    }
    auto &geometricPath = static_cast<PathGeometric &>(*path);
    pathSimplifier_->simplify(geometricPath, ptc);
}

void og::AOBiEST::reportIntermediateSolution(const ob::PathPtr &path) const
{
    const ob::ReportIntermediateSolutionFn callback = pdef_->getIntermediateSolutionCallback();
    if (!callback || !path)
        return;

    const auto &geometricPath = static_cast<const PathGeometric &>(*path);
    std::vector<const ob::State *> states;
    states.reserve(geometricPath.getStateCount());
    for (std::size_t i = 0; i < geometricPath.getStateCount(); ++i)
        states.push_back(geometricPath.getState(i));
    callback(this, states, ob::Cost(geometricPath.length()));
}

bool og::AOBiEST::updateBestPathIfImproved(const ob::PathPtr &path, const char *source)
{
    if (!path)
        return false;
    const auto &geometricPath = static_cast<const PathGeometric &>(*path);
    const double pathCost = geometricPath.length();
    if (pathCost + IMPROVEMENT_EPS >= bestCost_.value())
        return false;

    bestPath_ = path;
    bestCost_ = ob::Cost(pathCost);
    pdef_->addSolutionPath(bestPath_, false, 0.0, getName());
    reportIntermediateSolution(bestPath_);
    OMPL_INFORM("%s: %s solution cost %.5f", getName().c_str(), source, pathCost);
    return true;
}

ob::PlannerStatus og::AOBiEST::solve(const ob::PlannerTerminationCondition &ptc)
{
    checkValidity();
    if (!aoxPlanner_)
        setup();
    if (!aoxPlanner_)
        return ob::PlannerStatus::TIMEOUT;

    do
    {
        if (std::isfinite(bestCost_.value()) || pdef_->hasExactSolution())
        {
            if (!std::isfinite(bestCost_.value()) && pdef_->getSolutionPath())
            {
                bestPath_ = pdef_->getSolutionPath();
                bestCost_ = ob::Cost(bestPath_->length());
            }

            aoxPlanner_->setPathCost(bestCost_.value());
            solveStatus_ = aoxPlanner_->solve(ptc);
            if (solveStatus_ == ob::PlannerStatus::EXACT_SOLUTION)
            {
                const ob::PathPtr path = aoxPlanner_->getFoundPath();
                if (updateBestPathIfImproved(path, "AOX search"))
                {
                    simplifySolution(bestPath_, ptc);
                    updateBestPathIfImproved(bestPath_, "AOX simplified");
                }
            }
        }
        else
        {
            solveStatus_ = aoxPlanner_->solve(ptc);
            if (solveStatus_ == ob::PlannerStatus::EXACT_SOLUTION)
            {
                bestPath_ = aoxPlanner_->getFoundPath();
                if (!bestPath_)
                    bestPath_ = pdef_->getSolutionPath();
                updateBestPathIfImproved(bestPath_, "initial");
                simplifySolution(bestPath_, ptc);
                updateBestPathIfImproved(bestPath_, "initial simplified");
            }
        }

        if (solveStatus_ == ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE ||
            solveStatus_ == ob::PlannerStatus::INVALID_START || solveStatus_ == ob::PlannerStatus::INVALID_GOAL)
            return solveStatus_;

        if (solveStatus_ == ob::PlannerStatus::EXACT_SOLUTION)
            reset(true);
        else if (aoxPlanner_->internalResetCondition())
        {
            OMPL_INFORM("%s: Restarted before finding a solution", getName().c_str());
            reset(false);
        }

        if (bestCost_.value() == 0.0)
        {
            OMPL_ERROR("%s: Zero-length path found. May have a common start/goal.", getName().c_str());
            return ob::PlannerStatus::INVALID_GOAL;
        }
    } while (!ptc);

    return bestPath_ ? ob::PlannerStatus::EXACT_SOLUTION : ob::PlannerStatus::TIMEOUT;
}

void og::AOBiEST::getPlannerData(ob::PlannerData &data) const
{
    if (aoxPlanner_)
        aoxPlanner_->getPlannerData(data);
}
