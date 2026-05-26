/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Queen's University
 *  All rights reserved.
 *********************************************************************/

#include "ompl/geometric/planners/est/AOXBiEST.h"

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    constexpr double COST_EPS = 1e-9;
    constexpr std::size_t MAX_AOX_PARENT_CANDIDATES = 128;
}

og::AOXBiEST::AOXBiEST(const ob::SpaceInformationPtr &si) : ob::Planner(si, "AOXBiEST")
{
    specs_.recognizedGoal = ob::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &AOXBiEST::setRange, &AOXBiEST::getRange, "0.:1.:10000.");
}

og::AOXBiEST::~AOXBiEST()
{
    freeMemory();
}

void og::AOXBiEST::setup()
{
    Planner::setup();

    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    nbrhoodRadius_ = maxDistance_ / 3.0;

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

    allocateNearestNeighbors();
    reset(true);
}

void og::AOXBiEST::clear()
{
    Planner::clear();
    sampler_.reset();
    foundPath_.reset();
    bestCost_ = ob::Cost(std::numeric_limits<double>::infinity());
    sampleAttempts_ = 0;
    startState_ = nullptr;
    goalState_ = nullptr;
    connectionPoint_ = std::make_pair<ob::State *, ob::State *>(nullptr, nullptr);
    freeMemory();
}

void og::AOXBiEST::reset(bool solvedProblem)
{
    freeMemory();
    allocateNearestNeighbors();
    configureDistanceFunctions();
    foundPath_.reset();
    startTreeTurn_ = true;
    sampleAttempts_ = 0;
    startState_ = nullptr;
    goalState_ = nullptr;
    connectionPoint_ = std::make_pair<ob::State *, ob::State *>(nullptr, nullptr);

    if (!solvedProblem)
    {
        maxInternalSamples_ += maxInternalSamplesIncrement_;
        maxInternalVertices_ += maxInternalVerticesIncrement_;
    }
    else
    {
        maxInternalSamples_ = maxInternalSamplesIncrement_;
        maxInternalVertices_ = maxInternalVerticesIncrement_;
    }
}

void og::AOXBiEST::setPathCost(double cost)
{
    bestCost_ = ob::Cost(cost);
    if (opt_)
        opt_->setCostThreshold(bestCost_);
    configureDistanceFunctions();
}

bool og::AOXBiEST::internalResetCondition() const
{
    return startTree_.motions.size() + goalTree_.motions.size() >= maxInternalVertices_ ||
           sampleAttempts_ >= maxInternalSamples_;
}

void og::AOXBiEST::setMaxInternalSamples(std::size_t value)
{
    if (value == 0)
        return;
    maxInternalSamples_ = value;
    maxInternalSamplesIncrement_ = value;
}

void og::AOXBiEST::setMaxInternalVertices(std::size_t value)
{
    if (value == 0)
        return;
    maxInternalVertices_ = value;
    maxInternalVerticesIncrement_ = value;
}

void og::AOXBiEST::freeMemory()
{
    clearTree(startTree_);
    clearTree(goalTree_);
}

void og::AOXBiEST::clearTree(TreeData &tree)
{
    for (auto *motion : tree.motions)
    {
        if (motion->state != nullptr)
            si_->freeState(motion->state);
        delete motion;
    }
    tree.motions.clear();
    tree.pdf.clear();
    if (tree.spatial)
        tree.spatial->clear();
    if (tree.costAugmented)
        tree.costAugmented->clear();
}

void og::AOXBiEST::allocateNearestNeighbors()
{
    startTree_.start = true;
    goalTree_.start = false;
    if (!startTree_.spatial)
        startTree_.spatial.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!goalTree_.spatial)
        goalTree_.spatial.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!startTree_.costAugmented)
        startTree_.costAugmented.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!goalTree_.costAugmented)
        goalTree_.costAugmented.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
}

void og::AOXBiEST::configureDistanceFunctions()
{
    if (!startTree_.spatial || !goalTree_.spatial || !startTree_.costAugmented || !goalTree_.costAugmented)
        return;

    startTree_.spatial->setDistanceFunction([this](const Motion *a, const Motion *b) {
        return spatialDistanceFunction(a, b);
    });
    goalTree_.spatial->setDistanceFunction([this](const Motion *a, const Motion *b) {
        return spatialDistanceFunction(a, b);
    });

    if (hasFiniteCostBound())
    {
        startTree_.costAugmented->setDistanceFunction([this](const Motion *a, const Motion *b) {
            return costAugmentedDistanceFunction(a, b);
        });
        goalTree_.costAugmented->setDistanceFunction([this](const Motion *a, const Motion *b) {
            return costAugmentedDistanceFunction(a, b);
        });
    }
    else
    {
        startTree_.costAugmented->setDistanceFunction([this](const Motion *a, const Motion *b) {
            return spatialDistanceFunction(a, b);
        });
        goalTree_.costAugmented->setDistanceFunction([this](const Motion *a, const Motion *b) {
            return spatialDistanceFunction(a, b);
        });
    }
}

double og::AOXBiEST::spatialDistanceFunction(const Motion *a, const Motion *b) const
{
    return si_->distance(a->state, b->state);
}

double og::AOXBiEST::costAugmentedDistanceFunction(const Motion *a, const Motion *b) const
{
    const double stateDistance = si_->distance(a->state, b->state);
    const double costDistance = a->cost - b->cost;
    return std::sqrt(stateDistance * stateDistance + costDistance * costDistance);
}

bool og::AOXBiEST::hasFiniteCostBound() const
{
    return std::isfinite(bestCost_.value());
}

og::AOXBiEST::Motion *og::AOXBiEST::selectMotion(TreeData &tree)
{
    return tree.pdf.size() > 0 ? tree.pdf.sample(rng_.uniform01()) : nullptr;
}

og::AOXBiEST::Motion *og::AOXBiEST::addMotion(TreeData &tree, Motion *motion,
                                              const std::vector<Motion *> &neighbors)
{
    for (auto *neighbor : neighbors)
    {
        PDF<Motion *>::Element *elem = neighbor->element;
        const double weight = tree.pdf.getWeight(elem);
        tree.pdf.update(elem, weight / (weight + 1.0));
    }

    motion->element = tree.pdf.add(motion, 1.0 / (static_cast<double>(neighbors.size()) + 1.0));
    tree.motions.push_back(motion);
    tree.spatial->add(motion);
    tree.costAugmented->add(motion);
    return motion;
}

bool og::AOXBiEST::isMotionValidFromTree(const TreeData &tree, const Motion *parent, const ob::State *child) const
{
    return tree.start ? si_->checkMotion(parent->state, child) : si_->checkMotion(child, parent->state);
}

double og::AOXBiEST::lowerBoundToRoots(const TreeData &tree, const ob::State *state) const
{
    double best = std::numeric_limits<double>::infinity();
    for (const auto *motion : tree.motions)
    {
        if (motion->parent == nullptr)
            best = std::min(best, si_->distance(state, motion->state));
    }
    return best;
}

og::AOXBiEST::Motion *og::AOXBiEST::selectCostAugmentedParent(TreeData &tree, TreeData &otherTree,
                                                             const ob::State *sample, double targetCost) const
{
    Motion query;
    query.state = const_cast<ob::State *>(sample);
    query.cost = targetCost;

    const double oppositeLowerBound = lowerBoundToRoots(otherTree, sample);
    std::vector<Motion *> candidates;
    tree.costAugmented->nearestK(&query, std::min(MAX_AOX_PARENT_CANDIDATES, tree.motions.size()), candidates);

    Motion *best = nullptr;
    double bestCost = std::numeric_limits<double>::infinity();
    const auto consider = [&](Motion *candidate) {
        const double edgeCost = si_->distance(candidate->state, sample);
        if (edgeCost > maxDistance_ + COST_EPS)
            return;
        const double newCost = candidate->cost + edgeCost;
        if (newCost > targetCost + COST_EPS)
            return;
        if (newCost + oppositeLowerBound + COST_EPS >= bestCost_.value())
            return;
        if (newCost >= bestCost)
            return;
        if (!isMotionValidFromTree(tree, candidate, sample))
            return;
        best = candidate;
        bestCost = newCost;
    };

    for (auto *candidate : candidates)
        consider(candidate);
    if (best != nullptr)
        return best;

    for (auto *candidate : tree.motions)
        consider(candidate);
    return best;
}

og::AOXBiEST::Motion *og::AOXBiEST::tryAddSample(TreeData &tree, TreeData &otherTree, Motion *existing,
                                                 const ob::State *sample)
{
    Motion query;
    query.state = const_cast<ob::State *>(sample);

    std::vector<Motion *> neighbors;
    tree.spatial->nearestR(&query, nbrhoodRadius_, neighbors);
    if (!neighbors.empty())
    {
        const double rejectProbability = 1.0 - (1.0 / static_cast<double>(neighbors.size()));
        if (rng_.uniform01() < rejectProbability)
            return nullptr;
    }

    Motion *parent = existing;
    if (hasFiniteCostBound())
    {
        const double sameLowerBound = lowerBoundToRoots(tree, sample);
        const double oppositeLowerBound = lowerBoundToRoots(otherTree, sample);
        if (!std::isfinite(sameLowerBound) || !std::isfinite(oppositeLowerBound) ||
            sameLowerBound + oppositeLowerBound + COST_EPS >= bestCost_.value())
            return nullptr;

        const double costUpper = bestCost_.value() - oppositeLowerBound;
        if (costUpper <= sameLowerBound + COST_EPS)
            return nullptr;

        const double targetCost = rng_.uniformReal(sameLowerBound, costUpper);
        parent = selectCostAugmentedParent(tree, otherTree, sample, targetCost);
        if (parent == nullptr)
            return nullptr;
    }
    else if (!isMotionValidFromTree(tree, parent, sample))
        return nullptr;

    const double newCost = parent->cost + si_->distance(parent->state, sample);
    if (hasFiniteCostBound() && newCost + lowerBoundToRoots(otherTree, sample) + COST_EPS >= bestCost_.value())
        return nullptr;

    auto *motion = new Motion(si_);
    si_->copyState(motion->state, sample);
    motion->parent = parent;
    motion->root = parent->root;
    motion->cost = newCost;
    return addMotion(tree, motion, neighbors);
}

double og::AOXBiEST::pathCostThroughConnection(const Motion *startMotion, const Motion *goalMotion) const
{
    return startMotion->cost + si_->distance(startMotion->state, goalMotion->state) + goalMotion->cost;
}

ob::PathPtr og::AOXBiEST::constructPath(Motion *startMotion, Motion *goalMotion) const
{
    std::vector<Motion *> startPath;
    for (Motion *motion = startMotion; motion != nullptr; motion = motion->parent)
        startPath.push_back(motion);

    std::vector<Motion *> goalPath;
    for (Motion *motion = goalMotion; motion != nullptr; motion = motion->parent)
        goalPath.push_back(motion);

    auto path = std::make_shared<PathGeometric>(si_);
    path->getStates().reserve(startPath.size() + goalPath.size());
    for (int i = static_cast<int>(startPath.size()) - 1; i >= 0; --i)
        path->append(startPath[static_cast<std::size_t>(i)]->state);
    for (auto *motion : goalPath)
        path->append(motion->state);
    return path;
}

bool og::AOXBiEST::endpointsMatch(const PathGeometric &path) const
{
    return path.getStateCount() >= 2 && startState_ != nullptr && goalState_ != nullptr &&
           si_->distance(path.getState(0), startState_) <= COST_EPS &&
           si_->distance(path.getState(path.getStateCount() - 1), goalState_) <= COST_EPS;
}

bool og::AOXBiEST::tryConnect(TreeData &tree, TreeData &otherTree, Motion *motion,
                              const ob::GoalSampleableRegion *goal)
{
    std::vector<Motion *> candidates;
    otherTree.spatial->nearestR(motion, maxDistance_, candidates);
    for (auto *candidate : candidates)
    {
        Motion *startMotion = tree.start ? motion : candidate;
        Motion *goalMotion = tree.start ? candidate : motion;
        if (!goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            continue;

        const double candidateCost = pathCostThroughConnection(startMotion, goalMotion);
        if (hasFiniteCostBound() && candidateCost + COST_EPS >= bestCost_.value())
            continue;
        if (!si_->checkMotion(startMotion->state, goalMotion->state))
            continue;

        auto path = constructPath(startMotion, goalMotion);
        auto *geometricPath = static_cast<PathGeometric *>(path.get());
        if (!endpointsMatch(*geometricPath))
        {
            if (geometricPath->getStateCount() >= 2 && startState_ != nullptr && goalState_ != nullptr &&
                si_->distance(geometricPath->getState(0), goalState_) <= COST_EPS &&
                si_->distance(geometricPath->getState(geometricPath->getStateCount() - 1), startState_) <= COST_EPS)
            {
                geometricPath->reverse();
            }
            else if (startState_ != nullptr && goalState_ != nullptr && si_->checkMotion(startState_, goalState_))
            {
                path = std::make_shared<PathGeometric>(si_);
                static_cast<PathGeometric *>(path.get())->append(startState_);
                static_cast<PathGeometric *>(path.get())->append(goalState_);
                geometricPath = static_cast<PathGeometric *>(path.get());
            }
            else
                continue;
        }

        if (hasFiniteCostBound() && geometricPath->length() + COST_EPS >= bestCost_.value())
            continue;

        connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);
        pdef_->addSolutionPath(path, false, 0.0, getName());
        foundPath_ = path;
        OMPL_INFORM("%s: Found a solution of cost %.5f", getName().c_str(), geometricPath->length());
        return true;
    }
    return false;
}

ob::PlannerStatus og::AOXBiEST::solve(const ob::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    std::vector<Motion *> neighbors;
    while (const ob::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        motion->cost = 0.0;
        startTree_.spatial->nearestR(motion, nbrhoodRadius_, neighbors);
        addMotion(startTree_, motion, neighbors);
        if (startState_ == nullptr)
            startState_ = motion->state;
    }

    if (startTree_.motions.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return ob::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return ob::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

    OMPL_INFORM("%s: Started planning with %u states. Seeking a solution better than %.5f.", getName().c_str(),
                static_cast<unsigned int>(startTree_.motions.size() + goalTree_.motions.size()), bestCost_.value());

    ob::State *sample = si_->allocState();
    bool solved = false;
    ob::PlannerStatus::StatusType status = ob::PlannerStatus::TIMEOUT;

    while (!ptc && !solved && !internalResetCondition())
    {
        if (goalTree_.motions.empty() || pis_.getSampledGoalsCount() < goalTree_.motions.size() / 2)
        {
            const ob::State *st = goalTree_.motions.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                motion->cost = 0.0;
                goalTree_.spatial->nearestR(motion, nbrhoodRadius_, neighbors);
                addMotion(goalTree_, motion, neighbors);
                if (goalState_ == nullptr)
                    goalState_ = motion->state;
            }

            if (goalTree_.motions.empty())
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                status = ob::PlannerStatus::INVALID_GOAL;
                break;
            }
        }

        TreeData &tree = startTreeTurn_ ? startTree_ : goalTree_;
        TreeData &otherTree = startTreeTurn_ ? goalTree_ : startTree_;
        Motion *existing = selectMotion(tree);
        if (existing != nullptr && sampler_->sampleNear(sample, existing->state, maxDistance_))
        {
            ++sampleAttempts_;
            Motion *added = tryAddSample(tree, otherTree, existing, sample);
            if (added != nullptr)
                solved = tryConnect(tree, otherTree, added, goal);
        }
        else
            ++sampleAttempts_;

        startTreeTurn_ = !startTreeTurn_;
    }

    si_->freeState(sample);

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(),
                static_cast<unsigned int>(startTree_.motions.size() + goalTree_.motions.size()),
                static_cast<unsigned int>(startTree_.motions.size()), static_cast<unsigned int>(goalTree_.motions.size()));
    return solved ? ob::PlannerStatus::EXACT_SOLUTION : status;
}

void og::AOXBiEST::getPlannerData(ob::PlannerData &data) const
{
    Planner::getPlannerData(data);

    for (auto *motion : startTree_.motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(ob::PlannerDataVertex(motion->state, 1));
        else
            data.addEdge(ob::PlannerDataVertex(motion->parent->state, 1), ob::PlannerDataVertex(motion->state, 1));
    }

    for (auto *motion : goalTree_.motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(ob::PlannerDataVertex(motion->state, 2));
        else
            data.addEdge(ob::PlannerDataVertex(motion->state, 2), ob::PlannerDataVertex(motion->parent->state, 2));
    }

    if (connectionPoint_.first != nullptr && connectionPoint_.second != nullptr)
        data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}
