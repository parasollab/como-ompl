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

#include "ompl/geometric/planners/rrt/LubyAORRTC.h"

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/planners/rrt/AOXRRTConnect.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"
#include "ompl/util/RandomNumbers.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <stdexcept>
#include <string>

#if !defined(_WIN32)
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace
{
    constexpr std::uint32_t RESULT_MAGIC = 0x4c554259u;  // LUBY

    enum class CommandType : std::uint32_t
    {
        START = 1,
        SHUTDOWN = 2
    };

    enum class ResultType : std::uint32_t
    {
        SOLUTION = 1,
        DONE = 2,
        ERROR = 3
    };

    struct WorkerCommand
    {
        std::uint32_t type{0};
        std::uint64_t solveId{0};
        std::uint_fast32_t seed{1};
    };

    struct ResultHeader
    {
        std::uint32_t magic{RESULT_MAGIC};
        std::uint32_t type{0};
        std::uint32_t workerIndex{0};
        std::uint32_t status{0};
        std::uint64_t solveId{0};
        std::uint64_t generation{0};
        std::uint64_t lubyIndex{0};
        std::uint64_t lubyMultiplier{0};
        std::uint64_t stateCount{0};
        std::uint64_t stateLength{0};
        std::uint64_t payloadBytes{0};
        double pathLength{std::numeric_limits<double>::infinity()};
    };

    std::size_t scaledBudget(std::size_t base, std::uint64_t multiplier)
    {
        if (base == 0)
            return 0;
        const auto maxValue = std::numeric_limits<std::size_t>::max();
        if (multiplier > maxValue / base)
            return maxValue;
        return base * static_cast<std::size_t>(multiplier);
    }

    bool isCostImprovement(double cost, double bound)
    {
        return std::isfinite(cost) && cost > 0.0 && (!std::isfinite(bound) || cost + 1e-12 < bound);
    }

    double strictImprovementBound(double bound)
    {
        if (!std::isfinite(bound))
            return bound;
        return std::nextafter(bound, -std::numeric_limits<double>::infinity());
    }

#if !defined(_WIN32)
    bool writeAll(int fd, const void *data, std::size_t size)
    {
        const auto *ptr = static_cast<const unsigned char *>(data);
        while (size > 0)
        {
            const ssize_t n = ::write(fd, ptr, size);
            if (n < 0)
            {
                if (errno == EINTR)
                    continue;
                return false;
            }
            if (n == 0)
                return false;
            ptr += static_cast<std::size_t>(n);
            size -= static_cast<std::size_t>(n);
        }
        return true;
    }

    bool readAll(int fd, void *data, std::size_t size)
    {
        auto *ptr = static_cast<unsigned char *>(data);
        while (size > 0)
        {
            const ssize_t n = ::read(fd, ptr, size);
            if (n < 0)
            {
                if (errno == EINTR)
                    continue;
                return false;
            }
            if (n == 0)
                return false;
            ptr += static_cast<std::size_t>(n);
            size -= static_cast<std::size_t>(n);
        }
        return true;
    }

    bool sendCommand(int fd, const WorkerCommand &command)
    {
        return writeAll(fd, &command, sizeof(command));
    }

    bool sendResult(int fd, const ResultHeader &header, const std::vector<unsigned char> &payload = {})
    {
        if (!writeAll(fd, &header, sizeof(header)))
            return false;
        if (!payload.empty())
            return writeAll(fd, payload.data(), payload.size());
        return true;
    }

    void closeFd(int &fd)
    {
        if (fd >= 0)
        {
            ::close(fd);
            fd = -1;
        }
    }
#endif
}

struct ompl::geometric::LubyAORRTC::SharedState
{
#if !defined(_WIN32)
    pthread_mutex_t mutex;
#endif
    double bestCost{std::numeric_limits<double>::infinity()};
    std::uint64_t generation{0};
    std::uint64_t solveId{0};
    int stop{1};
};

ompl::geometric::LubyAORRTC::LubyAORRTC(const base::SpaceInformationPtr &si) : base::Planner(si, "LubyAORRTC")
{
    specs_.approximateSolutions = true;
    specs_.canReportIntermediateSolutions = true;
    specs_.directed = true;
    specs_.multithreaded = true;
    specs_.optimizingPaths = true;

    Planner::declareParam<double>("range", this, &LubyAORRTC::setRange, &LubyAORRTC::getRange, "0.:1.:10000.");
    Planner::declareParam<unsigned int>("worker_processes", this, &LubyAORRTC::setWorkerProcesses,
                                        &LubyAORRTC::getWorkerProcesses, "1:64");
    Planner::declareParam<std::size_t>("base_internal_samples", this, &LubyAORRTC::setBaseInternalSamples,
                                       &LubyAORRTC::getBaseInternalSamples, "1:10000000");
    Planner::declareParam<std::size_t>("base_internal_vertices", this, &LubyAORRTC::setBaseInternalVertices,
                                       &LubyAORRTC::getBaseInternalVertices, "1:10000000");

    addPlannerProgressProperty("best cost REAL", [this] { return std::to_string(bestCost_.value()); });
}

ompl::geometric::LubyAORRTC::~LubyAORRTC()
{
    freeMemory();
}

void ompl::geometric::LubyAORRTC::clear()
{
    Planner::clear();
    freeMemory();
    setup_ = false;
    psk_.reset();
    opt_.reset();
    bestPath_.reset();
    bestCost_ = base::Cost(std::numeric_limits<double>::infinity());
    lastStatus_ = base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::LubyAORRTC::setup()
{
    if (setup_)
    {
        OMPL_WARN("%s: Planner setup called multiple times", getName().c_str());
        return;
    }

    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!pdef_)
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
        return;
    }

    if (pdef_->hasOptimizationObjective())
        opt_ = pdef_->getOptimizationObjective();
    else
    {
        opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
        pdef_->setOptimizationObjective(opt_);
    }

    psk_ = std::make_shared<PathSimplifier>(si_);

    if (workerProcesses_ > 1)
        startWorkers();
}

void ompl::geometric::LubyAORRTC::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    const bool changed = pdef_ != pdef;
    if (changed && workersStarted())
    {
        stopWorkers();
        setup_ = false;
    }
    Planner::setProblemDefinition(pdef);
    if (changed)
        setup_ = false;
}

void ompl::geometric::LubyAORRTC::freeMemory()
{
    stopWorkers();
#if !defined(_WIN32)
    if (sharedState_)
    {
        pthread_mutex_destroy(&sharedState_->mutex);
        ::munmap(sharedState_, sharedStateBytes_);
        sharedState_ = nullptr;
        sharedStateBytes_ = 0;
    }
#else
    sharedState_ = nullptr;
    sharedStateBytes_ = 0;
#endif
}

ompl::base::Cost ompl::geometric::LubyAORRTC::bestCost() const
{
    return bestCost_;
}

void ompl::geometric::LubyAORRTC::setWorkerProcesses(unsigned int n)
{
    const unsigned int value = std::max(1u, n);
    if (value == workerProcesses_)
        return;
    if (workersStarted())
    {
        stopWorkers();
        setup_ = false;
    }
    workerProcesses_ = value;
}

void ompl::geometric::LubyAORRTC::setBaseInternalSamples(std::size_t value)
{
    if (value > 0)
        baseInternalSamples_ = value;
}

void ompl::geometric::LubyAORRTC::setBaseInternalVertices(std::size_t value)
{
    if (value > 0)
        baseInternalVertices_ = value;
}

std::uint64_t ompl::geometric::LubyAORRTC::lubyMultiplier(std::uint64_t index)
{
    if (index == 0)
        index = 1;

    std::uint64_t blockSize = 1;
    while (blockSize - 1 < index)
    {
        if (blockSize > (std::numeric_limits<std::uint64_t>::max() / 2))
            return blockSize;
        blockSize <<= 1u;
    }
    while (blockSize > 1)
    {
        const std::uint64_t half = blockSize >> 1u;
        if (index == blockSize - 1)
            return half;
        if (index > half - 1)
            index -= half - 1;
        blockSize = half;
    }
    return 1;
}

void ompl::geometric::LubyAORRTC::simplifySolution(const base::PathPtr &p,
                                                   const base::PlannerTerminationCondition &ptc)
{
    if (!pdef_ || !p)
    {
        OMPL_WARN("%s: No solution to simplify", getName().c_str());
        return;
    }

    auto &path = static_cast<PathGeometric &>(*p);
    psk_->simplify(path, ptc);
}

void ompl::geometric::LubyAORRTC::initializeBestFromProblemDefinition()
{
    bestPath_.reset();
    bestCost_ = base::Cost(std::numeric_limits<double>::infinity());

    if (pdef_ && pdef_->hasExactSolution())
    {
        bestPath_ = pdef_->getSolutionPath();
        if (bestPath_)
            bestCost_ = base::Cost(bestPath_->length());
    }
}

bool ompl::geometric::LubyAORRTC::considerSolutionPath(const base::PathPtr &path,
                                                       const base::PlannerTerminationCondition &ptc)
{
    if (!path)
        return false;

    const double pathLength = path->length();
    if (!std::isfinite(pathLength) || pathLength <= 0.0)
        return false;

    if (pathLength + 1e-12 < bestCost_.value())
    {
        bestPath_ = path;
        bestCost_ = base::Cost(pathLength);
        pdef_->addSolutionPath(bestPath_, false, 0.0, getName());
        updateSharedBestCost(pathLength);

        const base::ReportIntermediateSolutionFn callback = pdef_->getIntermediateSolutionCallback();
        if (callback)
        {
            const auto &geometricPath = static_cast<const PathGeometric &>(*bestPath_);
            std::vector<const base::State *> states;
            states.reserve(geometricPath.getStateCount());
            for (std::size_t i = 0; i < geometricPath.getStateCount(); ++i)
                states.push_back(geometricPath.getState(i));
            callback(this, states, bestCost_);
        }

        if (!ptc)
        {
            simplifySolution(bestPath_, ptc);
            const double simplifiedLength = bestPath_->length();
            if (std::isfinite(simplifiedLength) && simplifiedLength + 1e-12 < bestCost_.value())
            {
                bestCost_ = base::Cost(simplifiedLength);
                pdef_->addSolutionPath(bestPath_, false, 0.0, getName());
                updateSharedBestCost(simplifiedLength);
                if (callback)
                {
                    const auto &geometricPath = static_cast<const PathGeometric &>(*bestPath_);
                    std::vector<const base::State *> states;
                    states.reserve(geometricPath.getStateCount());
                    for (std::size_t i = 0; i < geometricPath.getStateCount(); ++i)
                        states.push_back(geometricPath.getState(i));
                    callback(this, states, bestCost_);
                }
            }
        }
        return true;
    }
    return false;
}

ompl::base::PlannerStatus ompl::geometric::LubyAORRTC::solve(const base::PlannerTerminationCondition &ptc)
{
    if (workerProcesses_ > 1 && !isSetup())
        throw Exception(getName() +
                        ": worker_processes > 1 requires setup() before solve() so workers are forked before any "
                        "caller-owned termination-condition threads are active");

    checkValidity();
    initializeBestFromProblemDefinition();

    if (workerProcesses_ <= 1)
        lastStatus_ = solveSerial(ptc);
    else
        lastStatus_ = solveParallel(ptc);

    return lastStatus_;
}

ompl::base::PlannerStatus ompl::geometric::LubyAORRTC::solveSerial(const base::PlannerTerminationCondition &ptc)
{
    std::uint64_t lubyIndex = 1;
    while (!ptc)
    {
        const std::uint64_t multiplier = lubyMultiplier(lubyIndex);
        auto planner = std::make_shared<AOXRRTConnect>(si_);
        planner->setProblemDefinition(pdef_);
        planner->setName("LubyAOXRRTConnect");
        planner->setRange(maxDistance_);
        planner->setMaxInternalSamples(scaledBudget(baseInternalSamples_, multiplier));
        planner->setMaxInternalVertices(scaledBudget(baseInternalVertices_, multiplier));
        planner->setup();
        if (std::isfinite(bestCost_.value()))
            planner->setPathCost(strictImprovementBound(bestCost_.value()));

        const auto status = planner->solve(ptc);
        if (status == base::PlannerStatus::EXACT_SOLUTION)
        {
            if (considerSolutionPath(planner->getFoundPath(), ptc))
                lubyIndex = 1;
            else
                ++lubyIndex;
            continue;
        }

        if (status == base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE ||
            status == base::PlannerStatus::INVALID_START || status == base::PlannerStatus::INVALID_GOAL)
        {
            return status;
        }

        ++lubyIndex;
    }

    return pdef_->hasExactSolution() ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

bool ompl::geometric::LubyAORRTC::serializeSolutionPath(const PathGeometric &path,
                                                        std::vector<unsigned char> &payload,
                                                        std::uint64_t &stateCount,
                                                        std::uint64_t &stateLength) const
{
    stateCount = path.getStateCount();
    stateLength = si_->getStateSpace()->getSerializationLength();
    if (stateCount == 0 || stateLength == 0)
        return false;
    payload.resize(static_cast<std::size_t>(stateCount * stateLength));
    for (std::uint64_t i = 0; i < stateCount; ++i)
    {
        si_->getStateSpace()->serialize(payload.data() + static_cast<std::size_t>(i * stateLength),
                                        path.getState(static_cast<unsigned int>(i)));
    }
    return true;
}

bool ompl::geometric::LubyAORRTC::deserializeSolutionPath(std::uint64_t stateCount, std::uint64_t stateLength,
                                                          const std::vector<unsigned char> &payload,
                                                          base::PathPtr &path) const
{
    const std::uint64_t expectedStateLength = si_->getStateSpace()->getSerializationLength();
    if (stateCount == 0 || stateLength == 0 || stateLength != expectedStateLength ||
        payload.size() != static_cast<std::size_t>(stateCount * stateLength))
    {
        return false;
    }

    auto geometricPath = std::make_shared<PathGeometric>(si_);
    base::State *state = si_->allocState();
    for (std::uint64_t i = 0; i < stateCount; ++i)
    {
        si_->getStateSpace()->deserialize(state, payload.data() + static_cast<std::size_t>(i * stateLength));
        geometricPath->append(state);
    }
    si_->freeState(state);
    path = geometricPath;
    return true;
}

void ompl::geometric::LubyAORRTC::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
}

void ompl::geometric::LubyAORRTC::resetSharedStateForSolve(double initialBestCost)
{
    if (!sharedState_)
        return;
#if !defined(_WIN32)
    pthread_mutex_lock(&sharedState_->mutex);
#endif
    sharedState_->bestCost = initialBestCost;
    sharedState_->generation = 0;
    sharedState_->solveId = nextSolveId_;
    sharedState_->stop = 0;
#if !defined(_WIN32)
    pthread_mutex_unlock(&sharedState_->mutex);
#endif
}

void ompl::geometric::LubyAORRTC::updateSharedBestCost(double cost)
{
    if (!sharedState_ || !std::isfinite(cost))
        return;
#if !defined(_WIN32)
    pthread_mutex_lock(&sharedState_->mutex);
#endif
    if (cost + 1e-12 < sharedState_->bestCost)
    {
        sharedState_->bestCost = cost;
        ++sharedState_->generation;
    }
#if !defined(_WIN32)
    pthread_mutex_unlock(&sharedState_->mutex);
#endif
}

double ompl::geometric::LubyAORRTC::sharedBestCost() const
{
    if (!sharedState_)
        return std::numeric_limits<double>::infinity();
    double value;
#if !defined(_WIN32)
    pthread_mutex_lock(&sharedState_->mutex);
#endif
    value = sharedState_->bestCost;
#if !defined(_WIN32)
    pthread_mutex_unlock(&sharedState_->mutex);
#endif
    return value;
}

std::uint64_t ompl::geometric::LubyAORRTC::sharedGeneration() const
{
    if (!sharedState_)
        return 0;
    std::uint64_t value;
#if !defined(_WIN32)
    pthread_mutex_lock(&sharedState_->mutex);
#endif
    value = sharedState_->generation;
#if !defined(_WIN32)
    pthread_mutex_unlock(&sharedState_->mutex);
#endif
    return value;
}

void ompl::geometric::LubyAORRTC::setSharedStop(bool stop)
{
    if (!sharedState_)
        return;
#if !defined(_WIN32)
    pthread_mutex_lock(&sharedState_->mutex);
#endif
    sharedState_->stop = stop ? 1 : 0;
#if !defined(_WIN32)
    pthread_mutex_unlock(&sharedState_->mutex);
#endif
}

bool ompl::geometric::LubyAORRTC::sharedStopRequested() const
{
    if (!sharedState_)
        return true;
    bool value;
#if !defined(_WIN32)
    pthread_mutex_lock(&sharedState_->mutex);
#endif
    value = sharedState_->stop != 0;
#if !defined(_WIN32)
    pthread_mutex_unlock(&sharedState_->mutex);
#endif
    return value;
}

void ompl::geometric::LubyAORRTC::startWorkers()
{
    if (workersStarted())
        return;

#if defined(_WIN32)
    throw Exception(getName() + ": worker_processes > 1 requires POSIX fork support");
#else
    sharedStateBytes_ = sizeof(SharedState);
    sharedState_ = static_cast<SharedState *>(::mmap(nullptr, sharedStateBytes_, PROT_READ | PROT_WRITE,
                                                     MAP_SHARED | MAP_ANONYMOUS, -1, 0));
    if (sharedState_ == MAP_FAILED)
    {
        sharedState_ = nullptr;
        sharedStateBytes_ = 0;
        throw Exception(getName() + ": failed to allocate shared worker state: " + std::strerror(errno));
    }
    new (sharedState_) SharedState();

    pthread_mutexattr_t attr;
    if (pthread_mutexattr_init(&attr) != 0 || pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED) != 0 ||
        pthread_mutex_init(&sharedState_->mutex, &attr) != 0)
    {
        pthread_mutexattr_destroy(&attr);
        throw Exception(getName() + ": failed to initialize process-shared mutex");
    }
    pthread_mutexattr_destroy(&attr);

    workers_.reserve(workerProcesses_);
    for (unsigned int i = 0; i < workerProcesses_; ++i)
    {
        int commandPipe[2] = {-1, -1};
        int resultPipe[2] = {-1, -1};
        if (::pipe(commandPipe) != 0 || ::pipe(resultPipe) != 0)
        {
            closeFd(commandPipe[0]);
            closeFd(commandPipe[1]);
            closeFd(resultPipe[0]);
            closeFd(resultPipe[1]);
            stopWorkers();
            throw Exception(getName() + ": failed to create worker pipes: " + std::strerror(errno));
        }

        const pid_t pid = ::fork();
        if (pid < 0)
        {
            closeFd(commandPipe[0]);
            closeFd(commandPipe[1]);
            closeFd(resultPipe[0]);
            closeFd(resultPipe[1]);
            stopWorkers();
            throw Exception(getName() + ": failed to fork worker: " + std::strerror(errno));
        }

        if (pid == 0)
        {
            for (auto &worker : workers_)
            {
                closeFd(worker.commandFd);
                closeFd(worker.resultFd);
            }
            closeFd(commandPipe[1]);
            closeFd(resultPipe[0]);
            workerLoop(static_cast<int>(i), commandPipe[0], resultPipe[1]);
            _exit(0);
        }

        closeFd(commandPipe[0]);
        closeFd(resultPipe[1]);
        WorkerProcess worker;
        worker.workerIndex = static_cast<int>(i);
        worker.pid = pid;
        worker.commandFd = commandPipe[1];
        worker.resultFd = resultPipe[0];
        workers_.push_back(worker);
    }
#endif
}

void ompl::geometric::LubyAORRTC::stopWorkers()
{
#if !defined(_WIN32)
    if (workers_.empty())
        return;

    setSharedStop(true);
    for (auto &worker : workers_)
    {
        if (worker.commandFd >= 0)
        {
            WorkerCommand command;
            command.type = static_cast<std::uint32_t>(CommandType::SHUTDOWN);
            sendCommand(worker.commandFd, command);
        }
    }

    for (auto &worker : workers_)
        closeFd(worker.commandFd);

    for (auto &worker : workers_)
    {
        if (worker.pid > 0)
        {
            int status = 0;
            while (::waitpid(worker.pid, &status, 0) < 0)
            {
                if (errno != EINTR)
                    break;
            }
            worker.pid = -1;
        }
        closeFd(worker.resultFd);
    }
#endif
    workers_.clear();
}

void ompl::geometric::LubyAORRTC::workerLoop(int workerIndex, int commandFd, int resultFd)
{
#if !defined(_WIN32)
    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    pdef_->setIntermediateSolutionCallback(nullptr);
    WorkerCommand command;
    while (readAll(commandFd, &command, sizeof(command)))
    {
        const auto type = static_cast<CommandType>(command.type);
        if (type == CommandType::SHUTDOWN)
            break;
        if (type == CommandType::START)
            workerSolveLoop(workerIndex, resultFd, command.solveId, command.seed);
    }
    closeFd(commandFd);
    closeFd(resultFd);
#else
    (void)workerIndex;
    (void)commandFd;
    (void)resultFd;
#endif
}

void ompl::geometric::LubyAORRTC::workerSolveLoop(int workerIndex, int resultFd, std::uint64_t solveId,
                                                  std::uint_fast32_t seed)
{
#if !defined(_WIN32)
    RNG::setSeed(seed);
    std::uint64_t lubyIndex = 1;
    std::uint64_t observedGeneration = sharedGeneration();

    while (!sharedStopRequested())
    {
        pdef_->clearSolutionPaths();
        const std::uint64_t multiplier = lubyMultiplier(lubyIndex);
        auto planner = std::make_shared<AOXRRTConnect>(si_);
        planner->setProblemDefinition(pdef_);
        planner->setName("LubyAOXRRTConnectWorker");
        planner->setRange(maxDistance_);
        planner->setMaxInternalSamples(scaledBudget(baseInternalSamples_, multiplier));
        planner->setMaxInternalVertices(scaledBudget(baseInternalVertices_, multiplier));
        planner->setup();

        const double bound = sharedBestCost();
        if (std::isfinite(bound))
            planner->setPathCost(strictImprovementBound(bound));

        base::PlannerTerminationCondition stopCondition([this] { return sharedStopRequested(); });
        const auto status = planner->solve(stopCondition);
        if (sharedStopRequested())
            break;

        if (status == base::PlannerStatus::EXACT_SOLUTION)
        {
            auto path = std::dynamic_pointer_cast<PathGeometric>(planner->getFoundPath());
            if (path && std::isfinite(path->length()))
            {
                const double pathLength = path->length();
                if (isCostImprovement(pathLength, sharedBestCost()))
                {
                    updateSharedBestCost(pathLength);
                    std::vector<unsigned char> payload;
                    std::uint64_t stateCount = 0;
                    std::uint64_t stateLength = 0;
                    if (serializeSolutionPath(*path, payload, stateCount, stateLength))
                    {
                        ResultHeader header;
                        header.type = static_cast<std::uint32_t>(ResultType::SOLUTION);
                        header.workerIndex = static_cast<std::uint32_t>(workerIndex);
                        header.status = static_cast<std::uint32_t>(
                            static_cast<base::PlannerStatus::StatusType>(status));
                        header.solveId = solveId;
                        header.generation = sharedGeneration();
                        header.lubyIndex = lubyIndex;
                        header.lubyMultiplier = multiplier;
                        header.stateCount = stateCount;
                        header.stateLength = stateLength;
                        header.payloadBytes = payload.size();
                        header.pathLength = pathLength;
                        if (!sendResult(resultFd, header, payload))
                            break;
                    }
                    lubyIndex = 1;
                    observedGeneration = sharedGeneration();
                    continue;
                }
            }

            const std::uint64_t currentGeneration = sharedGeneration();
            if (currentGeneration != observedGeneration)
            {
                lubyIndex = 1;
                observedGeneration = currentGeneration;
            }
            else
            {
                ++lubyIndex;
            }
            continue;
        }

        if (status == base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE ||
            status == base::PlannerStatus::INVALID_START || status == base::PlannerStatus::INVALID_GOAL)
        {
            ResultHeader header;
            header.type = static_cast<std::uint32_t>(ResultType::ERROR);
            header.workerIndex = static_cast<std::uint32_t>(workerIndex);
            header.status = static_cast<std::uint32_t>(static_cast<base::PlannerStatus::StatusType>(status));
            header.solveId = solveId;
            sendResult(resultFd, header);
            break;
        }

        const std::uint64_t currentGeneration = sharedGeneration();
        if (currentGeneration != observedGeneration)
        {
            lubyIndex = 1;
            observedGeneration = currentGeneration;
        }
        else
        {
            ++lubyIndex;
        }
    }

    ResultHeader done;
    done.type = static_cast<std::uint32_t>(ResultType::DONE);
    done.workerIndex = static_cast<std::uint32_t>(workerIndex);
    done.status = static_cast<std::uint32_t>(base::PlannerStatus::TIMEOUT);
    done.solveId = solveId;
    done.generation = sharedGeneration();
    sendResult(resultFd, done);
#else
    (void)workerIndex;
    (void)resultFd;
    (void)solveId;
    (void)seed;
#endif
}

ompl::base::PlannerStatus ompl::geometric::LubyAORRTC::solveParallel(const base::PlannerTerminationCondition &ptc)
{
#if defined(_WIN32)
    throw Exception(getName() + ": worker_processes > 1 requires POSIX fork support");
#else
    if (!workersStarted())
        startWorkers();

    const std::uint64_t solveId = nextSolveId_++;
    resetSharedStateForSolve(bestCost_.value());
    for (auto &worker : workers_)
    {
        worker.ready = false;
        WorkerCommand command;
        command.type = static_cast<std::uint32_t>(CommandType::START);
        command.solveId = solveId;
        command.seed = RNG::getSeed() + static_cast<std::uint_fast32_t>(worker.workerIndex + 1) * 7919u +
                       static_cast<std::uint_fast32_t>(solveId) * 104729u;
        if (!sendCommand(worker.commandFd, command))
            throw Exception(getName() + ": failed to start worker " + std::to_string(worker.workerIndex));
    }

    std::vector<pollfd> pollFds;
    pollFds.reserve(workers_.size());
    for (const auto &worker : workers_)
        pollFds.push_back({worker.resultFd, POLLIN, 0});

    base::PlannerStatus terminalStatus{base::PlannerStatus::UNKNOWN};
    unsigned int doneCount = 0;
    bool stopping = false;

    const auto processOneMessage = [&](std::size_t index) {
        ResultHeader header;
        if (!readAll(pollFds[index].fd, &header, sizeof(header)))
            return;
        if (header.magic != RESULT_MAGIC || header.solveId != solveId)
            return;

        std::vector<unsigned char> payload;
        if (header.payloadBytes > 0)
        {
            payload.resize(static_cast<std::size_t>(header.payloadBytes));
            if (!readAll(pollFds[index].fd, payload.data(), payload.size()))
                return;
        }

        const auto type = static_cast<ResultType>(header.type);
        if (type == ResultType::SOLUTION)
        {
            base::PathPtr path;
            if (deserializeSolutionPath(header.stateCount, header.stateLength, payload, path))
                considerSolutionPath(path, ptc);
        }
        else if (type == ResultType::ERROR)
        {
            terminalStatus = base::PlannerStatus(
                static_cast<base::PlannerStatus::StatusType>(header.status));
            setSharedStop(true);
            stopping = true;
        }
        else if (type == ResultType::DONE)
        {
            if (!workers_[index].ready)
            {
                workers_[index].ready = true;
                ++doneCount;
            }
        }
    };

    while (doneCount < workers_.size())
    {
        if (!stopping && ptc)
        {
            setSharedStop(true);
            stopping = true;
        }

        const int rc = ::poll(pollFds.data(), pollFds.size(), 10);
        if (rc < 0)
        {
            if (errno == EINTR)
                continue;
            setSharedStop(true);
            break;
        }
        if (rc == 0)
            continue;

        for (std::size_t i = 0; i < pollFds.size(); ++i)
        {
            if (pollFds[i].revents & POLLIN)
                processOneMessage(i);
            if (pollFds[i].revents & (POLLERR | POLLHUP | POLLNVAL))
            {
                if (!workers_[i].ready)
                {
                    workers_[i].ready = true;
                    ++doneCount;
                }
            }
            pollFds[i].revents = 0;
        }
    }

    setSharedStop(true);

    if (terminalStatus != base::PlannerStatus::UNKNOWN && !pdef_->hasExactSolution())
        return terminalStatus;

    return pdef_->hasExactSolution() ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
#endif
}
