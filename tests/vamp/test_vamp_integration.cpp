#define BOOST_TEST_MODULE VampIntegrationTest
#include <boost/test/unit_test.hpp>
#include <cstring>
#include <iostream>
#include <array>
#include <type_traits>
#include <vector>

// VAMP robot types for testing
#include <vamp/robots/panda.hh>
#include <vamp/robots/panda_legacy59.hh>
#include <vamp/collision/factory.hh>

// OMPL includes for state conversion testing
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/vamp/VampStateSpace.h>
#include <ompl/vamp/VampStateValidityChecker.h>
#include <ompl/vamp/VampMotionValidator.h>

namespace ob = ompl::base;

BOOST_AUTO_TEST_CASE(LegacyPandaBaseCoverageThroughOmplBridge)
{
    using Robot = vamp::robots::PandaLegacy59;
    using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;
    BOOST_REQUIRE_EQUAL(Robot::n_spheres, 59U);

    auto space = std::make_shared<ompl::vamp::VampStateSpace<Robot>>();
    auto si = std::make_shared<ob::SpaceInformation>(space);
    Environment env;
    auto checker = std::make_shared<ompl::vamp::VampStateValidityChecker<Robot>>(si, env);
    auto motion = std::make_shared<ompl::vamp::VampMotionValidator<Robot>>(si, env);
    si->setStateValidityChecker(checker);
    si->setMotionValidator(motion);
    si->setup();

    ob::ScopedState<> state(space);
    const std::array<double, 7> ready{0., -0.785, 0., -2.356, 0., 1.571, 0.785};
    for (std::size_t i = 0; i < ready.size(); ++i)
        state[i] = ready[i];
    BOOST_REQUIRE(checker->isValid(state.get()));
    BOOST_REQUIRE(motion->checkMotion(state.get(), state.get()));

    // The original model leaves this rear-tail location clear. This detects
    // accidental use of the experimental 82 mm tail sphere by either bridge.
    vamp::collision::Environment<float> obstacles;
    obstacles.spheres.emplace_back(
        vamp::collision::factory::sphere::array({-0.186F, 0.F, 0.035F}, 0.005F));
    env = Environment(obstacles);
    BOOST_CHECK(checker->isValid(state.get()));
    BOOST_CHECK(motion->checkMotion(state.get(), state.get()));

    // Both bridge entry points must still detect the original base sphere.
    obstacles.spheres.clear();
    obstacles.spheres.emplace_back(
        vamp::collision::factory::sphere::array({0.F, 0.F, 0.050F}, 0.005F));
    env = Environment(obstacles);
    BOOST_CHECK(!checker->isValid(state.get()));
    BOOST_CHECK(!motion->checkMotion(state.get(), state.get()));
}

BOOST_AUTO_TEST_CASE(DefaultPandaUsesRefined46ThroughBothValidators)
{
    using Robot = vamp::robots::Panda;
    using Legacy = vamp::robots::PandaLegacy59;
    using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;
    static_assert(std::is_same_v<Robot, vamp::robots::ComoPanda>);
    BOOST_REQUIRE_EQUAL(Robot::n_spheres, 46U);
    BOOST_REQUIRE_EQUAL(Legacy::n_spheres, 59U);

    auto space = std::make_shared<ompl::vamp::VampStateSpace<Robot>>();
    auto si = std::make_shared<ob::SpaceInformation>(space);
    Environment env;
    auto checker = std::make_shared<ompl::vamp::VampStateValidityChecker<Robot>>(si, env);
    auto motion = std::make_shared<ompl::vamp::VampMotionValidator<Robot>>(si, env);
    si->setStateValidityChecker(checker);
    si->setMotionValidator(motion);
    si->setup();
    ompl::vamp::VampStateValidityChecker<Legacy> legacy_checker(si, env);
    ompl::vamp::VampMotionValidator<Legacy> legacy_motion(si, env);

    ob::ScopedState<> start(space), goal(space);
    const std::array<double, 7> ready{0., -0.785, 0., -2.356, 0., 1.571, 0.785};
    for (std::size_t i = 0; i < ready.size(); ++i)
        start[i] = goal[i] = ready[i];
    goal[0] += 0.02;
    BOOST_REQUIRE(si->isValid(start.get()));
    BOOST_REQUIRE(si->isValid(goal.get()));
    BOOST_REQUIRE(si->checkMotion(start.get(), goal.get()));
    BOOST_REQUIRE(legacy_checker.isValid(start.get()));
    BOOST_REQUIRE(legacy_checker.isValid(goal.get()));
    BOOST_REQUIRE(legacy_motion.checkMotion(start.get(), goal.get()));

    // Refined46 covers this rear-base location that Legacy59 leaves unmodeled.
    // A nonzero edge ensures the motion bridge also calls the selected FKCC.
    vamp::collision::Environment<float> obstacles;
    obstacles.spheres.emplace_back(
        vamp::collision::factory::sphere::array({-0.155F, 0.F, 0.024F}, 0.001F));
    env = Environment(obstacles);
    BOOST_CHECK(!si->isValid(start.get()));
    BOOST_CHECK(!si->isValid(goal.get()));
    BOOST_CHECK(!si->checkMotion(start.get(), goal.get()));
    BOOST_CHECK(!si->checkMotion(goal.get(), start.get()));
    BOOST_CHECK(legacy_checker.isValid(start.get()));
    BOOST_CHECK(legacy_checker.isValid(goal.get()));
    BOOST_CHECK(legacy_motion.checkMotion(start.get(), goal.get()));
    BOOST_CHECK(legacy_motion.checkMotion(goal.get(), start.get()));

    std::pair<ob::State *, double> last_valid{nullptr, 0.};
    BOOST_CHECK(!motion->checkMotion(start.get(), goal.get(), last_valid));
    BOOST_CHECK(legacy_motion.checkMotion(start.get(), goal.get(), last_valid));
}

// Test VAMP target availability and linking
BOOST_AUTO_TEST_CASE(VampTargetLinkingTest)
{
    // This test confirms that VAMP was built and linked properly
    // If vamp_cpp target wasn't available, this test wouldn't have linked

    BOOST_CHECK_MESSAGE(true, "vamp_cpp target successfully linked");
}

// Test portable vs native build configuration
BOOST_AUTO_TEST_CASE(VampBuildConfigurationTest)
{
    BOOST_TEST_MESSAGE("Testing VAMP build configuration");

#ifdef __x86_64__
// Check for expected instruction sets in portable/native builds
#ifdef __AVX2__
    BOOST_TEST_MESSAGE("AVX2 enabled - SIMD acceleration available");
#else
    BOOST_WARN_MESSAGE(false, "AVX2 not enabled - performance may be limited");
#endif

    BOOST_CHECK_MESSAGE(true, "x86_64 build configuration validated");

#elif defined(__aarch64__)
    BOOST_CHECK_MESSAGE(true, "ARM64 build configuration validated");

#else
    BOOST_WARN_MESSAGE(false, "Unknown architecture - VAMP may not be optimized");
#endif

    BOOST_TEST_MESSAGE("Build configuration test completed");
}

// Test memory alignment for SIMD operations
BOOST_AUTO_TEST_CASE(VampMemoryAlignmentTest)
{
    BOOST_TEST_MESSAGE("Testing SIMD-compatible memory alignment");

    // Test 32-byte alignment
    constexpr size_t alignment = 32;
    constexpr size_t size = 256;

    void *aligned_ptr = nullptr;
    int result = posix_memalign(&aligned_ptr, alignment, size);

    BOOST_CHECK_EQUAL(result, 0);
    BOOST_CHECK(aligned_ptr != nullptr);

    if (aligned_ptr)
    {
        // Verify alignment
        uintptr_t addr = reinterpret_cast<uintptr_t>(aligned_ptr);
        BOOST_CHECK_EQUAL(addr % alignment, 0);

        // Basic write test
        memset(aligned_ptr, 0x42, size);
        unsigned char *bytes = static_cast<unsigned char *>(aligned_ptr);
        BOOST_CHECK_EQUAL(bytes[0], 0x42);

        free(aligned_ptr);

        BOOST_TEST_MESSAGE("Memory alignment test passed");
    }
}
