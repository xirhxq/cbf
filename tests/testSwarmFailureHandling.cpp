#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#ifndef PROJECT_ROOT
#define PROJECT_ROOT "."
#endif
#include "doctest/doctest.h"

#include "Swarm.hpp"

#include <exception>
#include <sstream>
#include <stdexcept>
#include <string>

TEST_CASE("FailureRecoveryPreservesOriginalWhenCleanupThrows") {
    const std::exception_ptr original =
        std::make_exception_ptr(std::runtime_error("original loop failure"));
    int updateAttempts = 0;
    int logAttempts = 0;
    std::ostringstream warnings;

    const std::exception_ptr recovered = recoverFailedIteration(
        original,
        false,
        [&]() {
            ++updateAttempts;
            throw std::runtime_error("cleanup update failed");
        },
        [&]() {
            ++logAttempts;
        },
        warnings
    );

    CHECK(updateAttempts == 1);
    CHECK(logAttempts == 1);
    CHECK(warnings.str().find("cleanup update failed") != std::string::npos);
    try {
        std::rethrow_exception(recovered);
        FAIL("the original exception must be rethrown");
    } catch (const std::runtime_error& error) {
        CHECK(std::string(error.what()) == "original loop failure");
    }
}

TEST_CASE("FailureRecoveryDoesNotRepeatAnAttemptedFrameLog") {
    const std::exception_ptr original =
        std::make_exception_ptr(std::runtime_error("failure after log"));
    int logAttempts = 0;
    std::ostringstream warnings;

    const std::exception_ptr recovered = recoverFailedIteration(
        original,
        true,
        []() {},
        [&]() {
            ++logAttempts;
        },
        warnings
    );

    CHECK(recovered == original);
    CHECK(logAttempts == 0);
    CHECK(warnings.str().empty());
}

TEST_CASE("SimulationErrorGateHandlesNonStandardExceptions") {
    int attempts = 0;
    std::ostringstream errors;

    const int returnCode = runSimulationWithErrorGate(
        [&]() {
            ++attempts;
            throw 7;
        },
        errors
    );

    CHECK(attempts == 1);
    CHECK(returnCode == 1);
    CHECK(errors.str() == "[SIMULATION_ERROR] Unknown error\n");
}
