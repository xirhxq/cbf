#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "diagnostics/EvidenceStream.hpp"

#include <nlohmann/json.hpp>

#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

TEST_CASE("every flushed evidence prefix is independently parseable") {
    std::ostringstream output;
    cbf2026::diagnostics::EvidenceStream stream(output);

    stream.write({{"record_type", "initialization"}, {"robot_id", 1}});
    stream.flush();
    stream.write({{"record_type", "edge"}, {"owner", 1}});
    stream.flush();
    stream.write({{"record_type", "controller_interval"}, {"complete", true}});
    stream.flush();

    std::istringstream lines(output.str());
    std::string line;
    std::vector<std::string> prefixes;
    std::string prefix;
    while (std::getline(lines, line)) {
        prefix += line + "\n";
        prefixes.push_back(prefix);
        const auto parsed = nlohmann::json::parse(line);
        CHECK(parsed.is_object());
    }
    REQUIRE(prefixes.size() == 3);
    for (const auto& candidate : prefixes) {
        std::istringstream prefixLines(candidate);
        std::size_t parsed = 0;
        while (std::getline(prefixLines, line)) {
            const auto row = nlohmann::json::parse(line);
            CHECK(row.is_object());
            ++parsed;
        }
        CHECK(parsed >= 1);
    }
}

TEST_CASE("evidence stream emits compact objects and rejects nonfinite data") {
    std::ostringstream output;
    cbf2026::diagnostics::EvidenceStream stream(output);

    stream.write({{"record_type", "initialization"}, {"robot_id", 1}});
    CHECK(
        output.str()
        == "{\"record_type\":\"initialization\",\"robot_id\":1}\n"
    );
    CHECK_THROWS_AS(stream.write(nlohmann::json::array()), std::invalid_argument);
    CHECK_THROWS_AS(
        stream.write({{"bad", std::numeric_limits<double>::infinity()}}),
        std::invalid_argument
    );
    CHECK_THROWS_AS(
        stream.write({{"nested", {{"bad", std::numeric_limits<double>::quiet_NaN()}}}}),
        std::invalid_argument
    );
    const auto discarded = nlohmann::json::parse("{", nullptr, false);
    CHECK(discarded.is_discarded());
    CHECK_THROWS_AS(stream.write(discarded), std::invalid_argument);
}

TEST_CASE("evidence stream reports write and flush failures") {
    std::ostringstream failedWrite;
    failedWrite.setstate(std::ios::badbit);
    cbf2026::diagnostics::EvidenceStream writeStream(failedWrite);
    CHECK_THROWS_AS(
        writeStream.write({{"record_type", "mission_terminal"}}),
        std::runtime_error
    );

    std::ostringstream failedFlush;
    cbf2026::diagnostics::EvidenceStream flushStream(failedFlush);
    failedFlush.setstate(std::ios::badbit);
    CHECK_THROWS_AS(flushStream.flush(), std::runtime_error);
}

TEST_CASE("reset witness store returns the exact guard solve without re-solving") {
    struct Witness {
        int solveSequence;
        std::vector<double> solution;
    };
    cbf2026::diagnostics::ExactResetWitnessStore<Witness> witnesses;
    witnesses.record(0, 3, Witness{1, {1.25, -2.5, 0.1}});
    const Witness hypotheticalSecondSolve{2, {9.0, 9.0, 0.0}};

    const Witness& emitted = witnesses.at(0, 3);

    CHECK(emitted.solveSequence == 1);
    CHECK(emitted.solution == std::vector<double>{1.25, -2.5, 0.1});
    CHECK(emitted.solveSequence != hypotheticalSecondSolve.solveSequence);
    CHECK_THROWS_AS(
        witnesses.record(0, 3, hypotheticalSecondSolve),
        std::logic_error
    );
    CHECK_THROWS_AS(witnesses.at(0, 4), std::out_of_range);
    witnesses.eraseFrame(0);
    CHECK_THROWS_AS(witnesses.at(0, 3), std::out_of_range);
}
