#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "Swarm.hpp"

#include <fstream>
#include <map>
#include <optional>

namespace {

json baseSettings() {
    return json::parse(std::ifstream(
        std::string(PROJECT_ROOT) + "/config/config_second_order.json"));
}

}  // namespace

TEST_CASE("Reusable Swarm step applies certified controls through real Robot ZOH states") {
    json settings = baseSettings();
    Swarm swarm(settings);
    const Point first_before = swarm.robots.at(0)->model->xy();
    const Eigen::VectorXd velocity_before =
        swarm.robots.at(0)->model->getVelocity();
    bool hook_saw_exchanged_state = false;

    const auto result = swarm.stepWithCertifiedControls(
        0.2, [&](Swarm& current)
            -> std::optional<std::map<int, Eigen::Vector2d>> {
            hook_saw_exchanged_state =
                current.robots.at(1)->comm->_othersPos.count(1) == 1;
            return std::map<int, Eigen::Vector2d>{
                {1, {0.4, 0.0}}, {2, {-0.4, 0.0}}};
        });

    REQUIRE(result.advanced);
    CHECK(hook_saw_exchanged_state);
    CHECK(swarm.robots.at(0)->runtime == doctest::Approx(0.2));
    const Point first_after = swarm.robots.at(0)->model->xy();
    CHECK(first_after.x == doctest::Approx(
        first_before.x + velocity_before.x() * 0.2 + 0.5 * 0.4 * 0.04));
    CHECK(first_after.y == doctest::Approx(
        first_before.y + velocity_before.y() * 0.2));
    CHECK(result.updated_truth_cells >= 1);
}

TEST_CASE("Reusable Swarm step does not advance or apply an absent certified batch") {
    json settings = baseSettings();
    Swarm swarm(settings);
    const Point before = swarm.robots.at(0)->model->xy();
    const Eigen::VectorXd acceleration_before =
        swarm.robots.at(0)->model->getAcceleration();

    const auto result = swarm.stepWithCertifiedControls(
        0.2, [](Swarm&)
            -> std::optional<std::map<int, Eigen::Vector2d>> {
            return std::nullopt;
        });

    CHECK_FALSE(result.advanced);
    CHECK(result.reason == "certified_control_unavailable");
    CHECK(swarm.robots.at(0)->runtime == doctest::Approx(0.0));
    CHECK(swarm.robots.at(0)->model->xy().x == doctest::Approx(before.x));
    CHECK((swarm.robots.at(0)->model->getAcceleration() - acceleration_before).norm()
          == doctest::Approx(0.0));
}
