#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "models/BaseModel.hpp"
#include "models/DoubleIntegrate2D.hpp"

TEST_CASE("BaseModelVectorJsonConversionsAcceptVectorsByConstReference") {
    using StateToJson = json (BaseModel::*)(const VectorXd&) const;
    using ControlToJson = json (BaseModel::*)(const VectorXd&) const;

    auto stateToJson = static_cast<StateToJson>(&BaseModel::state2Json);
    auto controlToJson = static_cast<ControlToJson>(&BaseModel::control2Json);

    CHECK(stateToJson != nullptr);
    CHECK(controlToJson != nullptr);
}

TEST_CASE("RandomSeedResolutionUsesConfiguredExecuteSeedWhenPresent") {
    json settings = {
        {"execute", {
            {"random-seed", 1234}
        }}
    };

    CHECK(resolveRandomSeed(settings, 99) == 1234u);
    CHECK(resolveRandomSeed(json::object(), 99) == 99u);
}

TEST_CASE("DoubleIntegratorExposesPlanarAccelerationControl") {
    json settings = {
        {"model-params", {
            {"discharge-rate", 0.1}
        }}
    };
    DoubleIntegrate2D model(settings);

    VectorXd u(3);
    u << 1.5, -0.25, 0.3;
    model.setControlInput(u);

    VectorXd acceleration = model.getAcceleration();
    CHECK(acceleration.size() == 2);
    CHECK(acceleration(0) == doctest::Approx(1.5));
    CHECK(acceleration(1) == doctest::Approx(-0.25));
}
