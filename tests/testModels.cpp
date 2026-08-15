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

TEST_CASE("DoubleIntegratorAdvancesWithExactZeroOrderHold") {
    json settings = {
        {"model-params", {
            {"discharge-rate", 0.1}
        }}
    };
    DoubleIntegrate2D model(settings);
    model.setStateVariable("x", 1.0);
    model.setStateVariable("y", -2.0);
    model.setStateVariable("vx", 3.0);
    model.setStateVariable("vy", -4.0);
    model.setStateVariable("battery", 4100.0);
    model.setStateVariable("yawRad", 0.2);

    VectorXd control(3);
    control << 2.0, 6.0, 0.4;
    model.setControlInput(control);
    model.stepTimeForward(0.5);

    CHECK(model.getStateVariable("x") == doctest::Approx(2.75));
    CHECK(model.getStateVariable("y") == doctest::Approx(-3.25));
    CHECK(model.getStateVariable("vx") == doctest::Approx(4.0));
    CHECK(model.getStateVariable("vy") == doctest::Approx(-1.0));
    CHECK(model.getStateVariable("battery") == doctest::Approx(4099.95));
    CHECK(model.getStateVariable("yawRad") == doctest::Approx(0.4));
}

TEST_CASE("DoubleIntegrator yaw uses exact ZOH and canonical angle wrap") {
    json settings = {{"model-params",{{"discharge-rate",0.1}}}};
    DoubleIntegrate2D model(settings);
    model.setStateVariable("battery",4100.0);
    model.setStateVariable("yawRad",M_PI-0.02);
    VectorXd control(3);
    control << 0.0,0.0,1.0;
    model.setControlInput(control);
    model.stepTimeForward(0.1);
    CHECK(model.getStateVariable("yawRad")==
          doctest::Approx(-M_PI+0.08));
}

TEST_CASE("Pure double-integrator ZOH propagation composes over multiple steps") {
    const Eigen::Vector2d position(1.0, -2.0);
    const Eigen::Vector2d velocity(3.0, -4.0);
    const Eigen::Vector2d acceleration(2.0, 6.0);

    const auto first = propagateDoubleIntegratorPlanarZoh(
        position, velocity, acceleration, 0.5);
    const auto second = propagateDoubleIntegratorPlanarZoh(
        first.position, first.velocity, acceleration, 0.5);

    CHECK(first.position(0) == doctest::Approx(2.75));
    CHECK(first.position(1) == doctest::Approx(-3.25));
    CHECK(first.velocity(0) == doctest::Approx(4.0));
    CHECK(first.velocity(1) == doctest::Approx(-1.0));
    CHECK(second.position(0) == doctest::Approx(5.0));
    CHECK(second.position(1) == doctest::Approx(-3.0));
    CHECK(second.velocity(0) == doctest::Approx(5.0));
    CHECK(second.velocity(1) == doctest::Approx(2.0));
}
