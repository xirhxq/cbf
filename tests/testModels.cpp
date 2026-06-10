#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "models/BaseModel.hpp"

TEST_CASE("BaseModelVectorJsonConversionsAcceptVectorsByConstReference") {
    using StateToJson = json (BaseModel::*)(const VectorXd&) const;
    using ControlToJson = json (BaseModel::*)(const VectorXd&) const;

    auto stateToJson = static_cast<StateToJson>(&BaseModel::state2Json);
    auto controlToJson = static_cast<ControlToJson>(&BaseModel::control2Json);

    CHECK(stateToJson != nullptr);
    CHECK(controlToJson != nullptr);
}
