#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "utils.h"
#include "cbf/CBFConfig.hpp"

TEST_CASE("ConfiguredRandomSeedOverridesFallback") {
    json settings = {{"execute", {{"random-seed", 1234}}}};

    CHECK(resolveRandomSeed(settings, 99) == 1234u);
    CHECK(resolveRandomSeed(json::object(), 99) == 99u);
}

TEST_CASE("NestedClassKConfigurationIsRead") {
    json config = {{"alpha", {{"coe", 0.5}, {"pow", 3}}}};

    ClassKParameters parameters = readClassKParameters(config, 0.1, 1);

    CHECK(parameters.coefficient == doctest::Approx(0.5));
    CHECK(parameters.power == 3);
}
