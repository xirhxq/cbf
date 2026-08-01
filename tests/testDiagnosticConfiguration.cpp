#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "utils.h"
#include "cbf/CBFConfig.hpp"
#include "cbf/HybridCertificateGuard.hpp"

#include <fstream>

namespace {

json loadJson(const std::string& path) {
    std::ifstream stream(path);
    REQUIRE(stream.good());
    return json::parse(stream);
}

void mergeOverlay(json& materialized, const json& overlay) {
    for (const auto& [key, value] : overlay.items()) {
        if (value.is_object()
            && materialized.contains(key)
            && materialized.at(key).is_object()) {
            mergeOverlay(materialized[key], value);
        } else {
            materialized[key] = value;
        }
    }
}

}

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

TEST_CASE("qualified materialized primary config passes strict startup gate") {
    json materialized = loadJson("config/config.json");
    mergeOverlay(
        materialized,
        loadJson(
            "config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json"
        )
    );

    CHECK(cbf2026::validateQualifiedMaterializedConfig(
        materialized, {"Gurobi"}
    ));
}

TEST_CASE("qualified materialized ablation passes the same startup gate") {
    json materialized = loadJson("config/config.json");
    mergeOverlay(
        materialized,
        loadJson(
            "config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json"
        )
    );

    CHECK(cbf2026::validateQualifiedMaterializedConfig(
        materialized, {"Gurobi"}
    ));
}

TEST_CASE("qualified materialized startup gate fails closed") {
    json valid = loadJson("config/config.json");
    mergeOverlay(
        valid,
        loadJson(
            "config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json"
        )
    );
    std::vector<json> invalid;
    auto candidate = valid;
    candidate["position_covariance"]["enable"] = false;
    invalid.push_back(candidate);
    candidate = valid;
    candidate["position_covariance"]["uncertainty-type"] = "std_avg";
    invalid.push_back(candidate);
    candidate = valid;
    candidate["model"] = "DoubleIntegrate2D";
    invalid.push_back(candidate);
    candidate = valid;
    candidate["cbfs"]["without-slack"]["method"] = "min";
    invalid.push_back(candidate);
    candidate = valid;
    candidate["cbfs"]["without-slack"]["energy"]["on"] = true;
    invalid.push_back(candidate);
    candidate = valid;
    candidate["cbfs"]["without-slack"]["comm-auto"]["on"] = true;
    invalid.push_back(candidate);
    candidate = valid;
    candidate["qualified-estimator"]["unexpected"] = true;
    invalid.push_back(candidate);
    candidate = valid;
    candidate["qualified-estimator"]["deployment"]["anchor-ids"][0] = false;
    invalid.push_back(candidate);
    candidate = valid;
    candidate["qualified-estimator"]["deployment"]["ocean-side"] = true;
    invalid.push_back(candidate);
    candidate = valid;
    candidate["qualified-estimator"]["deployment"]["unit-normal"] =
        json::array({true, false});
    invalid.push_back(candidate);
    candidate = valid;
    candidate["qualified-estimator"]["history"]["public-max-age-frames"] = true;
    invalid.push_back(candidate);
    candidate = valid;
    candidate["optimiser"] = "HiGHS";
    invalid.push_back(candidate);
    candidate = valid;
    candidate["cbfs"]["input-limits"]["on"] = 1;
    invalid.push_back(candidate);
    candidate = valid;
    candidate["cbfs"]["without-slack"]["safety"]["on"] = 1;
    invalid.push_back(candidate);
    candidate = valid;
    candidate["cbfs"]["without-slack"]["comm-fixed"]["on"] = 1;
    invalid.push_back(candidate);

    for (const auto& config : invalid) {
        CHECK_FALSE(cbf2026::validateQualifiedMaterializedConfig(
            config, {"Gurobi"}
        ));
    }
    CHECK_FALSE(cbf2026::validateQualifiedMaterializedConfig(
        valid, {"HiGHS"}
    ));
    CHECK_FALSE(cbf2026::validateQualifiedMaterializedConfig(
        invalid.back(), {"Gurobi", "HiGHS"}
    ));
}
