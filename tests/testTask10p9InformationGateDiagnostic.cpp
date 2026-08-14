#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/InformationGateDiagnostic.hpp"

namespace {
gf::InformationGateOwnerInput healthy() {
    gf::InformationGateOwnerInput value;
    value.owner = 2;
    value.initialized = true;
    value.posterior_margin_m2 = 0.01;
    value.nominal_fim_eigenvalue = 0.2;
    value.posterior_fim_proxy_eigenvalue = 0.18;
    value.robust_cone_fim_lower_bound = 0.12;
    value.minimum_fim_eigenvalue = 1e-6;
    value.candidate_requested = true;
    value.candidate_generated = true;
    value.solver_succeeded = true;
    value.edges = {
        {{100,2}, true, true, true, 1.0, 0.1, true},
        {{101,2}, true, true, true, 1.0, 0.1, true}};
    return value;
}
}

TEST_CASE("Task 10.9 information certificate keeps FIM meanings separate") {
    const auto certificate = gf::diagnoseInformationGate(healthy());
    CHECK(certificate.accepted);
    CHECK(certificate.nominal_fim_eigenvalue == doctest::Approx(0.2));
    CHECK(certificate.posterior_fim_proxy_eigenvalue == doctest::Approx(0.18));
    CHECK(certificate.robust_cone_fim_lower_bound == doctest::Approx(0.12));
    CHECK(certificate.effective_reference_count == 2);
}

TEST_CASE("Task 10.9 information root cause distinguishes initialization and data flow") {
    auto value = healthy();
    value.initialized = false;
    auto certificate = gf::diagnoseInformationGate(value);
    CHECK(certificate.cause ==
          gf::InformationFailureCause::InsufficientInitialization);
    CHECK(certificate.first_failed_gate == "initialization");

    value = healthy();
    value.edges.front().range_present = false;
    certificate = gf::diagnoseInformationGate(value);
    CHECK(certificate.cause == gf::InformationFailureCause::DataFlowError);
    CHECK(certificate.first_failed_gate == "range_missing:100->2");
}

TEST_CASE("Task 10.9 information root cause distinguishes scientific and candidate failures") {
    auto value = healthy();
    value.posterior_fim_proxy_eigenvalue = 0.0;
    auto certificate = gf::diagnoseInformationGate(value);
    CHECK(certificate.cause == gf::InformationFailureCause::ScientificRejection);
    CHECK(certificate.first_failed_gate == "posterior_fim_proxy");

    value = healthy();
    value.robust_cone_fim_lower_bound = 0.0;
    certificate = gf::diagnoseInformationGate(value);
    CHECK(certificate.cause == gf::InformationFailureCause::ScientificRejection);
    CHECK(certificate.first_failed_gate == "robust_cone_fim");

    value = healthy();
    value.candidate_generated = false;
    certificate = gf::diagnoseInformationGate(value);
    CHECK(certificate.cause ==
          gf::InformationFailureCause::CandidateGenerationDefect);

    value = healthy();
    value.solver_succeeded = false;
    certificate = gf::diagnoseInformationGate(value);
    CHECK(certificate.cause == gf::InformationFailureCause::SolverFailure);
}

TEST_CASE("Task 10.9 robust cone FIM is named and lower than nominal FIM") {
    gf::JointEstimateSnapshot estimate;
    estimate.mobile_ids = {1};
    estimate.fixed_positions = {{10,{10.0,0.0}}, {11,{0.0,10.0}}};
    estimate.mean = Eigen::Vector4d::Zero();
    estimate.covariance = 1e-4 * Eigen::Matrix4d::Identity();
    const std::vector<gf::DirectedEdge> edges{{10,1},{11,1}};
    const std::map<std::string,double> variance{
        {"1--10",1.0}, {"1--11",1.0}};
    const double nominal = Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
        gf::referenceFim(1, edges, estimate, variance))
        .eigenvalues().minCoeff();
    const double robust = gf::robustReferenceFimConeLowerBound(
        1, edges, estimate, variance, 3.0);
    CHECK(robust < nominal);
    CHECK(robust > 0.0);
}
