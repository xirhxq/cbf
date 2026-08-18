#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CentralizedEkfOracle.hpp"
#include "grand_finale/Task10p11rFimGraphSemantics.hpp"
#include "grand_finale/Task10p11rFixedBaseline.hpp"

namespace {

gf::GrandFinaleRuntimeSnapshot collinearReferenceSnapshot() {
    gf::GrandFinaleRuntimeSnapshot runtime;
    runtime.runtime_s=0.1;
    runtime.estimate.mobile_ids={9,10,11,12};
    runtime.estimate.mean=Eigen::VectorXd::Zero(16);
    runtime.estimate.mean.segment<2>(0)=Eigen::Vector2d(0.0,10.0);
    runtime.estimate.mean.segment<2>(4)=Eigen::Vector2d(-20.0,0.0);
    runtime.estimate.mean.segment<2>(8)=Eigen::Vector2d(-10.0,0.0);
    runtime.estimate.mean.segment<2>(12)=Eigen::Vector2d(0.0,0.0);
    runtime.estimate.covariance=Eigen::MatrixXd::Identity(16,16)*1.0e-4;
    runtime.topology={{10,12},{11,12}};
    for (const auto edge:std::vector<gf::UndirectedEdge>{
        gf::UndirectedEdge::canonical(9,12),
        gf::UndirectedEdge::canonical(10,12),
        gf::UndirectedEdge::canonical(11,12)})
        runtime.range_links[edge.id()]={0.0,1.0,1.0};
    return runtime;
}

gf::AcceptedRangeUpdateAudit accepted(gf::NodeId first,gf::NodeId second) {
    return {{0,gf::UndirectedEdge::canonical(first,second),10.0,1.0},
            first,0.0,1.0};
}

std::vector<gf::MobileEstimate> mobileEstimates(
    const gf::JointEstimateSnapshot& snapshot) {
    std::vector<gf::MobileEstimate> result;
    for (std::size_t index=0;index<snapshot.mobile_ids.size();++index) {
        const Eigen::Index offset=static_cast<Eigen::Index>(4*index);
        result.push_back({snapshot.mobile_ids[index],
            snapshot.mean.segment<4>(offset),
            snapshot.covariance.block<4,4>(offset,offset)});
    }
    return result;
}

std::vector<gf::RangeMeasurement> measurements(
    const std::vector<gf::AcceptedRangeUpdateAudit>& batch) {
    std::vector<gf::RangeMeasurement> result;
    for (const auto& update:batch) result.push_back(update.measurement);
    return result;
}

}

TEST_CASE("Adapter exposes the exact accepted stage-zero information batch") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture();
    const auto initialized=fixture->adapter.initializeStageZero();
    REQUIRE(initialized.initialized);
    const auto batch=fixture->adapter.lastAcceptedRangeBatchAudit();
    CHECK(batch.size()==133);
    std::set<std::string> ids;
    for (const auto& update:batch) {
        CHECK(update.measurement.variance_m2>0.0);
        CHECK(std::isfinite(update.innovation));
        CHECK(update.innovation_variance>0.0);
        CHECK(ids.insert(update.measurement.edge.id()).second);
    }
    auto copy=batch;
    copy.clear();
    CHECK(fixture->adapter.lastAcceptedRangeBatchAudit().size()==133);
}

TEST_CASE("Accepted transverse information edge repairs collinear reference-only FIM") {
    const auto runtime=collinearReferenceSnapshot();
    const std::vector<gf::AcceptedRangeUpdateAudit> batch{
        accepted(9,12),accepted(10,12),accepted(11,12)};
    const std::vector<gf::LeaderCoverageBranchSpec> branches{
        {{9,10,11,12},12,{0.0,0.0},0.0,{}}};
    const auto audit=gf::auditFimGraphSemantics(runtime,batch,12,branches,
        {850.0,1.0,0.5,0.0,1.0e-6});
    REQUIRE(audit.valid);
    CHECK(audit.reference.edges==
          std::vector<gf::DirectedEdge>{{10,12},{11,12}});
    CHECK(audit.authority_augmented.edges==
          std::vector<gf::DirectedEdge>{{9,12},{10,12},{11,12}});
    CHECK(audit.all_accepted.edges==audit.authority_augmented.edges);
    CHECK(audit.reference.robust_cone_lower_bound<1.0e-6);
    CHECK(audit.authority_augmented.robust_cone_lower_bound>1.0e-6);
    CHECK(audit.all_accepted.robust_cone_lower_bound>1.0e-6);
}

TEST_CASE("Information qualification does not alter the hard reference parent count") {
    auto runtime=collinearReferenceSnapshot();
    runtime.range_links.at("9--12").age_s=2.0;
    const std::vector<gf::AcceptedRangeUpdateAudit> batch{
        accepted(9,12),accepted(10,12),accepted(11,12)};
    const std::vector<gf::LeaderCoverageBranchSpec> branches{
        {{9,10,11,12},12,{0.0,0.0},0.0,{}}};
    const auto audit=gf::auditFimGraphSemantics(runtime,batch,12,branches,
        {850.0,1.0,0.5,0.0,1.0e-6});
    REQUIRE(audit.valid);
    CHECK(audit.reference.edges.size()==2);
    CHECK(audit.authority_augmented.edges.size()==2);
    CHECK(audit.rejected_information_edges.at("9--12")=="stale");
}

TEST_CASE("Reference range is not an accepted-information cutoff") {
    auto runtime=collinearReferenceSnapshot();
    runtime.estimate.mean.segment<2>(0)=Eigen::Vector2d(0.0,900.0);
    const std::vector<gf::AcceptedRangeUpdateAudit> batch{
        accepted(9,12),accepted(10,12),accepted(11,12)};
    const std::vector<gf::LeaderCoverageBranchSpec> branches{
        {{9,10,11,12},12,{0.0,0.0},0.0,{}}};
    const auto audit=gf::auditFimGraphSemantics(runtime,batch,12,branches,
        {850.0,1.0,0.5,0.0,1.0e-6});
    REQUIRE(audit.valid);
    CHECK(audit.authority_augmented.edges==
          std::vector<gf::DirectedEdge>{{10,12},{11,12}});
    CHECK(audit.all_accepted.edges==
          std::vector<gf::DirectedEdge>{{9,12},{10,12},{11,12}});
}

TEST_CASE("Frozen owner 12 information graph and D-EKF posterior match the centralized oracle") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture();
    const auto prior=fixture->adapter.runtimeSnapshot().estimate;
    gf::CentralizedEkfOracle centralized(
        mobileEstimates(prior),prior.fixed_positions);
    const auto initialized=fixture->adapter.initializeStageZero();
    REQUIRE(initialized.initialized);

    auto batch=fixture->adapter.lastAcceptedRangeBatchAudit();
    centralized.update(measurements(batch));
    auto runtime=fixture->adapter.runtimeSnapshot();
    auto central=centralized.snapshot();
    CHECK((central.mean-runtime.estimate.mean).cwiseAbs().maxCoeff()<1.0e-10);
    CHECK((central.covariance-runtime.estimate.covariance)
        .cwiseAbs().maxCoeff()<1.0e-10);
    const auto thresholds=gf::FimGraphSemanticsThresholds{
        fixture->adapter.config().reference_distance_m,
        fixture->adapter.config().maximum_range_aoi_s,
        fixture->adapter.config().minimum_range_quality,
        fixture->adapter.config().uncertainty_sigma,1.0e-6};
    const auto stage_zero=gf::auditFimGraphSemantics(runtime,batch,12,
        gf::task10p11rAuthorityContract().branches,thresholds);
    REQUIRE(stage_zero.valid);
    CHECK(stage_zero.reference.robust_cone_lower_bound<1.0e-6);
    CHECK(stage_zero.authority_augmented.robust_cone_lower_bound>1.0e-6);
    CHECK(stage_zero.all_accepted.robust_cone_lower_bound>1.0e-6);
    CHECK(gf::schurEffectivePositionInformation(central,12).isApprox(
        gf::schurEffectivePositionInformation(runtime.estimate,12),1.0e-9));

    const auto step=fixture->controller.advance();
    REQUIRE(step.step.advanced);
    std::vector<Eigen::Vector2d> controls;
    for (const auto id:prior.mobile_ids)
        controls.push_back(step.step.applied_controls.at(id));
    centralized.propagate(controls,fixture->adapter.config().dt_s,
        fixture->adapter.config().estimator_acceleration_variance);
    batch=fixture->adapter.lastAcceptedRangeBatchAudit();
    centralized.update(measurements(batch));
    runtime=fixture->adapter.runtimeSnapshot();
    central=centralized.snapshot();
    CHECK((central.mean-runtime.estimate.mean).cwiseAbs().maxCoeff()<1.0e-9);
    CHECK((central.covariance-runtime.estimate.covariance)
        .cwiseAbs().maxCoeff()<1.0e-9);
    const auto first_step=gf::auditFimGraphSemantics(runtime,batch,12,
        gf::task10p11rAuthorityContract().branches,thresholds);
    REQUIRE(first_step.valid);
    CHECK(first_step.reference.robust_cone_lower_bound<1.0e-6);
    CHECK(first_step.authority_augmented.robust_cone_lower_bound>1.0e-6);
    CHECK(first_step.all_accepted.robust_cone_lower_bound>1.0e-6);
}

TEST_CASE("Formal current FIM gate uses qualified accepted information edges") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture();
    const auto initialized=fixture->adapter.initializeStageZero();
    REQUIRE(initialized.initialized);
    const auto audit=fixture->adapter.currentReferenceAudit();
    CHECK(audit.minimum_effective_reference_count>=2);
    CHECK(audit.minimum_fim_eigenvalue>=1.0e-6);
    CHECK(audit.minimum_robust_fim_cone_lower_bound>=1.0e-6);
}
