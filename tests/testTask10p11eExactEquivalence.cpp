#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CanonicalGammaFeedbackEquivalence.hpp"

namespace {

gf::JointEstimateSnapshot snapshot() {
    gf::JointEstimateSnapshot value;
    value.mobile_ids={1,2};
    value.mean=Eigen::VectorXd::Zero(8);
    value.mean.segment<4>(0)<<2.0,2.0,0.1,0.0;
    value.mean.segment<4>(4)<<8.0,2.0,-0.1,0.0;
    value.covariance=1e-4*Eigen::MatrixXd::Identity(8,8);
    return value;
}

gf::CanonicalHardRowRequest request(const gf::JointEstimateSnapshot& value) {
    gf::CanonicalHardRowRequest result;
    result.mobile_ids=value.mobile_ids;
    result.fixed_ids={100};
    result.states[100]={{5.0,8.0},Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero()};
    for (std::size_t index=0;index<value.mobile_ids.size();++index) {
        const auto state=value.mean.segment<4>(4*index);
        result.states[value.mobile_ids[index]]={
            {state.x(),state.y()},state.tail<2>(),Eigen::Vector2d::Zero()};
        result.workspace_snapshot_tubes[value.mobile_ids[index]]={0.1,0.1};
    }
    result.workspace_facets={
        {"x-upper",{1.0,0.0},20.0},{"x-lower",{-1.0,0.0},0.0},
        {"y-upper",{0.0,1.0},20.0},{"y-lower",{0.0,-1.0},0.0}};
    result.reference_edges={{100,1},{100,2}};
    result.reference_spec={PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    result.reference_snapshot_tubes["100->1"]={0.1,0.1};
    result.reference_snapshot_tubes["100->2"]={0.1,0.1};
    result.collision_pairs={gf::UndirectedEdge::canonical(1,2)};
    result.collision_spec={PairwiseSecondOrderBarrierKind::CollisionLower,
        0.5,0.0,1.0,1.0,1.0,0.0};
    result.collision_snapshot_tubes["1--2"]={0.1,0.1};
    result.acceleration_half_box=0.4;
    result.require_snapshot_robust_rows=true;
    return result;
}

gf::CanonicalGammaFeedbackConfig config() {
    gf::CanonicalGammaFeedbackConfig value;
    value.acceleration_half_box=0.4;
    value.homotopy_segments=8;
    value.selection_mode=gf::GammaFeedbackSelectionMode::MaximumPredictedMargin;
    return value;
}

}

TEST_CASE("Optimized gamma batch is field-equivalent to the frozen reference") {
    const auto state=snapshot();
    const std::map<gf::NodeId,Eigen::Vector2d> nominal{
        {1,{0.4,0.1}},{2,{-0.4,-0.1}}};
    gf::CanonicalGammaFeedbackEvaluationContext reference_context;
    gf::CanonicalGammaFeedbackEvaluationContext optimized_context;
    const auto reference=gf::evaluateCanonicalGammaFeedbackBatchReference(
        state,nominal,config(),0.1,0.02,
        [](const auto& value) { return request(value); },reference_context);
    const auto optimized=gf::evaluateCanonicalGammaFeedbackBatchOptimized(
        state,nominal,config(),0.1,0.02,
        [](const auto& value) { return request(value); },optimized_context);

    const auto equivalent=gf::compareCanonicalGammaFeedbackBatches(
        reference,optimized,1e-10);
    INFO("reference owner1 selected=" <<
         reference.selections.at(1).selected_control.transpose() <<
         " gamma=" << reference.selections.at(1).selected_predicted_gamma <<
         " dominant=" << reference.selections.at(1).dominant_row);
    INFO("optimized owner1 selected=" <<
         optimized.selections.at(1).selected_control.transpose() <<
         " gamma=" << optimized.selections.at(1).selected_predicted_gamma <<
         " dominant=" << optimized.selections.at(1).dominant_row);
    CAPTURE(equivalent.first_mismatch);
    CHECK(equivalent.equivalent);
    CHECK(optimized.work.estimator_propagations <
          reference.work.estimator_propagations);
    CHECK(optimized.work.focused_owner_row_rebuilds > 0);
}

TEST_CASE("Equivalence oracle identifies a changed selected control") {
    gf::CanonicalGammaFeedbackBatchResult lhs;
    lhs.valid=true;
    lhs.reason="selected";
    lhs.selected_controls[1]={0.1,0.2};
    auto rhs=lhs;
    rhs.selected_controls[1].x()+=1e-4;
    const auto equivalent=gf::compareCanonicalGammaFeedbackBatches(
        lhs,rhs,1e-10);
    CHECK_FALSE(equivalent.equivalent);
    CHECK(equivalent.first_mismatch=="selected_controls[1]");
}
