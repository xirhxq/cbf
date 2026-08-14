#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CanonicalGammaStarFeedback.hpp"

namespace {

gf::CanonicalHardRow row(
    std::string id,const Eigen::Vector2d& coefficient,double constant) {
    return {std::move(id),gf::CanonicalHardRowKind::Workspace,1,std::nullopt,
            coefficient,coefficient,constant,1.0,true};
}

}

TEST_CASE("Canonical gamma feedback exact projection witness and homotopy share the current ledger") {
    const std::vector<gf::CanonicalHardRow> rows{
        row("x-lower",{1.0,0.0},0.2),
        row("x-upper",{-1.0,0.0},0.2),
        row("y-lower",{0.0,1.0},0.1),
        row("y-upper",{0.0,-1.0},0.3)};
    gf::CanonicalGammaFeedbackConfig config;
    config.acceleration_half_box=0.4;
    config.homotopy_segments=4;
    const auto stage=gf::buildCanonicalGammaFeedbackStage(
        rows,1,Eigen::Vector2d(0.4,0.4),config);

    REQUIRE(stage.valid);
    CHECK(stage.current_gamma==doctest::Approx(0.2));
    CHECK(stage.task_projection.x()==doctest::Approx(0.2));
    CHECK(stage.task_projection.y()==doctest::Approx(0.3));
    CHECK(stage.maximum_margin_control.x()==doctest::Approx(0.0));
    CHECK(stage.maximum_margin_control.y()==doctest::Approx(0.1));
    REQUIRE(stage.candidates.size()==5);
    for (const auto& candidate:stage.candidates)
        CHECK(gf::minimumCanonicalOwnerResidual(rows,1,candidate)>=-1e-12);
    CHECK(stage.current_row_ids==std::vector<std::string>{
        "x-lower","x-upper","y-lower","y-upper"});
    REQUIRE(stage.current_rows.size()==rows.size());
    for (std::size_t index=0;index<rows.size();++index) {
        CHECK(stage.current_rows[index].id==rows[index].id);
        CHECK(stage.current_rows[index].control_coefficient.isApprox(
            rows[index].control_coefficient,0.0));
        CHECK(stage.current_rows[index].constant==rows[index].constant);
    }
}

TEST_CASE("Frozen-neighbor local prediction is not an actual joint-next guarantee") {
    gf::JointEstimateSnapshot snapshot;
    snapshot.mobile_ids={1,2};
    snapshot.mean=Eigen::VectorXd::Zero(8);
    snapshot.mean.segment<4>(0)<<-1.0,0.0,0.2,0.0;
    snapshot.mean.segment<4>(4)<<1.0,0.0,-0.2,0.0;
    snapshot.covariance=1e-4*Eigen::MatrixXd::Identity(8,8);
    const std::map<gf::NodeId,Eigen::Vector2d> frozen_neighbor{
        {1,{0.4,0.0}},{2,{0.0,0.0}}};
    const std::map<gf::NodeId,Eigen::Vector2d> simultaneous_change{
        {1,{0.4,0.0}},{2,{-0.4,0.0}}};
    const auto local=gf::predictNoMeasurementSnapshot(
        snapshot,frozen_neighbor,0.1,0.0);
    const auto joint=gf::predictNoMeasurementSnapshot(
        snapshot,simultaneous_change,0.1,0.0);
    CHECK_FALSE(local.mean.isApprox(joint.mean,1e-12));
    CHECK(local.mean(4)-joint.mean(4)==doctest::Approx(0.002));
    CHECK(local.mean(6)-joint.mean(6)==doctest::Approx(0.04));
}

TEST_CASE("Feedback stages retain robust mobile half rows and full fixed workspace responsibility") {
    gf::CanonicalHardRowRequest request;
    request.mobile_ids={1,2};
    request.fixed_ids={100};
    request.states={
        {1,{{2.0,2.0},{0.0,0.0},{0.0,0.0}}},
        {2,{{5.0,2.0},{0.0,0.0},{0.0,0.0}}},
        {100,{{0.0,2.0},{0.0,0.0},{0.0,0.0}}}};
    request.reference_edges={{100,1},{1,2}};
    request.collision_pairs={gf::UndirectedEdge::canonical(1,2)};
    request.reference_spec={
        PairwiseSecondOrderBarrierKind::CommunicationUpper,
        10.0,0.0,1.0,1.0,1.0,0.0};
    request.collision_spec={
        PairwiseSecondOrderBarrierKind::CollisionLower,
        1.0,0.0,1.0,1.0,1.0,0.0};
    request.acceleration_half_box=0.4;
    request.require_snapshot_robust_rows=true;
    request.reference_snapshot_tubes={
        {"100->1",{0.1,0.1}}, {"1->2",{0.1,0.1}}};
    request.collision_snapshot_tubes={{"1--2",{0.1,0.1}}};
    request.workspace_facets={{"x-upper",{1.0,0.0},20.0}};
    request.workspace_snapshot_tubes={{1,{0.1,0.1}},{2,{0.1,0.1}}};
    const auto rows=gf::buildCanonicalHardRows(request);
    gf::CanonicalGammaFeedbackConfig config;
    config.acceleration_half_box=0.4;
    const auto first=gf::buildCanonicalGammaFeedbackStage(
        rows,1,Eigen::Vector2d::Zero(),config);
    const auto second=gf::buildCanonicalGammaFeedbackStage(
        rows,2,Eigen::Vector2d::Zero(),config);
    REQUIRE(first.valid);
    REQUIRE(second.valid);

    const auto first_half=std::find_if(rows.begin(),rows.end(),[](const auto& item) {
        return item.id=="collision:1--2:owner:1";
    });
    const auto second_half=std::find_if(rows.begin(),rows.end(),[](const auto& item) {
        return item.id=="collision:1--2:owner:2";
    });
    REQUIRE(first_half!=rows.end());
    REQUIRE(second_half!=rows.end());
    CHECK(first_half->responsibility==doctest::Approx(0.5));
    CHECK(second_half->responsibility==doctest::Approx(0.5));
    CHECK(first_half->margin(first.task_projection)>=-1e-10);
    CHECK(second_half->margin(second.task_projection)>=-1e-10);
    const auto coupled=gf::buildSnapshotRobustPairRow(
        request.states.at(1),request.states.at(2),request.collision_spec,
        request.collision_snapshot_tubes.at("1--2"),
        request.acceleration_half_box);
    const double coupled_lower=
        coupled.nominal_control_coefficient.dot(first.task_projection)-
        coupled.nominal_control_coefficient.dot(second.task_projection)+
        coupled.central_constant_lower;
    CHECK(coupled_lower>=-1e-10);
    CHECK(coupled_lower>=
        first_half->margin(first.task_projection)+
        second_half->margin(second.task_projection)-1e-10);

    const auto fixed=std::find_if(rows.begin(),rows.end(),[](const auto& item) {
        return item.id=="reference:100->1:owner:1";
    });
    REQUIRE(fixed!=rows.end());
    CHECK(fixed->responsibility==doctest::Approx(1.0));
    const auto workspace=std::find_if(rows.begin(),rows.end(),[](const auto& item) {
        return item.kind==gf::CanonicalHardRowKind::Workspace && item.owner==1;
    });
    REQUIRE(workspace!=rows.end());
    CHECK(workspace->responsibility==doctest::Approx(1.0));
}

TEST_CASE("Negative exact current gamma fails closed before prediction") {
    const std::vector<gf::CanonicalHardRow> impossible{
        row("left",{1.0,0.0},-0.5),row("right",{-1.0,0.0},-0.5)};
    gf::CanonicalGammaFeedbackConfig config;
    config.acceleration_half_box=0.4;
    const auto stage=gf::buildCanonicalGammaFeedbackStage(
        impossible,1,Eigen::Vector2d::Zero(),config);
    CHECK_FALSE(stage.valid);
    CHECK(stage.reason=="current_gamma_negative");
    CHECK(stage.current_gamma<0.0);
}

TEST_CASE("No-measurement prediction propagates the full joint covariance") {
    gf::JointEstimateSnapshot snapshot;
    snapshot.mobile_ids={1,2};
    snapshot.mean=Eigen::VectorXd::Zero(8);
    snapshot.mean.segment<4>(0)<<1.0,2.0,3.0,4.0;
    snapshot.mean.segment<4>(4)<<5.0,6.0,7.0,8.0;
    snapshot.covariance=Eigen::MatrixXd::Identity(8,8);
    snapshot.covariance(0,4)=snapshot.covariance(4,0)=0.25;
    const std::map<gf::NodeId,Eigen::Vector2d> controls{
        {1,{0.2,-0.1}},{2,{-0.3,0.4}}};
    const auto predicted=gf::predictNoMeasurementSnapshot(
        snapshot,controls,0.1,0.5);

    CHECK(predicted.mean(0)==doctest::Approx(1.301));
    CHECK(predicted.mean(1)==doctest::Approx(2.3995));
    CHECK(predicted.mean(2)==doctest::Approx(3.02));
    CHECK(predicted.mean(3)==doctest::Approx(3.99));
    CHECK(predicted.covariance(0,4)==doctest::Approx(0.25));
    CHECK(predicted.covariance(0,6)==doctest::Approx(0.0));
    CHECK(predicted.covariance(2,6)==doctest::Approx(0.0));
    CHECK(predicted.covariance.isApprox(
        predicted.covariance.transpose(),1e-12));
}

TEST_CASE("Predictive selection uses fallback semantics without claiming joint next state") {
    const std::vector<gf::CanonicalHardRow> rows{
        row("x-lower",{1.0,0.0},0.2),row("x-upper",{-1.0,0.0},0.2)};
    gf::CanonicalGammaFeedbackConfig config;
    config.acceleration_half_box=0.4;
    config.homotopy_segments=2;
    config.selection_mode=gf::GammaFeedbackSelectionMode::LeastIntervention;
    config.predictive_tau_mps2=0.5;
    const auto stage=gf::buildCanonicalGammaFeedbackStage(
        rows,1,Eigen::Vector2d(0.2,0.0),config);
    REQUIRE(stage.valid);

    std::size_t call=0;
    const auto selected=gf::selectCanonicalGammaFeedback(
        stage,config,[&](const Eigen::Vector2d&) {
            const std::vector<double> scores{0.1,0.3,0.4};
            return gf::CanonicalPredictedGammaScore{
                true,scores.at(call++),"predicted-row"};
        });
    REQUIRE(selected.valid);
    CHECK(selected.fallback_reason=="tau_unattained_maximum_predicted_margin");
    CHECK(selected.selected_predicted_gamma==doctest::Approx(0.4));
    CHECK_FALSE(selected.actual_next_gamma_guaranteed);

    call=0;
    const auto invalid=gf::selectCanonicalGammaFeedback(
        stage,config,[&](const Eigen::Vector2d&) {
            ++call;
            return gf::CanonicalPredictedGammaScore{};
        });
    REQUIRE(invalid.valid);
    CHECK(invalid.selected_control.isApprox(stage.task_projection));
    CHECK(invalid.fallback_reason=="invalid_prediction_use_current_projection");
}

TEST_CASE("Predictive tau is an explicit nonnegative reserve threshold") {
    gf::CanonicalGammaFeedbackConfig config;
    config.acceleration_half_box=0.4;
    config.selection_mode=gf::GammaFeedbackSelectionMode::LeastIntervention;
    config.predictive_tau_mps2=-0.01;
    CHECK_THROWS_AS(
        gf::buildCanonicalGammaFeedbackStage(
            {row("x",{1.0,0.0},1.0)},1,Eigen::Vector2d::Zero(),config),
        std::invalid_argument);
}
