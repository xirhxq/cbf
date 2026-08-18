#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CanonicalHocbfQpController.hpp"
#include "grand_finale/Task10p11gFrozenModel.hpp"
#include "grand_finale/CoverageScaleBudget.hpp"

#include <fstream>

TEST_CASE("Task 10.11g model constants are explicit and dimensionally unambiguous") {
    const auto model=gf::task10p11gFrozenModel();
    CHECK(model.speed_limit_mps==doctest::Approx(30.0));
    CHECK(model.acceleration_half_box_mps2==doctest::Approx(4.0));
    CHECK(model.collision_distance_m==doctest::Approx(10.0));
    CHECK(model.maximum_acceleration_norm_mps2==
          doctest::Approx(4.0*std::sqrt(2.0)));
    CHECK(model.control_period_s==doctest::Approx(0.1));
    CHECK(model.planner_period_s==doctest::Approx(1.0));
    CHECK(model.predictive_tau_mps2==doctest::Approx(0.0));
    CHECK(model.gamma_selection==gf::GammaFeedbackSelectionMode::LeastIntervention);
    const auto frozen=json::parse(std::ifstream(std::string(PROJECT_ROOT)+
        "/config/grand_finale/task10p11g_model_freeze.json"));
    CHECK(frozen.at("research_model").at("horizontal_speed_norm_limit_mps")
          .get<double>()==doctest::Approx(model.speed_limit_mps));
    CHECK(frozen.at("research_model").at("plant_speed_facet_count")==64);
    CHECK(frozen.at("research_model").at("speed_constraint_semantics")==
          "zoh_safe_plant_applied_control_domain");
    CHECK(frozen.at("research_model")
          .at("acceleration_component_half_box_mps2").get<double>()==
          doctest::Approx(model.acceleration_half_box_mps2));
    CHECK(frozen.at("research_model").at("collision_distance_m").get<double>()==
          doctest::Approx(model.collision_distance_m));
    CHECK(frozen.at("coverage_heading").at("initial_yaw_deg").get<double>()==
          doctest::Approx(model.initial_yaw_rad*180.0/M_PI));
    CHECK(frozen.at("scheduler").at("logical_planner_period_s").get<double>()==
          doctest::Approx(model.planner_period_s));
}

TEST_CASE("Position and yaw soft tasks cannot alter the hard-row ledger") {
    const auto model=gf::task10p11gFrozenModel();
    gf::Task10p11gSoftTaskRequest request;
    request.position={1.0,2.0};
    request.velocity={3.0,-1.0};
    request.yaw_rad=M_PI-0.05;
    request.committed_target=Eigen::Vector2d(-10.0,1.0);
    request.mode=gf::SupervisorMode::Search;
    const auto result=gf::task10p11gSoftTask(request,model);
    CHECK(result.acceleration.allFinite());
    CHECK(result.acceleration.lpNorm<Eigen::Infinity>()<=4.0+1.0e-12);
    CHECK(std::abs(result.yaw_rate_radps)<=model.maximum_yaw_rate_radps+1e-12);
    CHECK(result.target_tracking_active);

    const auto hold=gf::task10p11gSoftTask({
        request.position,request.velocity,request.yaw_rad,
        request.committed_target,gf::SupervisorMode::Hold},model);
    CHECK_FALSE(hold.target_tracking_active);
    CHECK(hold.yaw_rate_radps==doctest::Approx(0.0));
    CHECK(hold.acceleration.x()<0.0);
    CHECK(hold.acceleration.y()>0.0);
    for (const auto mode : {gf::SupervisorMode::Reform,
                            gf::SupervisorMode::Union}) {
        auto changed=request;
        changed.mode=mode;
        const auto mode_task=gf::task10p11gSoftTask(changed,model);
        CHECK_FALSE(mode_task.target_tracking_active);
        CHECK(mode_task.yaw_rate_radps==doctest::Approx(0.0));
    }
    auto retreat=request;
    retreat.mode=gf::SupervisorMode::Retreat;
    const auto retreat_task=gf::task10p11gSoftTask(retreat,model);
    CHECK_FALSE(retreat_task.target_tracking_active);
    CHECK(retreat_task.yaw_rate_radps==doctest::Approx(0.0));
}

TEST_CASE("Yaw exact ZOH wraps and bounded controller follows shortest angle") {
    const auto model=gf::task10p11gFrozenModel();
    CHECK(gf::wrapYawRad(M_PI)==doctest::Approx(-M_PI));
    CHECK(gf::propagateYawExactZoh(M_PI-0.02,1.0,0.1)==
          doctest::Approx(-M_PI+0.08));

    gf::Task10p11gSoftTaskRequest request;
    request.yaw_rad=M_PI-0.05;
    request.committed_target=Eigen::Vector2d(-10.0,-1.0);
    request.mode=gf::SupervisorMode::Search;
    const auto result=gf::task10p11gSoftTask(request,model);
    CHECK(result.yaw_error_rad>0.0);
    CHECK(result.yaw_rate_radps>0.0);

    const gf::SectorFootprintSpec sector{0.0,400.0,M_PI/3.0};
    const gf::SectorFootprintStage before{{0.0,0.0},0.0};
    const gf::SectorFootprintStage after{{0.0,0.0},M_PI/2.0};
    CHECK(gf::sectorContains(before,sector,{100.0,0.0}));
    CHECK_FALSE(gf::sectorContains(after,sector,{100.0,0.0}));
    CHECK(gf::sectorContains(after,sector,{0.0,100.0}));
}

TEST_CASE("Both QP profiles match exact projection with the 64-facet plant domain") {
    gf::CanonicalHardRowRequest hard;
    hard.mobile_ids={1};
    hard.states[1]={{0.0,0.0},{29.0,0.0},Eigen::Vector2d::Zero()};
    hard.reference_spec={PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    hard.collision_spec={PairwiseSecondOrderBarrierKind::CollisionLower,
        1.0,0.0,1.0,1.0,1.0,0.0};
    hard.acceleration_half_box=4.0;
    hard.speed_limit_mps=30.0;
    hard.plant_speed_facet_count=64;
    hard.plant_speed_dt_s=0.1;
    hard.plant_speed_snapshot_tubes[1]={0.0,0.2,
        gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    const auto rows=gf::buildCanonicalHardRows(hard);
    CHECK(std::count_if(rows.begin(),rows.end(),[](const auto& row) {
        return row.kind==gf::CanonicalHardRowKind::PlantSpeedAppliedControl;
    })==64);
    CHECK(std::none_of(rows.begin(),rows.end(),[](const auto& row) {
        return row.kind==gf::CanonicalHardRowKind::SpeedLimit;
    }));
    const auto gamma=gf::solveCanonicalGammaStar(rows,1,4.0);
    REQUIRE(gamma.valid);
    CHECK(gamma.gamma>=0.0);

    gf::CanonicalHocbfQpController controller;
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        const auto solved=controller.solve({profile,1,9,7,
            gf::SupervisorMode::Search,{4.0,0.5},4.0,rows,1.0e-7});
        REQUIRE(solved.control_available);
        CHECK(solved.residual_verified);
        CHECK(solved.exact_oracle_error<=1.0e-5);
        CHECK(solved.minimum_hard_residual>=-1.0e-7);
    }
}
