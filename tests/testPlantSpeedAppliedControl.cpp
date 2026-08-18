#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CanonicalHardRows.hpp"
#include "grand_finale/CanonicalHocbfQpController.hpp"
#include "grand_finale/PlantSpeedAppliedControl.hpp"
#include "grand_finale/Task10p11pOperationalEnvelope.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <fstream>

TEST_CASE("Continuous tangent speed condition does not certify a ZOH interval") {
    const Eigen::Vector2d velocity(30.0,0.0);
    const Eigen::Vector2d acceleration(0.0,4.0);
    CHECK(velocity.dot(acceleration)==doctest::Approx(0.0));
    const auto audit=gf::auditPlantSpeedExactZoh(
        velocity,acceleration,0.1,30.0,1.0e-12);
    CHECK_FALSE(audit.valid);
    CHECK(audit.maximum_interval_speed_mps>30.0);
}

TEST_CASE("All 64 applied-control facets contain the supported ZOH endpoint") {
    gf::PlantSpeedAppliedControlRequest request;
    request.owner=7;
    request.estimated_velocity_mps={29.98,0.0};
    request.velocity_support_mps=0.02;
    request.speed_limit_mps=30.0;
    request.dt_s=0.1;
    request.facet_count=64;
    const auto rows=gf::buildPlantSpeedAppliedControlRows(request);
    REQUIRE(rows.size()==64);

    const Eigen::Vector2d applied(-0.5,0.0);
    for (const auto& row:rows)
        REQUIRE(row.margin(applied)>=-1.0e-12);

    // Independent worst-direction check: every point on the support circle
    // around the predicted estimator velocity remains in the 30 m/s ball.
    const Eigen::Vector2d predicted=
        request.estimated_velocity_mps+request.dt_s*applied;
    for (int degree=0;degree<360;++degree) {
        const double angle=degree*M_PI/180.0;
        const Eigen::Vector2d error(
            request.velocity_support_mps*std::cos(angle),
            request.velocity_support_mps*std::sin(angle));
        CHECK((predicted+error).norm()<=30.0+1.0e-12);
    }
}

TEST_CASE("Velocity support tightens constants without an initial-set verdict") {
    gf::PlantSpeedAppliedControlRequest clear;
    clear.owner=1;
    clear.estimated_velocity_mps={20.0,3.0};
    clear.speed_limit_mps=30.0;
    clear.dt_s=0.1;
    clear.facet_count=64;
    const auto zero=gf::buildPlantSpeedAppliedControlRows(clear);
    clear.velocity_support_mps=0.25;
    const auto tightened=gf::buildPlantSpeedAppliedControlRows(clear);
    REQUIRE(zero.size()==tightened.size());
    for (std::size_t i=0;i<zero.size();++i) {
        CHECK(tightened[i].id==zero[i].id);
        CHECK(tightened[i].control_coefficient.isApprox(
            zero[i].control_coefficient,1.0e-14));
        CHECK(tightened[i].constant==
              doctest::Approx(zero[i].constant-2.5));
    }
}

TEST_CASE("Exact ZOH speed maximum is attained at an endpoint") {
    const Eigen::Vector2d velocity(12.0,-7.0);
    const Eigen::Vector2d acceleration(-4.0,3.0);
    const double dt=0.1;
    const auto audit=gf::auditPlantSpeedExactZoh(
        velocity,acceleration,dt,30.0,1.0e-12);
    const double endpoint=std::max(
        velocity.norm(),(velocity+dt*acceleration).norm());
    CHECK(audit.maximum_interval_speed_mps==doctest::Approx(endpoint));
    for (int index=0;index<=100;++index) {
        const double time=dt*index/100.0;
        CHECK((velocity+time*acceleration).norm()<=
              audit.maximum_interval_speed_mps+1.0e-12);
    }
}

TEST_CASE("Low-speed facet orientation is deterministic and truth-free") {
    gf::PlantSpeedAppliedControlRequest request;
    request.owner=9;
    request.estimated_velocity_mps={0.0,0.0};
    request.velocity_support_mps=0.01;
    request.speed_limit_mps=30.0;
    request.dt_s=0.1;
    request.facet_count=64;
    const auto first=gf::buildPlantSpeedAppliedControlRows(request);
    const auto second=gf::buildPlantSpeedAppliedControlRows(request);
    REQUIRE(first.size()==second.size());
    for (std::size_t i=0;i<first.size();++i) {
        CHECK(first[i].id==second[i].id);
        CHECK(first[i].facet_normal.isApprox(second[i].facet_normal,0.0));
        CHECK(first[i].constant==second[i].constant);
    }
}

namespace {

gf::CanonicalHardRowRequest canonicalPlantRequest(double support) {
    gf::CanonicalHardRowRequest request;
    request.mobile_ids={1};
    request.states[1]={Point(0.0,0.0),Eigen::Vector2d(29.98,0.0),
                       Eigen::Vector2d::Zero()};
    request.acceleration_half_box=4.0;
    request.reference_spec={PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    request.collision_spec={PairwiseSecondOrderBarrierKind::CollisionLower,
        10.0,0.0,1.0,1.0,1.0,0.0};
    request.speed_limit_mps=30.0;
    request.plant_speed_facet_count=64;
    request.plant_speed_dt_s=0.1;
    request.plant_speed_snapshot_tubes[1]={0.0,support,
        gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    return request;
}

} // namespace

TEST_CASE("Canonical plant-speed rows replace the formal speed HOCBF") {
    const auto rows=gf::buildCanonicalHardRows(canonicalPlantRequest(0.02));
    std::size_t plant=0;
    std::size_t legacy=0;
    for (const auto& row:rows) {
        if (row.kind==gf::CanonicalHardRowKind::PlantSpeedAppliedControl) {
            ++plant;
            CHECK(row.id.find("plant_speed_applied_control:1:facet:")==0);
            CHECK(std::isinf(row.barrier_h));
            CHECK(std::isinf(row.barrier_psi1));
        }
        if (row.kind==gf::CanonicalHardRowKind::SpeedLimit) ++legacy;
    }
    CHECK(plant==64);
    CHECK(legacy==0);
    CHECK(rows.size()==68);
}

TEST_CASE("Velocity tube cannot trigger the removed speed initial-set failure") {
    const auto rows=gf::buildCanonicalHardRows(canonicalPlantRequest(0.03));
    for (const auto profile:{gf::SolverProfile::Gurobi,
                            gf::SolverProfile::OpenSource}) {
        gf::CanonicalHocbfQpController controller;
        const auto solved=controller.solve({
            profile,1,7,3,gf::SupervisorMode::Search,
            Eigen::Vector2d(4.0,4.0),4.0,rows,1.0e-7});
        INFO(solved.failure_reason," status=",solved.solver_status);
        CHECK(solved.failure_reason!="speed_initial_set_violated");
        REQUIRE(solved.control_available);
        CHECK(solved.residual_verified);
        CHECK(solved.minimum_hard_residual>=-1.0e-7);
        CHECK((solved.control-solved.exact_projection).norm()<=1.0e-5);
    }
}

TEST_CASE("Plant-speed rows do not alter collision or reference robust rows") {
    gf::CanonicalHardRowRequest base;
    base.mobile_ids={1,2};
    base.fixed_ids={100};
    base.states[1]={Point(0.0,0.0),Eigen::Vector2d(1.0,2.0),{0.0,0.0}};
    base.states[2]={Point(50.0,0.0),Eigen::Vector2d(-1.0,0.0),{0.0,0.0}};
    base.states[100]={Point(-50.0,0.0),Eigen::Vector2d::Zero(),{0.0,0.0}};
    base.reference_edges={{100,1},{1,2}};
    base.collision_pairs={gf::UndirectedEdge::canonical(1,2),
                          gf::UndirectedEdge::canonical(1,100)};
    base.reference_spec={PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    base.collision_spec={PairwiseSecondOrderBarrierKind::CollisionLower,
        10.0,0.0,1.0,1.0,1.0,0.0};
    base.acceleration_half_box=4.0;
    base.require_snapshot_robust_rows=true;
    base.reference_snapshot_tubes["100->1"]={0.1,0.1};
    base.reference_snapshot_tubes["1->2"]={0.1,0.1};
    base.collision_snapshot_tubes["1--2"]={0.1,0.1};
    base.collision_snapshot_tubes["1--100"]={0.1,0.1};
    const auto without=gf::buildCanonicalHardRows(base);
    auto with=base;
    with.speed_limit_mps=30.0;
    with.plant_speed_facet_count=64;
    with.plant_speed_dt_s=0.1;
    with.plant_speed_snapshot_tubes[1]={0.0,0.02};
    with.plant_speed_snapshot_tubes[2]={0.0,0.02};
    const auto augmented=gf::buildCanonicalHardRows(with);
    for (const auto& expected:without) {
        if (expected.kind==gf::CanonicalHardRowKind::InputBox) continue;
        const auto found=std::find_if(augmented.begin(),augmented.end(),
            [&](const auto& candidate) { return candidate.id==expected.id; });
        REQUIRE(found!=augmented.end());
        CHECK(found->kind==expected.kind);
        CHECK(found->control_coefficient.isApprox(
            expected.control_coefficient,0.0));
        CHECK(found->constant==expected.constant);
        CHECK(found->coefficient_uncertainty_reserve==
              expected.coefficient_uncertainty_reserve);
    }
}

TEST_CASE("Plant preflight rejects one unsafe owner before any swarm state mutates") {
    const auto scenario=gf::task10p11pStandardCoastalScenario();
    auto settings=gf::task10p11pSwarmSettings(
        scenario,gf::SolverProfile::OpenSource);
    Swarm swarm(settings);
    Swarm::CertifiedControlBatch controls;
    std::vector<Eigen::VectorXd> before;
    for (const auto& robot:swarm.robots) {
        controls[robot->id]=Eigen::Vector2d::Zero();
        before.push_back(robot->model->getX());
    }
    swarm.robots.front()->model->setStateVariable("vx",30.0);
    swarm.robots.front()->model->setStateVariable("vy",0.0);
    controls.at(swarm.robots.front()->id)={0.0,4.0};
    before.front()=swarm.robots.front()->model->getX();

    const auto rejected=swarm.applyCertifiedControlsAndAdvance(
        0.1,controls,std::nullopt,30.0);
    CHECK_FALSE(rejected.advanced);
    CHECK(rejected.reason=="plant_speed_preflight_rejected");
    for (std::size_t index=0;index<swarm.robots.size();++index)
        CHECK(swarm.robots[index]->model->getX().isApprox(before[index],0.0));
}

TEST_CASE("Plant preflight applies the certified control unchanged") {
    const auto scenario=gf::task10p11pStandardCoastalScenario();
    auto settings=gf::task10p11pSwarmSettings(
        scenario,gf::SolverProfile::OpenSource);
    Swarm swarm(settings);
    Swarm::CertifiedControlBatch controls;
    for (const auto& robot:swarm.robots)
        controls[robot->id]=Eigen::Vector2d(0.25,-0.5);
    const Eigen::Vector2d velocity=swarm.robots.front()->model->getVelocity();
    const auto advanced=swarm.applyCertifiedControlsAndAdvance(
        0.1,controls,std::nullopt,30.0);
    INFO(advanced.reason);
    REQUIRE(advanced.advanced);
    CHECK(swarm.robots.front()->model->getVelocity().isApprox(
        velocity+0.1*controls.at(swarm.robots.front()->id),1.0e-12));
    CHECK(swarm.robots.front()->model->getAcceleration().isApprox(
        controls.at(swarm.robots.front()->id),0.0));
}

TEST_CASE("Preserved 11.5 second legacy speed snapshot is plant-control feasible") {
    const auto evidence=json::parse(std::ifstream(
        std::string(PROJECT_ROOT)+
        "/../docs/evidence/task10p11p/nominal-only.json"));
    REQUIRE(evidence.at("failure_reason")=="speed_initial_set_violated");
    const auto certificate=evidence.at("failure_certificate");
    REQUIRE(certificate.at("owner")==14);
    const Eigen::Vector2d estimated_velocity(
        certificate.at("estimated_velocity_mps").at(0).get<double>(),
        certificate.at("estimated_velocity_mps").at(1).get<double>());
    const double support=
        certificate.at("velocity_tube_radius_mps").get<double>();

    gf::CanonicalHardRowRequest request;
    request.mobile_ids={14};
    request.states[14]={Point(0.0,0.0),estimated_velocity,
                        Eigen::Vector2d::Zero()};
    request.acceleration_half_box=4.0;
    request.reference_spec={PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    request.collision_spec={PairwiseSecondOrderBarrierKind::CollisionLower,
        10.0,0.0,1.0,1.0,1.0,0.0};
    request.speed_limit_mps=30.0;
    request.plant_speed_facet_count=64;
    request.plant_speed_dt_s=0.1;
    request.plant_speed_snapshot_tubes[14]={0.0,support};
    const auto rows=gf::buildCanonicalHardRows(request);
    const auto gamma=gf::solveCanonicalGammaStar(rows,14,4.0);
    REQUIRE(gamma.valid);
    CHECK(gamma.gamma>=0.0);
    const Eigen::Vector2d gamma_control(gamma.accelX,gamma.accelY);
    CHECK(gamma_control.x()<0.0);
    const Eigen::Vector2d supported_endpoint=
        estimated_velocity+0.1*gamma_control;
    CHECK(supported_endpoint.norm()+support<=30.0+1.0e-9);
    // The preserved evidence did not serialize collision/reference rows.
    // This test therefore closes only the superseded speed-row blocker; the
    // complete union is decided by the fresh standard-scenario replay.
}
