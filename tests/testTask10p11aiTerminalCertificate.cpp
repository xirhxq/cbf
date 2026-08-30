#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/Task10p11aiTerminalCertificate.hpp"

#include <filesystem>
#include <random>

namespace {

std::filesystem::path packed157p8() {
    return std::filesystem::path(PROJECT_ROOT).parent_path()/"docs"/"evidence"/
        "task10p11ag-full-domain-predecessor-recovery"/"gate2"/
        "derived-from-sparse-157.8.json";
}

std::filesystem::path initializationManifest() {
    return std::filesystem::path(PROJECT_ROOT).parent_path()/"docs"/"evidence"/
        "task10p11ac-fixed-tau22-eight-cell-confirmation"/
        "initialization-manifest.json";
}

struct MirrorFixture {
    gf::CanonicalHardRowRequest request;
    std::string collision_mobile_pair_base;
    std::string reference_mobile_pair_base;
    std::string reference_mobile_fixed_base;
};

MirrorFixture makeMirrorFixture(std::mt19937& rng) {
    MirrorFixture fixture;
    auto& request=fixture.request;
    request.mobile_ids={1,2,3};
    request.fixed_ids={101};
    std::uniform_real_distribution<double> position(-300.0,300.0);
    std::uniform_real_distribution<double> velocity(-20.0,20.0);
    auto make_state=[&]() {
        PairwiseSecondOrderState2D state;
        state.position=Point(position(rng),position(rng));
        state.velocity=Eigen::Vector2d(velocity(rng),velocity(rng));
        state.acceleration=Eigen::Vector2d::Zero();
        return state;
    };
    for (gf::NodeId id:request.mobile_ids)
        request.states.emplace(id,make_state());
    request.states.emplace(101,PairwiseSecondOrderState2D{});
    // DirectedEdge(reference, owner): owner 2 references mobile 3; owner 1
    // references fixed 101.  Edge ids are "reference->owner".
    request.reference_edges.emplace_back(3,2);
    request.reference_edges.emplace_back(101,1);
    request.collision_pairs.push_back(gf::UndirectedEdge::canonical(1,2));
    request.collision_pairs.push_back(gf::UndirectedEdge::canonical(3,101));
    request.acceleration_half_box=4.0;
    request.reference_spec.kind=
        PairwiseSecondOrderBarrierKind::CommunicationUpper;
    request.reference_spec.distanceLimit=850.0;
    request.reference_spec.uncertainty=0.05;
    request.reference_spec.totalReserve=0.01;
    request.reference_spec.k=1.0;
    request.reference_spec.lambda1=1.0;
    request.reference_spec.lambda2=1.0;
    request.collision_spec=request.reference_spec;
    request.collision_spec.kind=
        PairwiseSecondOrderBarrierKind::CollisionLower;
    request.collision_spec.distanceLimit=10.0;
    request.plant_speed_facet_count=0;
    request.require_snapshot_robust_rows=true;
    gf::PairwiseSnapshotTube tube{0.05,0.05,
        gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    for (const auto& edge:request.reference_edges)
        request.reference_snapshot_tubes.emplace(edge.id(),tube);
    for (const auto& edge:request.collision_pairs)
        request.collision_snapshot_tubes.emplace(edge.id(),tube);
    fixture.collision_mobile_pair_base="collision:1--2";
    fixture.reference_mobile_pair_base="reference:3->2";
    fixture.reference_mobile_fixed_base="reference:101->1";
    return fixture;
}

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> makeP3(
    const nlohmann::json& manifest) {
    const auto initialization=gf::task10p11acInitializationFromManifest(
        manifest,"P3");
    auto scenario=gf::task10p11rFixedBaselineScenario();
    scenario.mobile_positions=initialization.positions;
    auto settings=gf::task10p11acSwarmSettings(
        scenario,initialization.velocities);
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        std::move(scenario),std::move(settings),
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
    if (!fixture->adapter.initializeStageZero().initialized)
        throw std::runtime_error("P3 stage-zero initialization failed");
    return fixture;
}

}  // namespace

TEST_CASE("Task 10.11ai freezes preregistration v1.2 protocol semantics") {
    const auto protocol=gf::task10p11aiProtocol();
    CHECK(protocol.preregistration=="v1.2-approved-2026-08-30");
    CHECK(protocol.tau_mps2==doctest::Approx(22.0));
    CHECK(protocol.dt_s==doctest::Approx(0.1));
    CHECK(protocol.half_box_mps2==doctest::Approx(4.0));
    CHECK(protocol.chain_beats==3);
    CHECK(protocol.strict_negative_bound_mps2==doctest::Approx(-1.0e-8));
    CHECK(protocol.stage_a_total_budget_s==doctest::Approx(28800.0));
    CHECK(protocol.terminality_no_followup_tasks);
    CHECK(gf::task10p11aiClassificationName(
        gf::Task10p11aiClassification::StrictlyInfeasible)==
        "strictly_infeasible");
    CHECK(gf::task10p11aiClassificationName(
        gf::Task10p11aiClassification::Undetermined)=="undetermined");
}

TEST_CASE("coupled mirror reproduces canonical builder residuals exactly") {
    std::mt19937 rng(20270830);
    MirrorFixture fixture=makeMirrorFixture(rng);
    const auto canonical=gf::buildCanonicalHardRows(fixture.request);
    std::uniform_real_distribution<double> control(-4.0,4.0);
    struct Case {
        std::string base_id;
        gf::NodeId owner;
        gf::NodeId peer;
        bool collision;
        bool mobile_pair;
    };
    const std::vector<Case> cases{
        {fixture.collision_mobile_pair_base,1,2,true,true},
        {fixture.reference_mobile_pair_base,2,3,false,true},
        {fixture.reference_mobile_fixed_base,1,101,false,false}};
    for (int trial=0;trial<32;++trial) {
        std::map<gf::NodeId,Eigen::Vector2d> controls;
        for (gf::NodeId id:fixture.request.mobile_ids)
            controls.emplace(id,Eigen::Vector2d(control(rng),control(rng)));
        for (const auto& item:cases) {
            const gf::CanonicalHardRow* owner_row=nullptr;
            for (const auto& candidate:canonical) {
                if (candidate.id==item.base_id+":owner:"+
                        std::to_string(item.owner))
                    owner_row=&candidate;
            }
            REQUIRE_MESSAGE(owner_row!=nullptr,item.base_id);
            gf::Task10p11aiPairSpec pair;
            pair.collision=item.collision;
            pair.mobile_pair=item.mobile_pair;
            pair.spec=item.collision?fixture.request.collision_spec:
                fixture.request.reference_spec;
            const auto& tubes=item.collision?
                fixture.request.collision_snapshot_tubes:
                fixture.request.reference_snapshot_tubes;
            const std::string key=item.collision?
                std::to_string(std::min(item.owner,item.peer))+"--"+
                    std::to_string(std::max(item.owner,item.peer)):
                std::to_string(item.peer)+"->"+std::to_string(item.owner);
            REQUIRE(tubes.count(key)==1);
            pair.tube_position_radius_m=tubes.at(key).position_radius_m;
            pair.tube_velocity_radius_mps=tubes.at(key).velocity_radius_mps;
            const Eigen::Vector2d rel_pos(
                fixture.request.states.at(item.owner).position.x-
                    fixture.request.states.at(item.peer).position.x,
                fixture.request.states.at(item.owner).position.y-
                    fixture.request.states.at(item.peer).position.y);
            const Eigen::Vector2d rel_vel=
                fixture.request.states.at(item.owner).velocity-
                    fixture.request.states.at(item.peer).velocity;
            const auto mirror=gf::task10p11aiCoupledMirror(pair,rel_pos,
                rel_vel,4.0);
            const Eigen::Vector2d delta=controls.at(item.owner)-
                (item.mobile_pair?controls.at(item.peer):
                    Eigen::Vector2d::Zero());
            double builder=0.0;
            if (item.mobile_pair) {
                const gf::CanonicalHardRow* peer_row=nullptr;
                for (const auto& candidate:canonical) {
                    if (candidate.id==item.base_id+":owner:"+
                            std::to_string(item.peer))
                        peer_row=&candidate;
                }
                REQUIRE(peer_row!=nullptr);
                builder=owner_row->margin(controls.at(item.owner))+
                    peer_row->margin(controls.at(item.peer))+
                    owner_row->coefficient_uncertainty_reserve;
            } else {
                builder=owner_row->margin(controls.at(item.owner));
            }
            const double mirror_value=mirror.normal.dot(delta)+
                mirror.constant;
            REQUIRE_MESSAGE(std::abs(builder-mirror_value)<1e-9,item.base_id);
        }
    }
}

TEST_CASE("interval mirror contains builder residual on enclosing boxes") {
    std::mt19937 rng(20270831);
    MirrorFixture fixture=makeMirrorFixture(rng);
    const auto& request=fixture.request;
    std::uniform_real_distribution<double> inflate(0.0,3.0);
    for (int trial=0;trial<16;++trial) {
        std::map<gf::NodeId,gf::Task10p11aiOwnerInterval> intervals;
        std::map<gf::NodeId,PairwiseSecondOrderState2D> mid_states;
        for (gf::NodeId id:request.mobile_ids) {
            const auto& state=request.states.at(id);
            const double dx=inflate(rng),dy=inflate(rng);
            gf::Task10p11aiOwnerInterval box;
            box.px={state.position.x-dx,state.position.x+dx};
            box.py={state.position.y-dy,state.position.y+dy};
            box.vx={state.velocity.x()-0.5,state.velocity.x()+0.5};
            box.vy={state.velocity.y()-0.5,state.velocity.y()+0.5};
            intervals.emplace(id,box);
            PairwiseSecondOrderState2D mid;
            mid.position=Point((box.px.lo+box.px.hi)/2.0,
                (box.py.lo+box.py.hi)/2.0);
            mid.velocity=Eigen::Vector2d((box.vx.lo+box.vx.hi)/2.0,
                (box.vy.lo+box.vy.hi)/2.0);
            mid_states.emplace(id,mid);
        }
        for (gf::NodeId id:request.fixed_ids) {
            gf::Task10p11aiOwnerInterval box;
            box.px=gf::Task10p11aiInterval(request.states.at(id).position.x);
            box.py=gf::Task10p11aiInterval(request.states.at(id).position.y);
            box.vx=gf::Task10p11aiInterval(0.0);
            box.vy=gf::Task10p11aiInterval(0.0);
            intervals.emplace(id,box);
        }
        gf::Task10p11aiPairSpec pair;
        pair.collision=true;
        pair.mobile_pair=true;
        pair.spec=request.collision_spec;
        const auto& tube=request.collision_snapshot_tubes.at("1--2");
        pair.tube_position_radius_m=tube.position_radius_m;
        pair.tube_velocity_radius_mps=tube.velocity_radius_mps;
        const auto mirror=gf::task10p11aiCoupledMirrorInterval(pair,
            intervals.at(1),intervals.at(2),4.0);
        const Eigen::Vector2d delta(2.5,-1.5);
        const gf::Task10p11aiInterval residual=
            mirror.normal_x*gf::Task10p11aiInterval(delta.x())+
            mirror.normal_y*gf::Task10p11aiInterval(delta.y())+
            mirror.constant;
        const Eigen::Vector2d rel_pos(
            mid_states.at(1).position.x-mid_states.at(2).position.x,
            mid_states.at(1).position.y-mid_states.at(2).position.y);
        const Eigen::Vector2d rel_vel=mid_states.at(1).velocity-
            mid_states.at(2).velocity;
        const auto exact=gf::task10p11aiCoupledMirror(pair,rel_pos,rel_vel,
            4.0);
        const double point_value=exact.normal.dot(delta)+exact.constant;
        CHECK(residual.lo<=point_value+1e-12);
        CHECK(residual.hi>=point_value-1e-12);
        const double sup=gf::task10p11aiCoupledRowSup(mirror,2.0*4.0);
        CHECK(sup>=point_value-1e-12);
    }
}

TEST_CASE("chain solver closes on a synthetic collision chain with agreement") {
    gf::Task10p11aiChainInput input;
    input.pair.collision=true;
    input.pair.mobile_pair=true;
    input.pair.spec.kind=
        PairwiseSecondOrderBarrierKind::CollisionLower;
    input.pair.spec.distanceLimit=10.0;
    input.pair.spec.uncertainty=0.05;
    input.pair.spec.totalReserve=0.01;
    input.pair.spec.k=1.0;
    input.pair.spec.lambda1=1.0;
    input.pair.spec.lambda2=1.0;
    input.pair.tube_position_radius_m=0.05;
    input.pair.tube_velocity_radius_mps=0.05;
    input.pair_owner_a=2;
    input.pair_owner_b=9;
    input.rel_pos_0=Eigen::Vector2d(-15.0,20.0);
    input.rel_vel_0=Eigen::Vector2d(3.0,-4.0);
    input.dt_s=0.1;
    input.half_box_mps2=4.0;
    input.tube_position_radius_m={0.05,0.06,0.07,0.08};
    input.tube_velocity_radius_mps={0.05,0.06,0.07,0.08};
    input.time_limit_s=300.0;
    const auto result=gf::solveTask10p11aiPairChainBound(input);
    const std::string fail_detail=result.fail_reason+" status="+
        std::to_string(result.gurobi_status);
    CHECK_MESSAGE(result.built,fail_detail);
    CHECK(result.fail_reason.empty());
    CHECK(result.sign_fixation[1]);
    CHECK(result.sign_fixation[2]);
    CHECK(result.sign_fixation[3]);
    REQUIRE(result.solved);
    CHECK(result.global_optimal);
    REQUIRE(std::isfinite(result.independent_formula_mps2));
    CHECK(std::abs(result.incumbent_mps2-result.independent_formula_mps2)<=
        1.0e-7);
}

TEST_CASE("limiting-row replay reproduces the frozen 10.11ah terminal") {
    const auto packed=gf::readTask10p11vJson(packed157p8().string());
    const auto manifest=gf::readTask10p11vJson(
        initializationManifest().string());
    auto fixture=makeP3(manifest);
    gf::restoreTask10p11vRestartState(*fixture,
        packed.at("restart_checkpoint"));
    REQUIRE(fixture->topologyFrozen());
    Eigen::Matrix<double,8,1> frozen_vector;
    frozen_vector<<-4,4,4,-4,-4,4,4,-4;
    const auto limiting=gf::task10p11aiIdentifyLimitingRow(*fixture,
        frozen_vector);
    CHECK(limiting.pair_row);
    CHECK(limiting.successor_minimum_residual_mps2==
        doctest::Approx(-0.02138986047537017).epsilon(1e-6));
}

TEST_CASE("full-row interval audit matches the canonical row count") {
    const auto snapshot=gf::readTask10p11vJson(packed157p8().string());
    const auto base_request=gf::task10p11s_capture_detail::requestFromJson(
        snapshot.at("canonical_request"));
    const auto canonical=gf::buildCanonicalHardRows(base_request);
    const auto problem=gf::buildTask10p11sRows28d(canonical,
        base_request.mobile_ids,true);
    const auto audit=gf::runTask10p11aiIntervalAudit(snapshot,4.0);
    for (std::size_t layer=0;layer<=3;++layer) {
        const auto& item=audit.layers[layer];
        CHECK(item.valid);
        CHECK(item.row_count==problem.rows.size());
        CHECK(item.row_count==1113);
        CHECK(std::isfinite(item.minimum_sup_mps2));
        CHECK(!item.limiting_row_id.empty());
    }
}
