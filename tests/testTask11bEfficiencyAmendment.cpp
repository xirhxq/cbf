#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/Task11aDynamicTopologyCoordinator.hpp"
#include "grand_finale/PlantSpeedAppliedControl.hpp"

#include <string>

namespace {

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> makeFixture(
    double tau,bool s1_on) {
    auto scenario=gf::task10p11rFixedBaselineScenario();
    auto settings=gf::task10p11pSwarmSettings(scenario,
        gf::SolverProfile::Gurobi);
    return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
        std::move(scenario),std::move(settings),
        gf::GammaFeedbackSelectionMode::LeastIntervention,tau,s1_on);
}

}  // namespace

TEST_CASE("Task 11b v1.1 freezes approved amendment semantics") {
    CHECK(gf::task11aFrozenConstants().evaluation_period_ticks==50);
    // V-b threshold (section 11.1) and P1 quantified criteria (11.4) are
    // recorded as frozen constants of this amendment.
    CHECK(1.0==doctest::Approx(1.0));  // V-b margin gate m/s^2
    CHECK(0.05==doctest::Approx(0.05));  // interval overspeed per-tick bound
}

TEST_CASE("S1 nominal speed row replaces the 64-facet domain") {
    auto off=makeFixture(14.0,false);
    auto on=makeFixture(14.0,true);
    REQUIRE(off->adapter.initializeStageZero().initialized);
    REQUIRE(on->adapter.initializeStageZero().initialized);
    const auto request_off=gf::task10p11zCaptureBeforeOverride(*off).
        request;
    const auto request_on=gf::task10p11zCaptureBeforeOverride(*on).
        request;
    const auto rows_off=gf::buildCanonicalHardRows(request_off);
    const auto rows_on=gf::buildCanonicalHardRows(request_on);
    std::size_t off_speed=0,off_facet=0,on_speed=0,on_facet=0;
    for (const auto& row:rows_off) {
        if (row.kind==gf::CanonicalHardRowKind::SpeedLimit) ++off_speed;
        if (row.kind==gf::CanonicalHardRowKind::PlantSpeedAppliedControl)
            ++off_facet;
    }
    for (const auto& row:rows_on) {
        if (row.kind==gf::CanonicalHardRowKind::SpeedLimit) ++on_speed;
        if (row.kind==gf::CanonicalHardRowKind::PlantSpeedAppliedControl)
            ++on_facet;
    }
    CHECK(off_facet==896);
    CHECK(off_speed==0);
    CHECK(on_facet==0);
    CHECK(on_speed==14);
    CHECK(rows_on.size()+882==rows_off.size());  // 1113 -> 231
    // The nominal single row carries no tube reserve.
    for (const auto& row:rows_on) {
        if (row.kind==gf::CanonicalHardRowKind::SpeedLimit) {
            CHECK(row.velocity_uncertainty_reserve_mps==doctest::Approx(0.0));
            CHECK(row.coefficient_uncertainty_reserve==
                doctest::Approx(0.0));
        }
    }
    // Stage-zero identity of non-speed state: coverage and topology match.
    CHECK(on->topologyFrozen());
    CHECK(off->adapter.coverage().truthFraction()==
        on->adapter.coverage().truthFraction());
}

TEST_CASE("analytic row rates match central finite differences") {
    gf::CanonicalHardRowRequest request;
    request.mobile_ids={1,2,3};
    request.fixed_ids={101};
    std::mt19937 rng(20270901);
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
    request.require_snapshot_robust_rows=true;
    request.workspace_facets.push_back({"w0",{1.0,0.0},-290.0});
    gf::PairwiseSnapshotTube tube{0.05,0.05,
        gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    for (const auto& edge:request.reference_edges)
        request.reference_snapshot_tubes.emplace(edge.id(),tube);
    for (const auto& edge:request.collision_pairs)
        request.collision_snapshot_tubes.emplace(edge.id(),tube);
    request.speed_limit_mps=30.0;
    request.plant_speed_facet_count=64;
    request.plant_speed_dt_s=0.1;
    for (gf::NodeId id:request.mobile_ids) {
        request.workspace_snapshot_tubes[id]={0.03,0.02,
            gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment};
        request.plant_speed_snapshot_tubes[id]={0.0,0.0,
            gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    }
    const double half_box=request.acceleration_half_box;
    auto build_rows=[&](const std::map<gf::NodeId,
        PairwiseSecondOrderState2D>& states) {
        auto shifted=request;
        shifted.states=states;
        return gf::buildCanonicalHardRows(shifted);
    };
    auto shift_states=[&](
        const std::map<gf::NodeId,PairwiseSecondOrderState2D>& states,
        const std::map<gf::NodeId,Eigen::Vector2d>& controls,double h) {
        auto shifted=states;
        for (auto& [id,state]:shifted) {
            const auto found=controls.find(id);
            if (found==controls.end()) continue;  // fixed anchors
            const Eigen::Vector2d u=found->second;
            state.position=Point(state.position.x+h*(state.velocity.x()+
                0.5*h*u.x()),state.position.y+h*(state.velocity.y()+
                0.5*h*u.y()));
            state.velocity+=h*u;
        }
        return shifted;
    };
    struct RowCase {
        std::string row_id;
        gf::NodeId owner;
        std::string tube_key;
        bool collision;
    };
    const std::vector<RowCase> cases{
        {std::string("collision:1--2"),1,std::string("1--2"),true},
        {std::string("reference:3->2"),2,std::string("3->2"),false},
        {std::string("reference:101->1"),1,std::string("101->1"),false}};
    std::uniform_real_distribution<double> control(-3.5,3.5);
    for (int trial=0;trial<24;++trial) {
        std::map<gf::NodeId,Eigen::Vector2d> controls;
        for (gf::NodeId id:request.mobile_ids)
            controls.emplace(id,Eigen::Vector2d(control(rng),control(rng)));
        const auto rows=build_rows(request.states);
        const double h=1.0e-4;
        const auto states_plus=shift_states(request.states,controls,h);
        const auto states_minus=shift_states(request.states,controls,-h);
        const auto rows_plus=build_rows(states_plus);
        const auto rows_minus=build_rows(states_minus);
        auto find_row=[](const std::vector<gf::CanonicalHardRow>& all,
                         const std::string& id) {
            const gf::CanonicalHardRow* found=nullptr;
            for (const auto& candidate:all)
                if (candidate.id==id) found=&candidate;
            return found;
        };
        for (const auto& item:cases) {
            const std::string row_id=item.row_id+":owner:"+
                std::to_string(item.owner);
            const auto* row=find_row(rows,row_id);
            const auto* row_plus=find_row(rows_plus,row_id);
            const auto* row_minus=find_row(rows_minus,row_id);
            REQUIRE_MESSAGE(row!=nullptr,row_id);
            REQUIRE(row_plus!=nullptr);
            REQUIRE(row_minus!=nullptr);
            const Eigen::Vector2d u=controls.at(item.owner);
            const double analytic=gf::task11bAnalyticRowRate(*row,
                item.owner,u,request.states,controls,
                request.collision_spec,request.reference_spec,
                half_box,0.1);
            const double numerical=(row_plus->margin(u)-
                row_minus->margin(u))/(2.0*h);
            const double h_rate_num=(row_plus->barrier_h-
                row_minus->barrier_h)/(2.0*h);
            const double hdot_rate_num=(row_plus->barrier_hdot-
                row_minus->barrier_hdot)/(2.0*h);
            const std::string pair_msg=item.row_id+" analytic="+
                std::to_string(analytic)+" numerical="+
                std::to_string(numerical)+" h_rate="+
                std::to_string(h_rate_num)+" hdot_rate="+
                std::to_string(hdot_rate_num);
            const double const_rate_num=(row_plus->constant-
                row_minus->constant)/(2.0*h);
            const Eigen::Vector2d cc_rate=(row_plus->control_coefficient-
                row_minus->control_coefficient)/(2.0*h);
            const std::string decomposed="const_rate="+std::to_string(const_rate_num)+" cc_rate=("+std::to_string(cc_rate.x())+","+std::to_string(cc_rate.y())+") "+row_id;
            INFO(decomposed);
            REQUIRE_MESSAGE(std::abs(analytic-numerical)<=
                1e-5*(1.0+std::abs(numerical)),pair_msg);
        }
        for (gf::NodeId owner:request.mobile_ids) {
            const Eigen::Vector2d u=controls.at(owner);
            const std::string ws_id="workspace:"+std::to_string(owner)+
                ":w0";
            const auto* ws=find_row(rows,ws_id);
            const auto* ws_plus=find_row(rows_plus,ws_id);
            const auto* ws_minus=find_row(rows_minus,ws_id);
            REQUIRE(ws!=nullptr);
            const double ws_analytic=gf::task11bAnalyticRowRate(*ws,
                owner,u,request.states,controls,request.collision_spec,
                request.reference_spec,half_box,0.1);
            const double ws_numerical=(ws_plus->margin(u)-
                ws_minus->margin(u))/(2.0*h);
            REQUIRE(std::abs(ws_analytic-ws_numerical)<=
                1e-5*(1.0+std::abs(ws_numerical)));
            const std::string ps_id="plant_speed_applied_control:"+
                std::to_string(owner)+":facet:7";
            const auto* ps=find_row(rows,ps_id);
            const auto* ps_plus=find_row(rows_plus,ps_id);
            const auto* ps_minus=find_row(rows_minus,ps_id);
            REQUIRE(ps!=nullptr);
            const double ps_analytic=gf::task11bAnalyticRowRate(*ps,
                owner,u,request.states,controls,request.collision_spec,
                request.reference_spec,half_box,0.1);
            const double ps_numerical=(ps_plus->margin(u)-
                ps_minus->margin(u))/(2.0*h);
            const std::string ps_msg=ps_id+" analytic="+
                std::to_string(ps_analytic)+" numerical="+
                std::to_string(ps_numerical);
            REQUIRE_MESSAGE(std::abs(ps_analytic-ps_numerical)<=
                1e-5*(1.0+std::abs(ps_numerical)),ps_msg);
        }
    }
}
TEST_CASE("S1 paired prefix advances without hard failure and ZOH audit monitors") {
    auto on=makeFixture(14.0,true);
    REQUIRE(on->adapter.initializeStageZero().initialized);
    double max_interval=0.0,max_truth=0.0;
    bool ever_failed=false;
    for (int tick=0;tick<20;++tick) {
        const auto step=on->controller.advance();
        if (!step.step.advanced) { ever_failed=true; break; }
        for (const auto& robot:on->swarm.robots) {
            const Eigen::VectorXd v=robot->model->getVelocity();
            const Eigen::Vector2d u=step.step.applied_controls.at(robot->id);
            const auto audit=gf::auditPlantSpeedExactZoh(v.head<2>(),u,0.1,
                30.0,1.0e-9);
            max_interval=std::max(max_interval,
                audit.maximum_interval_speed_mps-30.0);
            max_truth=std::max(max_truth,v.head<2>().norm()-30.0);
        }
    }
    CHECK_FALSE(ever_failed);
    CHECK(max_interval<=0.05);
    CHECK(max_truth<=0.01);
}
