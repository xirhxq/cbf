#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task29RoleCenterPath.hpp"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("Task29 role center preserves labelled endpoints and each contract unit centroid") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0);
    const auto from=gf::task20LiftTargets(old,fixed,gf::task26CompactFronts(old,fixed)).targets;
    for(int mode:{12,0,2}) {
        const auto goal=gf::task25DagContractFromCode(mode);
        const auto to=gf::task20LiftTargets(goal,fixed,gf::task26CompactFronts(goal,fixed)).targets;
        gf::Task29RoleCenterPath path(goal,from,to);
        gf::Task28LayerPath legacy(goal,from,to,gf::Task28LayerPath::Kind::CenteredFrame);
        for(const auto& [id,p]:from)CHECK((path.evaluate(0).at(id)-p).norm()==0);
        for(const auto& [id,p]:to)CHECK((path.evaluate(1).at(id)-p).norm()==0);
        for(int step=0;step<=500;++step) {
            const double h=step/500.;const auto q=path.evaluate(h);
            for(const auto& unit:goal.coverage_units) {
                Eigen::Vector2d centroid=Eigen::Vector2d::Zero(),expected=Eigen::Vector2d::Zero();
                for(auto id:unit.members){centroid+=q.at(id);expected+=(1-h)*from.at(id)+h*to.at(id);}
                CHECK((centroid-expected).norm()<1e-8);
            }
            if(mode==12) {
                for(const auto& [id,p]:q) {
                    for(const auto& [other,z]:q)if(id<other)CHECK((p-z).norm()>10);
                    for(const auto& [base,z]:fixed)CHECK((p-z).norm()>10);
                }
            }
        }
        CHECK_THROWS(path.evaluate(std::numeric_limits<double>::quiet_NaN()));
        CHECK_THROWS(path.evaluate(std::numeric_limits<double>::infinity()));
        auto invalid=goal;invalid.coverage_units.clear();CHECK_THROWS(gf::Task29RoleCenterPath(invalid,from,to));
        invalid=goal;invalid.coverage_units.front().members.push_back(invalid.coverage_units.front().members.front());
        CHECK_THROWS(gf::Task29RoleCenterPath(invalid,from,to));
        invalid=goal;invalid.member_roles.begin()->second.triangular_fraction=std::numeric_limits<double>::quiet_NaN();
        CHECK_THROWS(gf::Task29RoleCenterPath(invalid,from,to));
        invalid=goal;for(auto& [id,role]:invalid.member_roles){role.axial_fraction=0;role.triangular_fraction=0;}
        CHECK_THROWS(gf::Task29RoleCenterPath(invalid,from,to));
    }
}

TEST_CASE("Task29 role center endpoint tangent is common front affine motion not rigid velocity") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
    const auto a=gf::task20LiftTargets(old,fixed,gf::task26CompactFronts(old,fixed)).targets;
    const auto b=gf::task20LiftTargets(goal,fixed,gf::task26CompactFronts(goal,fixed)).targets;
    gf::Task29RoleCenterPath path(goal,a,b);const double eps=1e-7;const auto near=path.evaluate(1-eps);
    std::map<gf::NodeId,gf::Task29MotionState> states;
    for(const auto& [id,p]:b)states[id]={p,(p-near.at(id))/(60*eps),0,0};
    const auto audit=gf::task29MovingCompletion(goal,fixed,b,states,true,true);
    REQUIRE(audit.valid);CHECK(audit.maximum_coordinated_speed_bound<1e-4);
    CHECK(audit.moving_instant_ready);CHECK_FALSE(audit.legacy_instant_ready);
    // This is a reference tangent test with ideal position, not a plant certificate.
}

TEST_CASE("Task29 role center only enabled by exact external research action") {
    auto scenario=gf::task10p11rFixedBaselineScenario();auto cfg=gf::task19ProductionAdapterConfig();
    CHECK(cfg.target_policy_task18_cbf2026_outer);
    cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
    auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,scenario.initial_topology,cfg);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);REQUIRE(adapter.initializeStageZero().initialized);
    gf::Task26ExternalReconstructor c2(adapter,controller,"pinball-qualified-layered-centeredframe-moving-linearphase");
    CHECK_FALSE(c2.telemetry().contains("task29_path"));
    gf::Task26ExternalReconstructor c3(adapter,controller,"pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter");
    CHECK(c3.telemetry().at("task29_path").at("common_correction")=="final_role_matrix");
    CHECK(c3.telemetry().at("task28").at("expansion_path")=="terminal_first_role_center");
    CHECK(c3.telemetry().at("task29_timing").at("common_phase")=="linear");
    CHECK(c3.telemetry().contains("task29"));CHECK_FALSE(c3.telemetry().contains("task29_compact"));
}

TEST_CASE("Task29 C4 keeps the role path image and labelled endpoints with zero endpoint clock velocity") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0);
    const auto a=gf::task20LiftTargets(old,fixed,gf::task26CompactFronts(old,fixed)).targets;
    for(int mode:{12,0,2}) {
        const auto goal=gf::task25DagContractFromCode(mode);
        const auto b=gf::task20LiftTargets(goal,fixed,gf::task26CompactFronts(goal,fixed)).targets;
        const gf::Task29RoleCenterPath path(goal,a,b);
        const double dt=1e-4;
        const auto near_start=path.evaluate(gf::task26SmoothStep(dt/60));
        const auto near_end=path.evaluate(gf::task26SmoothStep(1-dt/60));
        for(const auto& [id,p]:a) {
            CHECK((path.evaluate(0).at(id)-p).norm()==0);
            CHECK((path.evaluate(1).at(id)-b.at(id)).norm()==0);
            CHECK((near_start.at(id)-p).norm()/dt<1e-3);
            CHECK((b.at(id)-near_end.at(id)).norm()/dt<1e-3);
        }
        for(int tick=0;tick<=600;++tick) {
            const double s=tick/600.,phase=gf::task26SmoothStep(s);
            CHECK(phase>=0);CHECK(phase<=1);
            const auto q=path.evaluate(phase);
            for(const auto& unit:goal.coverage_units) {
                Eigen::Vector2d mean=Eigen::Vector2d::Zero(),expected=Eigen::Vector2d::Zero();
                for(auto id:unit.members) {mean+=q.at(id);expected+=(1-phase)*a.at(id)+phase*b.at(id);}
                CHECK((mean-expected).norm()<1e-8);
            }
            if(mode==12)for(const auto& [id,p]:q) {
                for(const auto& [j,z]:q)if(id<j)CHECK((p-z).norm()>10);
                for(const auto& [base,z]:fixed)CHECK((p-z).norm()>10);
            }
        }
    }
}

TEST_CASE("Task29 continued front preserves the final lifting in nonzero coordinated motion") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
    const auto a=gf::task20LiftTargets(old,fixed,gf::task26CompactFronts(old,fixed)).targets;
    const auto b=gf::task20LiftTargets(goal,fixed,gf::task26CompactFronts(goal,fixed)).targets;
    const gf::Task29RoleCenterPath path(goal,a,b);const double eps=1e-6;
    const auto q=path.evaluateContinuingFront(1.25),next=path.evaluateContinuingFront(1.25+eps);
    std::map<gf::NodeId,gf::Task29MotionState> states;
    for(const auto& [id,p]:q)states[id]={p,(next.at(id)-p)/(60*eps),0,0};
    const auto audit=gf::task29MovingCompletion(goal,fixed,q,states,true,true);
    REQUIRE(audit.valid);CHECK(audit.moving_instant_ready);CHECK_FALSE(audit.legacy_instant_ready);
    CHECK(audit.maximum_coordinated_speed_bound<1e-5);
    const auto lifted=gf::task20LiftTargets(goal,fixed,audit.front_positions);
    REQUIRE(lifted.valid);
    for(const auto& [id,p]:q)CHECK((p-lifted.targets.at(id)).norm()<1e-8);
}

TEST_CASE("Task29 continued front exactly preserves C3 prefix and is C1 for one two three units") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0);
    const auto a=gf::task20LiftTargets(old,fixed,gf::task26CompactFronts(old,fixed)).targets;
    for(int mode:{12,0,2}) {
        const auto goal=gf::task25DagContractFromCode(mode);
        auto fronts=gf::task26CompactFronts(goal,fixed);
        if(mode==0) {
            int ordinal=0;
            for(auto& [unit,g]:fronts){++ordinal;g+=Eigen::Vector2d(80*ordinal,50*ordinal);}
        }
        const auto b=gf::task20LiftTargets(goal,fixed,fronts).targets;
        const gf::Task29RoleCenterPath path(goal,a,b);
        for(int k=-10;k<=1000;++k) {
            const double s=k/1000.;const auto q=path.evaluateContinuingFront(s),r=path.evaluate(s);
            for(const auto& [id,p]:q)CHECK((p-r.at(id)).norm()==0);
        }
        const double eps=1e-7;
        const auto left=path.evaluateContinuingFront(1-eps),right=path.evaluateContinuingFront(1+eps);
        const auto coarse_left=path.evaluateContinuingFront(1-10*eps),coarse_right=path.evaluateContinuingFront(1+10*eps);
        for(const auto& [id,p]:b) {
            const double fine=((p-left.at(id))-(right.at(id)-p)).norm()/eps;
            const double coarse=((p-coarse_left.at(id))-(coarse_right.at(id)-p)).norm()/(10*eps);
            CHECK(std::isfinite(fine));
            CHECK(fine<=.15*coarse+1e-5); // First-order finite-difference curvature must converge to zero.
        }
        for(double s:{1.,1.01,1.5,2.,6.}) {
            const auto q=path.evaluateContinuingFront(s),next=path.evaluateContinuingFront(s+eps);
            std::map<gf::NodeId,gf::Task29MotionState> state;
            for(const auto& [id,p]:q)state[id]={p,(next.at(id)-p)/(60*eps),0,0};
            const auto audit=gf::task29MovingCompletion(goal,fixed,q,state,true,true);
            REQUIRE(audit.valid);CHECK(audit.maximum_coordinated_speed_bound<1e-5);
            const auto lifted=gf::task20LiftTargets(goal,fixed,audit.front_positions);REQUIRE(lifted.valid);
            for(const auto& [id,p]:q)CHECK((p-lifted.targets.at(id)).norm()<1e-8);
        }
        CHECK_THROWS(path.evaluateContinuingFront(std::numeric_limits<double>::quiet_NaN()));
        CHECK_THROWS(path.evaluateContinuingFront(std::numeric_limits<double>::infinity()));
        CHECK_THROWS(path.evaluateContinuingFront(-std::numeric_limits<double>::infinity()));
    }
}
