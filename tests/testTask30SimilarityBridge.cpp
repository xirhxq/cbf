#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("Task30 two bridge endpoints scale every mobile around base101 and leave fixed anchors unchanged") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto fixed_before=fixed;
    for(int mode:{0,12}) {
        const auto contract=gf::task25DagContractFromCode(mode);
        const auto front=gf::task26CompactFronts(contract,fixed);
        const auto scaled=gf::task30SimilarityBridgeFronts(contract,fixed,front,1.5);
        const auto original=gf::task20LiftTargets(contract,fixed,front);
        const auto enlarged=gf::task20LiftTargets(contract,fixed,scaled);
        REQUIRE(original.valid);REQUIRE(enlarged.valid);
        for(const auto& [id,p]:original.targets)
            CHECK((enlarged.targets.at(id)-(fixed.at(101)+1.5*(p-fixed.at(101)))).norm()<1e-9);
        if(mode==0) {
            CHECK(scaled.at("A").x()==2025);CHECK(scaled.at("A").y()==700);
            CHECK(scaled.at("B").x()==2475);CHECK(scaled.at("B").y()==700);
        } else {CHECK(scaled.at("P").x()==2250);CHECK(scaled.at("P").y()==1450);}
        for(const auto& [id,p]:fixed)CHECK((p-fixed_before.at(id)).norm()==0);
        const auto identity=gf::task30SimilarityBridgeFronts(contract,fixed,front,1.0);
        for(const auto& [id,p]:front)CHECK((p.array()==identity.at(id).array()).all());
    }
}

TEST_CASE("Task30 scale is opt in through the C5 coordinator without changing default motion or safety") {
    auto scenario=gf::task10p11rFixedBaselineScenario();
    auto cfg=gf::task19ProductionAdapterConfig();CHECK(cfg.target_policy_task18_cbf2026_outer);
    cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
    auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);Swarm swarm(settings);
    gf::GrandFinaleSwarmAdapter adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,scenario.initial_topology,cfg);
    gf::Task10p11hSimpleCoverageController controller(swarm,adapter);
    REQUIRE(adapter.initializeStageZero().initialized);
    const std::string action="pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront";
    gf::Task26ExternalReconstructor legacy(adapter,controller,action);
    gf::Task26ExternalReconstructor identity(adapter,controller,action,60.0,1.0);
    CHECK(legacy.telemetry()==identity.telemetry());
    CHECK_FALSE(identity.telemetry().contains("task30_bridge"));
    gf::Task26ExternalReconstructor enlarged(adapter,controller,action,60.0,1.25);
    CHECK(enlarged.telemetry().at("task30_bridge").at("kappa")==1.25);
    CHECK(enlarged.telemetry().at("task30_bridge").at("center_id")==101);
    CHECK(enlarged.telemetry().at("task29_front").at("duration_s")==60.0);
    CHECK(enlarged.telemetry().at("task29").at("completion_contract")=="moving-v1");
    CHECK_FALSE(enlarged.telemetry().contains("task29_compact"));
    CHECK_THROWS(gf::Task26ExternalReconstructor(adapter,controller,"pinball",60.0,1.25));
    for(double invalid:{0.,.9,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()})
        CHECK_THROWS(gf::Task26ExternalReconstructor(adapter,controller,action,60.0,invalid));
}

TEST_CASE("Task30 similarity rejects noncommuting anchors and preserves labelled path covariance") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
    auto unequal=fixed;unequal.at(102).x()+=60;
    CHECK_THROWS(gf::task30SimilarityBridgeFronts(goal,unequal,gf::task26CompactFronts(goal,unequal),1.25));
    for(double invalid:{.5,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
        CHECK_THROWS(gf::task30SimilarityBridgeFronts(goal,fixed,gf::task26CompactFronts(goal,fixed),invalid));
    const auto a=gf::task20LiftTargets(old,fixed,gf::task26CompactFronts(old,fixed)).targets;
    const auto b=gf::task20LiftTargets(goal,fixed,gf::task26CompactFronts(goal,fixed)).targets;
    const gf::Task29RoleCenterPath baseline(goal,a,b);
    for(double kappa:{1.,1.25,1.5,1.75,2.}) {
        const auto old_front=gf::task30SimilarityBridgeFronts(old,fixed,gf::task26CompactFronts(old,fixed),kappa);
        const auto new_front=gf::task30SimilarityBridgeFronts(goal,fixed,gf::task26CompactFronts(goal,fixed),kappa);
        const auto start=gf::task20LiftTargets(old,fixed,old_front).targets;
        const auto end=gf::task20LiftTargets(goal,fixed,new_front).targets;
        const gf::Task29RoleCenterPath path(goal,start,end);
        double minimum=std::numeric_limits<double>::infinity(),at=0;gf::NodeId mi=0,mj=0;
        for(int j=0;j<=10000;++j) {
            const double phase=j/10000.;const auto q=path.evaluateContinuingFront(phase);
            const auto original=baseline.evaluateContinuingFront(phase);
            for(const auto& [id,p]:q) {
                CHECK((p-(fixed.at(101)+kappa*(original.at(id)-fixed.at(101)))).norm()<1e-8);
                for(const auto& [other,z]:q)if(other>id&&(p-z).norm()<minimum) {minimum=(p-z).norm();at=phase;mi=id;mj=other;}
                for(const auto& [other,z]:fixed)if((p-z).norm()<minimum) {minimum=(p-z).norm();at=phase;mi=id;mj=other;}
            }
        }
        nlohmann::json old_json=nlohmann::json::object(),new_json=nlohmann::json::object();
        for(const auto& [id,p]:start)old_json[std::to_string(id)]={p.x(),p.y()};
        for(const auto& [id,p]:end)new_json[std::to_string(id)]={p.x(),p.y()};
        std::cout<<"TASK30_SCALAR_GEOMETRY "<<nlohmann::json({{"kappa",kappa},{"minimum_m",minimum},
            {"pair",{mi,mj}},{"phase",at},{"old_targets",old_json},{"new_targets",new_json},
            {"sample_count",10001}}).dump()<<'\n';
        CHECK(minimum>10);
        Eigen::Matrix2d rotation;const double angle=.37;
        rotation<<std::cos(angle),-std::sin(angle),std::sin(angle),std::cos(angle);
        const Eigen::Vector2d translation(17,-93);
        auto moved_fixed=fixed;for(auto& [id,p]:moved_fixed)p=rotation*p+translation;
        auto moved_front=gf::task26CompactFronts(goal,fixed);for(auto& [u,p]:moved_front)p=rotation*p+translation;
        const auto moved=gf::task20LiftTargets(goal,moved_fixed,gf::task30SimilarityBridgeFronts(goal,moved_fixed,moved_front,kappa));
        REQUIRE(moved.valid);
        for(const auto& [id,p]:end)CHECK((moved.targets.at(id)-(rotation*p+translation)).norm()<1e-8);
    }
}

TEST_CASE("Task30 rejects finite anchors whose centroid accumulation overflows") {
    const auto goal=gf::task25DagContractFromCode(12);
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{
        {100,{1e308,1e308}},{101,{1e308,1e308}},{102,{1.1e308,1.1e308}}};
    const std::map<std::string,Eigen::Vector2d> front{{"P",{0,0}}};
    CHECK_THROWS(gf::task30SimilarityBridgeFronts(goal,fixed,front,1.25));
}

TEST_CASE("Task30 inner scale 1.1 independent scalar phase record") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
    const double kappa=1.1;
    const auto a=gf::task20LiftTargets(old,fixed,gf::task30SimilarityBridgeFronts(old,fixed,gf::task26CompactFronts(old,fixed),kappa)).targets;
    const auto b=gf::task20LiftTargets(goal,fixed,gf::task30SimilarityBridgeFronts(goal,fixed,gf::task26CompactFronts(goal,fixed),kappa)).targets;
    const gf::Task29RoleCenterPath path(goal,a,b);
    double minimum=std::numeric_limits<double>::infinity(),phase_at=0;gf::NodeId mi=0,mj=0;
    for(int tick=0;tick<=10000;++tick) {
        const double phase=tick/10000.;const auto q=path.evaluateContinuingFront(phase);
        REQUIRE(q.size()==14);
        for(const auto& [i,p]:q) {
            REQUIRE(p.allFinite());
            for(const auto& [j,z]:q)if(j>i&&(p-z).norm()<minimum){minimum=(p-z).norm();phase_at=phase;mi=i;mj=j;}
            for(const auto& [j,z]:fixed)if((p-z).norm()<minimum){minimum=(p-z).norm();phase_at=phase;mi=i;mj=j;}
        }
    }
    CHECK(minimum>10);
    nlohmann::json endpoints=nlohmann::json::object();
    for(const auto& [i,p]:b)endpoints[std::to_string(i)]={p.x(),p.y()};
    std::cout<<"TASK30_INNER_SCALAR_GEOMETRY "<<nlohmann::json({{"kappa",kappa},{"minimum_m",minimum},
        {"phase",phase_at},{"pair",{mi,mj}},{"new_targets",endpoints},{"sample_count",10001}}).dump()<<'\n';
}
