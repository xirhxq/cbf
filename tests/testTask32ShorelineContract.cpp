#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <fstream>
static gf::Task31AnchorScene scene(const char* path) {
    std::ifstream f(path);if(!f.good())throw std::runtime_error("missing shoreline asset");
    nlohmann::json j;f>>j;return gf::task31AnchorScene(j);
}
TEST_CASE("shoreline-only scene preserves DAG lattice roles final targets and original anchors") {
    const auto a=scene("docs/evidence/task32-fixed-library/observation-r1-anchor-asset.json");
    const auto b=scene("docs/evidence/task32-fixed-library/shoreline-left-r1-anchor-asset.json");
    REQUIRE(a.valid);REQUIRE(b.valid);CHECK(a.fixed.size()==6);CHECK(b.fixed.size()==6);
    CHECK(a.goal.reference_edges==b.goal.reference_edges);CHECK(a.terminal_ports==b.terminal_ports);
    CHECK(a.frame_origin==b.frame_origin);CHECK(a.direction==b.direction);CHECK(a.ranking_span_m==b.ranking_span_m);
    for(const auto& [id,p]:a.fixed)if(id!=103)CHECK(p==b.fixed.at(id));
    for(const auto& [id,cell]:a.cells) {
        CHECK(cell.row==b.cells.at(id).row);CHECK(cell.slot==b.cells.at(id).slot);
        CHECK(gf::task29RoleMatrix(a.goal.member_roles.at(id))==gf::task29RoleMatrix(b.goal.member_roles.at(id)));
    }
    for(const Eigen::Vector2d target:{Eigen::Vector2d(5,5),{4495,5},{5,2245},{4495,2245},{2255,1125}}) {
        std::map<std::string,Eigen::Vector2d> fronts;for(const auto& u:a.goal.coverage_units)fronts[u.id]=target;
        const auto qa=gf::task20LiftTargets(a.goal,a.fixed,fronts),qb=gf::task20LiftTargets(b.goal,b.fixed,fronts);
        REQUIRE(qa.valid);REQUIRE(qb.valid);CHECK(qa.targets==qb.targets);
        // Also protect the production lifting with all six physical anchors present.
        const auto h0=gf::task25DagContractFromCode(0);fronts.clear();for(const auto& u:h0.coverage_units)fronts[u.id]=target;
        const auto ha=gf::task20LiftTargets(h0,a.fixed,fronts),hb=gf::task20LiftTargets(h0,b.fixed,fronts);
        REQUIRE(ha.valid);REQUIRE(hb.valid);CHECK(ha.targets==hb.targets);
    }
}
TEST_CASE("both fixed modes initialize with all physical shoreline anchors and frozen old yaw") {
    const auto b=scene("docs/evidence/task32-fixed-library/shoreline-left-r1-anchor-asset.json");REQUIRE(b.valid);
    for(int mode:{0,12}) {
        auto s=gf::task10p11rFixedBaselineScenario();s.width_m=4500;s.height_m=2250;s.fixed_positions=b.fixed;
        for(auto& p:s.mobile_positions)p.x()+=750;
        if(mode==12)s.initial_topology=b.goal.reference_edges;
        auto cfg=gf::task19ProductionAdapterConfig();cfg.target_policy_task18_cbf2026_outer=false;cfg.target_policy_task20_dag_lattice=true;
        REQUIRE(cfg.task18_yaw_objective==gf::Task18YawObjective::ActualVelocity);
        cfg.distance_range_availability={true,134001};cfg.range_noise_std_m=0;cfg.range_dropout_probability=0;cfg.range_random_seed=2027;
        auto settings=gf::task10p11pSwarmSettings(s,gf::SolverProfile::Gurobi);
        gf::Task10p11rFixedBaselineFixture x(s,settings,cfg);const auto startup=x.adapter.initializeStageZero();REQUIRE(startup.initialized);
        auto pos=b.fixed;for(std::size_t k=0;k<s.mobile_ids.size();++k)pos[s.mobile_ids[k]]=s.mobile_positions[k];
        for(const auto& edge:s.initial_topology)CHECK((pos.at(edge.owner)-pos.at(edge.reference)).norm()<850);
        for(const auto& [id,p]:pos)if(id<=14)for(const auto& [other,q]:pos)if(other>id)CHECK((p-q).norm()>10);
        const auto info=gf::task31InformationTelemetry(x.adapter);
        CHECK(info["qualified_information"]["robust_fim_min"].get<double>()>=1e-6);
        CHECK(info["posterior_max_m2"].get<double>()<=.1);CHECK(info["aoi_margin_min_s"].get<double>()>=0.);
        for(const auto& row:x.adapter.currentSnapshotHardRows(s.initial_topology)) {
            if(row.kind==gf::CanonicalHardRowKind::ReferenceDistance||row.kind==gf::CanonicalHardRowKind::Collision)CHECK(row.barrier_h>0.);
            CHECK(row.kind!=gf::CanonicalHardRowKind::Workspace);
        }
        std::cout<<"SHORELINE_INITIAL "<<nlohmann::json({{"mode",mode},{"information",info},{"plant_advanced",false}}).dump()<<'\n';
    }
}
