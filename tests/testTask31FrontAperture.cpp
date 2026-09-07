#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task31AnchorScene.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include "grand_finale/Task29RoleCenterPath.hpp"
#include <fstream>

TEST_CASE("explicit front similarity changes geometry only about the shared task front") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-design-r1.json");
    REQUIRE(f.good());nlohmann::json design;f>>design;
    auto input=design.at("chosen");input["width_m"]=4500.;input["height_m"]=2250.;input["mode_code"]=12;
    input["front_similarity_gain"]=.5;
    const auto scene=gf::task31AnchorScene(input);REQUIRE(scene.valid);
    const Eigen::Vector2d g=scene.frame_origin+Eigen::Vector2d(0,1000);
    const auto q=gf::task20LiftTargets(scene.goal,scene.fixed,{{"P",g}});REQUIRE(q.valid);
    for(const auto& [id,p]:q.targets) {
        const auto old=input.at("final_positions").at(std::to_string(id));
        const Eigen::Vector2d expected=g+.5*(Eigen::Vector2d(old[0],old[1])-g);
        CHECK((p-expected).norm()<1e-8);
    }
}

TEST_CASE("front similarity preserves labelled units and rigid frame covariance") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    for(int code:{0,2,12,13}) {
        const auto c=gf::task25DagContractFromCode(code);
        const auto old=gf::task31TriangularLattice(c,fixed,{0,1},450);
        const auto same=gf::task31TriangularLattice(c,fixed,{0,1},450,1.);
        const auto half=gf::task31TriangularLattice(c,fixed,{0,1},450,.5);
        REQUIRE(old.valid);REQUIRE(same.valid);REQUIRE(half.valid);
        CHECK(half.contract.reference_edges==c.reference_edges);
        const Eigen::Rotation2Dd rotation(.73);const Eigen::Vector2d shift(93,-117);
        auto rc=c;std::map<gf::NodeId,Eigen::Vector2d> rf;
        for(const auto& [id,p]:fixed)rf[id]=rotation*p+shift;
        const auto moved=gf::task31TriangularLattice(rc,rf,rotation*Eigen::Vector2d(0,1),450,.5);
        REQUIRE(moved.valid);
        for(const auto& u:c.coverage_units) {
            const Eigen::Vector2d g(2300,1300);
            const auto q0=gf::task20LiftTargets(old.contract,fixed,{{u.id,g}});
            // All coverage-unit fronts are needed by the public lifting.
            std::map<std::string,Eigen::Vector2d> fronts,transformed;
            for(const auto& unit:c.coverage_units){fronts[unit.id]=g;transformed[unit.id]=rotation*g+shift;}
            const auto a=gf::task20LiftTargets(old.contract,fixed,fronts);
            const auto b=gf::task20LiftTargets(half.contract,fixed,fronts);
            const auto m=gf::task20LiftTargets(moved.contract,rf,transformed);
            REQUIRE(a.valid);REQUIRE(b.valid);REQUIRE(m.valid);
            for(auto id:u.members) {
                CHECK(same.contract.member_roles.at(id).axial_fraction==old.contract.member_roles.at(id).axial_fraction);
                CHECK(same.contract.member_roles.at(id).triangular_fraction==old.contract.member_roles.at(id).triangular_fraction);
                CHECK((b.targets.at(id)-(g+.5*(a.targets.at(id)-g))).norm()<1e-8);
                CHECK((m.targets.at(id)-(rotation*b.targets.at(id)+shift)).norm()<1e-8);
                CHECK(half.cells.at(id).row==old.cells.at(id).row);
                CHECK(half.cells.at(id).slot==old.cells.at(id).slot);
            }
        }
        for(double gain:{0.,-.1,1.0001,std::numeric_limits<double>::infinity()})
            CHECK_FALSE(gf::task31TriangularLattice(c,fixed,{0,1},450,gain).valid);
    }
}

TEST_CASE("aperture design reads the unchanged frozen coverage reserve") {
    const auto c=gf::task19ProductionAdapterConfig();
    CHECK(c.sensor_radius_m==400.);CHECK(c.uncertainty_sigma==3.);
    CHECK(c.maximum_posterior_eigenvalue_m2==.1);CHECK(c.certified_error_bound_m==.05);
    CHECK(c.certified_shadow_single_position_support_m==0.);
    CHECK(c.coverage_half_angle_rad==doctest::Approx(M_PI/3.));
    std::cout<<"TASK31_APERTURE_CONFIG "<<nlohmann::json{
        {"outer_radius_m",c.sensor_radius_m},{"inner_radius_m",c.coverage_inner_radius_m},
        {"half_angle_rad",c.coverage_half_angle_rad},{"uncertainty_sigma",c.uncertainty_sigma},
        {"posterior_limit_m2",c.maximum_posterior_eigenvalue_m2},
        {"certified_error_bound_m",c.certified_error_bound_m},
        {"shadow_support_m",c.certified_shadow_single_position_support_m},
        {"grid_spacing_m",gf::loadTask10p10Config().grid_spacing_m}}.dump()<<'\n';
}

TEST_CASE("explicit common front continuation is independent of final similarity") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-r1.json");
    REQUIRE(f.good());nlohmann::json asset;f>>asset;
    for(double gain:{1.,.5}) {
        asset["front_similarity_gain"]=gain;
        const auto scene=gf::task31AnchorScene(asset);REQUIRE(scene.valid);
        const Eigen::Vector2d g=scene.frame_origin+scene.final_front_offset;
        const auto to=gf::task20LiftTargets(scene.goal,scene.fixed,{{"P",g}});REQUIRE(to.valid);
        auto from=to.targets;for(auto& [id,p]:from)p-=Eigen::Vector2d(0,200);
        const gf::Task29RoleCenterPath path(scene.goal,from,to.targets);
        const std::map<std::string,Eigen::Vector2d> rates{{"P",{120,300}}};
        const auto q0=path.evaluateContinuingFrontWithRates(0.,rates);
        const auto q1=path.evaluateContinuingFrontWithRates(1.,rates);
        const auto q2=path.evaluateContinuingFrontWithRates(2.,rates);
        Eigen::Vector2d front1=Eigen::Vector2d::Zero(),front2=front1;
        const auto& members=scene.goal.coverage_units.front().front_members;
        for(auto id:members){front1+=q1.at(id);front2+=q2.at(id);}
        CHECK(((front2-front1)/members.size()-Eigen::Vector2d(120,300)).norm()<1e-8);
        for(const auto& [id,p]:from)CHECK((q0.at(id)-p).norm()==0.);
        CHECK_THROWS(path.evaluateContinuingFrontWithRates(2.,{}));
    }
}

TEST_CASE("continuation rates use the unscaled contract not the new role mean") {
    std::ifstream f("docs/evidence/task31-triangular-common-bridge/anchor-scene-r1.json");
    REQUIRE(f.good());nlohmann::json asset;f>>asset;
    const auto baseline=gf::task31AnchorScene(asset);REQUIRE(baseline.valid);
    const Eigen::Vector2d g=baseline.frame_origin+baseline.final_front_offset;
    auto canonical=gf::task20LiftTargets(baseline.goal,baseline.fixed,{{"P",g}}).targets;
    for(auto& [id,p]:canonical)p-=Eigen::Vector2d(120,300);
    for(double gain:{1.,.5}) {
        asset["front_similarity_gain"]=gain;
        const auto scene=gf::task31AnchorScene(asset);REQUIRE(scene.valid);
        const auto rates=gf::task31UnscaledContinuationRates(scene,canonical);
        // 5-4-3-2 rows have mean axial coefficient 15/28, mean slot0.
        CHECK((rates.at("P")-Eigen::Vector2d(224,560)).norm()<1e-8);
    }
}
