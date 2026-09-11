#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task32ModeContract.hpp"
#include <fstream>

using Json=nlohmann::json;

static gf::Task20DagLatticeContract original(int mode) {
    return mode==1?gf::task20DagLatticeContract(gf::Task20LatticeMode::MergedStrip)
        :gf::task25DagContractFromCode(mode);
}
static Json request(int mode,double width=4500,double height=2250) {
    Json anchors={{"100",{.4*width,-50}},{"101",{.5*width,-50}},{"102",{.6*width,-50}},
        {"103",{.45*width,-50}},{"104",{.55*width,-50}},{"105",{.65*width,-50}}};
    Json frames=Json::object();const auto prior=original(mode);
    for(const auto& u:prior.coverage_units) {
        Eigen::Vector2d p=Eigen::Vector2d::Zero();
        for(auto id:u.base_anchors)p+=gf::task32_fixed_asset_detail::point(anchors.at(std::to_string(id)));
        p/=static_cast<double>(u.base_anchors.size());frames[u.id]={p.x(),p.y()};
    }
    return {{"schema","task32-mode-contract-v1"},
        {"physical_scene",{{"width_m",width},{"height_m",height},{"physical_anchors",anchors}}},
        {"coverage",{{"mode_code",mode},{"mapping",{{"kind",mode==1?"native-task20-strip":"native-task25"},{"unit_frames",frames}}}}},
        {"initialization",{{"version","task31-original-physical-launch-v1"}}}};
}

TEST_CASE("versioned native contracts keep original fourteen roles refs and targets") {
    for(int mode:{0,1,2,11,13})for(const auto shape:std::vector<std::pair<double,double>>{{4500,2250},{2250,4500},{3000,3000}}) {
        const auto a=gf::task32ModeContract(request(mode,shape.first,shape.second));REQUIRE(a.valid);
        const auto old=original(mode);CHECK(a.contract.reference_edges==old.reference_edges);
        CHECK(a.mode_code==mode);CHECK_FALSE(a.runtime_qualified);
        CHECK(a.strip_role_reassignment_pending==(mode==1));
        CHECK(a.initialization_version=="task31-original-physical-launch-v1");
        REQUIRE(a.identity.at("coverage_contract").at("member_roles").size()==14);
        REQUIRE(a.identity.at("coverage_contract").at("reference_edges").size()==28);
        CHECK(a.identity.at("physical_scene").at("physical_anchors").size()==6);
        CHECK_FALSE(a.identity.at("qualification").at("runtime_qualified").get<bool>());
        for(std::size_t i=0;i<old.coverage_units.size();++i) {
            const auto& x=old.coverage_units[i];const auto& y=a.contract.coverage_units[i];
            CHECK(x.members==y.members);CHECK(x.front_members==y.front_members);
            CHECK(x.search_observation_members==y.search_observation_members);
        }
        for(int pose=0;pose<4;++pose) {
            std::map<std::string,Eigen::Vector2d> fronts;int unit=0;
            for(const auto& u:old.coverage_units)fronts[u.id]={105.+600*unit++,205.+410*pose};
            const auto before=gf::task20LiftTargets(old,a.fixed,fronts),after=gf::task20LiftTargets(a.contract,a.fixed,fronts);
            REQUIRE(before.valid);REQUIRE(after.valid);REQUIRE(after.targets.size()==14);
            CHECK(before.targets==after.targets);
        }
    }
}

TEST_CASE("native wrapper exactly reuses fixed asset and Cross only changes DAG") {
    auto input=request(0);const auto a=gf::task32ModeContract(input);REQUIRE(a.valid);
    Json old={{"schema","task32-fixed-coverage-asset-v1"},{"width_m",4500},{"height_m",2250},
        {"physical_anchors",input["physical_scene"]["physical_anchors"]},{"mode_code",0},{"mapping",input["coverage"]["mapping"]}};
    const auto b=gf::task32FixedCoverageAsset(old);REQUIRE(b.valid);
    CHECK(a.contract.reference_edges==b.contract.reference_edges);
    const auto cross=gf::task32ModeContract(request(11));REQUIRE(cross.valid);
    CHECK(a.contract.reference_edges!=cross.contract.reference_edges);
    const std::map<std::string,Eigen::Vector2d> fronts{{"A",{115,1795}},{"B",{4325,2245}}};
    CHECK(gf::task20LiftTargets(a.contract,a.fixed,fronts).targets==gf::task20LiftTargets(cross.contract,cross.fixed,fronts).targets);
    CHECK(gf::task20LiftTargets(a.contract,a.fixed,fronts).targets==gf::task20LiftTargets(b.contract,b.fixed,fronts).targets);
    CHECK(a.identity.at("coverage_contract").at("member_roles")==cross.identity.at("coverage_contract").at("member_roles"));
    input["physical_scene"]["physical_anchors"]["103"]={17,29};
    const auto moved=gf::task32ModeContract(input);REQUIRE(moved.valid);
    CHECK(a.identity.at("coverage_contract").at("front_frames")==moved.identity.at("coverage_contract").at("front_frames"));
}

TEST_CASE("registered Pinball embeds unchanged legacy mapping and observation") {
    std::ifstream f("docs/evidence/task32-fixed-library/observation-r1-anchor-asset.json");REQUIRE(f.good());
    Json registered;f>>registered;auto input=request(0);
    input["coverage"]={{"mode_code",12},{"mapping",{{"kind","frozen-task31"},{"asset",registered}}}};
    input["physical_scene"]["physical_anchors"]=registered["physical_anchors"];
    CHECK_FALSE(gf::task32ModeContract(input).valid);
    const auto parsed=gf::task32ModeContract(input,&registered);REQUIRE(parsed.valid);
    const auto old=gf::task31AnchorScene(registered);REQUIRE(old.valid);
    CHECK(parsed.contract.reference_edges==old.goal.reference_edges);
    CHECK(parsed.contract.coverage_units[0].search_observation_members==old.goal.coverage_units[0].search_observation_members);
    for(int k=0;k<4;++k) {
        std::map<std::string,Eigen::Vector2d> fronts;for(const auto& u:old.goal.coverage_units)fronts[u.id]={105.+1000*k,205.+500*k};
        CHECK(gf::task20LiftTargets(parsed.contract,parsed.fixed,fronts).targets==gf::task20LiftTargets(old.goal,old.fixed,fronts).targets);
    }
    input["coverage"]["mapping"]["asset"]["search_observation"]="legacy_front";
    CHECK_FALSE(gf::task32ModeContract(input,&registered).valid);
}

TEST_CASE("unregistered overrides modes initializations and Strip reorder fail closed") {
    auto j=request(1);j["coverage"]["mapping"]["roles"]={{"1",{99,99}}};CHECK_FALSE(gf::task32ModeContract(j).valid);
    j=request(1);j["coverage"]["mapping"]["kind"]="native-task25";CHECK_FALSE(gf::task32ModeContract(j).valid);
    j=request(1);j["coverage"]["mapping"]["unit_frames"]["M"]={0,0};CHECK_FALSE(gf::task32ModeContract(j).valid);
    j=request(1);j["coverage"]["role_permutation"]={8,1};CHECK_FALSE(gf::task32ModeContract(j).valid);
    j=request(0);j["initialization"]["version"]="arbitrary-restart";CHECK_FALSE(gf::task32ModeContract(j).valid);
    j=request(0);j["initialization"]["state"]={0,0};CHECK_FALSE(gf::task32ModeContract(j).valid);
    j=request(0);j["coverage"]["mode_code"]=std::uint64_t{4294967296ULL};CHECK_FALSE(gf::task32ModeContract(j).valid);
    j=request(0);j["coverage"]["mode_code"]=10;CHECK_FALSE(gf::task32ModeContract(j).valid);
    j=request(0);j["physical_scene"]["physical_anchors"]["1"]={0,0};CHECK_FALSE(gf::task32ModeContract(j).valid);
    j=request(0);j["physical_scene"]["initialization"]="ignored";CHECK_FALSE(gf::task32ModeContract(j).valid);
}

TEST_CASE("finite atlas inputs parse without claiming a missing registered scene") {
    const std::string root="docs/evidence/task32-fixed-library/";
    std::ifstream index_stream(root+"mode-contract-assets-index-r1.json");REQUIRE(index_stream.good());
    Json index;index_stream>>index;std::size_t parsed_count=0,missing_count=0;
    for(const auto& cell:index.at("cells")) {
        if(cell.at("asset").is_null()) {++missing_count;CHECK(cell.at("mode_code")==12);continue;}
        std::ifstream input(root+cell.at("asset").get<std::string>());REQUIRE(input.good());Json asset;input>>asset;
        Json registered;const Json* registry=nullptr;
        if(!cell.at("registered_task31_source").is_null()) {
            std::ifstream source(root+cell.at("registered_task31_source").get<std::string>());REQUIRE(source.good());source>>registered;registry=&registered;
        }
        const auto result=gf::task32ModeContract(asset,registry);REQUIRE(result.valid);++parsed_count;
        CHECK(result.mode_code==cell.at("mode_code").get<int>());
        CHECK(result.width_m==cell.at("width_m").get<double>());CHECK(result.height_m==cell.at("height_m").get<double>());
        CHECK_FALSE(result.runtime_qualified);
        std::map<std::string,Eigen::Vector2d> fronts;
        for(const auto& u:result.contract.coverage_units)fronts[u.id]={.55*result.width_m,.7*result.height_m};
        const auto targets=gf::task20LiftTargets(result.contract,result.fixed,fronts);REQUIRE(targets.valid);CHECK(targets.targets.size()==14);
    }
    CHECK(parsed_count==9);CHECK(missing_count==1);
}
