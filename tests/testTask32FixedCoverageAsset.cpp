#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task32FixedCoverageAsset.hpp"
#include "grand_finale/Task31AnchorScene.hpp"
#include <fstream>

static nlohmann::json nativeAsset(int mode) {
    nlohmann::json frames;
    // Adopted 2026-09-18: DualLadder unit frames are the centroids of the
    // anchors each unit's reference edges cite ({100,101} and {101,102}).
    if(mode==0||mode==11)frames={{"A",{2025,-50}},{"B",{2475,-50}}};
    else if(mode==2)frames={{"L",{2025,-50}},{"C",{2475,-50}},{"R",{2250,-50}}};
    else frames={{"T",{2250,-50}}};
    return {{"schema","task32-fixed-coverage-asset-v1"},{"width_m",4500},{"height_m",2250},{"mode_code",mode},
        {"physical_anchors",{{"100",{1800,-50}},{"101",{2250,-50}},{"102",{2700,-50}},
            {"103",{2025,-50}},{"104",{2475,-50}},{"105",{2925,-50}}}},
        {"mapping",{{"kind","native-task25"},{"unit_frames",frames}}}};
}

TEST_CASE("Explicit native H0 asset preserves all fourteen original targets") {
    nlohmann::json asset={{"schema","task32-fixed-coverage-asset-v1"},
        {"width_m",4500},{"height_m",2250},{"mode_code",0},
        {"physical_anchors",{{"100",{1800,-50}},{"101",{2250,-50}},{"102",{2700,-50}},
            {"103",{2025,-50}},{"104",{2475,-50}},{"105",{2925,-50}}}},
        {"mapping",{{"kind","native-task25"},{"unit_frames",{{"A",{2025,-50}},{"B",{2475,-50}}}}}}};
    const auto parsed=gf::task32FixedCoverageAsset(asset);
    REQUIRE(parsed.valid);CHECK(parsed.mode_code==0);CHECK(parsed.fixed.size()==6);
    const auto old=gf::task25DagContractFromCode(0);
    const std::map<std::string,Eigen::Vector2d> fronts{{"A",{300,2200}},{"B",{4200,2200}}};
    const auto expected=gf::task20LiftTargets(old,parsed.fixed,fronts);
    const auto result=gf::task20LiftTargets(parsed.contract,parsed.fixed,fronts);
    REQUIRE(expected.valid);REQUIRE(result.valid);REQUIRE(result.targets.size()==14);
    for(const auto& [id,p]:expected.targets)CHECK((result.targets.at(id)-p).norm()==0);
    CHECK(parsed.contract.reference_edges==old.reference_edges);
    CHECK(parsed.contract.coverage_units.size()==2);
}

TEST_CASE("Native one two three units retain role maps and distinct reference frames") {
    for(int mode:{0,11,2,13}) {
        auto asset=nativeAsset(mode);const auto parsed=gf::task32FixedCoverageAsset(asset);REQUIRE(parsed.valid);
        const auto old=gf::task25DagContractFromCode(mode);
        CHECK(parsed.contract.reference_edges==old.reference_edges);
        CHECK(parsed.contract.coverage_units.size()==(mode==2?3:(mode==13?1:2)));
        for(int pose=0;pose<4;++pose) {
            std::map<std::string,Eigen::Vector2d> fronts;int k=0;
            for(const auto& unit:old.coverage_units)fronts[unit.id]={100.+850*k++ +50*pose,200.+600*pose};
            const auto expected=gf::task20LiftTargets(old,parsed.fixed,fronts);
            const auto actual=gf::task20LiftTargets(parsed.contract,parsed.fixed,fronts);
            REQUIRE(expected.valid);REQUIRE(actual.valid);REQUIRE(actual.targets.size()==14);
            for(const auto& [id,p]:expected.targets)CHECK((actual.targets.at(id)-p).norm()==0);
        }
        asset["physical_anchors"]["103"]={17,29};
        const auto shifted=gf::task32FixedCoverageAsset(asset);REQUIRE(shifted.valid);
        for(size_t i=0;i<old.coverage_units.size();++i)
            CHECK((*shifted.contract.coverage_units[i].frame_origin-*parsed.contract.coverage_units[i].frame_origin).norm()==0);
    }
    const auto h=gf::task32FixedCoverageAsset(nativeAsset(0)),c=gf::task32FixedCoverageAsset(nativeAsset(11));
    CHECK(h.contract.reference_edges!=c.contract.reference_edges);
    const std::map<std::string,Eigen::Vector2d> fronts{{"A",{100,2000}},{"B",{4100,1800}}};
    CHECK(gf::task20LiftTargets(h.contract,h.fixed,fronts).targets==gf::task20LiftTargets(c.contract,c.fixed,fronts).targets);
}

TEST_CASE("Fixed asset refuses missing frames silent overrides and identity mismatches") {
    auto a=nativeAsset(2);a["mapping"]["unit_frames"].erase("C");CHECK_FALSE(gf::task32FixedCoverageAsset(a).valid);
    a=nativeAsset(2);a["mapping"]["unit_frames"]["L"]={2250,-50};CHECK_FALSE(gf::task32FixedCoverageAsset(a).valid);
    a=nativeAsset(0);a["mapping"]["roles"]={{"1",{99,99}}};CHECK_FALSE(gf::task32FixedCoverageAsset(a).valid);
    a=nativeAsset(0);a["reference_edges"]={{100,1},{102,1}};CHECK_FALSE(gf::task32FixedCoverageAsset(a).valid);
    a=nativeAsset(0);a["physical_anchors"]["1"]={0,0};CHECK_FALSE(gf::task32FixedCoverageAsset(a).valid);
    a=nativeAsset(0);a["physical_anchors"]["100"]={0,0};CHECK_FALSE(gf::task32FixedCoverageAsset(a).valid);
    a=nativeAsset(0);a["mode_code"]=12;CHECK_FALSE(gf::task32FixedCoverageAsset(a).valid);
    a=nativeAsset(0);a["mode_code"]=std::uint64_t{4294967296ULL};CHECK_FALSE(gf::task32FixedCoverageAsset(a).valid);
}

TEST_CASE("Frozen Pinball mapping stays exactly the confirmed legacy asset") {
    std::ifstream f("docs/evidence/task32-fixed-library/observation-r1-anchor-asset.json");REQUIRE(f.good());
    nlohmann::json legacy;f>>legacy;const auto expected=gf::task31AnchorScene(legacy);REQUIRE(expected.valid);
    auto a=nativeAsset(0);a["mode_code"]=12;a["physical_anchors"]=legacy["physical_anchors"];
    a["mapping"]={{"kind","frozen-task31"},{"asset",legacy}};
    CHECK_FALSE(gf::task32FixedCoverageAsset(a).valid);
    const auto parsed=gf::task32FixedCoverageAsset(a,&legacy);REQUIRE(parsed.valid);
    CHECK(parsed.contract.reference_edges==expected.goal.reference_edges);
    for(int pose=0;pose<4;++pose) {
        std::map<std::string,Eigen::Vector2d> fronts;
        for(const auto& unit:expected.goal.coverage_units)fronts[unit.id]={100.+1000*pose,200.+600*pose};
        const auto old=gf::task20LiftTargets(expected.goal,expected.fixed,fronts),now=gf::task20LiftTargets(parsed.contract,parsed.fixed,fronts);
        REQUIRE(old.valid);REQUIRE(now.valid);for(const auto& [id,p]:old.targets)CHECK((now.targets.at(id)-p).norm()==0);
    }
    CHECK(parsed.contract.coverage_units[0].search_observation_members==expected.goal.coverage_units[0].search_observation_members);
    auto changed=a;changed["mapping"]["asset"]["search_observation"]="legacy_front";CHECK_FALSE(gf::task32FixedCoverageAsset(changed,&legacy).valid);
    changed=a;changed["mapping"]["asset"]["frame_origin"]={2200,-50};CHECK_FALSE(gf::task32FixedCoverageAsset(changed,&legacy).valid);
    changed=a;changed["mapping"]["asset"]["roles"]={{"1",{99,99}}};CHECK_FALSE(gf::task32FixedCoverageAsset(changed,&legacy).valid);
    a["physical_anchors"]["103"]={17,29};CHECK_FALSE(gf::task32FixedCoverageAsset(a,&legacy).valid);
    a["physical_anchors"]=legacy["physical_anchors"];a["mode_code"]=0;CHECK_FALSE(gf::task32FixedCoverageAsset(a,&legacy).valid);
}
