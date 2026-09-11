#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task32FixedModeDispatch.hpp"
#include <fstream>

static nlohmann::json nativeMode(int mode=0,double width=4500.,double height=2250.) {
    nlohmann::json frames;
    if(mode==0||mode==11)frames={{"A",{width*.5,-50}},{"B",{width*.5,-50}}};
    else if(mode==2)frames={{"L",{width*9./20.,-50}},{"C",{width*11./20.,-50}},{"R",{width*.5,-50}}};
    else frames={{"T",{width*.5,-50}}};
    return {{"schema","task32-mode-contract-v1"},
        {"physical_scene",{{"width_m",width},{"height_m",height},{"physical_anchors",{
            {"100",{width*.4,-50}},{"101",{width*.5,-50}},{"102",{width*.6,-50}},
            {"103",{width*.45,-50}},{"104",{width*.55,-50}},{"105",{width*.65,-50}}}}}},
        {"coverage",{{"mode_code",mode},{"mapping",{{"kind","native-task25"},{"unit_frames",frames}}}}},
        {"initialization",{{"version","task31-original-physical-launch-v1"}}}};
}

TEST_CASE("Omitted fixed-mode dispatch preserves legacy CLI and emits no research identity") {
    const std::vector<std::string> legacy{"runner","origin","result.json","progress","telemetry.jsonl","14","900","task25-h0-p0"};
    const auto request=gf::task32FixedModeDispatchRequest(legacy);
    REQUIRE(request.valid);
    CHECK_FALSE(request.enabled);
    CHECK(request.legacy_arguments==legacy);
    CHECK(request.asset_path.empty());
    const auto plan=gf::task32FixedModeDispatch(request,nlohmann::json("unused malformed asset"));
    REQUIRE(plan.valid);
    CHECK_FALSE(plan.enabled);
    CHECK(plan.telemetry_identity.is_null());
    CHECK_FALSE(plan.asset.has_value());
}

TEST_CASE("Explicit fixed-mode suffix prepares a sealed scene contract without granting runtime admission") {
    const std::vector<std::string> legacy{"runner","origin","result.json","progress","telemetry.jsonl","14","900","task25-h0-p0","range-availability=frozen.json"};
    auto args=legacy;args.push_back("fixed-mode-asset=mode.json");
    const auto request=gf::task32FixedModeDispatchRequest(args);
    REQUIRE(request.valid);REQUIRE(request.enabled);
    CHECK(request.legacy_arguments==legacy);CHECK(request.asset_path=="mode.json");
    const auto plan=gf::task32FixedModeDispatch(request,nativeMode());
    REQUIRE(plan.valid);REQUIRE(plan.enabled);REQUIRE(plan.asset.has_value());
    CHECK(plan.asset->mode_code==0);CHECK(plan.asset->fixed.size()==6);
    CHECK(plan.asset->contract.coverage_units.size()==2);CHECK(plan.asset->contract.member_roles.size()==14);
    CHECK_FALSE(plan.asset->runtime_qualified);
    CHECK(plan.telemetry_identity.at("physical_scene")==nativeMode().at("physical_scene"));
    CHECK(plan.telemetry_identity.at("coverage_contract").at("mode_code")==0);
    CHECK(plan.telemetry_identity.at("qualification").at("runtime_qualified")==false);
    CHECK(plan.telemetry_identity.at("initialization").at("executed")==false);
    CHECK(plan.telemetry_identity.at("dispatch").at("control_configuration_changed")==false);
    CHECK_FALSE(gf::task32FixedModeDispatch(request,nlohmann::json::object()).valid);
    auto bad=args;bad.back()="fixed-mode-asset=";
    CHECK_FALSE(gf::task32FixedModeDispatchRequest(bad).valid);
    bad=args;bad.push_back("fixed-mode-asset=other.json");
    CHECK_FALSE(gf::task32FixedModeDispatchRequest(bad).valid);
    bad=args;bad.push_back("range-availability=wrong-order.json");
    CHECK_FALSE(gf::task32FixedModeDispatchRequest(bad).valid);
}

TEST_CASE("Pure scene preparation preserves the registered physical launch and separates DAG from all anchors") {
    const auto baseline=gf::task10p11pStandardCoastalScenario();
    const auto disabled=gf::task32FixedModeDispatch(gf::task32FixedModeDispatchRequest({"runner"}),{});
    auto legacy=baseline;legacy.id="legacy-sentinel";legacy.mobile_positions[0].x()=42.;
    const auto untouched=gf::task32FixedModeScenario(disabled,legacy);
    CHECK(untouched.id==legacy.id);CHECK(untouched.mobile_positions==legacy.mobile_positions);
    CHECK(untouched.fixed_positions==legacy.fixed_positions);CHECK(untouched.initial_topology==legacy.initial_topology);
    CHECK(gf::task32FixedModeGeometryIdentity(disabled,legacy).is_null());
    const auto request=gf::task32FixedModeDispatchRequest({"runner","fixed-mode-asset=mode.json"});
    for(int mode:{0,2,11,13})for(double width:{2250.,3000.,4500.}) {
        const double height=width==2250.?4500.:(width==4500.?2250.:3000.);
        const auto plan=gf::task32FixedModeDispatch(request,nativeMode(mode,width,height));
        INFO("mode="<<mode<<" width="<<width<<" reason="<<plan.reason);REQUIRE(plan.valid);
        const auto scene=gf::task32FixedModeScenario(plan,baseline);
        CHECK(scene.width_m==width);CHECK(scene.height_m==height);CHECK(scene.fixed_positions.size()==6);
        CHECK(scene.mobile_ids==baseline.mobile_ids);CHECK(scene.initial_topology==plan.asset->contract.reference_edges);
        REQUIRE(scene.mobile_positions.size()==14);
        for(std::size_t i=0;i<14;++i) {
            CHECK(scene.mobile_positions[i].x()==baseline.mobile_positions[i].x()+(width-3000.)*.5);
            CHECK(scene.mobile_positions[i].y()==baseline.mobile_positions[i].y());
        }
        const auto identity=gf::task32FixedModeGeometryIdentity(plan,scene);
        CHECK(identity.at("initialization").at("executed")==false);
        CHECK(identity.at("initialization").at("prepared_mobile_ids").size()==14);
        CHECK(identity.at("initialization").at("prepared_mobile_positions").size()==14);
        CHECK(identity.at("qualification").at("runtime_qualified")==false);
        auto wrong=scene;wrong.fixed_positions.at(103).x()+=1;
        CHECK_THROWS(gf::task32FixedModeGeometryIdentity(plan,wrong));
        wrong=scene;wrong.initial_topology.pop_back();CHECK_THROWS(gf::task32FixedModeGeometryIdentity(plan,wrong));
        wrong=scene;wrong.mobile_positions[0].x()+=1;CHECK_THROWS(gf::task32FixedModeGeometryIdentity(plan,wrong));
        CHECK_THROWS(gf::task32FixedModeScenario(plan,legacy));
    }
}

TEST_CASE("Frozen Pinball registry and original Strip status survive dispatch identity") {
    std::ifstream f("docs/evidence/task32-fixed-library/observation-r1-anchor-asset.json");REQUIRE(f.good());
    nlohmann::json registered;f>>registered;
    auto asset=nativeMode();asset["coverage"]={{"mode_code",12},{"mapping",{{"kind","frozen-task31"},{"asset",registered}}}};
    asset["physical_scene"]["physical_anchors"]=registered["physical_anchors"];
    const auto request=gf::task32FixedModeDispatchRequest({"runner","fixed-mode-asset=pinball.json"});
    CHECK_FALSE(gf::task32FixedModeDispatch(request,asset).valid);
    const auto pinball=gf::task32FixedModeDispatch(request,asset,&registered);REQUIRE(pinball.valid);
    const auto scene=gf::task32FixedModeScenario(pinball,gf::task10p11pStandardCoastalScenario());
    CHECK(scene.initial_topology==pinball.asset->contract.reference_edges);CHECK(scene.fixed_positions==pinball.asset->fixed);
    CHECK(pinball.telemetry_identity.at("coverage_contract").at("observation").at("P").at("effective_members").size()==1);
    auto changed=registered;changed["search_observation"]="legacy_front";
    CHECK_FALSE(gf::task32FixedModeDispatch(request,asset,&changed).valid);
    asset=nativeMode();asset["coverage"]={{"mode_code",1},{"mapping",{{"kind","native-task20-strip"},{"unit_frames",{{"M",{2250,-50}}}}}}};
    const auto strip=gf::task32FixedModeDispatch(request,asset);REQUIRE(strip.valid);
    CHECK(strip.asset->strip_role_reassignment_pending);
    CHECK(strip.telemetry_identity.at("coverage_contract").at("strip_role_reassignment_pending")==true);
    CHECK(strip.asset->contract.coverage_units[0].members==std::vector<gf::NodeId>{1,8,2,9,3,10,4,11,5,12,6,13,7,14});
}

TEST_CASE("Nine registered candidate assets prepare geometry while the tenth registry gap stays explicit") {
    const std::string root="docs/evidence/task32-fixed-library/";
    std::ifstream index_file(root+"mode-contract-assets-index-r1.json");REQUIRE(index_file.good());
    nlohmann::json index;index_file>>index;int prepared_count=0,missing_count=0;
    for(const auto& cell:index.at("cells")) {
        if(cell.at("asset").is_null()) {
            ++missing_count;CHECK(cell.at("mode_code")==12);CHECK(cell.at("width_m")==3000);
            CHECK(cell.at("runtime_qualified")==false);continue;
        }
        const auto path=root+cell.at("asset").get<std::string>();
        std::ifstream input(path);REQUIRE(input.good());nlohmann::json asset;input>>asset;
        nlohmann::json registration;const nlohmann::json* registered=nullptr;
        if(!cell.at("registered_task31_source").is_null()) {
            std::ifstream source(root+cell.at("registered_task31_source").get<std::string>());REQUIRE(source.good());
            source>>registration;registered=&registration;
        }
        const auto request=gf::task32FixedModeDispatchRequest({"runner","fixed-mode-asset="+path});
        const auto plan=gf::task32FixedModeDispatch(request,asset,registered);INFO(path<<" "<<plan.reason);REQUIRE(plan.valid);
        const auto scene=gf::task32FixedModeScenario(plan,gf::task10p11pStandardCoastalScenario());
        const auto identity=gf::task32FixedModeGeometryIdentity(plan,scene);
        CHECK(scene.width_m==cell.at("width_m").get<double>());CHECK(scene.height_m==cell.at("height_m").get<double>());
        CHECK(scene.fixed_positions.size()==(scene.width_m==4500.?6:3));
        CHECK(identity.at("coverage_contract").at("mode_code")==cell.at("mode_code"));
        CHECK(identity.at("qualification").at("runtime_qualified")==false);
        CHECK(identity.at("qualification").at("service_qualified")==false);
        CHECK(identity.at("initialization").at("executed")==false);
        ++prepared_count;
    }
    CHECK(prepared_count==9);CHECK(missing_count==1);
}
