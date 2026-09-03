#include "grand_finale/Task21PersistentRibbon.hpp"
#include "grand_finale/Task20GridOracle.hpp"
#include "grand_finale/Task10p11rFixedBaseline.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>

namespace {

gf::Task20DagLatticeContract contractFor(const std::string& mode) {
    if (mode=="dual-ladder")
        return gf::task20DagLatticeContract(gf::Task20LatticeMode::DualLadder);
    if (mode=="whole-strip")
        return gf::task20DagLatticeContract(gf::Task20LatticeMode::MergedStrip);
    if (mode=="pinball") return gf::task21PinballContract();
    throw std::invalid_argument("unknown Task 21 DAG mode");
}

bool containsCell(const Eigen::Vector2d& pose,double yaw,int x,int y) {
    constexpr double reserve=0.05+5.0*1.4142135623730950488;
    const Eigen::Vector2d center{5.0+10.0*x,5.0+10.0*y};
    const Eigen::Vector2d delta=center-pose;
    const double distance=delta.norm();
    if (distance<=reserve||distance+reserve>400.0) return false;
    const double bearing=std::atan2(delta.y(),delta.x());
    const double error=std::abs(gf::wrapYawRad(bearing-yaw));
    return error+std::asin(std::min(1.0,reserve/distance))<=M_PI/3.0+1.0e-12;
}

nlohmann::json point(const Eigen::Vector2d& value) {
    return nlohmann::json::array({value.x(),value.y()});
}

struct ScanResult {
    double spacing=0.0;
    std::size_t planned_count=0;
    std::size_t joint_count=0;
    double maximum_reference=0.0;
    double minimum_separation=std::numeric_limits<double>::infinity();
    double minimum_fim=std::numeric_limits<double>::infinity();
    double maximum_adjacent_target_change=0.0;
    nlohmann::json maximum_reference_witness;
    nlohmann::json minimum_separation_witness;
    nlohmann::json minimum_fim_witness;
    std::vector<bool> covered;
    std::vector<nlohmann::json> witnesses;
    std::vector<nlohmann::json> segment_records;
    std::vector<nlohmann::json> transition_records;
    std::vector<nlohmann::json> configuration_records;
};

ScanResult scan(double spacing,const gf::Task20DagLatticeContract& contract,
    const std::map<gf::NodeId,Eigen::Vector2d>& fixed,
    const std::vector<bool>& initial,
    const std::map<std::string,Eigen::Vector2d>& initial_fronts,
    bool save_witnesses) {
    std::vector<gf::FrontierCell> cells;
    cells.reserve(90000);
    for (int x=0;x<300;++x) for (int y=0;y<300;++y)
        cells.push_back({x,y,{5.0+10.0*x,5.0+10.0*y}});
    std::vector<std::string> unit_ids;
    for (const auto& unit:contract.coverage_units) unit_ids.push_back(unit.id);
    const auto field=gf::task21AffineCoordinateField(
        {0.0,0.0},{0.0,1.0},{1.0,0.0});
    const auto plan=gf::task21BuildRibbonPlan(
        cells,unit_ids,field,spacing,1024,initial_fronts);
    if (!plan.valid) throw std::runtime_error(plan.reason);
    ScanResult result;
    result.spacing=spacing;
    result.covered.assign(90000,false);
    if (save_witnesses) result.witnesses.resize(90000);
    std::map<std::string,const gf::Task20CoverageUnit*> units;
    for (const auto& unit:contract.coverage_units) units[unit.id]=&unit;
    std::map<std::string,const gf::Task21RibbonCorridor*> corridors;
    for (const auto& corridor:plan.corridors)
        corridors[corridor.coverage_unit]=&corridor;
    for (const auto& [unit_id,route]:plan.routes) {
        const auto& unit=*units.at(unit_id);
        const auto& corridor=*corridors.at(unit_id);
        for (const auto& segment:route.segments) {
            std::vector<bool> segment_covered;
            if (save_witnesses) segment_covered.assign(90000,false);
            std::map<gf::NodeId,Eigen::Vector2d> previous_targets;
            double previous_cross=0.0;
            const double yaw=std::atan2(segment.tangent.y(),segment.tangent.x());
            for (double cross=5.0;cross<3000.0;cross+=10.0) {
                if (cross<corridor.cross_min||cross>=corridor.cross_max) continue;
                std::map<std::string,Eigen::Vector2d> fronts;
                for (const auto& other:contract.coverage_units) {
                    const auto& other_corridor=*corridors.at(other.id);
                    const double other_cross=other.id==unit_id?cross:
                        0.5*(other_corridor.cross_min+other_corridor.cross_max);
                    fronts[other.id]=field.origin+
                        segment.route_progress*field.progress_axis+
                        other_cross*field.cross_axis;
                }
                const auto lifted=gf::task20LiftTargets(contract,fixed,fronts);
                if (!lifted.valid) throw std::runtime_error(lifted.reason);
                const auto geometry=gf::task20EvaluateOracleGeometry(
                    contract,fixed,lifted.targets);
                const std::string configuration_id=unit_id+":"+
                    std::to_string(segment.band_index)+":"+
                    std::to_string(static_cast<int>(std::llround(cross)));
                nlohmann::json fronts_json=nlohmann::json::object();
                for (const auto& [front_id,front_pose]:fronts)
                    fronts_json[front_id]=point(front_pose);
                const nlohmann::json pose_key={{"coverage_unit",unit_id},
                    {"route_band",segment.band_index},
                    {"route_progress_m",segment.route_progress},
                    {"route_cross_m",cross},{"fronts",fronts_json}};
                if (geometry.maximum_reference_edge_m>
                    result.maximum_reference) {
                    result.maximum_reference=geometry.maximum_reference_edge_m;
                    result.maximum_reference_witness=pose_key;
                }
                if (geometry.minimum_target_separation_m<
                    result.minimum_separation) {
                    result.minimum_separation=geometry.minimum_target_separation_m;
                    result.minimum_separation_witness=pose_key;
                }
                if (geometry.minimum_nominal_fim_proxy<result.minimum_fim) {
                    result.minimum_fim=geometry.minimum_nominal_fim_proxy;
                    result.minimum_fim_witness=pose_key;
                }
                if (save_witnesses&&!previous_targets.empty()) {
                    double maximum_change=0.0;
                    for (const auto& [member,target]:lifted.targets)
                        maximum_change=std::max(maximum_change,
                            (target-previous_targets.at(member)).norm());
                    result.maximum_adjacent_target_change=std::max(
                        result.maximum_adjacent_target_change,maximum_change);
                    result.transition_records.push_back({
                        {"coverage_unit",unit_id},
                        {"route_band",segment.band_index},
                        {"from_cross_m",previous_cross},{"to_cross_m",cross},
                        {"maximum_member_target_change_m",maximum_change}});
                }
                previous_targets=lifted.targets;
                previous_cross=cross;
                if (save_witnesses) {
                    nlohmann::json targets_json=nlohmann::json::object();
                    for (const auto& [member,target]:lifted.targets)
                        targets_json[std::to_string(member)]=point(target);
                    nlohmann::json reference_edges=nlohmann::json::array();
                    for (const auto& edge:contract.reference_edges) {
                        const auto mobile=lifted.targets.find(edge.reference);
                        const Eigen::Vector2d reference=
                            mobile==lifted.targets.end()?fixed.at(edge.reference):
                            mobile->second;
                        reference_edges.push_back({
                            {"reference",edge.reference},{"owner",edge.owner},
                            {"distance_m",(reference-lifted.targets.at(edge.owner)).norm()}});
                    }
                    result.configuration_records.push_back({
                        {"configuration_id",configuration_id},
                        {"coverage_unit",unit_id},
                        {"route_band",segment.band_index},
                        {"route_progress_m",segment.route_progress},
                        {"route_cross_m",cross},{"route_tangent",point(segment.tangent)},
                        {"fronts",fronts_json},{"targets",targets_json},
                        {"reference_edges",reference_edges},
                        {"maximum_nominal_reference_m",geometry.maximum_reference_edge_m},
                        {"minimum_nominal_separation_m",geometry.minimum_target_separation_m},
                        {"minimum_nominal_fim_proxy",geometry.minimum_nominal_fim_proxy}});
                }
                for (gf::NodeId member:unit.members) {
                    const Eigen::Vector2d pose=lifted.targets.at(member);
                    const int xmin=std::max(0,static_cast<int>(
                        std::floor((pose.x()-400.0)/10.0)));
                    const int xmax=std::min(299,static_cast<int>(
                        std::floor((pose.x()+400.0)/10.0)));
                    const int ymin=std::max(0,static_cast<int>(
                        std::floor((pose.y()-400.0)/10.0)));
                    const int ymax=std::min(299,static_cast<int>(
                        std::floor((pose.y()+400.0)/10.0)));
                    for (int x=xmin;x<=xmax;++x) for (int y=ymin;y<=ymax;++y) {
                        const std::size_t index=static_cast<std::size_t>(x*300+y);
                        if (!containsCell(pose,yaw,x,y)) continue;
                        if (save_witnesses) segment_covered[index]=true;
                        if (result.covered[index]) continue;
                        result.covered[index]=true;
                        if (save_witnesses&&!initial[index])
                            result.witnesses[index]={{"cell_id",
                                std::to_string(x)+":"+std::to_string(y)},
                                {"configuration_id",configuration_id},
                                {"coverage_unit",unit_id},
                                {"responsible_member",member},
                                {"route_band",segment.band_index},
                                {"route_progress_m",segment.route_progress},
                                {"route_cross_m",cross},
                                {"route_tangent",point(segment.tangent)},
                                {"service_pose",point(pose)},
                                {"service_yaw_rad",yaw},
                                {"fronts",fronts_json},
                                {"maximum_nominal_reference_m",
                                    geometry.maximum_reference_edge_m},
                                {"minimum_nominal_separation_m",
                                    geometry.minimum_target_separation_m},
                                {"minimum_nominal_fim_proxy",
                                    geometry.minimum_nominal_fim_proxy}};
                    }
                }
            }
            if (save_witnesses) {
                const std::size_t count=static_cast<std::size_t>(std::count(
                    segment_covered.begin(),segment_covered.end(),true));
                result.segment_records.push_back({
                    {"coverage_unit",unit_id},{"route_band",segment.band_index},
                    {"route_progress_m",segment.route_progress},
                    {"route_tangent",point(segment.tangent)},
                    {"covered_count",count},
                    {"covered_bits_hex",
                        gf::task17_grid_detail::bitHex(segment_covered)}});
            }
        }
    }
    for (std::size_t index=0;index<result.covered.size();++index) {
        result.planned_count+=result.covered[index];
        result.joint_count+=initial[index]||result.covered[index];
    }
    return result;
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=3) {
        std::cerr<<"usage: GrandFinaleTask21RouteOracle MODE OUTPUT_DIR\n";
        return 2;
    }
    const auto contract=contractFor(argv[1]);
    if (!contract.valid) throw std::runtime_error(contract.reason);
    const std::filesystem::path output(argv[2]);
    std::filesystem::create_directories(output);
    const auto scenario=gf::task10p11rFixedBaselineScenario();
    const auto initial_snapshot=gf::task20FormalInitialCoverage(scenario,300,300);
    const auto initial=gf::task17_grid_detail::bitsFromHex(
        initial_snapshot.certified_bits_hex,90000);
    std::map<gf::NodeId,Eigen::Vector2d> mobile_positions;
    for (std::size_t index=0;index<scenario.mobile_ids.size();++index)
        mobile_positions[scenario.mobile_ids[index]]=scenario.mobile_positions[index];
    std::map<std::string,Eigen::Vector2d> initial_fronts;
    for (const auto& unit:contract.coverage_units) {
        Eigen::Vector2d front=Eigen::Vector2d::Zero();
        for (gf::NodeId member:unit.front_members)
            front+=mobile_positions.at(member);
        initial_fronts[unit.id]=front/static_cast<double>(unit.front_members.size());
    }
    const std::vector<double> candidates{
        800,700,650,600,550,500,450,400,350,300,250,220,200,190,
        180,160,140,120,100,80,60,40,20,10};
    nlohmann::json sweep=nlohmann::json::array();
    std::optional<double> selected;
    for (double spacing:candidates) {
        const auto result=scan(spacing,contract,scenario.fixed_positions,
            initial,initial_fronts,false);
        sweep.push_back({{"spacing_m",spacing},
            {"planned_route_service_count",result.planned_count},
            {"initial_union_route_count",result.joint_count}});
        if (!selected&&result.joint_count==90000) selected=spacing;
    }
    if (!selected) selected=10.0;
    const auto final=scan(*selected,contract,scenario.fixed_positions,
        initial,initial_fronts,true);
    std::ofstream witnesses(output/"route-witnesses.jsonl");
    std::ofstream segments(output/"route-segments.jsonl");
    for (const auto& record:final.segment_records)
        segments<<record.dump()<<'\n';
    std::ofstream transitions(output/"route-transitions.jsonl");
    for (const auto& record:final.transition_records)
        transitions<<record.dump()<<'\n';
    std::ofstream configurations(output/"route-configurations.jsonl");
    for (const auto& record:final.configuration_records)
        configurations<<record.dump()<<'\n';
    std::vector<std::string> holes;
    for (int x=0;x<300;++x) for (int y=0;y<300;++y) {
        const std::size_t index=static_cast<std::size_t>(x*300+y);
        if (!initial[index]&&!final.covered[index])
            holes.push_back(std::to_string(x)+":"+std::to_string(y));
        if (!initial[index]&&!final.witnesses[index].is_null())
            witnesses<<final.witnesses[index].dump()<<'\n';
    }
    nlohmann::json edges=nlohmann::json::array();
    for (const auto& edge:contract.reference_edges)
        edges.push_back({edge.reference,edge.owner});
    nlohmann::json units_json=nlohmann::json::array();
    for (const auto& unit:contract.coverage_units)
        units_json.push_back({{"id",unit.id},{"members",unit.members},
            {"front_members",unit.front_members},{"base_anchors",unit.base_anchors}});
    nlohmann::json fixed_json=nlohmann::json::object();
    for (const auto& [id,position]:scenario.fixed_positions)
        fixed_json[std::to_string(id)]=point(position);
    nlohmann::json summary{{"schema",1},{"mode",contract.id},
        {"reference_edges",edges},{"coverage_units",units_json},
        {"fixed_positions",fixed_json},
        {"initial_certified_count",initial_snapshot.certified_count},
        {"initial_certified_hash",initial_snapshot.certified_hash},
        {"spacing_sweep",sweep},{"maximum_hole_free_spacing_m",*selected},
        {"planned_route_service_count",final.planned_count},
        {"initial_union_route_count",final.joint_count},
        {"no_service_count",holes.size()},{"no_service_cells",holes},
        {"maximum_nominal_target_reference_m",final.maximum_reference},
        {"maximum_nominal_target_reference_witness",
            final.maximum_reference_witness},
        {"minimum_nominal_target_separation_m",final.minimum_separation},
        {"minimum_nominal_target_separation_witness",
            final.minimum_separation_witness},
        {"minimum_nominal_fim_proxy",final.minimum_fim},
        {"minimum_nominal_fim_witness",final.minimum_fim_witness},
        {"maximum_adjacent_member_target_change_m",
            final.maximum_adjacent_target_change},
        {"segment_bitset_file","route-segments.jsonl"},
        {"cell_witness_file","route-witnesses.jsonl"},
        {"transition_file","route-transitions.jsonl"},
        {"configuration_file","route-configurations.jsonl"},
        {"nominal_actual_boundary",
            "route oracle evaluates nominal target geometry only; >850 m is not an actual closed-loop reference violation"}};
    std::ofstream(output/"summary.json")<<summary.dump(2)<<'\n';
    std::cout<<summary.dump(2)<<'\n';
    return holes.empty()?0:3;
}
