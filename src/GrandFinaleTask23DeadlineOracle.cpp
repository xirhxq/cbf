#include "grand_finale/Task23PersistentDeadlineRibbon.hpp"
#include "grand_finale/Task20GridOracle.hpp"
#include "grand_finale/Task10p11rFixedBaseline.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>

namespace {

using json=nlohmann::json;

gf::Task20DagLatticeContract contractFor(const std::string& mode) {
    if (mode=="dual-ladder")
        return gf::task20DagLatticeContract(gf::Task20LatticeMode::DualLadder);
    if (mode=="merged-strip")
        return gf::task20DagLatticeContract(gf::Task20LatticeMode::MergedStrip);
    if (mode=="split-three-front")
        return gf::task20DagLatticeContract(
            gf::Task20LatticeMode::SplitThreeFront);
    if (mode=="pinball-5-4-3-2") return gf::task23Pinball5432Contract();
    throw std::invalid_argument("unknown Task23 contract:"+mode);
}

json point(const Eigen::Vector2d& value) {
    return json::array({value.x(),value.y()});
}

}  // namespace

int main(int argc,char** argv) {
    if (argc<4||argc>6) {
        std::cerr<<"usage: GrandFinaleTask23DeadlineOracle MODE OUTPUT_DIR "
            "SPACING_M [GRID_X GRID_Y]\n";
        return 2;
    }
    const std::string mode=argv[1];
    const std::filesystem::path output=argv[2];
    const double spacing=std::stod(argv[3]);
    int grid_x=300,grid_y=300;
    if (argc==6) {
        grid_x=std::stoi(argv[4]);
        grid_y=std::stoi(argv[5]);
    }
    std::filesystem::create_directories(output);
    const auto contract=contractFor(mode);
    const auto contract_audit=gf::task23AuditCoverageContract(contract);
    if (!contract_audit.valid) throw std::runtime_error(contract_audit.reason);
    const auto scenario=gf::task10p11rFixedBaselineScenario();
    std::vector<gf::FrontierCell> cells;
    for (int x=0;x<grid_x;++x) for (int y=0;y<grid_y;++y)
        cells.push_back({x,y,{5.0+10.0*x,5.0+10.0*y}});
    std::set<std::string> initially_certified;
    std::string initial_bits_hex;
    std::string initial_hash;
    if (grid_x==300&&grid_y==300) {
        const auto initial=gf::task20FormalInitialCoverage(scenario,300,300);
        const auto bits=gf::task17_grid_detail::bitsFromHex(
            initial.certified_bits_hex,90000);
        for (std::size_t index=0;index<bits.size();++index)
            if (bits[index]) initially_certified.insert(cells[index].id());
        initial_bits_hex=initial.certified_bits_hex;
        initial_hash=initial.certified_hash;
    }
    std::map<gf::NodeId,Eigen::Vector2d> mobile;
    for (std::size_t index=0;index<scenario.mobile_ids.size();++index)
        mobile[scenario.mobile_ids[index]]=scenario.mobile_positions[index];
    std::map<std::string,Eigen::Vector2d> initial_fronts;
    for (const auto& unit:contract.coverage_units) {
        Eigen::Vector2d value=Eigen::Vector2d::Zero();
        const auto& members=unit.front_members.empty()
            ?std::vector<gf::NodeId>{unit.leader}:unit.front_members;
        for (gf::NodeId member:members) value+=mobile.at(member);
        initial_fronts[unit.id]=value/static_cast<double>(members.size());
    }
    const auto field=gf::task21AffineCoordinateField(
        {0.0,0.0},{0.0,1.0},{1.0,0.0});
    const auto plan=gf::task23BuildDeadlinePlan(cells,field,spacing,1024,
        contract,scenario.fixed_positions,initial_fronts,initially_certified);

    std::ofstream witness_file(output/"witnesses.jsonl");
    std::ofstream queue_file(output/"deadline-queues.jsonl");
    double maximum_reference=0.0;
    double minimum_separation=std::numeric_limits<double>::infinity();
    double minimum_fim=std::numeric_limits<double>::infinity();
    double minimum_sensing_margin=std::numeric_limits<double>::infinity();
    for (const auto& [cell_id,witness]:plan.witnesses) {
        maximum_reference=std::max(maximum_reference,
            witness.maximum_target_reference_m);
        minimum_separation=std::min(minimum_separation,
            witness.minimum_target_separation_m);
        minimum_fim=std::min(minimum_fim,witness.nominal_fim_proxy);
        minimum_sensing_margin=std::min(minimum_sensing_margin,
            witness.sensing_margin_m);
        const auto cell=plan.route_geometry.cells.at(
            plan.route_geometry.cell_lookup.at(cell_id));
        witness_file<<json({
            {"cell_id",cell_id},{"cell_center",point(cell.center)},
            {"coverage_unit",witness.coverage_unit},
            {"service_member",witness.service_member},
            {"pass_index",witness.pass_index},
            {"first_service_s",witness.first_service_s},
            {"last_service_s",witness.last_service_s},
            {"canonical_s",witness.canonical_s},
            {"canonical_front",point(witness.canonical_front)},
            {"route_tangent",point(witness.route_tangent)},
            {"sensing_margin_m",witness.sensing_margin_m},
            {"yaw_margin_rad",witness.yaw_margin_rad},
            {"maximum_target_reference_m",
                witness.maximum_target_reference_m},
            {"minimum_target_separation_m",
                witness.minimum_target_separation_m},
            {"nominal_fim_proxy",witness.nominal_fim_proxy},
            {"tier",witness.tier==gf::Task23WitnessTier::NominalCompatible
                ?"nominal-compatible":"virtual-dependent"}}).dump()<<'\n';
    }
    for (const auto& [unit,queue]:plan.queues)
        for (std::size_t index=0;index<queue.size();++index)
            queue_file<<json({{"coverage_unit",unit},{"queue_index",index},
                {"cell_id",queue[index].cell_id},
                {"canonical_s",queue[index].canonical_s}}).dump()<<'\n';
    json edges=json::array();
    for (const auto& edge:contract.reference_edges)
        edges.push_back({edge.reference,edge.owner});
    json units=json::array();
    for (const auto& unit:contract.coverage_units)
        units.push_back({{"id",unit.id},{"members",unit.members},
            {"anchors",unit.base_anchors},{"leader",unit.leader},
            {"front_members",unit.front_members}});
    json roles=json::object();
    for (const auto& [member,role]:contract.member_roles)
        roles[std::to_string(member)]={{"unit",role.coverage_unit},
            {"axial",role.axial_fraction},
            {"triangular",role.triangular_fraction}};
    json fixed=json::object();
    for (const auto& [id,pose]:scenario.fixed_positions)
        fixed[std::to_string(id)]=point(pose);
    json front_json=json::object();
    for (const auto& [id,pose]:initial_fronts) front_json[id]=point(pose);
    json queue_counts=json::object();
    for (const auto& [unit,queue]:plan.queues) queue_counts[unit]=queue.size();
    json routes=json::object();
    for (const auto& [unit,route]:plan.route_geometry.routes) {
        json passes=json::array();
        for (const auto& pass:route.passes)
            passes.push_back({{"pass_index",pass.pass_index},
                {"direction",pass.direction},{"progress",pass.progress},
                {"cross_begin",pass.cross_begin},{"cross_end",pass.cross_end},
                {"inset_low",pass.inset_low},{"inset_high",pass.inset_high}});
        routes[unit]={{"total_length_m",route.total_length},
            {"sample_count",route.samples.size()},{"passes",passes}};
    }
    const json summary={
        {"protocol","task23-deadline-oracle-v1"},
        {"mode",mode},{"contract_id",contract.id},
        {"valid",plan.valid},{"reason",plan.reason},
        {"grid",{{"x",grid_x},{"y",grid_y},{"cells",cells.size()},
            {"cell_size_m",10.0}}},
        {"pass_spacing_m",spacing},
        {"deadline_rule",plan.deadline_rule},
        {"route_validity_lateral_m",plan.route_validity_lateral_m},
        {"route_validity_yaw_rad",plan.route_validity_yaw_rad},
        {"initial_certified_count",initially_certified.size()},
        {"initial_certified_bits_hex",initial_bits_hex},
        {"initial_certified_hash",initial_hash},
        {"witness_count",plan.witnesses.size()},
        {"joint_count",plan.witnesses.size()+initially_certified.size()},
        {"no_witness_count",plan.no_witness_cells.size()},
        {"no_witness_cells",plan.no_witness_cells},
        {"nominal_compatible_count",plan.nominal_compatible_count},
        {"virtual_dependent_count",plan.virtual_dependent_count},
        {"queue_counts",queue_counts},
        {"routes",routes},
        {"maximum_target_reference_m",maximum_reference},
        {"minimum_target_separation_m",minimum_separation},
        {"minimum_nominal_fim_proxy",minimum_fim},
        {"minimum_sensing_margin_m",minimum_sensing_margin},
        {"reference_edges",edges},{"coverage_units",units},
        {"member_roles",roles},{"fixed_positions",fixed},
        {"initial_fronts",front_json}};
    std::ofstream(output/"summary.json")<<summary.dump(2)<<'\n';
    std::cout<<summary.dump(2)<<'\n';
    return plan.valid?0:1;
}
