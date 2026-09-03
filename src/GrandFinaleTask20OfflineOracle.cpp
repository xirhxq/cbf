#include "grand_finale/Task20GridOracle.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>

namespace {

gf::Task20LatticeMode parseMode(const std::string& value) {
    if (value=="dual-ladder") return gf::Task20LatticeMode::DualLadder;
    if (value=="merged-strip") return gf::Task20LatticeMode::MergedStrip;
    if (value=="split-three-front") return gf::Task20LatticeMode::SplitThreeFront;
    if (value=="cross-braced-diamond")
        return gf::Task20LatticeMode::CrossBracedDiamond;
    throw std::invalid_argument("unknown Task 20 lattice mode");
}

nlohmann::json point(const Eigen::Vector2d& value) {
    return nlohmann::json::array({value.x(),value.y()});
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=3&&argc!=6) {
        std::cerr<<"usage: GrandFinaleTask20OfflineOracle MODE OUTPUT_DIR "
            "[WIDTH_M HEIGHT_M SUMMARY_ONLY]\n";
        return 2;
    }
    const auto contract=gf::task20DagLatticeContract(parseMode(argv[1]));
    if (!contract.valid) throw std::runtime_error(contract.reason);
    const std::filesystem::path output(argv[2]);
    std::filesystem::create_directories(output);
    auto scenario=gf::task10p11pStandardCoastalScenario();
    const double width_m=argc==6?std::stod(argv[3]):3000.0;
    const double height_m=argc==6?std::stod(argv[4]):3000.0;
    const bool summary_only=argc==6&&std::stoi(argv[5])!=0;
    if (width_m<=0.0||height_m<=0.0||
        std::fmod(width_m,10.0)!=0.0||std::fmod(height_m,10.0)!=0.0)
        throw std::invalid_argument("Task 20 atlas dimensions must be positive 10 m multiples");
    const int x_cells=static_cast<int>(std::llround(width_m/10.0));
    const int y_cells=static_cast<int>(std::llround(height_m/10.0));
    scenario.id="task20_atlas_"+std::to_string(x_cells)+"x"+
        std::to_string(y_cells);
    scenario.width_m=width_m;
    scenario.height_m=height_m;
    scenario.fixed_positions={{100,{0.4*width_m,-50.0}},
        {101,{0.5*width_m,-50.0}},{102,{0.6*width_m,-50.0}}};
    const double center_x=0.5*width_m;
    scenario.mobile_positions={{center_x-120.0,10.0},
        {center_x-130.0,30.0},{center_x-140.0,50.0},
        {center_x-150.0,70.0},{center_x-160.0,90.0},
        {center_x-170.0,110.0},{center_x-180.0,130.0},
        {center_x+120.0,10.0},{center_x+130.0,30.0},
        {center_x+140.0,50.0},{center_x+150.0,70.0},
        {center_x+160.0,90.0},{center_x+170.0,110.0},
        {center_x+180.0,130.0}};
    const auto initial=gf::task20FormalInitialCoverage(
        scenario,x_cells,y_cells);
    const std::size_t cell_count=static_cast<std::size_t>(x_cells)*
        static_cast<std::size_t>(y_cells);
    const auto initial_bits=gf::task17_grid_detail::bitsFromHex(
        initial.certified_bits_hex,cell_count);
    std::ofstream witnesses;
    if (!summary_only) {
        witnesses.open(output/"witnesses.jsonl");
        if (!witnesses) throw std::runtime_error("cannot create witness evidence");
    }
    std::size_t serviceable_count=0;
    std::size_t nominal_reference_compatible_count=0;
    std::size_t safety_layer_required_count=0;
    std::vector<std::string> no_service;
    double global_max_reference=0.0;
    double global_min_separation=std::numeric_limits<double>::infinity();
    double global_min_fim=std::numeric_limits<double>::infinity();
    std::string max_reference_cell,min_separation_cell,min_fim_cell;
    for (int x=0;x<x_cells;++x) for (int y=0;y<y_cells;++y) {
        const std::size_t index=static_cast<std::size_t>(x*y_cells+y);
        const std::string id=std::to_string(x)+":"+std::to_string(y);
        nlohmann::json row{{"cell_id",id},{"initial_certified",initial_bits[index]}};
        if (!initial_bits[index]) {
            const Eigen::Vector2d center{5.0+10.0*x,5.0+10.0*y};
            const auto witness=gf::task20FindServiceWitness(
                contract,scenario.fixed_positions,center);
            row["serviceable"]=witness.serviceable;
            if (!witness.serviceable) {
                row["reason"]=witness.reason;
                no_service.push_back(id);
            } else {
                ++serviceable_count;
                if (witness.nominal_reference_compatible)
                    ++nominal_reference_compatible_count;
                else ++safety_layer_required_count;
                row["coverage_unit"]=witness.coverage_unit;
                row["responsible_member"]=witness.responsible_member;
                row["service_pose"]=point(witness.service_pose);
                row["service_yaw_rad"]=witness.service_yaw_rad;
                row["radial_certified_margin_m"]=
                    witness.radial_certified_margin_m;
                row["angular_certified_margin_rad"]=
                    witness.angular_certified_margin_rad;
                row["nominal_reference_compatible"]=
                    witness.nominal_reference_compatible;
                row["maximum_reference_edge_m"]=
                    witness.geometry.maximum_reference_edge_m;
                row["minimum_target_separation_m"]=
                    witness.geometry.minimum_target_separation_m;
                row["minimum_nominal_fim_proxy"]=
                    witness.geometry.minimum_nominal_fim_proxy;
                row["fronts"]=nlohmann::json::object();
                for (const auto& [unit,front]:witness.fronts)
                    row["fronts"][unit]=point(front);
                if (witness.geometry.maximum_reference_edge_m>
                    global_max_reference) {
                    global_max_reference=
                        witness.geometry.maximum_reference_edge_m;
                    max_reference_cell=id;
                }
                if (witness.geometry.minimum_target_separation_m<
                    global_min_separation) {
                    global_min_separation=
                        witness.geometry.minimum_target_separation_m;
                    min_separation_cell=id;
                }
                if (witness.geometry.minimum_nominal_fim_proxy<global_min_fim) {
                    global_min_fim=witness.geometry.minimum_nominal_fim_proxy;
                    min_fim_cell=id;
                }
            }
        } else row["serviceable"]=nullptr;
        if (!summary_only) witnesses<<row.dump()<<'\n';
    }
    nlohmann::json edges=nlohmann::json::array();
    for (const auto& edge:contract.reference_edges)
        edges.push_back({{"reference",edge.reference},{"owner",edge.owner},
                         {"id",edge.id()}});
    nlohmann::json roles=nlohmann::json::object();
    for (const auto& [member,role]:contract.member_roles)
        roles[std::to_string(member)]={{"coverage_unit",role.coverage_unit},
            {"axial_fraction",role.axial_fraction},
            {"triangular_fraction",role.triangular_fraction}};
    nlohmann::json summary{
        {"schema",1},{"mode",contract.id},
        {"structural_signature",contract.structural_signature},
        {"width_m",width_m},{"height_m",height_m},
        {"cell_size_m",10.0},{"entrance_edge","y=0"},
        {"anchor_rule","x={0.4,0.5,0.6}*width; y=-50 m"},
        {"launch_rule","Task18 center-relative x offsets and y=10..130 m"},
        {"summary_only",summary_only},
        {"cell_count",cell_count},{"initial_certified_count",initial.certified_count},
        {"initial_truth_count",initial.truth_count},
        {"initial_certified_hash",initial.certified_hash},
        {"initial_truth_hash",initial.truth_hash},
        {"task_domain_count",cell_count-initial.certified_count},
        {"serviceable_task_cells",serviceable_count},
        {"joint_gate_count",serviceable_count+initial.certified_count},
        {"nominal_reference_compatible_task_cells",
            nominal_reference_compatible_count},
        {"safety_layer_required_task_cells",safety_layer_required_count},
        {"no_service_count",no_service.size()},{"no_service_cells",no_service},
        {"maximum_witness_reference_edge_m",global_max_reference},
        {"maximum_witness_reference_edge_cell",max_reference_cell},
        {"minimum_witness_target_separation_m",global_min_separation},
        {"minimum_witness_target_separation_cell",min_separation_cell},
        {"minimum_witness_nominal_fim_proxy",global_min_fim},
        {"minimum_witness_nominal_fim_cell",min_fim_cell},
        {"reference_edges",edges},{"member_roles",roles},
        {"witness_reconstruction",
            "fronts + member_roles + fixed anchors deterministically reconstruct all 14 targets and all edge lengths"},
        {"evidence_boundary",
            "nominal FIM is a static proxy; runtime robust-FIM, posterior and AoI remain separate hard gates"}};
    std::ofstream(output/"summary.json")<<summary.dump(2)<<'\n';
    std::cout<<summary.dump(2)<<'\n';
    return no_service.empty()?0:3;
}
