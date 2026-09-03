#include "grand_finale/Task22FootprintInsetSweep.hpp"
#include "grand_finale/Task20GridOracle.hpp"
#include "grand_finale/Task10p11rFixedBaseline.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

namespace {

gf::Task20DagLatticeContract contractFor(const std::string& mode) {
    if (mode=="dual-ladder")
        return gf::task20DagLatticeContract(gf::Task20LatticeMode::DualLadder);
    if (mode=="whole-strip")
        return gf::task20DagLatticeContract(gf::Task20LatticeMode::MergedStrip);
    if (mode=="split-three-front")
        return gf::task20DagLatticeContract(
            gf::Task20LatticeMode::SplitThreeFront);
    if (mode=="pinball") return gf::task21PinballContract();
    if (mode=="pinball-5-4-3-2") return gf::task22Pinball5432Contract();
    if (mode=="tiling-fan") return gf::task22SwathTilingContract("fan");
    if (mode=="tiling-trailing-fan")
        return gf::task22SwathTilingContract("trailing-fan");
    if (mode=="lanes-14") return gf::task22Lanes14Contract();
    if (mode=="lanes-7") return gf::task22Lanes7Contract();
    throw std::invalid_argument("unknown Task 22 DAG mode:"+mode);
}

// P10 standing rule (b): deflated-sector reachability -- cone 60-6 deg,
// range [40, 352] m (delta 6 deg / 40 m from the diagnostics).  A cell
// counts reachable only with a deflated witness; the no-hole union alone
// is not sufficient.
bool deflatedInSector(const Eigen::Vector2d& pose,double yaw,
    const Eigen::Vector2d& center) {
    constexpr double min_range=40.0;
    constexpr double max_range=352.0;
    constexpr double sin54=0.80901699437494742410;
    const Eigen::Vector2d delta=center-pose;
    const double d2=delta.squaredNorm();
    if (d2<min_range*min_range||d2>max_range*max_range) return false;
    const double d=std::sqrt(d2);
    const double lateral=std::abs(-std::sin(yaw)*delta.x()+
        std::cos(yaw)*delta.y());
    return lateral<=sin54*d;
}

nlohmann::json point(const Eigen::Vector2d& value) {
    return nlohmann::json::array({value.x(),value.y()});
}

struct ScanResult {
    double spacing=0.0;
    std::size_t grid_cells=0;
    std::size_t initial_count=0;
    std::size_t planned_count=0;
    std::size_t joint_count=0;
    std::size_t no_service_count=0;
    double maximum_reference=0.0;
    double minimum_separation=std::numeric_limits<double>::infinity();
    double minimum_fim=std::numeric_limits<double>::infinity();
    double maximum_tangent_step_rad=0.0;
    double total_route_length_m=0.0;
    double fillet_length_m=0.0;
    double transition_budget_s=0.0;
    double overlap_rate=0.0;
    double mean_predicted_cells_per_100m=0.0;
    std::vector<nlohmann::json> witnesses;
    std::vector<nlohmann::json> sample_records;
    std::vector<nlohmann::json> pass_records;
    std::vector<nlohmann::json> bucket_records;
    nlohmann::json maximum_reference_witness;
    nlohmann::json minimum_separation_witness;
    nlohmann::json minimum_fim_witness;
    nlohmann::json plan_reason;
};

ScanResult scan(double spacing,const gf::Task20DagLatticeContract& contract,
    const std::map<gf::NodeId,Eigen::Vector2d>& fixed,
    const std::vector<gf::FrontierCell>& cells,
    const std::vector<bool>& initial,
    const std::map<std::string,Eigen::Vector2d>& initial_fronts,
    bool save_witnesses) {
    ScanResult result;
    result.spacing=spacing;
    result.grid_cells=cells.size();
    for (std::size_t index=0;index<initial.size();++index)
        result.initial_count+=initial[index];
    std::vector<std::string> units;
    for (const auto& unit:contract.coverage_units) units.push_back(unit.id);
    const auto field=gf::task21AffineCoordinateField(
        {0.0,0.0},{0.0,1.0},{1.0,0.0});
    const auto plan=gf::task22BuildSweepPlan(cells,units,field,spacing,1024,
        contract,fixed,initial_fronts);
    result.plan_reason=plan.valid?nlohmann::json("valid")
        :nlohmann::json(plan.reason);
    if (!plan.valid) return result;
    const std::size_t cell_count=cells.size();
    std::vector<bool> covered(cell_count,false);
    if (save_witnesses) result.witnesses.resize(cell_count);
    std::map<std::string,const gf::Task20CoverageUnit*> unit_by_id;
    for (const auto& unit:contract.coverage_units)
        unit_by_id[unit.id]=&unit;

    for (const auto& [unit_id,route]:plan.routes) {
        const auto& unit=*unit_by_id.at(unit_id);
        result.total_route_length_m+=route.total_length;
        double previous_reference=0.0;
        for (std::size_t index=0;index<route.samples.size();++index) {
            const auto& sample=route.samples[index];
            if (sample.on_fillet) result.fillet_length_m+=0.0;
            std::map<std::string,Eigen::Vector2d> fronts;
            for (const auto& other:contract.coverage_units)
                fronts[other.id]=other.id==unit_id?sample.position:
                    plan.routes.at(other.id).samples.front().position;
            const auto lifted=gf::task20LiftTargets(contract,fixed,fronts);
            if (!lifted.valid) throw std::runtime_error(lifted.reason);
            const auto geometry=gf::task20EvaluateOracleGeometry(
                contract,fixed,lifted.targets);
            if (geometry.maximum_reference_edge_m>result.maximum_reference) {
                result.maximum_reference=geometry.maximum_reference_edge_m;
                result.maximum_reference_witness={
                    {"coverage_unit",unit_id},{"s",sample.s},
                    {"position",point(sample.position)}};
            }
            if (geometry.minimum_target_separation_m<
                result.minimum_separation) {
                result.minimum_separation=
                    geometry.minimum_target_separation_m;
                result.minimum_separation_witness={
                    {"coverage_unit",unit_id},{"s",sample.s},
                    {"position",point(sample.position)}};
            }
            if (geometry.minimum_nominal_fim_proxy<result.minimum_fim) {
                result.minimum_fim=geometry.minimum_nominal_fim_proxy;
                result.minimum_fim_witness={
                    {"coverage_unit",unit_id},{"s",sample.s},
                    {"position",point(sample.position)}};
            }
            const double yaw=std::atan2(sample.tangent.y(),
                sample.tangent.x());
            double min_x=1.0e18,max_x=-1.0e18,min_y=1.0e18,max_y=-1.0e18;
            for (gf::NodeId member:unit.members) {
                const Eigen::Vector2d pose=lifted.targets.at(member);
                min_x=std::min(min_x,pose.x());
                max_x=std::max(max_x,pose.x());
                min_y=std::min(min_y,pose.y());
                max_y=std::max(max_y,pose.y());
            }
            std::size_t sample_serving=0;
            for (std::size_t cell_index=0;cell_index<cell_count;
                 ++cell_index) {
                const auto& cell=cells[cell_index];
                if (cell.center.x()<min_x-400.0||
                    cell.center.x()>max_x+400.0||
                    cell.center.y()<min_y-400.0||
                    cell.center.y()>max_y+400.0) continue;
                for (gf::NodeId member:unit.members) {
                    if (!gf::task22CellInCertifiedSector(
                        lifted.targets.at(member),yaw,cell.center))
                        continue;
                    ++sample_serving;
                    if (!covered[cell_index]) {
                        covered[cell_index]=true;
                        if (save_witnesses&&!initial[cell_index])
                            result.witnesses[cell_index]={
                                {"cell_id",cell.id()},
                                {"coverage_unit",unit_id},
                                {"responsible_member",member},
                                {"route_s_m",sample.s},
                                {"on_fillet",sample.on_fillet},
                                {"service_pose",
                                    point(lifted.targets.at(member))},
                                {"service_yaw_rad",yaw},
                                {"front",point(sample.position)},
                                {"tangent",point(sample.tangent)}};
                    }
                    break;
                }
            }
            if (save_witnesses)
                result.sample_records.push_back({
                    {"coverage_unit",unit_id},{"s",sample.s},
                    {"position",point(sample.position)},
                    {"tangent",point(sample.tangent)},
                    {"on_fillet",sample.on_fillet},
                    {"serving_cells",sample_serving},
                    {"maximum_nominal_reference_m",
                        geometry.maximum_reference_edge_m},
                    {"minimum_nominal_separation_m",
                        geometry.minimum_target_separation_m},
                    {"minimum_nominal_fim_proxy",
                        geometry.minimum_nominal_fim_proxy}});
        }
        // Continuity statistics over consecutive samples.
        for (std::size_t index=0;index+1<route.samples.size();++index) {
            const double dot=std::max(-1.0,std::min(1.0,
                route.samples[index].tangent.dot(
                    route.samples[index+1].tangent)));
            result.maximum_tangent_step_rad=std::max(
                result.maximum_tangent_step_rad,std::acos(dot));
        }
        if (save_witnesses)
            for (const auto& pass:route.passes)
                result.pass_records.push_back({
                    {"coverage_unit",unit_id},
                    {"pass_index",pass.pass_index},
                    {"direction",pass.direction},
                    {"progress_m",pass.progress},
                    {"cross_begin_m",pass.cross_begin},
                    {"cross_end_m",pass.cross_end},
                    {"inset_low_m",pass.inset_low},
                    {"inset_high_m",pass.inset_high}});
        // Fillet arclength and efficiency buckets.
        double previous_s=0.0;
        double fillet_length=0.0;
        for (const auto& sample:route.samples) {
            if (sample.on_fillet) fillet_length+=sample.s-previous_s;
            previous_s=sample.s;
        }
        result.fillet_length_m+=fillet_length;
        (void)result.mean_predicted_cells_per_100m;
        // Route-order first-service buckets (predicted cells per 100 m).
        if (save_witnesses) {
            std::map<std::size_t,std::size_t> buckets;
            for (const auto& service:route.cell_order)
                ++buckets[static_cast<std::size_t>(
                    std::floor(service.first_service_s/100.0))];
            for (const auto& [bucket,count]:buckets)
                result.bucket_records.push_back({
                    {"coverage_unit",unit_id},
                    {"arclength_bucket_m",bucket*100},
                    {"predicted_new_cells",count}});
        }
    }
    result.transition_budget_s=result.fillet_length_m/30.0;
    std::size_t joint=0,planned=0;
    for (std::size_t index=0;index<cell_count;++index) {
        planned+=covered[index];
        joint+=initial[index]||covered[index];
    }
    result.planned_count=planned;
    result.joint_count=joint;
    result.no_service_count=cell_count-joint;
    // Overlap rate: unique served cells vs cumulative per-sample footprint
    // cell counts (equal sample weighting; recorded with the definition).
    std::size_t cumulative_serving=0;
    for (const auto& record:result.sample_records)
        cumulative_serving+=record["serving_cells"].get<std::size_t>();
    if (cumulative_serving>0)
        result.overlap_rate=1.0-static_cast<double>(planned)/
            static_cast<double>(cumulative_serving);
    std::size_t bucket_total=0;
    for (const auto& record:result.bucket_records)
        bucket_total+=record["predicted_new_cells"].get<std::size_t>();
    if (result.total_route_length_m>0.0)
        result.mean_predicted_cells_per_100m=100.0*
            static_cast<double>(bucket_total)/result.total_route_length_m;
    return result;
}

}  // namespace

struct DeflatedResult {
    std::size_t missing_count=0;
    std::vector<std::string> missing_sample;
    std::map<std::string,double> effective_path_m;
    // Global timing simulation: every unit advances its arclength at the
    // reference cruise speed; a cell certifies at the earliest cross-unit
    // deflated witness time.  This is the path pre-screen's T100 bound
    // (per-lane max-path overstates it by ignoring cross-lane
    // certification -- the P8 smoke vs first pre-screen discrepancy).
    double predicted_t100_s=0.0;
};

DeflatedResult deflatedScan(const gf::Task20DagLatticeContract& contract,
    const std::map<gf::NodeId,Eigen::Vector2d>& fixed,
    const std::vector<gf::FrontierCell>& cells,
    const std::vector<bool>& initial,
    const gf::Task22SweepPlan& plan,double cruise_mps=17.5) {
    DeflatedResult result;
    std::vector<bool> covered(cells.size(),false);
    std::vector<double> certify_time_s(cells.size(),
        std::numeric_limits<double>::infinity());
    std::map<std::string,const gf::Task20CoverageUnit*> unit_by_id;
    for (const auto& unit:contract.coverage_units)
        unit_by_id[unit.id]=&unit;
    for (const auto& [unit_id,route]:plan.routes) {
        const auto& unit=*unit_by_id.at(unit_id);
        double last_growth_s=0.0;
        for (const auto& sample:route.samples) {
            std::map<std::string,Eigen::Vector2d> fronts;
            for (const auto& other:contract.coverage_units)
                fronts[other.id]=other.id==unit_id?sample.position:
                    plan.routes.at(other.id).samples.front().position;
            const auto lifted=gf::task20LiftTargets(contract,fixed,fronts);
            if (!lifted.valid) continue;
            const double yaw=std::atan2(sample.tangent.y(),
                sample.tangent.x());
            bool grew=false;
            const double sample_time_s=sample.s/cruise_mps;
            for (std::size_t index=0;index<cells.size();++index) {
                if (initial[index]) continue;
                if (covered[index]&&
                    certify_time_s[index]<=sample_time_s) continue;
                for (gf::NodeId member:unit.members)
                    if (deflatedInSector(lifted.targets.at(member),yaw,
                        cells[index].center)) {
                        covered[index]=true;
                        certify_time_s[index]=std::min(
                            certify_time_s[index],sample_time_s);
                        grew=true;
                        break;
                    }
            }
            if (grew) last_growth_s=sample.s;
        }
        result.effective_path_m[unit_id]=last_growth_s;
    }
    for (std::size_t index=0;index<cells.size();++index)
        if (!initial[index]&&!covered[index]) {
            ++result.missing_count;
            if (result.missing_sample.size()<10)
                result.missing_sample.push_back(cells[index].id());
        }
        else if (!initial[index])
            result.predicted_t100_s=std::max(result.predicted_t100_s,
                certify_time_s[index]);
    return result;
}

int main(int argc,char** argv) {
    if (argc<3||argc>5) {
        std::cerr<<"usage: GrandFinaleTask22RouteOracle MODE OUTPUT_DIR "
                    "[GRID_X GRID_Y]\n";
        return 2;
    }
    const auto contract=contractFor(argv[1]);
    if (!contract.valid) throw std::runtime_error(contract.reason);
    int grid_x=300,grid_y=300;
    if (argc==5) {
        grid_x=std::stoi(argv[3]);
        grid_y=std::stoi(argv[4]);
    }
    const std::filesystem::path output(argv[2]);
    std::filesystem::create_directories(output);
    const auto scenario=gf::task10p11rFixedBaselineScenario();
    std::vector<gf::FrontierCell> cells;
    for (int x=0;x<grid_x;++x) for (int y=0;y<grid_y;++y)
        cells.push_back({x,y,{5.0+10.0*x,5.0+10.0*y}});
    std::vector<bool> initial(cells.size(),false);
    bool formal_initial=grid_x==300&&grid_y==300;
    std::string initial_bits_hex;
    std::size_t initial_certified_count=0;
    if (formal_initial) {
        const auto snapshot=gf::task20FormalInitialCoverage(
            scenario,300,300);
        const auto bits=gf::task17_grid_detail::bitsFromHex(
            snapshot.certified_bits_hex,90000);
        initial=bits;
        initial_bits_hex=snapshot.certified_bits_hex;
        initial_certified_count=snapshot.certified_count;
    }
    std::map<gf::NodeId,Eigen::Vector2d> mobile_positions;
    for (std::size_t index=0;index<scenario.mobile_ids.size();++index)
        mobile_positions[scenario.mobile_ids[index]]=
            scenario.mobile_positions[index];
    std::map<std::string,Eigen::Vector2d> initial_fronts;
    for (const auto& unit:contract.coverage_units) {
        Eigen::Vector2d front=Eigen::Vector2d::Zero();
        for (gf::NodeId member:unit.front_members)
            front+=mobile_positions.at(member);
        initial_fronts[unit.id]=
            front/static_cast<double>(unit.front_members.size());
    }
    const std::vector<double> candidates{
        2900,2400,2000,1600,1400,1200,1000,900,800,700,650,600,550,
        500,450,400,350,300,250,220,200,190,
        180,160,140,120,100,80,60,40,20,10};
    nlohmann::json sweep=nlohmann::json::array();
    std::optional<double> selected;
    for (double spacing:candidates) {
        const auto result=scan(spacing,contract,scenario.fixed_positions,
            cells,initial,initial_fronts,false);
        sweep.push_back({{"spacing_m",spacing},
            {"plan_valid",result.plan_reason=="valid"},
            {"plan_reason",result.plan_reason},
            {"planned_route_service_count",result.planned_count},
            {"initial_union_route_count",result.joint_count}});
        if (result.plan_reason=="valid"&&!selected&&
            result.joint_count==cells.size()) {
            selected=spacing;
            break;  // descending scan: the first hole-free spacing is the
                    // maximum; smaller spacings are not scanned.
        }
    }
    if (!selected) selected=10.0;
    const auto final=scan(*selected,contract,scenario.fixed_positions,
        cells,initial,initial_fronts,true);
    // Standing rules (a)+(b): rebuild the selected plan for the deflated
    // reachability check and the per-unit effective-path pre-screen.
    std::vector<std::string> unit_ids;
    for (const auto& unit:contract.coverage_units) unit_ids.push_back(unit.id);
    const auto field=gf::task21AffineCoordinateField({0.0,0.0},{0.0,1.0},
        {1.0,0.0});
    const auto plan=gf::task22BuildSweepPlan(cells,unit_ids,field,*selected,
        1024,contract,scenario.fixed_positions,initial_fronts);
    DeflatedResult deflated;
    if (plan.valid)
        deflated=deflatedScan(contract,scenario.fixed_positions,cells,
            initial,plan);
    double max_effective_path_m=0.0;
    for (const auto& [unit,path]:deflated.effective_path_m)
        max_effective_path_m=std::max(max_effective_path_m,path);
    const double predicted_t100_s=deflated.predicted_t100_s;
    std::ofstream witnesses(output/"route-witnesses.jsonl");
    std::ofstream samples(output/"route-samples.jsonl");
    for (const auto& record:final.sample_records)
        samples<<record.dump()<<'\n';
    std::ofstream passes(output/"route-passes.jsonl");
    for (const auto& record:final.pass_records)
        passes<<record.dump()<<'\n';
    std::ofstream buckets(output/"route-buckets.jsonl");
    for (const auto& record:final.bucket_records)
        buckets<<record.dump()<<'\n';
    std::vector<std::string> holes;
    for (std::size_t index=0;index<cells.size();++index) {
        if (!initial[index]&&!final.witnesses[index].is_null())
            witnesses<<final.witnesses[index].dump()<<'\n';
        if (!initial[index]&&
            !final.witnesses[index].is_null()) continue;
        if (!initial[index]) holes.push_back(cells[index].id());
    }
    nlohmann::json edges=nlohmann::json::array();
    for (const auto& edge:contract.reference_edges)
        edges.push_back({edge.reference,edge.owner});
    nlohmann::json units_json=nlohmann::json::array();
    for (const auto& unit:contract.coverage_units)
        units_json.push_back({{"id",unit.id},{"members",unit.members},
            {"base_anchors",unit.base_anchors},{"leader",unit.leader},
            {"front_members",unit.front_members}});
    nlohmann::json roles_json=nlohmann::json::object();
    for (const auto& [member,role]:contract.member_roles)
        roles_json[std::to_string(member)]={{"unit",role.coverage_unit},
            {"axial",role.axial_fraction},{"triangular",
            role.triangular_fraction}};
    nlohmann::json fixed_json=nlohmann::json::object();
    for (const auto& [id,position]:scenario.fixed_positions)
        fixed_json[std::to_string(id)]=point(position);
    nlohmann::json summary{{"schema",1},{"mode",contract.id},
        {"coverage_units",units_json},{"member_roles",roles_json},
        {"fixed_positions",fixed_json},
        {"grid",{grid_x,grid_y}},{"grid_cells",cells.size()},
        {"initial_semantics",formal_initial
            ?"task20FormalInitialCoverage_300x300"
            :"empty_initial_static_lower_bound_venue_gate"},
        {"initial_certified_count",initial_certified_count},
        {"initial_certified_bits_hex",initial_bits_hex},
        {"reference_edges",edges},
        {"spacing_sweep",sweep},{"maximum_hole_free_spacing_m",*selected},
        {"planned_route_service_count",final.planned_count},
        {"initial_union_route_count",final.joint_count},
        {"no_service_count",final.no_service_count},
        {"no_service_cells",holes},
        {"total_route_length_m",final.total_route_length_m},
        {"fillet_length_m",final.fillet_length_m},
        {"transition_budget_s",final.transition_budget_s},
        {"overlap_rate_definition",
            "one_minus_unique_served_over_cumulative_per_sample_serving"},
        {"overlap_rate",final.overlap_rate},
        {"mean_predicted_cells_per_100m",
            final.mean_predicted_cells_per_100m},
        {"maximum_consecutive_tangent_step_rad",
            final.maximum_tangent_step_rad},
        {"maximum_nominal_target_reference_m",final.maximum_reference},
        {"maximum_nominal_target_reference_witness",
            final.maximum_reference_witness},
        {"minimum_nominal_target_separation_m",final.minimum_separation},
        {"minimum_nominal_target_separation_witness",
            final.minimum_separation_witness},
        {"minimum_nominal_fim_proxy",final.minimum_fim},
        {"minimum_nominal_fim_witness",final.minimum_fim_witness},
        {"witness_file","route-witnesses.jsonl"},
        {"sample_file","route-samples.jsonl"},
        {"pass_file","route-passes.jsonl"},
        {"bucket_file","route-buckets.jsonl"},
        {"deflated_reachability_missing_count",deflated.missing_count},
        {"deflated_reachability_missing_sample",deflated.missing_sample},
        {"deflated_effective_path_m",deflated.effective_path_m},
        {"path_prescreen_max_effective_path_m",max_effective_path_m},
        {"path_prescreen_predicted_t100_s_parallel",predicted_t100_s},
        {"path_prescreen_max_effective_path_m_per_lane",max_effective_path_m},
        {"path_prescreen_pass",predicted_t100_s<=296.7},
        {"nominal_actual_boundary",
            "route oracle evaluates nominal target geometry only; >850 m "
            "is not an actual closed-loop reference violation"}};
    std::ofstream(output/"summary.json")<<summary.dump(2)<<'\n';
    std::cout<<summary.dump(2)<<'\n';
    if (final.no_service_count!=0) return 3;
    return deflated.missing_count==0?0:5;
}
