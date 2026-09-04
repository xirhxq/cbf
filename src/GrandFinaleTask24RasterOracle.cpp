#include "grand_finale/Task24PersistentRasterSweep.hpp"
#include "grand_finale/Task20GridOracle.hpp"
#include "grand_finale/Task22FootprintInsetSweep.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>

namespace {

using json=nlohmann::json;

gf::Task24LatticeMode parseMode(const std::string& value) {
    if (value=="h0") return gf::Task24LatticeMode::H0DualLadder;
    if (value=="pinball") return gf::Task24LatticeMode::Pinball5432;
    if (value=="triangle")
        return gf::Task24LatticeMode::LongTriangleSingleLadder;
    throw std::invalid_argument("unknown Task24 mode:"+value);
}

json point(const Eigen::Vector2d& value) {
    return json::array({value.x(),value.y()});
}

gf::Task20OracleGeometry geometry(
    const gf::Task24DagContract& contract,
    const std::map<gf::NodeId,Eigen::Vector2d>& targets) {
    gf::Task20OracleGeometry result;
    std::vector<std::pair<gf::NodeId,Eigen::Vector2d>> all;
    for (const auto& item:contract.fixed_positions) all.push_back(item);
    for (const auto& item:targets) all.push_back(item);
    for (std::size_t i=0;i<all.size();++i)
        for (std::size_t j=i+1;j<all.size();++j)
            result.minimum_target_separation_m=std::min(
                result.minimum_target_separation_m,
                (all[i].second-all[j].second).norm());
    std::map<gf::NodeId,std::vector<Eigen::Vector2d>> bearings;
    for (const auto& edge:contract.reference_edges) {
        const auto mobile=targets.find(edge.reference);
        const Eigen::Vector2d reference=mobile==targets.end()
            ?contract.fixed_positions.at(edge.reference):mobile->second;
        const Eigen::Vector2d delta=reference-targets.at(edge.owner);
        result.maximum_reference_edge_m=std::max(
            result.maximum_reference_edge_m,delta.norm());
        if (delta.norm()>1e-12)
            bearings[edge.owner].push_back(delta.normalized());
    }
    for (gf::NodeId owner=1;owner<=14;++owner) {
        Eigen::Matrix2d fim=Eigen::Matrix2d::Zero();
        for (const auto& bearing:bearings[owner])
            fim+=bearing*bearing.transpose();
        result.minimum_nominal_fim_proxy=std::min(
            result.minimum_nominal_fim_proxy,
            fim.eigenvalues().real().minCoeff());
    }
    return result;
}

std::vector<gf::FrontierCell> cellsFor(
    const gf::Task24DagContract& contract) {
    std::vector<gf::FrontierCell> cells;
    const int nx=static_cast<int>(std::llround(contract.width_m/10.0));
    const int ny=static_cast<int>(std::llround(contract.height_m/10.0));
    cells.reserve(static_cast<std::size_t>(nx*ny));
    for (int x=0;x<nx;++x) for (int y=0;y<ny;++y)
        cells.push_back({x,y,{5.0+10.0*x,5.0+10.0*y}});
    return cells;
}

struct InitialState {
    std::vector<Eigen::Vector2d> mobiles;
    std::vector<bool> certified;
    gf::Task20OracleGeometry geometry;
};

InitialState initialState(const gf::Task24DagContract& contract,
    const gf::Task24RasterPlan& plan) {
    InitialState result;
    double yaw=0.0;
    if (contract.mode==gf::Task24LatticeMode::H0DualLadder) {
        result.mobiles=gf::task10p11pStandardLaunchPositions();
        yaw=M_PI/2.0;
    } else {
        const auto& corridor=plan.corridors.front();
        const double cross=0.5*(corridor.cross_min+corridor.cross_max);
        const Eigen::Vector2d g=plan.field.origin+
            plan.pass_progress_m.front()*plan.field.progress_axis+
            cross*plan.field.cross_axis;
        const auto lifted=gf::task24LiftTargets(contract,
            {{contract.coverage_units.front().id,g}});
        if (!lifted.valid) throw std::runtime_error(lifted.reason);
        for (gf::NodeId id=1;id<=14;++id)
            result.mobiles.push_back(lifted.targets.at(id));
    }
    std::map<gf::NodeId,Eigen::Vector2d> mobile_map;
    for (std::size_t i=0;i<result.mobiles.size();++i)
        mobile_map[static_cast<gf::NodeId>(i+1)]=result.mobiles[i];
    result.geometry=geometry(contract,mobile_map);
    gf::CertifiedCoverageTracker tracker({0.0,contract.width_m},
        static_cast<int>(std::llround(contract.width_m/10.0)),
        {0.0,contract.height_m},
        static_cast<int>(std::llround(contract.height_m/10.0)));
    for (const auto& position:result.mobiles)
        tracker.observeSector(Point(position.x(),position.y()),
            Point(position.x(),position.y()),0.05,0.0,400.0,M_PI/3.0,yaw);
    const auto snapshot=gf::task17GridSnapshot(
        tracker.certifiedGrid(),tracker.truthGrid());
    const std::size_t cell_count=static_cast<std::size_t>(snapshot.x_cells)*
        static_cast<std::size_t>(snapshot.y_cells);
    result.certified=gf::task17_grid_detail::bitsFromHex(
        snapshot.certified_bits_hex,cell_count);
    return result;
}

struct Scan {
    bool complete=false;
    double spacing=0.0;
    std::size_t initial_count=0;
    std::size_t route_count=0;
    std::size_t joint_count=0;
    std::size_t nominal_compatible_count=0;
    double maximum_reference=0.0;
    double minimum_separation=1e18;
    double minimum_fim=1e18;
    gf::Task24RasterPlan plan;
    InitialState initial;
    std::vector<bool> covered;
    std::vector<json> witnesses;
};

Scan scan(const gf::Task24DagContract& contract,double spacing,
    bool save) {
    Scan result;
    result.spacing=spacing;
    const auto cells=cellsFor(contract);
    std::vector<std::string> units;
    for (const auto& unit:contract.coverage_units) units.push_back(unit.id);
    const auto field=gf::task21AffineCoordinateField(
        {0.0,0.0},{0.0,1.0},{1.0,0.0});
    // H0 corridor balancing uses its formal initial certified set.  The two
    // one-unit modes are independent of the cut, so their generated initial
    // pose may be evaluated after the same deterministic plan is built.
    std::set<std::string> initial_ids;
    if (contract.mode==gf::Task24LatticeMode::H0DualLadder) {
        const auto scenario=gf::task10p11pStandardCoastalScenario();
        const auto initial=gf::task20FormalInitialCoverage(scenario,300,300);
        const auto certified=gf::task17_grid_detail::bitsFromHex(
            initial.certified_bits_hex,cells.size());
        for (std::size_t index=0;index<cells.size();++index)
            if (certified[index])
                initial_ids.insert(cells[index].id());
    }
    result.plan=gf::task24BuildRasterPlan(
        cells,units,field,spacing,initial_ids);
    if (!result.plan.valid) return result;
    result.initial=initialState(contract,result.plan);
    result.covered=result.initial.certified;
    result.witnesses.resize(save?cells.size():0);
    for (bool value:result.initial.certified) result.initial_count+=value;
    std::map<std::string,const gf::Task20CoverageUnit*> unit_map;
    for (const auto& unit:contract.coverage_units) unit_map[unit.id]=&unit;
    const int nx=static_cast<int>(std::llround(contract.width_m/10.0));
    const int ny=static_cast<int>(std::llround(contract.height_m/10.0));
    for (const auto& corridor:result.plan.corridors) {
        const auto& unit=*unit_map.at(corridor.coverage_unit);
        for (std::size_t band=0;band<result.plan.pass_progress_m.size();++band) {
            for (double cross=corridor.cross_min;
                 cross<=corridor.cross_max+1e-9;cross+=10.0) {
                const double bounded_cross=std::min(cross,corridor.cross_max);
                std::map<std::string,Eigen::Vector2d> fronts;
                for (const auto& other:contract.coverage_units) {
                    const auto found=std::find_if(result.plan.corridors.begin(),
                        result.plan.corridors.end(),[&](const auto& value) {
                            return value.coverage_unit==other.id;
                        });
                    const double other_cross=other.id==unit.id?bounded_cross:
                        0.5*(found->cross_min+found->cross_max);
                    fronts[other.id]=field.origin+
                        result.plan.pass_progress_m[band]*field.progress_axis+
                        other_cross*field.cross_axis;
                }
                const auto lifted=gf::task24LiftTargets(contract,fronts);
                if (!lifted.valid) continue;
                const auto geom=geometry(contract,lifted.targets);
                result.maximum_reference=std::max(
                    result.maximum_reference,geom.maximum_reference_edge_m);
                result.minimum_separation=std::min(
                    result.minimum_separation,geom.minimum_target_separation_m);
                result.minimum_fim=std::min(
                    result.minimum_fim,geom.minimum_nominal_fim_proxy);
                for (int direction:{1,-1}) {
                    const double yaw=direction>0?0.0:M_PI;
                    for (gf::NodeId member:unit.members) {
                        const Eigen::Vector2d pose=lifted.targets.at(member);
                        const int xmin=std::max(0,static_cast<int>(
                            std::floor((pose.x()-400.0)/10.0)));
                        const int xmax=std::min(nx-1,static_cast<int>(
                            std::floor((pose.x()+400.0)/10.0)));
                        const int ymin=std::max(0,static_cast<int>(
                            std::floor((pose.y()-400.0)/10.0)));
                        const int ymax=std::min(ny-1,static_cast<int>(
                            std::floor((pose.y()+400.0)/10.0)));
                        for (int x=xmin;x<=xmax;++x) for (int y=ymin;y<=ymax;++y) {
                            const std::size_t index=static_cast<std::size_t>(x*ny+y);
                            const auto assigned=result.plan.cell_assignments.find(
                                cells[index].id());
                            if (assigned->second.coverage_unit!=unit.id||
                                assigned->second.band_index!=band||
                                !gf::task22CellInCertifiedSector(
                                    pose,yaw,cells[index].center)) continue;
                            if (!result.covered[index]) {
                                result.covered[index]=true;
                                if (geom.maximum_reference_edge_m<850.0&&
                                    geom.minimum_target_separation_m>10.0)
                                    ++result.nominal_compatible_count;
                                if (save) result.witnesses[index]={
                                    {"cell_id",cells[index].id()},
                                    {"coverage_unit",unit.id},
                                    {"band",band},{"direction",direction},
                                    {"responsible_member",member},
                                    {"front",point(fronts.at(unit.id))},
                                    {"service_pose",point(pose)},
                                    {"yaw_rad",yaw},
                                    {"maximum_nominal_reference_m",
                                        geom.maximum_reference_edge_m},
                                    {"minimum_nominal_separation_m",
                                        geom.minimum_target_separation_m},
                                    {"minimum_nominal_fim_proxy",
                                        geom.minimum_nominal_fim_proxy}};
                            }
                        }
                    }
                }
            }
        }
    }
    for (std::size_t i=0;i<result.covered.size();++i) {
        result.joint_count+=result.covered[i];
        if (result.covered[i]&&!result.initial.certified[i])
            ++result.route_count;
    }
    result.complete=result.joint_count==cells.size();
    return result;
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=3) {
        std::cerr<<"usage: GrandFinaleTask24RasterOracle MODE OUTPUT_DIR\n";
        return 2;
    }
    try {
        const auto contract=gf::task24Contract(parseMode(argv[1]));
        if (!contract.valid) throw std::runtime_error(contract.reason);
        std::filesystem::create_directories(argv[2]);
        std::vector<json> attempts;
        double coarse_pass=0.0;
        for (double spacing=800.0;spacing>=100.0;spacing-=50.0) {
            const auto value=scan(contract,spacing,false);
            attempts.push_back({{"spacing_m",spacing},
                {"joint_count",value.joint_count},{"complete",value.complete}});
            if (value.complete) { coarse_pass=spacing; break; }
        }
        if (coarse_pass==0.0) {
            const json summary={{"protocol","task24-raster-oracle-v1"},
                {"mode",contract.id},{"complete",false},
                {"reason","no_hole_free_spacing_in_100_to_800m"},
                {"attempts",attempts}};
            std::ofstream(std::string(argv[2])+"/summary.json")
                <<summary.dump(2)<<'\n';
            std::cout<<summary.dump(2)<<'\n';
            return 0;
        }
        double maximum=coarse_pass;
        for (double spacing=coarse_pass+40.0;
             spacing>coarse_pass;spacing-=10.0) {
            const auto value=scan(contract,spacing,false);
            attempts.push_back({{"spacing_m",spacing},
                {"joint_count",value.joint_count},{"complete",value.complete}});
            if (value.complete) { maximum=spacing; break; }
        }
        const double adopted=maximum-10.0;
        const auto final=scan(contract,adopted,true);
        std::ofstream witness(std::string(argv[2])+"/witnesses.jsonl");
        for (std::size_t i=0;i<final.witnesses.size();++i)
            if (!final.initial.certified[i]) witness<<final.witnesses[i].dump()<<'\n';
        std::ofstream mask(std::string(argv[2])+"/final-mask.bin",
            std::ios::binary);
        for (bool covered:final.covered) {
            const unsigned char value=covered?1:0;
            mask.write(reinterpret_cast<const char*>(&value),1);
        }
        const json summary={{"protocol","task24-raster-oracle-v1"},
            {"mode",contract.id},{"complete",final.complete},
            {"grid_cells",final.covered.size()},
            {"maximum_no_hole_spacing_m",maximum},
            {"adopted_spacing_m",adopted},
            {"initial_certified_count",final.initial_count},
            {"route_service_count",final.route_count},
            {"joint_count",final.joint_count},
            {"nominal_compatible_new_count",final.nominal_compatible_count},
            {"virtual_dependent_new_count",
                final.route_count-final.nominal_compatible_count},
            {"maximum_nominal_reference_m",final.maximum_reference},
            {"minimum_nominal_separation_m",final.minimum_separation},
            {"minimum_nominal_fim_proxy",final.minimum_fim},
            {"initial_maximum_reference_m",
                final.initial.geometry.maximum_reference_edge_m},
            {"initial_minimum_separation_m",
                final.initial.geometry.minimum_target_separation_m},
            {"initial_minimum_fim_proxy",
                final.initial.geometry.minimum_nominal_fim_proxy},
            {"attempts",attempts}};
        std::ofstream(std::string(argv[2])+"/summary.json")
            <<summary.dump(2)<<'\n';
        std::cout<<summary.dump(2)<<'\n';
        return final.complete?0:4;
    } catch (const std::exception& error) {
        std::cerr<<error.what()<<'\n';
        return 3;
    }
}
