#pragma once

#include "grand_finale/CertifiedCoverageTracker.hpp"
#include "grand_finale/Task17GridTelemetry.hpp"
#include "grand_finale/Task20DagLatticeContract.hpp"

namespace gf {

struct Task20OracleGeometry {
    double maximum_reference_edge_m=0.0;
    double minimum_target_separation_m=
        std::numeric_limits<double>::infinity();
    double minimum_nominal_fim_proxy=
        std::numeric_limits<double>::infinity();
};

struct Task20ServiceWitness {
    bool serviceable=false;
    bool nominal_reference_compatible=false;
    std::string reason;
    std::string coverage_unit;
    NodeId responsible_member=0;
    Eigen::Vector2d cell_center=Eigen::Vector2d::Zero();
    Eigen::Vector2d service_pose=Eigen::Vector2d::Zero();
    double service_yaw_rad=0.0;
    double radial_certified_margin_m=0.0;
    double angular_certified_margin_rad=0.0;
    std::map<std::string,Eigen::Vector2d> fronts;
    std::map<NodeId,Eigen::Vector2d> targets;
    Task20OracleGeometry geometry;
};

inline Task20OracleGeometry task20EvaluateOracleGeometry(
    const Task20DagLatticeContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& fixed_positions,
    const std::map<NodeId,Eigen::Vector2d>& targets) {
    Task20OracleGeometry result;
    std::vector<std::pair<NodeId,Eigen::Vector2d>> all;
    for (const auto& [id,position]:fixed_positions) all.push_back({id,position});
    for (const auto& [id,position]:targets) all.push_back({id,position});
    for (std::size_t first=0;first<all.size();++first)
        for (std::size_t second=first+1;second<all.size();++second)
            result.minimum_target_separation_m=std::min(
                result.minimum_target_separation_m,
                (all[first].second-all[second].second).norm());
    std::map<NodeId,std::vector<Eigen::Vector2d>> bearings;
    for (const auto& edge:contract.reference_edges) {
        const Eigen::Vector2d owner=targets.at(edge.owner);
        const auto mobile=targets.find(edge.reference);
        const Eigen::Vector2d reference=mobile==targets.end()
            ?fixed_positions.at(edge.reference):mobile->second;
        const Eigen::Vector2d delta=reference-owner;
        result.maximum_reference_edge_m=std::max(
            result.maximum_reference_edge_m,delta.norm());
        if (delta.norm()>1.0e-12) bearings[edge.owner].push_back(delta.normalized());
    }
    for (NodeId owner=1;owner<=14;++owner) {
        Eigen::Matrix2d fim=Eigen::Matrix2d::Zero();
        for (const auto& bearing:bearings[owner]) fim+=bearing*bearing.transpose();
        result.minimum_nominal_fim_proxy=std::min(
            result.minimum_nominal_fim_proxy,fim.eigenvalues().real().minCoeff());
    }
    return result;
}

inline Task17GridSnapshot task20FormalInitialCoverage(
    const Task10p10Scenario& scenario,int x_cells,int y_cells) {
    if (scenario.mobile_ids.size()!=scenario.mobile_positions.size()||
        scenario.width_m<=0.0||scenario.height_m<=0.0||x_cells<=0||
        y_cells<=0)
        throw std::invalid_argument("invalid Task 20 initial coverage request");
    const auto frozen=loadTask10p10Config();
    CertifiedCoverageTracker tracker({0.0,scenario.width_m},x_cells,
        {0.0,scenario.height_m},y_cells);
    constexpr double formal_initial_error_bound_m=0.05;
    constexpr double formal_initial_yaw_rad=M_PI/2.0;
    for (const auto& position:scenario.mobile_positions)
        tracker.observeSector(Point(position.x(),position.y()),
            Point(position.x(),position.y()),formal_initial_error_bound_m,
            frozen.sector_inner_radius_m,frozen.sector_outer_radius_m,
            frozen.sector_half_angle_deg*M_PI/180.0,
            formal_initial_yaw_rad);
    return task17GridSnapshot(tracker.certifiedGrid(),tracker.truthGrid());
}

inline Task20ServiceWitness task20FindServiceWitness(
    const Task20DagLatticeContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& fixed_positions,
    const Eigen::Vector2d& cell_center,double certified_error_bound_m=0.05) {
    Task20ServiceWitness result;
    result.cell_center=cell_center;
    if (!contract.valid||!cell_center.allFinite()||
        certified_error_bound_m<0.0) {
        result.reason="invalid_service_request";
        return result;
    }
    constexpr double outer_radius_m=400.0;
    constexpr double half_angle_rad=M_PI/3.0;
    constexpr double cell_half_diagonal_m=5.0*1.4142135623730950488;
    const std::array<double,3> standoffs{{200.0,250.0,300.0}};
    constexpr std::size_t direction_count=12;
    for (double standoff:standoffs)
        for (std::size_t direction_index=0;
             direction_index<direction_count;++direction_index) {
            const double yaw=2.0*M_PI*static_cast<double>(direction_index)/
                static_cast<double>(direction_count);
            const Eigen::Vector2d direction{std::cos(yaw),std::sin(yaw)};
            const Eigen::Vector2d pose=cell_center-standoff*direction;
            for (const auto& [member,role]:contract.member_roles) {
                const auto inverse=task20FrontForMemberPose(
                    contract,fixed_positions,member,pose);
                if (!inverse.valid) continue;
                std::map<std::string,Eigen::Vector2d> fronts;
                std::size_t active_index=0;
                for (std::size_t index=0;index<contract.coverage_units.size();++index)
                    if (contract.coverage_units[index].id==role.coverage_unit)
                        active_index=index;
                const Eigen::Vector2d transverse{-direction.y(),direction.x()};
                for (std::size_t index=0;index<contract.coverage_units.size();++index) {
                    const auto& unit=contract.coverage_units[index];
                    if (unit.id==role.coverage_unit) fronts[unit.id]=inverse.front;
                    else fronts[unit.id]=cell_center+
                        (static_cast<double>(index)-static_cast<double>(active_index))*
                            650.0*transverse+100.0*direction;
                }
                const auto lifted=task20LiftTargets(contract,fixed_positions,fronts);
                if (!lifted.valid) continue;
                const auto geometry=task20EvaluateOracleGeometry(
                    contract,fixed_positions,lifted.targets);
                if (!(geometry.minimum_target_separation_m>10.0)) continue;
                const double reserve=certified_error_bound_m+cell_half_diagonal_m;
                const double radial_margin=outer_radius_m-standoff-reserve;
                const double angular_margin=half_angle_rad-
                    std::asin(std::min(1.0,reserve/standoff));
                if (!(radial_margin>=0.0&&angular_margin>=0.0)) continue;
                result.serviceable=true;
                result.nominal_reference_compatible=
                    geometry.maximum_reference_edge_m<850.0;
                result.coverage_unit=role.coverage_unit;
                result.responsible_member=member;
                result.service_pose=pose;
                result.service_yaw_rad=yaw;
                result.radial_certified_margin_m=radial_margin;
                result.angular_certified_margin_rad=angular_margin;
                result.fronts=std::move(fronts);
                result.targets=lifted.targets;
                result.geometry=geometry;
                return result;
            }
        }
    result.reason="no_lattice_service_witness";
    return result;
}

}  // namespace gf
