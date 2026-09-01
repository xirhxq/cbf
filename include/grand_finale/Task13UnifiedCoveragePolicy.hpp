#pragma once

#include "grand_finale/Task10p11hLeaderCoveragePolicy.hpp"

#include <array>
#include <cstring>
#include <optional>
#include <tuple>

namespace gf {

struct Task13UnifiedCoverageConfig {
    double minimum_half_width_m=7.0;
    double fan_ratio=0.0075;
    double reference_limit_m=850.0;
    double separation_limit_m=10.0;
    double forward_focus_distance_m=400.0;
    double comparison_epsilon_m=1.0e-9;
    std::size_t shortlist_per_squad=64;
    bool compute_nominal_fim_proxy=true;
    double certified_service_standoff_m=0.0;
};

struct Task13UnifiedCoverageSquad {
    std::string name;
    std::array<NodeId,7> members;
    NodeId leader=0;
    NodeId outer_anchor=0;
    double mirror_sign=0.0;
    std::array<DirectedEdge,14> edges;
};

inline std::array<Task13UnifiedCoverageSquad,2>
task13UnifiedCoverageSquads() {
    return {{
        {"A",{1,2,3,4,5,6,7},7,100,-1.0,{
            DirectedEdge{101,1},DirectedEdge{100,1},
            DirectedEdge{101,2},DirectedEdge{1,2},
            DirectedEdge{1,3},DirectedEdge{2,3},
            DirectedEdge{2,4},DirectedEdge{3,4},
            DirectedEdge{3,5},DirectedEdge{4,5},
            DirectedEdge{4,6},DirectedEdge{5,6},
            DirectedEdge{5,7},DirectedEdge{6,7}}},
        {"B",{8,9,10,11,12,13,14},14,102,1.0,{
            DirectedEdge{101,8},DirectedEdge{102,8},
            DirectedEdge{101,9},DirectedEdge{8,9},
            DirectedEdge{8,10},DirectedEdge{9,10},
            DirectedEdge{9,11},DirectedEdge{10,11},
            DirectedEdge{10,12},DirectedEdge{11,12},
            DirectedEdge{11,13},DirectedEdge{12,13},
            DirectedEdge{12,14},DirectedEdge{13,14}}}
    }};
}

struct Task13UnifiedCoverageWitness {
    std::string squad;
    FrontierCell cell;
    NodeId responsible_member=0;
    NodeId leader=0;
    std::map<NodeId,Eigen::Vector2d> targets;
    std::map<NodeId,std::string> target_ids;
    double maximum_reference_edge_m=0.0;
    double minimum_target_separation_m=
        std::numeric_limits<double>::infinity();
    double nominal_fim_proxy=0.0;
    std::uint64_t digest=0;
};

struct Task13UnifiedAgentState {
    NodeId id=0;
    Eigen::Vector2d position=Eigen::Vector2d::Zero();
    double yaw_rad=0.0;
};

struct Task13UnifiedCoverageRequest {
    std::vector<Task13UnifiedAgentState> agents;
    std::vector<FrontierCell> uncovered_cells;
    std::map<NodeId,Eigen::Vector2d> fixed_positions;
    std::map<std::string,Task13UnifiedCoverageWitness> retained;
    Task13UnifiedCoverageConfig config;
};

struct Task13UnifiedCoverageAssignment {
    Task13UnifiedCoverageWitness witness;
    bool active=false;
    std::string reason;
};

struct Task13UnifiedCoverageResult {
    bool valid=false;
    std::string reason;
    std::map<std::string,Task13UnifiedCoverageAssignment> assignments;
    std::map<NodeId,FrontierCell> targets;
    std::size_t active_squads=0;
    double minimum_cross_target_separation_m=
        std::numeric_limits<double>::infinity();
    double maximum_target_displacement_m=0.0;
    double sum_target_displacement_m=0.0;
    double total_forward_focus_cost_m=0.0;
    std::size_t evaluated_joint_combinations=0;
};

namespace task13_unified_detail {

inline constexpr std::array<double,7> kLayer{
    0.25,0.25,0.50,0.50,0.75,0.75,1.0};
inline constexpr std::array<double,7> kRail{
    1.0,-1.0,1.0,-1.0,1.0,-1.0,0.0};

inline std::size_t localIndex(
    const Task13UnifiedCoverageSquad& squad,NodeId member) {
    const auto found=std::find(squad.members.begin(),squad.members.end(),member);
    if (found==squad.members.end())
        throw std::invalid_argument("responsible member outside squad");
    return static_cast<std::size_t>(
        std::distance(squad.members.begin(),found));
}

inline Eigen::Matrix2d lateralRotation(
    const Task13UnifiedCoverageSquad& squad) {
    Eigen::Matrix2d value;
    value<<0.0,-squad.mirror_sign,squad.mirror_sign,0.0;
    return value;
}

inline std::uint64_t hashWitness(
    const Task13UnifiedCoverageWitness& witness) {
    std::ostringstream text;
    text.precision(17);
    text<<witness.squad<<':'<<witness.cell.id()<<':'
        <<witness.responsible_member<<';';
    for (const auto& [member,target]:witness.targets)
        text<<member<<':'<<target.x()<<','<<target.y()<<';';
    return leader_coverage_detail::hashText(text.str());
}

inline Eigen::Vector2d referencePosition(
    NodeId reference,const std::map<NodeId,Eigen::Vector2d>& targets,
    const std::map<NodeId,Eigen::Vector2d>& fixed) {
    const auto fixed_it=fixed.find(reference);
    if (fixed_it!=fixed.end()) return fixed_it->second;
    const auto mobile_it=targets.find(reference);
    if (mobile_it==targets.end())
        throw std::invalid_argument("missing target reference");
    return mobile_it->second;
}

inline double nominalFimProxy(
    const Task13UnifiedCoverageSquad& squad,
    const std::map<NodeId,Eigen::Vector2d>& targets,
    const std::map<NodeId,Eigen::Vector2d>& fixed) {
    double minimum=std::numeric_limits<double>::infinity();
    for (NodeId owner:squad.members) {
        Eigen::Matrix2d fim=Eigen::Matrix2d::Zero();
        std::size_t incoming=0;
        for (const auto& edge:squad.edges) if (edge.owner==owner) {
            const Eigen::Vector2d delta=referencePosition(
                edge.reference,targets,fixed)-targets.at(owner);
            if (delta.norm()<=1e-12) return 0.0;
            const Eigen::Vector2d unit=delta.normalized();
            fim+=unit*unit.transpose();
            ++incoming;
        }
        if (incoming<2) return 0.0;
        minimum=std::min(minimum,
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(fim).
                eigenvalues().minCoeff());
    }
    return minimum;
}

inline std::tuple<int,int,NodeId,NodeId> identity(
    const Task13UnifiedCoverageWitness& witness) {
    return {witness.cell.x_index,witness.cell.y_index,
            witness.responsible_member,witness.leader};
}

struct Choice {
    Task13UnifiedCoverageWitness witness;
    bool active=false;
    std::string reason;
    double maximum_displacement_m=0.0;
    double sum_displacement_m=0.0;
    double forward_cost_m=0.0;
};

inline std::pair<double,double> displacement(
    const Task13UnifiedCoverageWitness& old,
    const Task13UnifiedCoverageWitness& next) {
    double maximum=0.0,sum=0.0;
    for (const auto& [member,target]:next.targets) {
        const double value=(target-old.targets.at(member)).norm();
        maximum=std::max(maximum,value);
        sum+=value;
    }
    return {maximum,sum};
}

inline bool choiceLess(const Choice& lhs,const Choice& rhs) {
    return std::tuple(lhs.maximum_displacement_m,lhs.sum_displacement_m,
                      lhs.forward_cost_m,identity(lhs.witness))<
           std::tuple(rhs.maximum_displacement_m,rhs.sum_displacement_m,
                      rhs.forward_cost_m,identity(rhs.witness));
}

inline void insertShortlist(
    std::vector<Choice>& values,Choice value,std::size_t capacity) {
    const auto where=std::lower_bound(values.begin(),values.end(),value,
        [](const Choice& lhs,const Choice& rhs) { return choiceLess(lhs,rhs); });
    values.insert(where,std::move(value));
    if (values.size()>capacity) values.pop_back();
}

inline double forwardCost(
    const Task13UnifiedCoverageWitness& witness,
    const std::vector<Task13UnifiedAgentState>& agents,double distance) {
    const auto found=std::find_if(agents.begin(),agents.end(),
        [&](const auto& value) { return value.id==witness.responsible_member; });
    if (found==agents.end())
        throw std::invalid_argument("missing unified coverage agent");
    const Eigen::Vector2d focus=found->position+distance*Eigen::Vector2d(
        std::cos(found->yaw_rad),std::sin(found->yaw_rad));
    return (witness.cell.center-focus).norm();
}

inline Choice makeChoice(
    Task13UnifiedCoverageWitness witness,bool active,std::string reason,
    const std::optional<Task13UnifiedCoverageWitness>& retained,
    const Task13UnifiedCoverageRequest& request) {
    Choice value{std::move(witness),active,std::move(reason)};
    if (retained.has_value()) {
        const auto burden=displacement(*retained,value.witness);
        value.maximum_displacement_m=burden.first;
        value.sum_displacement_m=burden.second;
    }
    if (active) value.forward_cost_m=forwardCost(
        value.witness,request.agents,request.config.forward_focus_distance_m);
    return value;
}

struct FastWitnessGeometry {
    FrontierCell cell;
    NodeId responsible_member=0;
    std::array<Eigen::Vector2d,7> targets;
    double maximum_reference_edge_m=0.0;
    double minimum_target_separation_m=
        std::numeric_limits<double>::infinity();
};

inline std::optional<FastWitnessGeometry> fastTaperedGeometry(
    const Task13UnifiedCoverageSquad& squad,NodeId responsible_member,
    const FrontierCell& cell,
    const std::map<NodeId,Eigen::Vector2d>& fixed_positions,
    const Task13UnifiedCoverageConfig& config) {
    if (cell.x_index<0 || cell.y_index<0 || !cell.center.allFinite() ||
        !(config.minimum_half_width_m>5.0) || config.fan_ratio<0.0 ||
        config.reference_limit_m<=0.0 || config.separation_limit_m<=0.0)
        return std::nullopt;
    const auto base_it=fixed_positions.find(101);
    if (base_it==fixed_positions.end()) return std::nullopt;
    const std::size_t responsible=localIndex(squad,responsible_member);
    const double t=kLayer[responsible];
    const double rail=kRail[responsible];
    const double width=config.minimum_half_width_m;
    const double fan=config.fan_ratio;
    const Eigen::Matrix2d j=lateralRotation(squad);
    Eigen::Vector2d responsible_target=cell.center;
    if (config.certified_service_standoff_m>0.0) {
        const Eigen::Vector2d domain_center(1500.0,1500.0);
        const Eigen::Vector2d radial=cell.center-domain_center;
        const double denominator=std::max(radial.norm(),
            config.certified_service_standoff_m);
        responsible_target-=config.certified_service_standoff_m*
            radial/denominator;
    }
    const Eigen::Vector2d x=responsible_target-base_it->second;
    Eigen::Vector2d d;
    if (rail==0.0) {
        d=x;
    } else {
        const double discriminant=(1.0+fan*fan)*x.squaredNorm()-width*width;
        if (!(discriminant>0.0)) return std::nullopt;
        const double axial=(-fan*width+std::sqrt(discriminant))/
            (1.0+fan*fan);
        const double lateral=width+fan*axial;
        const double denominator=axial*axial+lateral*lateral;
        if (!(axial>0.0) || !(denominator>0.0)) return std::nullopt;
        const Eigen::Vector2d unit=(axial*x-rail*lateral*j*x)/denominator;
        d=(axial/t)*unit;
    }
    const double radius=d.norm();
    if (!(radius>0.0)) return std::nullopt;
    const Eigen::Vector2d normal=j*(d/radius);
    FastWitnessGeometry value;
    value.cell=cell;
    value.responsible_member=responsible_member;
    for (std::size_t local=0;local<squad.members.size();++local)
        value.targets[local]=base_it->second+kLayer[local]*d+
            kRail[local]*(width+fan*kLayer[local]*radius)*normal;
    if ((value.targets[responsible]-responsible_target).norm()>1e-8)
        return std::nullopt;
    for (const auto& edge:squad.edges) {
        const Eigen::Vector2d owner=value.targets[localIndex(squad,edge.owner)];
        const auto fixed=fixed_positions.find(edge.reference);
        const Eigen::Vector2d reference=fixed!=fixed_positions.end()
            ?fixed->second:value.targets[localIndex(squad,edge.reference)];
        value.maximum_reference_edge_m=std::max(
            value.maximum_reference_edge_m,(owner-reference).norm());
    }
    for (std::size_t first=0;first<squad.members.size();++first) {
        for (std::size_t second=first+1;second<squad.members.size();++second)
            value.minimum_target_separation_m=std::min(
                value.minimum_target_separation_m,
                (value.targets[first]-value.targets[second]).norm());
        for (const auto& [fixed_id,position]:fixed_positions) {
            (void)fixed_id;
            value.minimum_target_separation_m=std::min(
                value.minimum_target_separation_m,
                (value.targets[first]-position).norm());
        }
    }
    if (!(value.maximum_reference_edge_m<
            config.reference_limit_m-config.comparison_epsilon_m) ||
        !(value.minimum_target_separation_m>
            config.separation_limit_m+config.comparison_epsilon_m))
        return std::nullopt;
    return value;
}

inline Task13UnifiedCoverageWitness materializeWitness(
    const Task13UnifiedCoverageSquad& squad,
    const FastWitnessGeometry& geometry,
    const std::map<NodeId,Eigen::Vector2d>& fixed_positions,
    const Task13UnifiedCoverageConfig& config) {
    Task13UnifiedCoverageWitness witness;
    witness.squad=squad.name;
    witness.cell=geometry.cell;
    witness.responsible_member=geometry.responsible_member;
    witness.leader=squad.leader;
    witness.maximum_reference_edge_m=geometry.maximum_reference_edge_m;
    witness.minimum_target_separation_m=
        geometry.minimum_target_separation_m;
    for (std::size_t local=0;local<squad.members.size();++local) {
        witness.targets[squad.members[local]]=geometry.targets[local];
        witness.target_ids[squad.members[local]]=geometry.cell.id();
    }
    if (config.compute_nominal_fim_proxy)
        witness.nominal_fim_proxy=nominalFimProxy(
            squad,witness.targets,fixed_positions);
    witness.digest=hashWitness(witness);
    return witness;
}

struct FastChoice {
    FastWitnessGeometry geometry;
    bool active=false;
    std::string reason;
    double maximum_displacement_m=0.0;
    double sum_displacement_m=0.0;
    double forward_cost_m=0.0;
};

inline auto fastChoiceIdentity(const FastChoice& value) {
    return std::tuple(value.geometry.cell.x_index,value.geometry.cell.y_index,
        value.geometry.responsible_member);
}

inline bool fastChoiceLess(const FastChoice& lhs,const FastChoice& rhs) {
    return std::tuple(lhs.maximum_displacement_m,lhs.sum_displacement_m,
        lhs.forward_cost_m,fastChoiceIdentity(lhs))<
        std::tuple(rhs.maximum_displacement_m,rhs.sum_displacement_m,
        rhs.forward_cost_m,fastChoiceIdentity(rhs));
}

inline void insertFastShortlist(
    std::vector<FastChoice>& values,FastChoice value,std::size_t capacity) {
    const auto where=std::lower_bound(values.begin(),values.end(),value,
        [](const FastChoice& lhs,const FastChoice& rhs) {
            return fastChoiceLess(lhs,rhs);
        });
    values.insert(where,std::move(value));
    if (values.size()>capacity) values.pop_back();
}

}  // namespace task13_unified_detail

inline std::optional<Task13UnifiedCoverageWitness> task13TaperedWitness(
    const Task13UnifiedCoverageSquad& squad,NodeId responsible_member,
    const FrontierCell& cell,
    const std::map<NodeId,Eigen::Vector2d>& fixed_positions,
    const Task13UnifiedCoverageConfig& config={}) {
    const auto geometry=task13_unified_detail::fastTaperedGeometry(
        squad,responsible_member,cell,fixed_positions,config);
    if (!geometry.has_value()) return std::nullopt;
    return task13_unified_detail::materializeWitness(
        squad,*geometry,fixed_positions,config);
}

inline double task13CrossMinimum(
    const Task13UnifiedCoverageWitness& first,
    const Task13UnifiedCoverageWitness& second) {
    double minimum=std::numeric_limits<double>::infinity();
    for (const auto& [first_id,a]:first.targets) {
        (void)first_id;
        for (const auto& [second_id,b]:second.targets) {
            (void)second_id;
            minimum=std::min(minimum,(a-b).norm());
        }
    }
    return minimum;
}

inline Task13UnifiedCoverageResult allocateTask13UnifiedCoverage(
    Task13UnifiedCoverageRequest request) {
    using namespace task13_unified_detail;
    Task13UnifiedCoverageResult result;
    if (request.config.shortlist_per_squad==0 || request.agents.size()!=14 ||
        request.fixed_positions.count(100)==0 ||
        request.fixed_positions.count(101)==0 ||
        request.fixed_positions.count(102)==0) {
        result.reason="invalid_unified_coverage_request";
        return result;
    }
    std::sort(request.uncovered_cells.begin(),request.uncovered_cells.end(),
        [](const auto& lhs,const auto& rhs) { return lhs.id()<rhs.id(); });
    std::set<std::string> uncovered_ids;
    for (const auto& cell:request.uncovered_cells)
        if (cell.x_index<0 || cell.y_index<0 || !cell.center.allFinite() ||
            !uncovered_ids.insert(cell.id()).second) {
            result.reason="invalid_unified_uncovered_cells";
            return result;
        }
    const auto squads=task13UnifiedCoverageSquads();
    std::map<std::string,std::vector<Choice>> choices;
    for (const auto& squad:squads) {
        const auto retained_it=request.retained.find(squad.name);
        const std::optional<Task13UnifiedCoverageWitness> retained=
            retained_it==request.retained.end()
                ?std::optional<Task13UnifiedCoverageWitness>{}
                :std::optional<Task13UnifiedCoverageWitness>{retained_it->second};
        std::vector<FastChoice> fast_active;
        std::vector<Choice> active,idle;
        const bool persistent=retained.has_value() &&
            uncovered_ids.count(retained->cell.id());
        const auto append_cell=[&](const FrontierCell& cell,
                                   const std::string& reason) {
            for (NodeId member:squad.members) {
                auto geometry=fastTaperedGeometry(squad,member,cell,
                    request.fixed_positions,request.config);
                if (!geometry.has_value()) continue;
                FastChoice choice;
                choice.geometry=std::move(*geometry);
                choice.active=true;
                choice.reason=reason;
                if (retained.has_value()) {
                    for (std::size_t local=0;
                         local<squad.members.size();++local) {
                        const double displacement=(choice.geometry.targets[local]-
                            retained->targets.at(squad.members[local])).norm();
                        choice.maximum_displacement_m=std::max(
                            choice.maximum_displacement_m,displacement);
                        choice.sum_displacement_m+=displacement;
                    }
                }
                const auto agent=std::find_if(request.agents.begin(),
                    request.agents.end(),[&](const auto& value) {
                        return value.id==member;
                    });
                if (agent==request.agents.end()) continue;
                const Eigen::Vector2d focus=agent->position+
                    request.config.forward_focus_distance_m*Eigen::Vector2d(
                        std::cos(agent->yaw_rad),std::sin(agent->yaw_rad));
                choice.forward_cost_m=(cell.center-focus).norm();
                insertFastShortlist(fast_active,std::move(choice),
                    request.config.shortlist_per_squad);
            }
        };
        if (persistent) {
            append_cell(retained->cell,"persistent_same_real_id");
        } else {
            for (const auto& cell:request.uncovered_cells)
                append_cell(cell,"new_real_cell");
            if (retained.has_value())
                for (NodeId member:squad.members) {
                    auto witness=task13TaperedWitness(squad,member,
                        retained->cell,request.fixed_positions,request.config);
                    if (!witness.has_value()) continue;
                    insertShortlist(idle,makeChoice(std::move(*witness),false,
                        "retained_same_real_id",retained,request),7);
                }
        }
        for (const auto& value:fast_active) {
            Choice choice;
            choice.witness=materializeWitness(
                squad,value.geometry,request.fixed_positions,request.config);
            choice.active=value.active;
            choice.reason=value.reason;
            choice.maximum_displacement_m=value.maximum_displacement_m;
            choice.sum_displacement_m=value.sum_displacement_m;
            choice.forward_cost_m=value.forward_cost_m;
            active.push_back(std::move(choice));
        }
        active.insert(active.end(),idle.begin(),idle.end());
        choices[squad.name]=std::move(active);
    }
    if (request.uncovered_cells.empty() && request.retained.empty()) {
        result.valid=true;
        result.reason="no_task_no_retained_state";
        return result;
    }
    struct Joint {
        Choice a,b;
        std::size_t active=0;
        double cross=0.0,max_displacement=0.0,sum_displacement=0.0,
            forward=0.0;
    };
    std::optional<Joint> best;
    const auto joint_less=[](const Joint& lhs,const Joint& rhs) {
        return std::tuple(-static_cast<long long>(lhs.active),
            lhs.max_displacement,lhs.sum_displacement,lhs.forward,
            identity(lhs.a.witness),identity(lhs.b.witness))<
            std::tuple(-static_cast<long long>(rhs.active),
            rhs.max_displacement,rhs.sum_displacement,rhs.forward,
            identity(rhs.a.witness),identity(rhs.b.witness));
    };
    for (const auto& a:choices["A"]) for (const auto& b:choices["B"]) {
        ++result.evaluated_joint_combinations;
        if (a.active&&b.active&&a.witness.cell.id()==b.witness.cell.id())
            continue;
        const double cross=task13CrossMinimum(a.witness,b.witness);
        if (!(cross>request.config.separation_limit_m+
                request.config.comparison_epsilon_m)) continue;
        Joint candidate{a,b,static_cast<std::size_t>(a.active)+b.active,cross,
            std::max(a.maximum_displacement_m,b.maximum_displacement_m),
            a.sum_displacement_m+b.sum_displacement_m,
            a.forward_cost_m+b.forward_cost_m};
        if (!best.has_value()||joint_less(candidate,*best))
            best=std::move(candidate);
    }
    if (!best.has_value() ||
        (!request.uncovered_cells.empty()&&best->active==0)) {
        result.reason="unified_allocator_failure";
        return result;
    }
    result.valid=true;
    result.reason="unified_real_cell_targets";
    result.active_squads=best->active;
    result.minimum_cross_target_separation_m=best->cross;
    result.maximum_target_displacement_m=best->max_displacement;
    result.sum_target_displacement_m=best->sum_displacement;
    result.total_forward_focus_cost_m=best->forward;
    for (const auto& [name,choice]:std::array<std::pair<std::string,Choice>,2>{{
            {"A",best->a},{"B",best->b}}}) {
        result.assignments[name]={choice.witness,choice.active,choice.reason};
        for (const auto& [member,target]:choice.witness.targets)
            result.targets[member]={choice.witness.cell.x_index,
                choice.witness.cell.y_index,target};
    }
    return result;
}

}  // namespace gf
