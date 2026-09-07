#pragma once
#include "grand_finale/Task31TriangularLattice.hpp"

namespace gf {
struct Task31TerminalPort {
    bool valid=false;
    std::string reason;
    Task20DagLatticeContract contract;
    std::map<std::string,NodeId> ports;
};

// An offline/initialization-only reparameterization of one shared front.
// q_i=b+M_i M_port^{-1}(cell-b). Fixed anchors are not transformed.
// This preserves internal similarity, not reference feasibility or sensing.
inline Task31TerminalPort task31PositiveTerminalPort(const Task31Lattice& lattice) {
    Task31TerminalPort out;out.contract=lattice.contract;
    auto reject=[&](const std::string& r){out.reason=r;return out;};
    if(!lattice.valid||!lattice.contract.valid)return reject("invalid_lattice");
    constexpr double h=0.86602540378443864676;
    for(const auto& u:lattice.contract.coverage_units) {
        if(u.front_members.empty())return reject("missing_front_members");
        NodeId port{};double maximum=-std::numeric_limits<double>::infinity();bool tied=false;
        for(auto id:u.front_members) {
            if(std::find(u.members.begin(),u.members.end(),id)==u.members.end()||
               !lattice.cells.count(id)||!lattice.contract.member_roles.count(id))return reject("invalid_front_role");
            const double slot=lattice.cells.at(id).slot;
            if(!std::isfinite(slot))return reject("nonfinite_front_slot");
            if(slot>maximum){port=id;maximum=slot;tied=false;}
            else if(slot==maximum)tied=true;
        }
        if(tied)return reject("ambiguous_terminal_slot");
        const auto& p=lattice.contract.member_roles.at(port);
        const double a=p.axial_fraction+.5*std::abs(p.triangular_fraction),b=h*p.triangular_fraction;
        const double det=a*a+b*b;
        if(!std::isfinite(det)||det<1e-12)return reject("singular_terminal_map");
        for(auto id:u.members) {
            auto& r=out.contract.member_roles.at(id);
            const double ai=r.axial_fraction+.5*std::abs(r.triangular_fraction),bi=h*r.triangular_fraction;
            const double next_a=(ai*a+bi*b)/det,next_b=(bi*a-ai*b)/det;
            if(!std::isfinite(next_a)||!std::isfinite(next_b))return reject("nonfinite_role_map");
            r.triangular_fraction=next_b/h;
            r.axial_fraction=next_a-.5*std::abs(r.triangular_fraction);
        }
        out.ports.emplace(u.id,port);
    }
    out.contract.id+="-positive-terminal-port";
    out.contract.structural_signature+=";positive-terminal-port";
    out.valid=true;out.reason="nominal_mapping_not_actual_service_certificate";return out;
}
} // namespace gf
