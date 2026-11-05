#ifndef CBF_TVT_LITERATURE_BASELINES_HPP
#define CBF_TVT_LITERATURE_BASELINES_HPP
#include "utils.h"

namespace tvt_literature {
// Explicit planar adaptations. Original method equations and changed graph /
// sequential-command assumptions are documented outside the main candidate.
inline json pairRows(int id, const std::vector<Point>& positions,
                     const std::vector<Point>& bases, const json& formation,
                     const std::map<int, Eigen::VectorXd>& commands,
                     const std::string& mode, double E_node, double T,
                     double component, double alpha, double D, double R) {
    json rows=json::array();
    auto add=[&](int peer,bool base,bool collision) {
        const Point r=positions.at(id-1)-(base?bases.at(peer):positions.at(peer-1));
        const Point other=base?Point(0,0):Point(commands.at(peer));
        const double sign=collision?1.:-1.;
        const double h=collision?r*r-D*D:R*R-r*r;
        const double E=E_node*(base?1.:2.);
        const double V=std::sqrt(2.)*component*(base?1.:2.);
        const double rho=E+T*V,M=r.len()+rho;
        const double constant=alpha*h-sign*2.*(r*other)
            -(mode=="sd-drcbf-socp"?2.*alpha*M*rho:0.);
        json row={{"name",collision?"safetyCBF(#"+std::to_string(peer)+")":base?"fixedCommCBF(base-"+std::to_string(peer)+")":"fixedCommCBF(#"+std::to_string(peer)+")"},
            {"row_mode",mode},{"coe",{{"vx",sign*2.*r.x},{"vy",sign*2.*r.y},{"yawRateRad",0.}}},
            {"const",constant},{"edge",{{"kind",collision?"collision":"link"},{"peer",peer},{"is_base",base}}},
            {"relative_estimate",{r.x,r.y}},{"other_command",{other.x,other.y}},
            {"h_snapshot",h},{"alpha",alpha},{"E",E},{"T",T},{"V",V},{"rho",rho},{"M",M}};
        if(mode=="sd-drcbf-socp")row["norm_coefficient"]=-2.*rho;
        rows.push_back(row);
    };
    for(int peer=1;peer<=static_cast<int>(positions.size());++peer)
        if(peer!=id && (mode=="gdms-min" || peer<id))add(peer,false,true);
    for(const auto& peer:formation.at("baseIds"))add(peer.get<int>(),true,false);
    for(const auto& peer:formation.at("anchorIds"))add(peer.get<int>(),false,false);
    if(mode=="gdms-min" && !rows.empty()) {
        double minimum=std::numeric_limits<double>::infinity();
        for(const auto& row:rows)minimum=std::min(minimum,row.at("h_snapshot").get<double>());
        json active=json::array();
        for(auto row:rows)if(std::abs(row.at("h_snapshot").get<double>()-minimum)<=1e-9*std::max(1.,std::abs(minimum))) {
            row["all_edge_branch_count"]=rows.size();row["active_min_value"]=minimum;
            active.push_back(row);
        }
        return active; // Preserve all tied active branches; do not replace min by all.
    }
    for(size_t k=0;k<rows.size();++k) {
        rows[k]["aux_index"]=k;
        rows[k]["aux_coe"]=std::vector<double>(rows.size(),0.);
        rows[k]["aux_coe"][k]=rows[k]["norm_coefficient"];
    }
    return rows;
}
} // namespace tvt_literature
#endif
