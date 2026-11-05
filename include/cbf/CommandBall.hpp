#ifndef CBF_COMMAND_BALL_HPP
#define CBF_COMMAND_BALL_HPP
#include "utils.h"
namespace cbf2026 {
inline json commandBallRows(json rows,double alpha,double T) {
    if(!(alpha>0)||!(T>0)||!std::isfinite(alpha)||!std::isfinite(T))throw std::invalid_argument("invalid command ball parameters");
    for(auto& row:rows) {
        const double E=row["E"],s=std::hypot(row["relative_estimate"][0].get<double>(),row["relative_estimate"][1].get<double>());
        row["original_fixed_ball_const"]=row["const"];row["original_fixed_ball_rho"]=row["rho"];row["original_fixed_ball_M"]=row["M"];
        row["const"]=alpha*row["h_snapshot"].get<double>()-row["coe"]["vx"].get<double>()*row["other_command"][0].get<double>()-row["coe"]["vy"].get<double>()*row["other_command"][1].get<double>()-2*alpha*(s+E)*E;
        row["norm_coefficient"]=-2*((1+alpha*T)*E+alpha*T*(s+E));row["q_squared_coefficient"]=-2*T*(1+alpha*T);row["row_mode"]="oruganti-command-ball";
        row["aux_coe"]=std::vector<double>(rows.size(),0.);row["aux_coe"][row["aux_index"].get<int>()]=row["norm_coefficient"];
        row["radius_rule"]="E+T*q; candidate-specific ball; not original global-Delta Eq9";row.erase("rho");row.erase("M");
    }
    return rows;
}
}
#endif
