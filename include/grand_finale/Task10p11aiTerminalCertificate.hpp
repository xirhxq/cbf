#pragma once

#include "grand_finale/Task10p11ahTerminalRecoveryOptimizer.hpp"
#include "grand_finale/Task10p11xRecoveryCampaign.hpp"

#ifdef ENABLE_GUROBI
#include "gurobi_c++.h"
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

struct Task10p11aiProtocol {
    std::string protocol="task10p11ai-terminal-certificate";
    std::string preregistration="v1.2-approved-2026-08-30";
    double tau_mps2=22.0;
    double dt_s=0.1;
    double half_box_mps2=4.0;
    std::size_t chain_beats=3;
    std::set<NodeId> component{2,9};
    double strict_negative_bound_mps2=-1.0e-8;
    double independent_agreement_mps2=1.0e-7;
    double anchor_successor_residual_mps2=-0.02138986047537017;
    double anchor_tolerance_mps2=1.0e-9;
    double stage_a_total_budget_s=28800.0;
    bool terminality_no_followup_tasks=true;
};

inline Task10p11aiProtocol task10p11aiProtocol() { return {}; }

enum class Task10p11aiClassification { StrictlyInfeasible, Undetermined };

inline std::string task10p11aiClassificationName(
    Task10p11aiClassification value) {
    return value==Task10p11aiClassification::StrictlyInfeasible
        ?"strictly_infeasible":"undetermined";
}

struct Task10p11aiPairSpec {
    bool collision=true;
    bool mobile_pair=true;
    PairwiseSecondOrderRowSpec spec;
    double tube_position_radius_m=0.0;
    double tube_velocity_radius_mps=0.0;
};

inline double task10p11aiPositionRadius(const Task10p11aiPairSpec& pair) {
    return pair.spec.uncertainty+pair.tube_position_radius_m;
}

inline double task10p11aiRelativeDomain(const Task10p11aiPairSpec& pair,
                                        double half_box) {
    return pair.mobile_pair?2.0*half_box:half_box;
}

inline double task10p11aiDirectionRadius(double q) {
    return std::sqrt(2.0*(1.0-std::sqrt(std::max(0.0,1.0-q*q))));
}

struct Task10p11aiCoupledMirror {
    Eigen::Vector2d normal=Eigen::Vector2d::Zero();
    double constant=0.0;
};

// Closed-form mirror of the coupled once-reserve full-pair row:
// residual == normal.dot(delta_u) + constant, where delta_u is the control
// difference of the two owners.  For reference rows the builder's
// nominal_control_coefficient is -nominal_normal; the sign is carried here so
// the mirror matches the canonical row exactly.  Fidelity against the builder
// is a focused-test obligation, not an assumption.
inline Task10p11aiCoupledMirror task10p11aiCoupledMirror(
    const Task10p11aiPairSpec& pair,const Eigen::Vector2d& rel_pos,
    const Eigen::Vector2d& rel_vel,double half_box) {
    const double distance=rel_pos.norm();
    const double position_radius=task10p11aiPositionRadius(pair);
    if (!std::isfinite(distance)||distance<=position_radius||distance<1e-9)
        throw std::invalid_argument("task10p11ai mirror radial singularity");
    const Eigen::Vector2d unit_normal=rel_pos/distance;
    const double q=position_radius/distance;
    const double direction_radius=task10p11aiDirectionRadius(q);
    const double radial=unit_normal.dot(rel_vel);
    const double speed=rel_vel.norm();
    const double reserve=half_box*std::sqrt(2.0)*direction_radius;
    double central=0.0;
    if (pair.collision) {
        const double h_lower=distance-position_radius-pair.spec.distanceLimit;
        const double hdot_lower=radial-direction_radius*speed-
            pair.tube_velocity_radius_mps;
        central=pair.spec.lambda1*pair.spec.lambda2*h_lower+
            (pair.spec.lambda1+pair.spec.lambda2)*hdot_lower-
            pair.spec.totalReserve;
    } else {
        const double h_lower=pair.spec.distanceLimit-
            (distance+position_radius);
        const double hdot_lower=-radial-direction_radius*speed-
            pair.tube_velocity_radius_mps;
        const double speed_upper=speed+pair.tube_velocity_radius_mps;
        central=pair.spec.lambda1*pair.spec.lambda2*h_lower+
            (pair.spec.lambda1+pair.spec.lambda2)*hdot_lower-
            speed_upper*speed_upper/(distance-position_radius)-
            pair.spec.totalReserve;
    }
    const Eigen::Vector2d normal=pair.collision?unit_normal:-unit_normal;
    return {normal,central-reserve};
}

struct Task10p11aiChainInput {
    Task10p11aiPairSpec pair;
    NodeId pair_owner_a=0;
    NodeId pair_owner_b=0;
    Eigen::Vector2d rel_pos_0=Eigen::Vector2d::Zero();
    Eigen::Vector2d rel_vel_0=Eigen::Vector2d::Zero();
    std::array<double,4> tube_position_radius_m{};
    std::array<double,4> tube_velocity_radius_mps{};
    double dt_s=0.1;
    double half_box_mps2=4.0;
    double time_limit_s=3600.0;
};

// Per-beat relative-position coefficients on the x (same as y) coordinate.
struct Task10p11aiChainResult {
    bool built=false;
    std::string fail_reason;
    std::array<bool,4> sign_fixation{{false,false,false,false}};
    bool solved=false;
    int gurobi_status=-1;
    bool global_optimal=false;
    double incumbent_mps2=std::numeric_limits<double>::quiet_NaN();
    double bound_mps2=std::numeric_limits<double>::quiet_NaN();
    double mip_gap=std::numeric_limits<double>::quiet_NaN();
    double solve_time_s=0.0;
    std::array<Eigen::Vector2d,3> relative_controls{};
    double independent_formula_mps2=
        std::numeric_limits<double>::quiet_NaN();
    double canonical_builder_mps2=std::numeric_limits<double>::quiet_NaN();
    bool strict=false;
};

inline std::array<double,3> task10p11aiLayerPositionCoefficients(
    std::size_t layer,double dt) {
    if (layer==1) return {0.5*dt*dt,0.0,0.0};
    if (layer==2) return {1.5*dt*dt,0.5*dt*dt,0.0};
    return {2.5*dt*dt,1.5*dt*dt,0.5*dt*dt};
}

// Closed-form chain value at explicit relative controls (all four chain
// conditions: rows at layers 0..2 and the maximized successor support at
// layer 3).  Used by the solver, the runner, and the independent verifier.
inline double task10p11aiChainIndependentValue(
    const Task10p11aiChainInput& input,
    const std::array<Eigen::Vector2d,3>& controls) {
    const double dt=input.dt_s;
    const double half_box=input.half_box_mps2;
    Eigen::Vector2d pos=input.rel_pos_0;
    Eigen::Vector2d vel=input.rel_vel_0;
    Task10p11aiPairSpec layer0=input.pair;
    layer0.tube_position_radius_m=input.tube_position_radius_m[0];
    layer0.tube_velocity_radius_mps=input.tube_velocity_radius_mps[0];
    const auto mirror0=task10p11aiCoupledMirror(layer0,pos,vel,half_box);
    double minimum=mirror0.normal.dot(controls[0])+mirror0.constant;
    for (std::size_t beat=0;beat<3;++beat) {
        pos+=dt*vel+0.5*dt*dt*controls[beat];
        vel+=dt*controls[beat];
        Task10p11aiPairSpec layer=input.pair;
        layer.tube_position_radius_m=input.tube_position_radius_m[beat+1];
        layer.tube_velocity_radius_mps=input.tube_velocity_radius_mps[beat+1];
        const auto mirror=task10p11aiCoupledMirror(layer,pos,vel,half_box);
        minimum=std::min(minimum,mirror.normal.dot(controls[beat])+
            mirror.constant);
    }
    Task10p11aiPairSpec layer3=input.pair;
    layer3.tube_position_radius_m=input.tube_position_radius_m[3];
    layer3.tube_velocity_radius_mps=input.tube_velocity_radius_mps[3];
    const auto mirror3=task10p11aiCoupledMirror(layer3,pos,vel,half_box);
    const double support=input.pair.mobile_pair
        ?2.0*half_box*(std::abs(mirror3.normal.x())+
            std::abs(mirror3.normal.y()))
        :half_box*(std::abs(mirror3.normal.x())+
            std::abs(mirror3.normal.y()));
    minimum=std::min(minimum,support+mirror3.constant);
    return minimum;
}

#ifdef ENABLE_GUROBI

namespace task10p11ai_detail {

struct LayerBox {
    double xl=0.0,xu=0.0,yl=0.0,yu=0.0;
    double vxl=0.0,vxu=0.0,vyl=0.0,vyu=0.0;
    int sx=0,sy=0;
    double dl=0.0,du=0.0,sl=0.0,su=0.0;
};

inline LayerBox layerBox(const Task10p11aiChainInput& input,std::size_t layer,
                         const std::array<double,3>& pos_coeff) {
    const double dt=input.dt_s;
    const double domain=task10p11aiRelativeDomain(input.pair,
        input.half_box_mps2);
    LayerBox box;
    const double px=input.rel_pos_0.x()+static_cast<double>(layer)*dt*
        input.rel_vel_0.x();
    const double py=input.rel_pos_0.y()+static_cast<double>(layer)*dt*
        input.rel_vel_0.y();
    double px_span=0.0,py_span=0.0;
    for (std::size_t beat=0;beat<3;++beat) {
        px_span+=std::abs(pos_coeff[beat])*domain;
        py_span+=std::abs(pos_coeff[beat])*domain;
    }
    box.xl=px-px_span; box.xu=px+px_span;
    box.yl=py-py_span; box.yu=py+py_span;
    const double vel_span=static_cast<double>(layer)*dt*domain;
    box.vxl=input.rel_vel_0.x()-vel_span;
    box.vxu=input.rel_vel_0.x()+vel_span;
    box.vyl=input.rel_vel_0.y()-vel_span;
    box.vyu=input.rel_vel_0.y()+vel_span;
    auto strict_sign=[](double lo,double hi) {
        if (hi<0.0) return -1;
        if (lo>0.0) return 1;
        return 0;
    };
    box.sx=strict_sign(box.xl,box.xu);
    box.sy=strict_sign(box.yl,box.yu);
    if (box.sx==0||box.sy==0) return box;
    const double ax_min=std::min(std::abs(box.xl),std::abs(box.xu));
    const double ax_max=std::max(std::abs(box.xl),std::abs(box.xu));
    const double ay_min=std::min(std::abs(box.yl),std::abs(box.yu));
    const double ay_max=std::max(std::abs(box.yl),std::abs(box.yu));
    box.dl=std::hypot(ax_min,ay_min);
    box.du=std::hypot(ax_max,ay_max);
    const double svx_min=std::min(std::abs(box.vxl),std::abs(box.vxu));
    const double svx_max=std::max(std::abs(box.vxl),std::abs(box.vxu));
    const double svy_min=std::min(std::abs(box.vyl),std::abs(box.vyu));
    const double svy_max=std::max(std::abs(box.vyl),std::abs(box.vyu));
    box.sl=std::hypot(svx_min,svy_min);
    box.su=std::hypot(svx_max,svy_max);
    return box;
}

}  // namespace task10p11ai_detail

inline Task10p11aiChainResult solveTask10p11aiPairChainBound(
    const Task10p11aiChainInput& input) {
    Task10p11aiChainResult result;
    const double dt=input.dt_s;
    const double half_box=input.half_box_mps2;
    const double domain=task10p11aiRelativeDomain(input.pair,
        input.half_box_mps2);
    std::array<task10p11ai_detail::LayerBox,4> boxes;
    for (std::size_t layer=1;layer<=3;++layer) {
        boxes[layer]=task10p11ai_detail::layerBox(input,layer,
            task10p11aiLayerPositionCoefficients(layer,dt));
        result.sign_fixation[layer]=boxes[layer].sx!=0&&boxes[layer].sy!=0;
        if (!result.sign_fixation[layer]) {
            result.fail_reason="sign_fixation_failed_layer_"+
                std::to_string(layer);
            return result;
        }
        const double layer_position_radius=input.pair.spec.uncertainty+
            input.tube_position_radius_m[layer];
        if (boxes[layer].dl<=layer_position_radius||boxes[layer].dl<=0.0) {
            result.fail_reason="distance_interval_not_above_position_radius_"+
                std::to_string(layer);
            return result;
        }
    }
    try {
        GRBEnv env(true);
        env.set(GRB_IntParam_OutputFlag,0);
        env.set(GRB_IntParam_Seed,2027);
        env.set(GRB_IntParam_Threads,1);
        env.set(GRB_IntParam_NumericFocus,3);
        env.set(GRB_IntParam_NonConvex,2);
        env.set(GRB_DoubleParam_TimeLimit,input.time_limit_s);
        env.set(GRB_DoubleParam_FeasibilityTol,1.0e-9);
        env.set(GRB_DoubleParam_OptimalityTol,1.0e-9);
        env.set(GRB_DoubleParam_MIPGap,1.0e-10);
        env.start();
        GRBModel model(env);
        std::array<GRBVar,3> dux,duy;
        for (std::size_t beat=0;beat<3;++beat) {
            dux[beat]=model.addVar(-domain,domain,0,GRB_CONTINUOUS,
                "dux"+std::to_string(beat));
            duy[beat]=model.addVar(-domain,domain,0,GRB_CONTINUOUS,
                "duy"+std::to_string(beat));
        }
        GRBVar t=model.addVar(-1.0e3,1.0e3,0,GRB_CONTINUOUS,"t");
        {
            Task10p11aiPairSpec layer0=input.pair;
            layer0.tube_position_radius_m=input.tube_position_radius_m[0];
            layer0.tube_velocity_radius_mps=input.tube_velocity_radius_mps[0];
            const auto mirror=task10p11aiCoupledMirror(layer0,input.rel_pos_0,
                input.rel_vel_0,half_box);
            model.addConstr(mirror.normal.x()*dux[0]+mirror.normal.y()*duy[0]+
                mirror.constant>=t);
        }
        for (std::size_t layer=1;layer<=3;++layer) {
            const auto& box=boxes[layer];
            const auto coeff=task10p11aiLayerPositionCoefficients(layer,dt);
            const double base_x=input.rel_pos_0.x()+static_cast<double>(layer)*
                dt*input.rel_vel_0.x();
            const double base_y=input.rel_pos_0.y()+static_cast<double>(layer)*
                dt*input.rel_vel_0.y();
            GRBLinExpr pos_x=base_x;
            GRBLinExpr pos_y=base_y;
            GRBLinExpr vel_x=input.rel_vel_0.x();
            GRBLinExpr vel_y=input.rel_vel_0.y();
            for (std::size_t beat=0;beat<layer;++beat) {
                pos_x+=coeff[beat]*dux[beat];
                pos_y+=coeff[beat]*duy[beat];
                vel_x+=dt*dux[beat];
                vel_y+=dt*duy[beat];
            }
            const std::string suffix=std::to_string(layer);
            GRBVar px=model.addVar(box.xl,box.xu,0,GRB_CONTINUOUS,
                "px"+suffix);
            GRBVar py=model.addVar(box.yl,box.yu,0,GRB_CONTINUOUS,
                "py"+suffix);
            GRBVar pvx=model.addVar(box.vxl,box.vxu,0,GRB_CONTINUOUS,
                "pvx"+suffix);
            GRBVar pvy=model.addVar(box.vyl,box.vyu,0,GRB_CONTINUOUS,
                "pvy"+suffix);
            model.addConstr(px==pos_x);
            model.addConstr(py==pos_y);
            model.addConstr(pvx==vel_x);
            model.addConstr(pvy==vel_y);
            GRBVar d=model.addVar(box.dl,box.du,0,GRB_CONTINUOUS,"d"+suffix);
            GRBVar speed=model.addVar(box.sl,box.su,0,GRB_CONTINUOUS,
                "speed"+suffix);
            model.addQConstr(d*d==px*px+py*py);
            model.addQConstr(speed*speed==pvx*pvx+pvy*pvy);
            const double layer_position_radius=input.pair.spec.uncertainty+
                input.tube_position_radius_m[layer];
            const double q_lo=layer_position_radius/box.du;
            const double q_hi=layer_position_radius/box.dl;
            GRBVar q=model.addVar(q_lo,q_hi,0,GRB_CONTINUOUS,"q"+suffix);
            model.addQConstr(q*d==layer_position_radius);
            GRBVar cosine=model.addVar(
                std::sqrt(std::max(0.0,1.0-q_hi*q_hi)),
                std::sqrt(std::max(0.0,1.0-q_lo*q_lo)),0,GRB_CONTINUOUS,
                "cosine"+suffix);
            model.addQConstr(cosine*cosine+q*q==1.0);
            // The direction radius is increasing in q, so the smaller
            // bound comes from q_lo.
            const double dr_lo=task10p11aiDirectionRadius(q_lo);
            const double dr_hi=task10p11aiDirectionRadius(q_hi);
            GRBVar dir=model.addVar(dr_lo,dr_hi,0,GRB_CONTINUOUS,
                "dir"+suffix);
            model.addQConstr(dir*dir==2.0*(1.0-cosine));
            GRBVar radial=model.addVar(-box.su,box.su,0,GRB_CONTINUOUS,
                "radial"+suffix);
            model.addQConstr(radial*d==px*pvx+py*pvy);
            const double tube_v=input.tube_velocity_radius_mps[layer];
            GRBVar ds=model.addVar(dr_lo*box.sl,dr_hi*box.su,0,GRB_CONTINUOUS,
                "ds"+suffix);
            model.addQConstr(ds==dir*speed);
            GRBLinExpr constant=0.0;
            if (input.pair.collision) {
                constant=input.pair.spec.lambda1*input.pair.spec.lambda2*
                    (d-layer_position_radius-input.pair.spec.distanceLimit)+
                    (input.pair.spec.lambda1+input.pair.spec.lambda2)*
                    (radial-ds-tube_v)-input.pair.spec.totalReserve-
                    half_box*std::sqrt(2.0)*dir;
            } else {
                const double ratio_lo=(box.sl+tube_v)*(box.sl+tube_v)/
                    (box.du-layer_position_radius);
                const double ratio_hi=(box.su+tube_v)*(box.su+tube_v)/
                    (box.dl-layer_position_radius);
                GRBVar ratio=model.addVar(ratio_lo,ratio_hi,0,GRB_CONTINUOUS,
                    "ratio"+suffix);
                model.addQConstr(ratio*(d-layer_position_radius)==
                    (speed+tube_v)*(speed+tube_v));
                constant=input.pair.spec.lambda1*input.pair.spec.lambda2*
                    (input.pair.spec.distanceLimit-d-layer_position_radius)+
                    (input.pair.spec.lambda1+input.pair.spec.lambda2)*
                    (-radial-ds-tube_v)-ratio-input.pair.spec.totalReserve-
                    half_box*std::sqrt(2.0)*dir;
            }
            const double kind_sign=input.pair.collision?1.0:-1.0;
            if (layer<=2) {
                const double nx_lo=std::min(std::abs(box.xl),
                    std::abs(box.xu))/box.du;
                const double nx_hi=std::max(std::abs(box.xl),
                    std::abs(box.xu))/box.dl;
                const double ny_lo=std::min(std::abs(box.yl),
                    std::abs(box.yu))/box.du;
                const double ny_hi=std::max(std::abs(box.yl),
                    std::abs(box.yu))/box.dl;
                GRBVar nx=model.addVar(nx_lo,nx_hi,0,GRB_CONTINUOUS,
                    "nx"+suffix);
                GRBVar ny=model.addVar(ny_lo,ny_hi,0,GRB_CONTINUOUS,
                    "ny"+suffix);
                model.addQConstr(nx*d==static_cast<double>(box.sx)*px);
                model.addQConstr(ny*d==static_cast<double>(box.sy)*py);
                model.addQConstr(kind_sign*static_cast<double>(box.sx)*nx*
                    dux[layer-1]+kind_sign*static_cast<double>(box.sy)*ny*
                    duy[layer-1]+constant>=t);
            } else {
                GRBVar support=model.addVar(0.0,domain*std::sqrt(2.0),0,
                    GRB_CONTINUOUS,"support");
                model.addQConstr(support*d==static_cast<double>(domain)*
                    (static_cast<double>(box.sx)*px+
                     static_cast<double>(box.sy)*py));
                model.addConstr(support+constant>=t);
            }
        }
        model.setObjective(GRBLinExpr(t),GRB_MAXIMIZE);
        model.optimize();
        result.gurobi_status=model.get(GRB_IntAttr_Status);
        if (const char* dump=std::getenv("TASK10P11AI_DUMP_LP");
            dump!=nullptr&&result.gurobi_status==GRB_INFEASIBLE)
            model.write(std::string(dump)+"/chain-infeasible.lp");
        result.solved=model.get(GRB_IntAttr_SolCount)>0;
        result.solve_time_s=model.get(GRB_DoubleAttr_Runtime);
        result.global_optimal=result.gurobi_status==GRB_OPTIMAL;
        try {
            result.bound_mps2=model.get(GRB_DoubleAttr_ObjBound);
        } catch (const GRBException&) {
            // ObjBound can be unavailable for some terminal statuses; the
            // status field records what happened.
        }
        if (result.solved) {
            result.incumbent_mps2=model.get(GRB_DoubleAttr_ObjVal);
            try {
                result.mip_gap=model.get(GRB_DoubleAttr_MIPGap);
            } catch (const GRBException&) {
            }
            for (std::size_t beat=0;beat<3;++beat)
                result.relative_controls[beat]=Eigen::Vector2d(
                    dux[beat].get(GRB_DoubleAttr_X),
                    duy[beat].get(GRB_DoubleAttr_X));
            result.independent_formula_mps2=task10p11aiChainIndependentValue(
                input,result.relative_controls);
        }
    } catch (const GRBException& error) {
        result.fail_reason="gurobi_error_"+std::to_string(
            error.getErrorCode())+":"+error.getMessage();
        return result;
    } catch (const std::exception& error) {
        result.fail_reason=std::string("error:")+error.what();
        return result;
    }
    result.built=true;
    result.strict=result.global_optimal&&result.solved&&
        result.bound_mps2<task10p11aiProtocol().strict_negative_bound_mps2&&
        std::isfinite(result.independent_formula_mps2)&&
        std::abs(result.incumbent_mps2-result.independent_formula_mps2)<=
            task10p11aiProtocol().independent_agreement_mps2;
    return result;
}

#else

inline Task10p11aiChainResult solveTask10p11aiPairChainBound(
    const Task10p11aiChainInput&) {
    Task10p11aiChainResult result;
    result.fail_reason="gurobi_not_enabled";
    return result;
}

#endif

struct Task10p11aiLimitingRow {
    std::string row_id;
    bool pair_row=false;
    bool coupled_mobile_pair=false;
    bool collision=false;
    bool reference=false;
    NodeId first=0;
    NodeId second=0;
    NodeId owner=0;
    NodeId peer=0;
    bool second_fixed=false;
    double successor_minimum_residual_mps2=
        std::numeric_limits<double>::quiet_NaN();
};

inline void task10p11aiParseRowId(Task10p11aiLimitingRow& row,
                                  const std::string& limiting) {
    row.row_id=limiting;
    std::string base=limiting;
    const std::string coupled_suffix=":full-pair-once-reserve";
    if (base.size()>coupled_suffix.size()&&base.substr(
            base.size()-coupled_suffix.size())==coupled_suffix) {
        base=base.substr(0,base.size()-coupled_suffix.size());
        row.coupled_mobile_pair=true;
        row.pair_row=true;
    }
    const std::string owner_suffix=":owner:";
    auto owner_position=base.rfind(owner_suffix);
    if (!row.coupled_mobile_pair&&owner_position!=std::string::npos) {
        base=base.substr(0,owner_position);
        row.pair_row=true;
    }
    if (base.rfind("collision:",0)==0) {
        row.collision=true;
        const std::string pair=base.substr(10);
        const auto separator=pair.find("--");
        const NodeId a=static_cast<NodeId>(std::stoull(
            pair.substr(0,separator)));
        const NodeId b=static_cast<NodeId>(std::stoull(
            pair.substr(separator+2)));
        row.first=std::min(a,b);
        row.second=std::max(a,b);
        row.owner=0;
        row.peer=0;
    } else if (base.rfind("reference:",0)==0) {
        row.reference=true;
        // DirectedEdge::id() is "reference->owner".
        const std::string pair=base.substr(10);
        const auto separator=pair.find("->");
        row.first=static_cast<NodeId>(std::stoull(pair.substr(0,separator)));
        row.second=static_cast<NodeId>(std::stoull(pair.substr(separator+2)));
        row.owner=row.second;
        row.peer=row.first;
    } else {
        row.pair_row=false;
    }
}

// Replays the frozen Task 10.11ah best plan through the established evaluator
// machinery and identifies the limiting terminal-successor row by independent
// residual recomputation at the successor margin optimum.
inline Task10p11aiLimitingRow task10p11aiIdentifyLimitingRow(
    const Task10p11rFixedBaselineFixture& source,
    const Eigen::Matrix<double,8,1>& plan_vector) {
    const auto plan=task10p11ahPlanFromVector(plan_vector);
    auto fixture=task10p11ah_optimizer_detail::cloneFixture(source,
        std::nullopt);
    auto frame0=task10p11ah_optimizer_detail::nativeFrame(*fixture,
        std::nullopt);
    auto active_pair=frame0.prepared.active_pair;
    const auto selected_u0=task10p11ah_optimizer_detail::
        projectedComponentControls(frame0.boundary.request,
            frame0.prepared.control.step.applied_controls,plan.owner2_u0,
            plan.owner9_u0);
    const auto successor0=task10p11af_detail::successorFullPair(
        frame0.snapshot,selected_u0);
    if (!successor0.feasible)
        throw std::runtime_error(
            "limiting-row replay x0 successor infeasible");
    const auto applied0=task10p11ah_optimizer_detail::applyVerified(*fixture,
        frame0,selected_u0,successor0.minimum_residual);
    if (!applied0.step.advanced)
        throw std::runtime_error("limiting-row replay x0 advance failed");
    auto frame1=task10p11ah_optimizer_detail::nativeFrame(*fixture,
        active_pair);
    active_pair=frame1.prepared.active_pair;
    const auto selected_u1=task10p11ah_optimizer_detail::
        projectedComponentControls(frame1.boundary.request,
            frame1.prepared.control.step.applied_controls,plan.owner2_u1,
            plan.owner9_u1);
    const auto successor1=task10p11af_detail::successorFullPair(
        frame1.snapshot,selected_u1);
    if (!successor1.feasible)
        throw std::runtime_error(
            "limiting-row replay x1 successor infeasible");
    const auto applied1=task10p11ah_optimizer_detail::applyVerified(*fixture,
        frame1,selected_u1,successor1.minimum_residual);
    if (!applied1.step.advanced)
        throw std::runtime_error("limiting-row replay x1 advance failed");
    auto frame2=task10p11ah_optimizer_detail::nativeFrame(*fixture,
        active_pair);
    const auto distributed_u2=frame2.prepared.control.step.applied_controls;
    const auto successor2=task10p11af_detail::successorFullPair(
        frame2.snapshot,distributed_u2);
    if (successor2.feasible)
        throw std::runtime_error(
            "frozen plan terminal successor unexpectedly feasible");
    const auto estimate0=task10p11s_capture_detail::estimateFromJson(
        frame2.snapshot.at("estimator"));
    const auto estimate1=task10p11aa_detail::predictEstimate(frame2.snapshot,
        estimate0,distributed_u2);
    const auto request1=task10p11x_detail::requestAtEstimate(frame2.snapshot,
        estimate1);
    const auto audit=task10p11aa_detail::fullStateAudit(request1,
        task10p11sOrderedControls(request1.mobile_ids,distributed_u2));
    if (!audit.margin.feasible)
        throw std::runtime_error(
            "limiting-row successor margin solve failed");
    std::string limiting;
    task10p11af_detail::independentMinimumResidual(audit.problem,
        audit.margin.controls,&limiting);
    Task10p11aiLimitingRow row;
    task10p11aiParseRowId(row,limiting);
    row.successor_minimum_residual_mps2=successor2.minimum_residual;
    return row;
}

// ---------------------------------------------------------------------------
// Level A3: interval sup audit over the full row set.  Evidence-only per the
// preregistration: it never upgrades the classification.  Every residual is
// evaluated with interval arithmetic over the free-control reachable box, so
// each reported sup is a rigorous upper bound of that row residual.
// ---------------------------------------------------------------------------

struct Task10p11aiInterval {
    double lo=0.0;
    double hi=0.0;

    Task10p11aiInterval()=default;
    Task10p11aiInterval(double value):lo(value),hi(value) {}
    Task10p11aiInterval(double lower,double upper):lo(lower),hi(upper) {}
};

inline Task10p11aiInterval operator-(const Task10p11aiInterval& a) {
    return {-a.hi,-a.lo};
}

inline Task10p11aiInterval operator+(const Task10p11aiInterval& a,
                                     const Task10p11aiInterval& b) {
    return {a.lo+b.lo,a.hi+b.hi};
}

inline Task10p11aiInterval operator-(const Task10p11aiInterval& a,
                                     const Task10p11aiInterval& b) {
    return {a.lo-b.hi,a.hi-b.lo};
}

inline Task10p11aiInterval operator*(const Task10p11aiInterval& a,
                                     const Task10p11aiInterval& b) {
    const std::array<double,4> products{a.lo*b.lo,a.lo*b.hi,a.hi*b.lo,
        a.hi*b.hi};
    return {std::min({products[0],products[1],products[2],products[3]}),
        std::max({products[0],products[1],products[2],products[3]})};
}

inline Task10p11aiInterval operator*(double scalar,
                                     const Task10p11aiInterval& b) {
    return scalar>=0.0?Task10p11aiInterval{scalar*b.lo,scalar*b.hi}
        :Task10p11aiInterval{scalar*b.hi,scalar*b.lo};
}

inline Task10p11aiInterval operator+(double scalar,
                                     const Task10p11aiInterval& b) {
    return {scalar+b.lo,scalar+b.hi};
}

inline Task10p11aiInterval operator+(const Task10p11aiInterval& a,
                                     double scalar) {
    return {a.lo+scalar,a.hi+scalar};
}

inline Task10p11aiInterval operator-(const Task10p11aiInterval& a,
                                     double scalar) {
    return {a.lo-scalar,a.hi-scalar};
}

inline Task10p11aiInterval operator/(const Task10p11aiInterval& a,
                                     const Task10p11aiInterval& b) {
    if (b.lo<=0.0&&b.hi>=0.0)
        throw std::invalid_argument("interval division crosses zero");
    if (b.lo>0.0) return {a.lo/b.hi,a.hi/b.lo};
    return {a.hi/b.lo,a.lo/b.hi};
}

inline Task10p11aiInterval task10p11aiSqrt(const Task10p11aiInterval& a) {
    if (a.hi<0.0) throw std::invalid_argument("interval sqrt of negative");
    return {std::sqrt(std::max(0.0,a.lo)),std::sqrt(a.hi)};
}

inline Task10p11aiInterval task10p11aiAbs(const Task10p11aiInterval& a) {
    if (a.lo>=0.0) return a;
    if (a.hi<=0.0) return -a;
    return {0.0,std::max(-a.lo,a.hi)};
}

// The norm is monotone in |x| and |y|, so both bounds are attained at the
// extreme coordinate magnitudes.
inline Task10p11aiInterval task10p11aiNorm(const Task10p11aiInterval& x,
                                           const Task10p11aiInterval& y) {
    const auto ax=task10p11aiAbs(x);
    const auto ay=task10p11aiAbs(y);
    return {std::hypot(ax.lo,ay.lo),std::hypot(ax.hi,ay.hi)};
}

// Interval state of one owner (position and velocity boxes).
struct Task10p11aiOwnerInterval {
    Task10p11aiInterval px,py,vx,vy;
};

struct Task10p11aiCoupledMirrorInterval {
    Task10p11aiInterval normal_x,normal_y,constant;
};

inline Task10p11aiCoupledMirrorInterval task10p11aiCoupledMirrorInterval(
    const Task10p11aiPairSpec& pair,const Task10p11aiOwnerInterval& owner,
    const Task10p11aiOwnerInterval& peer,double half_box) {
    const Task10p11aiInterval rx=owner.px-peer.px;
    const Task10p11aiInterval ry=owner.py-peer.py;
    const Task10p11aiInterval vx=owner.vx-peer.vx;
    const Task10p11aiInterval vy=owner.vy-peer.vy;
    const Task10p11aiInterval distance=task10p11aiNorm(rx,ry);
    const double position_radius=task10p11aiPositionRadius(pair);
    if (!(distance.lo>position_radius)||!(distance.lo>0.0))
        throw std::invalid_argument(
            "task10p11ai interval mirror radial singularity");
    const Task10p11aiInterval ux=rx/distance;
    const Task10p11aiInterval uy=ry/distance;
    const Task10p11aiInterval q=position_radius/distance;
    if (!(q.hi<1.0))
        throw std::invalid_argument(
            "task10p11ai interval direction radius undefined");
    const Task10p11aiInterval direction_radius=task10p11aiSqrt(
        2.0*(1.0-task10p11aiSqrt(1.0-(q*q))));
    const Task10p11aiInterval radial=ux*vx+uy*vy;
    const Task10p11aiInterval speed=task10p11aiNorm(vx,vy);
    const Task10p11aiInterval reserve=half_box*std::sqrt(2.0)*
        direction_radius;
    Task10p11aiInterval central;
    if (pair.collision) {
        const auto h_lower=distance-(position_radius+
            pair.spec.distanceLimit);
        const auto hdot_lower=radial-(direction_radius*speed)-
            pair.tube_velocity_radius_mps;
        central=pair.spec.lambda1*pair.spec.lambda2*h_lower+
            (pair.spec.lambda1+pair.spec.lambda2)*hdot_lower-
            pair.spec.totalReserve;
    } else {
        const auto h_lower=pair.spec.distanceLimit-(distance+
            position_radius);
        const auto hdot_lower=0.0-(radial+direction_radius*speed)-
            pair.tube_velocity_radius_mps;
        const auto speed_upper=speed+pair.tube_velocity_radius_mps;
        central=pair.spec.lambda1*pair.spec.lambda2*h_lower+
            (pair.spec.lambda1+pair.spec.lambda2)*hdot_lower-
            (speed_upper*speed_upper)/(distance-position_radius)-
            pair.spec.totalReserve;
    }
    const Task10p11aiInterval signed_x=pair.collision?ux:-ux;
    const Task10p11aiInterval signed_y=pair.collision?uy:-uy;
    return {signed_x,signed_y,central-reserve};
}

// Rigorous sup of the coupled row residual over the free relative-control box.
inline double task10p11aiCoupledRowSup(
    const Task10p11aiCoupledMirrorInterval& mirror,double domain) {
    const double normal_x=task10p11aiAbs(mirror.normal_x).hi;
    const double normal_y=task10p11aiAbs(mirror.normal_y).hi;
    return domain*(normal_x+normal_y)+mirror.constant.hi;
}

// Rigour note: the plant-speed facet normals rotate with the velocity
// orientation.  Treating the orientation as free over the full circle is a
// conservative but exact-in-direction sup: for any fixed orientation the
// maximum of -normal.u over the control box is bounded by the box norm, and
// the bound is attained for the aligned orientation.
inline double task10p11aiPlantSpeedRowSup(double apothem,double tube_v,
                                          double dt,double speed_limit_box_u,
                                          double speed_limit_box_v) {
    const double control_norm=speed_limit_box_u;
    const double velocity_norm=speed_limit_box_v;
    return control_norm+(apothem+velocity_norm-tube_v)/dt;
}

struct Task10p11aiLayerAudit {
    bool valid=false;
    std::string fail_reason;
    std::size_t row_count=0;
    double minimum_sup_mps2=std::numeric_limits<double>::infinity();
    std::string limiting_row_id;
};

namespace task10p11ai_detail {

inline double boxNormUpper(const Task10p11aiInterval& x,
                           const Task10p11aiInterval& y) {
    return std::hypot(task10p11aiAbs(x).hi,task10p11aiAbs(y).hi);
}

}  // namespace task10p11ai_detail

// Audits one layer.  The row enumeration mirrors buildCanonicalHardRows and
// the 28D coupling rule (one coupled row per mobile-mobile reference or
// collision pair), so the audited row set equals the canonical 1113-row set.
inline Task10p11aiLayerAudit task10p11aiAuditLayer(
    const CanonicalHardRowRequest& request,
    const std::map<NodeId,Task10p11aiOwnerInterval>& intervals,
    double half_box,double dt) {
    Task10p11aiLayerAudit audit;
    auto consider=[&](const std::string& id,double sup) {
        ++audit.row_count;
        if (!std::isfinite(sup))
            throw std::invalid_argument("non-finite row sup for "+id);
        if (sup<audit.minimum_sup_mps2) {
            audit.minimum_sup_mps2=sup;
            audit.limiting_row_id=id;
        }
    };
    try {
        const double u_norm=half_box*std::sqrt(2.0);
        for (const auto& facet:request.workspace_facets) {
            const double norm=facet.outward_normal.norm();
            const Eigen::Vector2d normal=facet.outward_normal/norm;
            const double offset=facet.offset_m/norm;
            for (NodeId owner:request.mobile_ids) {
                const auto& state=intervals.at(owner);
                const auto tube=request.workspace_snapshot_tubes.at(owner);
                const Task10p11aiInterval nx=normal.x(),ny=normal.y();
                const auto h=offset-(nx*state.px+ny*state.py)-
                    tube.position_radius_m;
                const auto hdot=0.0-(nx*state.vx+ny*state.vy)-
                    tube.velocity_radius_mps;
                const auto constant=h+2.0*hdot;
                const double sup=u_norm+constant.hi;
                consider("workspace:"+std::to_string(owner)+":"+facet.id,sup);
            }
        }
        std::set<std::string> seen_edges;
        for (const auto& edge:request.reference_edges) {
            const std::string id=edge.id();
            if (!seen_edges.insert(id).second) continue;
            const bool mobile_peer=request.states.count(edge.reference)!=0&&
                std::find(request.mobile_ids.begin(),request.mobile_ids.end(),
                    edge.reference)!=request.mobile_ids.end();
            Task10p11aiPairSpec pair;
            pair.collision=false;
            pair.mobile_pair=mobile_peer;
            pair.spec=request.reference_spec;
            const auto tube=request.reference_snapshot_tubes.at(id);
            pair.tube_position_radius_m=tube.position_radius_m;
            pair.tube_velocity_radius_mps=tube.velocity_radius_mps;
            const std::string base="reference:"+id;
            const auto& owner=intervals.at(edge.owner);
            if (mobile_peer) {
                const auto mirror=task10p11aiCoupledMirrorInterval(pair,
                    owner,intervals.at(edge.reference),half_box);
                consider(base+":full-pair-once-reserve",
                    task10p11aiCoupledRowSup(mirror,2.0*half_box));
            } else {
                const auto mirror=task10p11aiCoupledMirrorInterval(pair,
                    owner,intervals.at(edge.reference),half_box);
                consider(base+":owner:"+std::to_string(edge.owner),
                    task10p11aiCoupledRowSup(mirror,half_box));
            }
        }
        for (const auto& raw_edge:request.collision_pairs) {
            const auto edge=UndirectedEdge::canonical(raw_edge.first,
                raw_edge.second);
            const std::string id=edge.id();
            if (!seen_edges.insert("c"+id).second) continue;
            const bool first_mobile=std::find(request.mobile_ids.begin(),
                request.mobile_ids.end(),edge.first)!=request.mobile_ids.end();
            const bool second_mobile=std::find(request.mobile_ids.begin(),
                request.mobile_ids.end(),edge.second)!=request.mobile_ids.end();
            Task10p11aiPairSpec pair;
            pair.collision=true;
            pair.mobile_pair=first_mobile&&second_mobile;
            pair.spec=request.collision_spec;
            const auto tube=request.collision_snapshot_tubes.at(id);
            pair.tube_position_radius_m=tube.position_radius_m;
            pair.tube_velocity_radius_mps=tube.velocity_radius_mps;
            const std::string base="collision:"+id;
            const auto mirror=task10p11aiCoupledMirrorInterval(pair,
                intervals.at(edge.first),intervals.at(edge.second),half_box);
            if (pair.mobile_pair)
                consider(base+":full-pair-once-reserve",
                    task10p11aiCoupledRowSup(mirror,2.0*half_box));
            else
                consider(base+":owner:"+std::to_string(first_mobile?
                    edge.first:edge.second),
                    task10p11aiCoupledRowSup(mirror,half_box));
        }
        if (request.speed_limit_mps>0.0&&request.plant_speed_facet_count>0) {
            const double half_vertex_angle=std::acos(-1.0)/
                static_cast<double>(request.plant_speed_facet_count);
            const double apothem=request.speed_limit_mps*
                std::cos(half_vertex_angle);
            for (NodeId owner:request.mobile_ids) {
                const auto& state=intervals.at(owner);
                const auto& tube=request.plant_speed_snapshot_tubes.at(owner);
                const double sup=task10p11aiPlantSpeedRowSup(apothem,
                    tube.velocity_radius_mps,request.plant_speed_dt_s,
                    u_norm,task10p11ai_detail::boxNormUpper(state.vx,
                        state.vy));
                for (std::size_t index=0;index<request.plant_speed_facet_count;
                     ++index)
                    consider("plant_speed_applied_control:"+
                        std::to_string(owner)+":facet:"+std::to_string(index),
                        sup);
            }
        }
        for (NodeId owner:request.mobile_ids) {
            consider("input:"+std::to_string(owner)+":ax:lower",
                half_box+half_box);
            consider("input:"+std::to_string(owner)+":ax:upper",
                half_box+half_box);
            consider("input:"+std::to_string(owner)+":ay:lower",
                half_box+half_box);
            consider("input:"+std::to_string(owner)+":ay:upper",
                half_box+half_box);
        }
        audit.valid=true;
    } catch (const std::exception& error) {
        audit.valid=false;
        audit.fail_reason=error.what();
    }
    (void)dt;
    return audit;
}

// Owner interval boxes at layer `layer` (0 = frozen points) under the fully
// free per-owner control box used by the relaxed audit.
inline std::map<NodeId,Task10p11aiOwnerInterval>
task10p11aiLayerIntervals(const CanonicalHardRowRequest& request,
                          std::size_t layer,double half_box,double dt) {
    std::map<NodeId,Task10p11aiOwnerInterval> result;
    const double position_span_coefficient=layer==0?0.0:
        (layer==1?0.5:(layer==2?2.0:4.5))*dt*dt*half_box;
    const double velocity_span=static_cast<double>(layer)*dt*half_box;
    for (NodeId owner:request.mobile_ids) {
        const auto& state=request.states.at(owner);
        Task10p11aiOwnerInterval box;
        if (layer==0) {
            box.px=Task10p11aiInterval(state.position.x);
            box.py=Task10p11aiInterval(state.position.y);
            box.vx=Task10p11aiInterval(state.velocity.x());
            box.vy=Task10p11aiInterval(state.velocity.y());
        } else {
            const double px=state.position.x+static_cast<double>(layer)*dt*
                state.velocity.x();
            const double py=state.position.y+static_cast<double>(layer)*dt*
                state.velocity.y();
            box.px={px-position_span_coefficient,
                px+position_span_coefficient};
            box.py={py-position_span_coefficient,
                py+position_span_coefficient};
            box.vx={state.velocity.x()-velocity_span,
                state.velocity.x()+velocity_span};
            box.vy={state.velocity.y()-velocity_span,
                state.velocity.y()+velocity_span};
        }
        result.emplace(owner,box);
    }
    for (NodeId fixed:request.fixed_ids) {
        const auto& state=request.states.at(fixed);
        Task10p11aiOwnerInterval box;
        box.px=Task10p11aiInterval(state.position.x);
        box.py=Task10p11aiInterval(state.position.y);
        box.vx=Task10p11aiInterval(0.0);
        box.vy=Task10p11aiInterval(0.0);
        result.emplace(fixed,box);
    }
    return result;
}

struct Task10p11aiIntervalAuditResult {
    std::array<Task10p11aiLayerAudit,4> layers;
};

struct Task10p11aiCanonicalChainCheck {
    bool valid=false;
    std::string fail_reason;
    double value_mps2=std::numeric_limits<double>::quiet_NaN();
};

// Canonical-builder recomputation of the chain value at explicit relative
// controls: rows are rebuilt from the restored fixture snapshot at every
// layer through predictEstimate/requestAtEstimate, mirroring the established
// 10.11ag verifier pattern (closed formula vs Gurobi vs canonical builder).
inline Task10p11aiCanonicalChainCheck task10p11aiCanonicalChainValue(
    const Task10p11rFixedBaselineFixture& source,
    const Task10p11aiChainInput& input,
    const std::array<Eigen::Vector2d,3>& controls) {
    Task10p11aiCanonicalChainCheck check;
    try {
        auto fixture=task10p11ah_optimizer_detail::cloneFixture(source,
            std::nullopt);
        auto frame=task10p11ah_optimizer_detail::nativeFrame(*fixture,
            std::nullopt);
        const auto& request0=frame.boundary.request;
        const double half_box=input.half_box_mps2;
        const double domain=task10p11aiRelativeDomain(input.pair,half_box);
        std::map<NodeId,Eigen::Vector2d> controls_map=
            frame.prepared.control.step.applied_controls;
        const auto half=domain/2.0;
        const Eigen::Vector2d delta0=0.5*controls[0];
        const Eigen::Vector2d delta1=0.5*controls[1];
        const Eigen::Vector2d delta2=0.5*controls[2];
        if (input.pair.mobile_pair) {
            controls_map[input.pair_owner_a]=delta0;
            controls_map[input.pair_owner_b]=-delta0;
        } else {
            controls_map[input.pair_owner_a]=controls[0];
        }
        double minimum=std::numeric_limits<double>::infinity();
        auto layer_residual=[&](const CanonicalHardRowRequest& request,
                                const Eigen::Vector2d& delta,
                                bool support_layer) {
            const auto problem=buildTask10p11sRows28d(
                buildCanonicalHardRows(request),request.mobile_ids,true);
            std::map<NodeId,Eigen::Vector2d> layer_controls=controls_map;
            if (input.pair.mobile_pair) {
                layer_controls[input.pair_owner_a]=0.5*delta;
                layer_controls[input.pair_owner_b]=-0.5*delta;
            } else {
                layer_controls[input.pair_owner_a]=delta;
            }
            const Eigen::VectorXd ordered=task10p11sOrderedControls(
                request.mobile_ids,layer_controls);
            const std::string pair_id=input.pair.collision?
                "collision:"+std::to_string(input.pair_owner_a)+"--"+
                    std::to_string(input.pair_owner_b):
                "reference:"+std::to_string(input.pair_owner_a)+"->"+
                    std::to_string(input.pair_owner_b);
            const std::string row_id=pair_id+
                (input.pair.mobile_pair?":full-pair-once-reserve":
                    ":owner:"+std::to_string(input.pair_owner_a));
            const Task10p11sRow28d* row=nullptr;
            for (const auto& candidate:problem.rows)
                if (candidate.id==row_id) row=&candidate;
            if (row==nullptr)
                throw std::runtime_error("chain canonical row absent:"+row_id);
            if (!support_layer) return row->residual(ordered);
            const Eigen::Vector2d coefficient=
                row->coefficient.segment<2>(2*static_cast<Eigen::Index>(
                    std::find(request.mobile_ids.begin(),
                        request.mobile_ids.end(),input.pair_owner_a)-
                    request.mobile_ids.begin()));
            return row->constant+domain*(std::abs(coefficient.x())+
                std::abs(coefficient.y()));
        };
        minimum=std::min(minimum,layer_residual(request0,controls[0],false));
        auto estimate=task10p11s_capture_detail::estimateFromJson(
            frame.snapshot.at("estimator"));
        for (std::size_t layer=1;layer<=3;++layer) {
            const Eigen::Vector2d delta=
                layer==1?controls[0]:(layer==2?controls[1]:controls[2]);
            estimate=task10p11aa_detail::predictEstimate(frame.snapshot,
                estimate,controls_map);
            const auto request=task10p11x_detail::requestAtEstimate(
                frame.snapshot,estimate);
            minimum=std::min(minimum,layer_residual(request,delta,layer==3));
            if (input.pair.mobile_pair) {
                controls_map[input.pair_owner_a]=
                    layer==1?delta1:(layer==2?delta2:delta2);
                controls_map[input.pair_owner_b]=
                    -controls_map[input.pair_owner_a];
            } else {
                controls_map[input.pair_owner_a]=
                    layer==1?delta1:(layer==2?delta2:delta2);
            }
        }
        check.value_mps2=minimum;
        check.valid=true;
    } catch (const std::exception& error) {
        check.fail_reason=error.what();
    }
    return check;
}

inline Task10p11aiIntervalAuditResult runTask10p11aiIntervalAudit(
    const nlohmann::json& snapshot,double half_box) {
    Task10p11aiIntervalAuditResult result;
    const auto base_request=task10p11s_capture_detail::requestFromJson(
        snapshot.at("canonical_request"));
    const auto estimate0=task10p11s_capture_detail::estimateFromJson(
        snapshot.at("estimator"));
    std::map<NodeId,Eigen::Vector2d> zero;
    for (NodeId owner:base_request.mobile_ids)
        zero.emplace(owner,Eigen::Vector2d::Zero());
    std::array<JointEstimateSnapshot,4> estimates{estimate0,{},{},{}};
    for (std::size_t layer=1;layer<=3;++layer)
        estimates[layer]=task10p11aa_detail::predictEstimate(snapshot,
            estimates[layer-1],zero);
    std::array<CanonicalHardRowRequest,4> requests;
    requests[0]=task10p11x_detail::requestAtEstimate(snapshot,estimates[0]);
    for (std::size_t layer=1;layer<=3;++layer)
        requests[layer]=task10p11x_detail::requestAtEstimate(snapshot,
            estimates[layer]);
    for (std::size_t layer=0;layer<=3;++layer) {
        auto intervals=task10p11aiLayerIntervals(base_request,layer,half_box,
            snapshot.at("successor_parameters").at("dt_s").get<double>());
        result.layers[layer]=task10p11aiAuditLayer(requests[layer],intervals,
            half_box,snapshot.at("successor_parameters").at("dt_s").get<
            double>());
    }
    return result;
}

}  // namespace gf
