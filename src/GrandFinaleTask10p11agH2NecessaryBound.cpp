#include "grand_finale/Task10p11agFullDomainPredecessor.hpp"

#include <iostream>

namespace {

using json=nlohmann::json;

double exactUpper(const Eigen::Vector2d& a0,const Eigen::Vector2d& a1,
    const Eigen::Vector2d& r0,const Eigen::Vector2d& v0,double dt,
    double position_radius,double velocity_radius,
    const PairwiseSecondOrderRowSpec& spec,double half_box) {
    const Eigen::Vector2d r2=r0+2.0*dt*v0+
        1.5*dt*dt*a0+0.5*dt*dt*a1;
    const Eigen::Vector2d v2=v0+dt*(a0+a1);
    const double distance=r2.norm();
    const Eigen::Vector2d normal=r2/distance;
    const double direction=std::sqrt(2.0*(1.0-std::sqrt(
        1.0-std::pow(position_radius/distance,2.0))));
    const double radial=normal.dot(v2);
    const double central=spec.lambda1*spec.lambda2*
        (distance-position_radius-spec.distanceLimit)+
        (spec.lambda1+spec.lambda2)*(radial-
            direction*v2.norm()-velocity_radius)-spec.totalReserve;
    return 2.0*half_box*normal.cwiseAbs().sum()+central-
        half_box*std::sqrt(2.0)*direction;
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=3 && argc!=4) return 2;
#ifndef ENABLE_GUROBI
    (void)argv;
    return 4;
#else
    try {
        const auto snapshot=gf::readTask10p11vJson(argv[1]);
        const auto request=gf::task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        const auto estimate0=gf::task10p11s_capture_detail::estimateFromJson(
            snapshot.at("estimator"));
        std::map<gf::NodeId,Eigen::Vector2d> zero;
        for (gf::NodeId owner:request.mobile_ids)
            zero.emplace(owner,Eigen::Vector2d::Zero());
        const auto estimate1=gf::task10p11aa_detail::predictEstimate(
            snapshot,estimate0,zero);
        const auto estimate2=gf::task10p11aa_detail::predictEstimate(
            snapshot,estimate1,zero);
        const auto request2=gf::task10p11x_detail::requestAtEstimate(
            snapshot,estimate2);
        const auto tube=request2.collision_snapshot_tubes.at("2--9");
        const auto first=request.states.at(2);
        const auto second=request.states.at(9);
        const Eigen::Vector2d r0(first.position.x-second.position.x,
                                  first.position.y-second.position.y);
        const Eigen::Vector2d v0=first.velocity-second.velocity;
        const double dt=snapshot.at("successor_parameters").at("dt_s");
        const double half_box=request.acceleration_half_box;
        const double limit=argc==4?std::stod(argv[3]):43200.0;
        const Eigen::Vector2d center=r0+2.0*dt*v0;
        const double position_delta=4.0*dt*dt*half_box;
        const Eigen::Vector2d velocity_center=v0;
        const double velocity_delta=4.0*dt*half_box;
        const double xl=center.x()-position_delta;
        const double xu=center.x()+position_delta;
        const double yl=center.y()-position_delta;
        const double yu=center.y()+position_delta;
        if (!(xu<0.0 && yl>0.0))
            throw std::runtime_error("H2 pair normal signs are not fixed");
        const double dl=std::hypot(std::min(std::abs(xl),std::abs(xu)),
                                   std::min(std::abs(yl),std::abs(yu)));
        const double du=std::hypot(std::max(std::abs(xl),std::abs(xu)),
                                   std::max(std::abs(yl),std::abs(yu)));
        const double vxl=velocity_center.x()-velocity_delta;
        const double vxu=velocity_center.x()+velocity_delta;
        const double vyl=velocity_center.y()-velocity_delta;
        const double vyu=velocity_center.y()+velocity_delta;
        const double sl=std::hypot(std::min(std::abs(vxl),std::abs(vxu)),
                                   std::min(std::abs(vyl),std::abs(vyu)));
        const double su=std::hypot(std::max(std::abs(vxl),std::abs(vxu)),
                                   std::max(std::abs(vyl),std::abs(vyu)));

        GRBEnv env(true);
        env.set(GRB_IntParam_OutputFlag,0);
        env.set(GRB_IntParam_Seed,2027);
        env.set(GRB_IntParam_Threads,1);
        env.set(GRB_IntParam_NumericFocus,3);
        env.set(GRB_IntParam_NonConvex,2);
        env.set(GRB_DoubleParam_TimeLimit,limit);
        env.set(GRB_DoubleParam_FeasibilityTol,1.0e-9);
        env.set(GRB_DoubleParam_OptimalityTol,1.0e-9);
        env.set(GRB_DoubleParam_MIPGap,1.0e-10);
        env.start();
        GRBModel model(env);
        const auto a0x=model.addVar(-8,8,0,GRB_CONTINUOUS,"a0x");
        const auto a0y=model.addVar(-8,8,0,GRB_CONTINUOUS,"a0y");
        const auto a1x=model.addVar(-8,8,0,GRB_CONTINUOUS,"a1x");
        const auto a1y=model.addVar(-8,8,0,GRB_CONTINUOUS,"a1y");
        const auto x=model.addVar(xl,xu,0,GRB_CONTINUOUS,"x2");
        const auto y=model.addVar(yl,yu,0,GRB_CONTINUOUS,"y2");
        const auto vx=model.addVar(vxl,vxu,0,GRB_CONTINUOUS,"vx2");
        const auto vy=model.addVar(vyl,vyu,0,GRB_CONTINUOUS,"vy2");
        const auto distance=model.addVar(dl,du,0,GRB_CONTINUOUS,"distance");
        const auto speed=model.addVar(sl,su,0,GRB_CONTINUOUS,"speed");
        const double ql=tube.position_radius_m/du;
        const double qu=tube.position_radius_m/dl;
        const auto q=model.addVar(ql,qu,0,GRB_CONTINUOUS,"q");
        const auto cosine=model.addVar(std::sqrt(1-qu*qu),
            std::sqrt(1-ql*ql),0,GRB_CONTINUOUS,"cosine");
        const double drl=std::sqrt(2*(1-std::sqrt(1-ql*ql)));
        const double dru=std::sqrt(2*(1-std::sqrt(1-qu*qu)));
        const auto direction=model.addVar(drl,dru,0,GRB_CONTINUOUS,"direction");
        const auto radial=model.addVar(-su,su,0,GRB_CONTINUOUS,"radial");
        const auto ds=model.addVar(drl*sl,dru*su,0,GRB_CONTINUOUS,"direction_speed");
        const auto support=model.addVar(0,2*half_box*std::sqrt(2.0),0,
            GRB_CONTINUOUS,"U2_support");
        const auto upper=model.addVar(-100,100,0,GRB_CONTINUOUS,"upper");
        model.addConstr(x==center.x()+1.5*dt*dt*a0x+0.5*dt*dt*a1x);
        model.addConstr(y==center.y()+1.5*dt*dt*a0y+0.5*dt*dt*a1y);
        model.addConstr(vx==v0.x()+dt*(a0x+a1x));
        model.addConstr(vy==v0.y()+dt*(a0y+a1y));
        model.addQConstr(distance*distance==x*x+y*y);
        model.addQConstr(speed*speed==vx*vx+vy*vy);
        model.addQConstr(q*distance==tube.position_radius_m);
        model.addQConstr(cosine*cosine+q*q==1.0);
        model.addQConstr(direction*direction==2.0*(1.0-cosine));
        model.addQConstr(radial*distance==x*vx+y*vy);
        model.addQConstr(ds==direction*speed);
        model.addQConstr(support*distance==2.0*half_box*(-x+y));
        model.addConstr(upper==support+
            request.collision_spec.lambda1*request.collision_spec.lambda2*
                (distance-tube.position_radius_m-
                    request.collision_spec.distanceLimit)+
            (request.collision_spec.lambda1+request.collision_spec.lambda2)*
                (radial-ds-tube.velocity_radius_mps)-
            request.collision_spec.totalReserve-
            half_box*std::sqrt(2.0)*direction);
        GRBLinExpr objective=upper;
        model.setObjective(objective,GRB_MAXIMIZE);
        model.optimize();
        const int status=model.get(GRB_IntAttr_Status);
        const bool solved=model.get(GRB_IntAttr_SolCount)>0;
        const double incumbent=solved?model.get(GRB_DoubleAttr_ObjVal):
            std::numeric_limits<double>::quiet_NaN();
        const double bound=model.get(GRB_DoubleAttr_ObjBound);
        const Eigen::Vector2d a0(solved?a0x.get(GRB_DoubleAttr_X):0.0,
                                  solved?a0y.get(GRB_DoubleAttr_X):0.0);
        const Eigen::Vector2d a1(solved?a1x.get(GRB_DoubleAttr_X):0.0,
                                  solved?a1y.get(GRB_DoubleAttr_X):0.0);
        const double independent=solved?exactUpper(a0,a1,r0,v0,dt,
            tube.position_radius_m,tube.velocity_radius_mps,
            request.collision_spec,half_box):
            std::numeric_limits<double>::quiet_NaN();
        double canonical=std::numeric_limits<double>::quiet_NaN();
        if (solved) {
            auto controls0=gf::task10p11afAppliedControlsFromCheckpoint(snapshot);
            controls0[2]=0.5*a0; controls0[9]=-0.5*a0;
            const auto x1=gf::task10p11aa_detail::predictEstimate(
                snapshot,estimate0,controls0);
            auto controls1=controls0;
            controls1[2]=0.5*a1; controls1[9]=-0.5*a1;
            const auto x2=gf::task10p11aa_detail::predictEstimate(
                snapshot,x1,controls1);
            const auto req2=gf::task10p11x_detail::requestAtEstimate(snapshot,x2);
            const auto problem2=gf::buildTask10p11sRows28d(
                gf::buildCanonicalHardRows(req2),req2.mobile_ids,true);
            for (const auto& row:problem2.rows)
                if (row.id=="collision:2--9:full-pair-once-reserve")
                    canonical=row.constant+half_box*
                        row.coefficient.cwiseAbs().sum();
        }
        const bool optimal=status==GRB_OPTIMAL;
        const bool strict=optimal && solved && bound< -1.0e-8 &&
            std::abs(incumbent-independent)<=1.0e-7 &&
            std::abs(incumbent-canonical)<=1.0e-7;
        const json output={{"protocol","task10p11ag-h2-necessary-upper-bound-v1"},
            {"valid",optimal&&solved},{"gurobi_status",status},
            {"global_optimal",optimal},{"solve_time_s",model.get(GRB_DoubleAttr_Runtime)},
            {"incumbent_upper_row_residual_mps2",solved?json(incumbent):json(nullptr)},
            {"global_objective_bound_mps2",bound},
            {"mip_gap",solved?json(model.get(GRB_DoubleAttr_MIPGap)):json(nullptr)},
            {"maximizing_relative_controls",solved?json{{"U0",{a0.x(),a0.y()}},
                {"U1",{a1.x(),a1.y()}}}:json(nullptr)},
            {"independent_formula_residual_mps2",solved?json(independent):json(nullptr)},
            {"canonical_builder_support_residual_mps2",solved?json(canonical):json(nullptr)},
            {"strict_full_domain_H2_infeasible",strict},
            {"necessary_condition",{{"row_id","collision:2--9:full-pair-once-reserve"},
                {"x0_to_x1_to_x2",true},{"all_current_and_x1_constraints_omitted",true},
                {"U0_and_U1_relative_domains",json::array({-8.0,8.0})},
                {"U2_full_input_box_support_maximized_exactly",true},
                {"uncertainty_reserve_counted_once",true}}},
            {"claim_boundary",{{"negative_global_upper_bound_is_H2_infeasibility_certificate",true},
                {"recursive_infeasibility_claimed",false},
                {"mpc_claimed",false}}}};
        gf::writeTask10p11vJson(argv[2],output);
        std::cout<<output.dump(2)<<'\n';
        return strict?0:(optimal?5:4);
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ag H2 necessary bound failed: "<<error.what()<<'\n';
        return 4;
    }
#endif
}
