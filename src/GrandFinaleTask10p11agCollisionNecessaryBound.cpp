#include "grand_finale/Task10p11agFullDomainPredecessor.hpp"

#include <iostream>

namespace {

using json=nlohmann::json;

double exactUpperRow(const Eigen::Vector2d& relative_acceleration,
    const Eigen::Vector2d& relative_position,
    const Eigen::Vector2d& relative_velocity,double dt,double position_radius,
    double velocity_radius,const PairwiseSecondOrderRowSpec& spec,
    double half_box) {
    const Eigen::Vector2d position1=relative_position+
        dt*relative_velocity+0.5*dt*dt*relative_acceleration;
    const Eigen::Vector2d velocity1=relative_velocity+
        dt*relative_acceleration;
    const double distance=position1.norm();
    const Eigen::Vector2d normal=position1/distance;
    const double direction_radius=std::sqrt(2.0*(1.0-std::sqrt(
        1.0-std::pow(position_radius/distance,2.0))));
    const double radial=normal.dot(velocity1);
    const double radial_uncertainty=direction_radius*velocity1.norm()+
        velocity_radius;
    const double central=spec.lambda1*spec.lambda2*
        (distance-position_radius-spec.distanceLimit)+
        (spec.lambda1+spec.lambda2)*(radial-radial_uncertainty)-
        spec.totalReserve;
    const double coefficient_reserve=half_box*std::sqrt(2.0)*
        direction_radius;
    return 2.0*half_box*normal.cwiseAbs().sum()+central-
        coefficient_reserve;
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=3 && argc!=4) {
        std::cerr<<"usage: GrandFinaleTask10p11agCollisionNecessaryBound "
            "INPUT_PACKED OUTPUT_JSON [TIME_LIMIT_S]\n";
        return 2;
    }
#ifndef ENABLE_GUROBI
    (void)argv;
    std::cerr<<"Gurobi is required\n";
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
        const auto request1=gf::task10p11x_detail::requestAtEstimate(
            snapshot,estimate1);
        const auto tube=request1.collision_snapshot_tubes.at("2--9");
        const auto first=request.states.at(2);
        const auto second=request.states.at(9);
        const Eigen::Vector2d relative_position(
            first.position.x-second.position.x,
            first.position.y-second.position.y);
        const Eigen::Vector2d relative_velocity=
            first.velocity-second.velocity;
        const double dt=snapshot.at("successor_parameters").at("dt_s");
        const double half_box=request.acceleration_half_box;
        const double time_limit=argc==4?std::stod(argv[3]):43200.0;

        const Eigen::Vector2d position_center=
            relative_position+dt*relative_velocity;
        const double position_delta=dt*dt*half_box;
        const Eigen::Vector2d velocity_center=relative_velocity;
        const double velocity_delta=2.0*dt*half_box;
        const double x_lower=position_center.x()-position_delta;
        const double x_upper=position_center.x()+position_delta;
        const double y_lower=position_center.y()-position_delta;
        const double y_upper=position_center.y()+position_delta;
        if (!(x_upper<0.0 && y_lower>0.0))
            throw std::runtime_error(
                "pair normal signs are not fixed over expanded input box");
        const double distance_lower=std::hypot(
            std::min(std::abs(x_lower),std::abs(x_upper)),
            std::min(std::abs(y_lower),std::abs(y_upper)));
        const double distance_upper=std::hypot(
            std::max(std::abs(x_lower),std::abs(x_upper)),
            std::max(std::abs(y_lower),std::abs(y_upper)));
        const double vx_lower=velocity_center.x()-velocity_delta;
        const double vx_upper=velocity_center.x()+velocity_delta;
        const double vy_lower=velocity_center.y()-velocity_delta;
        const double vy_upper=velocity_center.y()+velocity_delta;
        const double speed_lower=std::hypot(
            std::min(std::abs(vx_lower),std::abs(vx_upper)),
            std::min(std::abs(vy_lower),std::abs(vy_upper)));
        const double speed_upper=std::hypot(
            std::max(std::abs(vx_lower),std::abs(vx_upper)),
            std::max(std::abs(vy_lower),std::abs(vy_upper)));

        GRBEnv environment(true);
        environment.set(GRB_IntParam_OutputFlag,0);
        environment.set(GRB_IntParam_Seed,2027);
        environment.set(GRB_IntParam_Threads,1);
        environment.set(GRB_IntParam_NumericFocus,3);
        environment.set(GRB_IntParam_NonConvex,2);
        environment.set(GRB_DoubleParam_TimeLimit,time_limit);
        environment.set(GRB_DoubleParam_FeasibilityTol,1.0e-9);
        environment.set(GRB_DoubleParam_OptimalityTol,1.0e-9);
        environment.set(GRB_DoubleParam_MIPGap,1.0e-10);
        environment.start();
        GRBModel model(environment);
        const auto ax=model.addVar(-2.0*half_box,2.0*half_box,0.0,
            GRB_CONTINUOUS,"relative_ax");
        const auto ay=model.addVar(-2.0*half_box,2.0*half_box,0.0,
            GRB_CONTINUOUS,"relative_ay");
        const auto x=model.addVar(x_lower,x_upper,0.0,GRB_CONTINUOUS,"x1");
        const auto y=model.addVar(y_lower,y_upper,0.0,GRB_CONTINUOUS,"y1");
        const auto vx=model.addVar(vx_lower,vx_upper,0.0,GRB_CONTINUOUS,"vx1");
        const auto vy=model.addVar(vy_lower,vy_upper,0.0,GRB_CONTINUOUS,"vy1");
        const auto distance=model.addVar(distance_lower,distance_upper,0.0,
            GRB_CONTINUOUS,"distance");
        const auto speed=model.addVar(speed_lower,speed_upper,0.0,
            GRB_CONTINUOUS,"relative_speed");
        const double q_lower=tube.position_radius_m/distance_upper;
        const double q_upper=tube.position_radius_m/distance_lower;
        const auto q=model.addVar(q_lower,q_upper,0.0,GRB_CONTINUOUS,"q");
        const auto cosine=model.addVar(std::sqrt(1.0-q_upper*q_upper),
            std::sqrt(1.0-q_lower*q_lower),0.0,GRB_CONTINUOUS,"cosine");
        const double direction_lower=std::sqrt(2.0*(1.0-
            std::sqrt(1.0-q_lower*q_lower)));
        const double direction_upper=std::sqrt(2.0*(1.0-
            std::sqrt(1.0-q_upper*q_upper)));
        const auto direction=model.addVar(direction_lower,direction_upper,0.0,
            GRB_CONTINUOUS,"direction_radius");
        const auto radial=model.addVar(-speed_upper,speed_upper,0.0,
            GRB_CONTINUOUS,"radial_velocity");
        const auto direction_speed=model.addVar(
            direction_lower*speed_lower,direction_upper*speed_upper,0.0,
            GRB_CONTINUOUS,"direction_speed");
        const auto input_support=model.addVar(0.0,2.0*half_box*std::sqrt(2.0),
            0.0,GRB_CONTINUOUS,"successor_input_support");
        const auto upper=model.addVar(-100.0,100.0,0.0,GRB_CONTINUOUS,
            "necessary_row_upper_bound");

        model.addConstr(x==position_center.x()+0.5*dt*dt*ax);
        model.addConstr(y==position_center.y()+0.5*dt*dt*ay);
        model.addConstr(vx==velocity_center.x()+dt*ax);
        model.addConstr(vy==velocity_center.y()+dt*ay);
        model.addQConstr(distance*distance==x*x+y*y);
        model.addQConstr(speed*speed==vx*vx+vy*vy);
        model.addQConstr(q*distance==tube.position_radius_m);
        model.addQConstr(cosine*cosine+q*q==1.0);
        model.addQConstr(direction*direction==2.0*(1.0-cosine));
        model.addQConstr(radial*distance==x*vx+y*vy);
        model.addQConstr(direction_speed==direction*speed);
        model.addQConstr(input_support*distance==
            2.0*half_box*(-x+y));
        model.addConstr(upper==input_support+
            request.collision_spec.lambda1*request.collision_spec.lambda2*
                (distance-tube.position_radius_m-
                    request.collision_spec.distanceLimit)+
            (request.collision_spec.lambda1+
                request.collision_spec.lambda2)*
                (radial-direction_speed-tube.velocity_radius_mps)-
            request.collision_spec.totalReserve-
            half_box*std::sqrt(2.0)*direction);
        GRBLinExpr objective=upper;
        model.setObjective(objective,GRB_MAXIMIZE);
        model.optimize();
        const int status=model.get(GRB_IntAttr_Status);
        const bool has_solution=model.get(GRB_IntAttr_SolCount)>0;
        const double incumbent=has_solution?model.get(GRB_DoubleAttr_ObjVal):
            std::numeric_limits<double>::quiet_NaN();
        const double bound=model.get(GRB_DoubleAttr_ObjBound);
        const Eigen::Vector2d relative_acceleration(
            has_solution?ax.get(GRB_DoubleAttr_X):0.0,
            has_solution?ay.get(GRB_DoubleAttr_X):0.0);
        const double independent=has_solution?exactUpperRow(
            relative_acceleration,relative_position,relative_velocity,dt,
            tube.position_radius_m,tube.velocity_radius_mps,
            request.collision_spec,half_box):
            std::numeric_limits<double>::quiet_NaN();

        double canonical=std::numeric_limits<double>::quiet_NaN();
        if (has_solution) {
            auto controls=gf::task10p11afAppliedControlsFromCheckpoint(snapshot);
            controls[2]=0.5*relative_acceleration;
            controls[9]=-0.5*relative_acceleration;
            const auto predicted=gf::task10p11aa_detail::predictEstimate(
                snapshot,estimate0,controls);
            const auto successor_request=gf::task10p11x_detail::requestAtEstimate(
                snapshot,predicted);
            const auto successor_problem=gf::buildTask10p11sRows28d(
                gf::buildCanonicalHardRows(successor_request),
                successor_request.mobile_ids,true);
            for (const auto& row:successor_problem.rows)
                if (row.id=="collision:2--9:full-pair-once-reserve")
                    canonical=row.constant+half_box*
                        row.coefficient.cwiseAbs().sum();
        }
        const bool global_optimal=status==GRB_OPTIMAL;
        const bool strict_infeasible=global_optimal && bound< -1.0e-8 &&
            has_solution && std::abs(incumbent-independent)<=1.0e-7 &&
            std::abs(incumbent-canonical)<=1.0e-7;
        const json output={{"protocol",
                "task10p11ag-collision-necessary-upper-bound-v1"},
            {"valid",global_optimal && has_solution},
            {"gurobi_status",status},{"global_optimal",global_optimal},
            {"solve_time_s",model.get(GRB_DoubleAttr_Runtime)},
            {"incumbent_upper_row_residual_mps2",
                std::isfinite(incumbent)?json(incumbent):json(nullptr)},
            {"global_objective_bound_mps2",bound},
            {"mip_gap",has_solution?json(model.get(GRB_DoubleAttr_MIPGap)):
                json(nullptr)},
            {"maximizing_relative_U0_mps2",has_solution?
                json::array({relative_acceleration.x(),
                    relative_acceleration.y()}):json(nullptr)},
            {"independent_formula_residual_mps2",
                std::isfinite(independent)?json(independent):json(nullptr)},
            {"canonical_builder_support_residual_mps2",
                std::isfinite(canonical)?json(canonical):json(nullptr)},
            {"strict_full_domain_predecessor_infeasible",strict_infeasible},
            {"necessary_condition",{{"row_id",
                    "collision:2--9:full-pair-once-reserve"},
                {"current_and_other_successor_rows_omitted",true},
                {"U0_relative_domain",json::array({-8.0,8.0})},
                {"U1_full_input_box_support_maximized_exactly",true},
                {"normal_signs_fixed_over_domain",true},
                {"uncertainty_reserve_counted_once",true}}},
            {"claim_boundary",{{"negative_global_upper_bound_is_infeasibility_certificate",true},
                {"recursive_infeasibility_claimed",false},
                {"fixed_topology_only",true}}}};
        gf::writeTask10p11vJson(argv[2],output);
        std::cout<<output.dump(2)<<'\n';
        return strict_infeasible?0:(global_optimal?5:4);
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ag necessary bound failed: "
                 <<error.what()<<'\n';
        return 4;
    }
#endif
}
