#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11lSFrozenQp.hpp"
#include "optimisers/OSQP.hpp"

TEST_CASE("Task 10.11l-S authority QP is literal and trajectory independent") {
    const auto& frozen=gf::task10p11lSOwner11FrozenQp();
    CHECK(frozen.source_runtime_s==doctest::Approx(4.3));
    CHECK(frozen.request.owner==11);
    CHECK(frozen.request.acceleration_half_box==doctest::Approx(4.0));
    CHECK(frozen.request.residual_tolerance==doctest::Approx(1.0e-7));
    CHECK(frozen.current_gamma_mps2==doctest::Approx(0.0102692).epsilon(1e-6));
    CHECK(frozen.request.rows.size()>4);
    CHECK(frozen.raw_hessian.isApprox(2.0*Eigen::Matrix2d::Identity(),1e-15));
    CHECK(frozen.raw_gradient.isApprox(-2.0*frozen.request.nominal,1e-15));
    CHECK_FALSE(frozen.snapshot_digest.empty());

    const auto exact=gf::evaluateProgressCompatibility(
        frozen.request.rows,frozen.request.owner,frozen.request.nominal,
        frozen.request.acceleration_half_box,
        {std::numeric_limits<double>::max(),0.0,
         frozen.request.residual_tolerance,true});
    REQUIRE(exact.polytope_nonempty);
    const auto gamma=gf::solveCanonicalGammaStar(
        frozen.request.rows,frozen.request.owner,
        frozen.request.acceleration_half_box);
    REQUIRE(gamma.valid);
    CHECK(gamma.gamma==doctest::Approx(frozen.current_gamma_mps2).epsilon(1e-13));
}

TEST_CASE("Task 10.11l-S never accepts a max-iteration iterate") {
    const auto& frozen=gf::task10p11lSOwner11FrozenQp();
    json settings={{"k_delta",1.0},{"maximum-iterations",1},
        {"absolute-tolerance",1.0e-8},{"relative-tolerance",1.0e-8}};
    OSQP osqp(settings);
    osqp.start(2,2);
    osqp.setObjective(frozen.request.nominal);
    for (const auto& row : frozen.request.rows) {
        osqp.addLinearConstraint(row.control_coefficient,-row.constant);
    }
    const auto iterate=osqp.solve();
    CHECK(iterate.size()==0);
    CHECK(osqp.getStatus().value("status","")=="max_iter_reached");
    CHECK_THROWS_AS(osqp.resolveWarmStarted(),std::logic_error);
}

TEST_CASE("Task 10.11l-S bounded one-million-iteration probe") {
    const auto& frozen=gf::task10p11lSOwner11FrozenQp();
    json settings={{"k_delta",1.0},{"maximum-iterations",1000000},
        {"absolute-tolerance",1.0e-8},{"relative-tolerance",1.0e-8},
        {"primal-infeasibility-tolerance",1.0e-8},
        {"dual-infeasibility-tolerance",1.0e-8},
        {"rho",10.0},{"adaptive-rho",true},
        {"adaptive-rho-interval",100},{"adaptive-rho-tolerance",5.0},
        {"scaling-iterations",0},{"explicit-row-scaling",true},
        {"polishing",true},
        {"termination-check-interval",25},{"warm-start",true},
        {"scaled-termination",false}};
    OSQP osqp(settings);
    osqp.start(2,2);
    osqp.setObjective(frozen.request.nominal);
    for (const auto& row : frozen.request.rows)
        osqp.addLinearConstraint(row.control_coefficient,-row.constant);
    const auto solution=osqp.solve();
    const auto status=osqp.getStatus();
    MESSAGE("TASK10P11L_S_1M status=",status.value("status",""),
        " iter=",status.value("iteration_count",-1),
        " primal=",status.value("primal_residual",-1.0),
        " dual=",status.value("dual_residual",-1.0));
    REQUIRE(status.value("explicit_row_scaling",false));
    REQUIRE(status.value("status","")=="optimal");
    REQUIRE(solution.size()==2);
    CHECK(gf::minimumCanonicalOwnerResidual(
        frozen.request.rows,11,solution)>=-frozen.request.residual_tolerance);
    const int cold_iterations=status.value("iteration_count",-1);
    const auto warm=osqp.resolveWarmStarted();
    const auto warm_status=osqp.getStatus();
    REQUIRE(warm_status.value("status","")=="optimal");
    REQUIRE(warm.size()==2);
    CHECK((warm-solution).norm()<=1.0e-9);
    CHECK(warm_status.value("iteration_count",cold_iterations)<=cold_iterations);
}

TEST_CASE("Task 10.11l-S three solvers expose strict near-boundary diagnostics") {
    const auto& frozen=gf::task10p11lSOwner11FrozenQp();
    auto gurobi_request=frozen.request;
    gurobi_request.profile=gf::SolverProfile::Gurobi;
    auto osqp_request=frozen.request;
    osqp_request.profile=gf::SolverProfile::OpenSource;
    gf::CanonicalHocbfQpController controller;
    const auto gurobi=controller.solve(gurobi_request);
    const auto osqp=controller.solve(osqp_request);
    MESSAGE("TASK10P11L_S_OSQP status=",osqp.solver_status,
        " iter=",osqp.solver_iteration_count,
        " primal=",osqp.solver_primal_residual,
        " dual=",osqp.solver_dual_residual,
        " reason=",osqp.failure_reason);
    REQUIRE(gurobi.control_available);
    REQUIRE(osqp.control_available);
    CHECK(gurobi.solver_status=="optimal");
    CHECK(osqp.solver_status=="optimal");
    CHECK(osqp.solver_iteration_count>0);
    CHECK(std::isfinite(osqp.solver_primal_residual));
    CHECK(std::isfinite(osqp.solver_dual_residual));
    CHECK(osqp.minimum_hard_residual>=-frozen.request.residual_tolerance);
    CHECK(osqp.exact_oracle_error<=1.0e-6);
    CHECK((osqp.control-gurobi.control).norm()<=1.0e-6);
}
