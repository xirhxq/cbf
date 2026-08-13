#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/ReferenceGeometry.hpp"

#include <Eigen/Dense>

#include <map>
#include <string>
#include <vector>

namespace {

gf::JointEstimateSnapshot geometrySnapshot() {
    Eigen::VectorXd mean(8);
    mean << 0.0, 0.0, 0.0, 0.0,
            10.0, 0.0, 0.0, 0.0;
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(8, 8);
    covariance *= 0.25;
    return gf::JointEstimateSnapshot{
        {1, 2}, mean, covariance,
        {{10, Eigen::Vector2d{10.0, 0.0}},
         {11, Eigen::Vector2d{0.0, 10.0}},
         {12, Eigen::Vector2d{-10.0, 0.0}}}};
}

std::map<std::string, double> unitVariances() {
    return {{"1--2", 1.0}, {"1--10", 1.0},
            {"1--11", 1.0}, {"1--12", 1.0}};
}

double minimumEigenvalue(const Eigen::Matrix2d& matrix) {
    return Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(matrix)
        .eigenvalues().minCoeff();
}

}  // namespace

TEST_CASE("Collinear references are degenerate and orthogonal references are informative") {
    const gf::JointEstimateSnapshot estimate = geometrySnapshot();
    const Eigen::Matrix2d collinear = gf::referenceFim(
        1, {gf::DirectedEdge{10, 1}, gf::DirectedEdge{12, 1}},
        estimate, unitVariances());
    const Eigen::Matrix2d orthogonal = gf::referenceFim(
        1, {gf::DirectedEdge{10, 1}, gf::DirectedEdge{11, 1}},
        estimate, unitVariances());

    CHECK(std::abs(minimumEigenvalue(collinear)) <= 1e-12);
    CHECK(minimumEigenvalue(orthogonal) == doctest::Approx(1.0).epsilon(1e-12));
}

TEST_CASE("A third reference adds positive semidefinite information") {
    const gf::JointEstimateSnapshot estimate = geometrySnapshot();
    const Eigen::Matrix2d two = gf::referenceFim(
        1, {gf::DirectedEdge{10, 1}, gf::DirectedEdge{11, 1}},
        estimate, unitVariances());
    const Eigen::Matrix2d three = gf::referenceFim(
        1, {gf::DirectedEdge{10, 1}, gf::DirectedEdge{11, 1},
            gf::DirectedEdge{12, 1}}, estimate, unitVariances());

    CHECK(minimumEigenvalue(three - two) >= -1e-12);
    CHECK(three.trace() > two.trace());
}

TEST_CASE("Reference counts enforce the per-owner two-through-rmax hard gate") {
    CHECK_FALSE(gf::referenceCountsValid(
        {1}, {gf::DirectedEdge{10, 1}}, 3));
    CHECK(gf::referenceCountsValid(
        {1}, {gf::DirectedEdge{10, 1}, gf::DirectedEdge{11, 1}}, 3));
    CHECK_FALSE(gf::referenceCountsValid(
        {1}, {gf::DirectedEdge{10, 1}, gf::DirectedEdge{11, 1},
              gf::DirectedEdge{12, 1}, gf::DirectedEdge{2, 1}}, 3));
}

TEST_CASE("Schur effective information equals inverse marginal position covariance") {
    Eigen::MatrixXd generator(8, 8);
    generator <<
        2,0,1,0, 1,0,0,0,
        0,2,0,1, 0,1,0,0,
        1,0,2,0, 0,0,1,0,
        0,1,0,2, 0,0,0,1,
        1,0,0,0, 2,0,1,0,
        0,1,0,0, 0,2,0,1,
        0,0,1,0, 1,0,2,0,
        0,0,0,1, 0,1,0,2;
    const Eigen::MatrixXd covariance =
        generator * generator.transpose() +
        0.5 * Eigen::MatrixXd::Identity(8, 8);
    gf::JointEstimateSnapshot estimate{
        {1, 2}, Eigen::VectorXd::Zero(8), covariance, {}};

    const Eigen::Matrix2d marginal =
        gf::marginalPositionCovariance(estimate, 1);
    const Eigen::Matrix2d schur =
        gf::schurEffectivePositionInformation(estimate, 1);

    CHECK(schur.isApprox(marginal.inverse(), 1e-10));
    CHECK(gf::posteriorPositionHealthy(estimate, 1, 100.0));
    CHECK_FALSE(gf::posteriorPositionHealthy(estimate, 1, 0.01));
}
