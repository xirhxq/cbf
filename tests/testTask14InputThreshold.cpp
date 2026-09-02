#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task14InputThreshold.hpp"

namespace {

gf::CanonicalHardRow lower(gf::NodeId owner,const std::string& id,
                            const Eigen::Vector2d& coefficient,
                            double constant) {
    return gf::makeCanonicalGammaRow(id,owner,coefficient,constant);
}

}  // namespace

TEST_CASE("Task 14 distinguishes per-axis box and horizontal norm thresholds") {
    const std::vector<gf::CanonicalHardRow> rows={
        lower(1,"ux>=3",{1.0,0.0},-3.0),
        lower(1,"uy>=4",{0.0,1.0},-4.0)};
    const auto axis=gf::task14MinimumAxisBox(rows,1,0.0,8.0,1e-10);
    const auto norm=gf::task14MinimumHorizontalNorm(rows,1,0.0);
    REQUIRE(axis.valid);
    REQUIRE(norm.valid);
    CHECK(axis.bound_mps2==doctest::Approx(4.0).epsilon(1e-8));
    CHECK(norm.bound_mps2==doctest::Approx(5.0).epsilon(1e-10));
    CHECK(norm.witness.x()==doctest::Approx(3.0));
    CHECK(norm.witness.y()==doctest::Approx(4.0));
}

TEST_CASE("Task 14 disk maximum-margin witness stays inside the disk") {
    const std::vector<gf::CanonicalHardRow> rows={
        lower(2,"ux>=3",{1.0,0.0},-3.0),
        lower(2,"uy>=4",{0.0,1.0},-4.0),
        lower(2,"ux<=6",{-1.0,0.0},6.0)};
    const auto solution=gf::task14SolveDiskGamma(rows,2,5.0,1e-10);
    REQUIRE(solution.valid);
    CHECK(solution.gamma>=-1e-9);
    CHECK(solution.control.norm()<=5.0+1e-9);
    for (const auto& row:rows)
        CHECK(row.margin(solution.control)>=solution.gamma-1e-8);
}

TEST_CASE("Task 14 minimum norm detects an empty hard-row intersection") {
    const std::vector<gf::CanonicalHardRow> rows={
        lower(3,"ux>=2",{1.0,0.0},-2.0),
        lower(3,"ux<=1",{-1.0,0.0},1.0)};
    CHECK_FALSE(gf::task14MinimumHorizontalNorm(rows,3,0.0).valid);
}
