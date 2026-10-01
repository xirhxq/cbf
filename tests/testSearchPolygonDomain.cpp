#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "world/world"
#include "grand_finale/CertifiedCoverageTracker.hpp"
#include "grand_finale/SearchPolygonDomain.hpp"

TEST_CASE("Opt-in search polygon uses closed cell-center membership") {
    const json settings={{"boundary",{{0.,0.},{20.,0.},{0.,20.}}},
        {"spacing",10.},{"cell-domain","closed-polygon-centers-v1"}};
    GridWorld grid(settings);
    CHECK(grid.validCount==3);
    CHECK(grid.valid[grid.getIndex(0,0)]);
    CHECK(grid.valid[grid.getIndex(0,1)]);
    CHECK(grid.valid[grid.getIndex(1,0)]);
    CHECK_FALSE(grid.valid[grid.getIndex(1,1)]);
}

TEST_CASE("Certified and truth coverage share the supplied task domain") {
    GridWorld domain(json{{"boundary",{{0.,0.},{20.,0.},{0.,20.}}},
        {"spacing",10.},{"cell-domain","closed-polygon-centers-v1"}});
    gf::CertifiedCoverageTracker coverage(domain);
    coverage.observe(Point(10,10),Point(10,10),0.,100.);
    CHECK(coverage.truthGrid().valid==domain.valid);
    CHECK(coverage.certifiedGrid().valid==domain.valid);
    CHECK(coverage.certifiedCoveredCount()==3);
    CHECK(coverage.truthCoveredCount()==3);
    CHECK(coverage.reachedCertifiedT100());
}

TEST_CASE("Rectangle default and explicit center domain are identical") {
    const json legacy={{"boundary",{{0.,0.},{4500.,0.},{4500.,2250.},{0.,2250.}}},
        {"spacing",10.}};
    auto explicitDomain=legacy;
    explicitDomain["cell-domain"]="closed-polygon-centers-v1";
    GridWorld first(legacy),second(explicitDomain);
    CHECK(first.valid==second.valid);
    CHECK(first.validCount==101250);
    gf::CertifiedCoverageTracker oldCoverage(first.xLim,first.xNum,first.yLim,first.yNum);
    gf::CertifiedCoverageTracker newCoverage(second);
    for (int i=0;i<14;++i) {
        const Point p(1800.+40*i,50.+30*i);
        oldCoverage.observeSector(p,p,10.,0.,400.,M_PI/3.,M_PI/2.);
        newCoverage.observeSector(p,p,10.,0.,400.,M_PI/3.,M_PI/2.);
    }
    CHECK(oldCoverage.truthGrid().vis==newCoverage.truthGrid().vis);
    CHECK(oldCoverage.certifiedGrid().vis==newCoverage.certifiedGrid().vis);
}

TEST_CASE("Polygon tasks omit partly intersecting cells with outside centers") {
    GridWorld grid(json{{"boundary",{{0.,0.},{20.,0.},{0.,20.}}},
        {"spacing",10.},{"cell-domain","closed-polygon-centers-v1"}});
    const auto candidates=grid.getUnexploredCellCenters();
    REQUIRE(candidates.size()==3);
    for (const auto& cell:candidates) CHECK(cell.center.x+cell.center.y<=20.);
    auto invalid=json{{"boundary",{{0.,0.},{20.,0.},{0.,20.}}},
        {"spacing",10.},{"cell-domain","typo"}};
    CHECK_THROWS_AS((GridWorld(invalid)),std::invalid_argument);
}

TEST_CASE("Search region adapter changes only the world domain") {
    const json original={{"world",{{"boundary",{{0,0},{4500,0},{4500,2250},{0,2250}}},
        {"spacing",10}}},{"initial",{{"yawDeg",90}}},{"marker",42}};
    const json site={{"schema","gf-search-polygon-v1"},{"id","triangle"},
        {"vertices_m",{{0,0},{4500,0},{2250,4500}}},{"cell_size_m",10}};
    const auto applied=gf::searchPolygonSettings(original,site);
    CHECK(applied.at("initial")==original.at("initial"));
    CHECK(applied.at("marker")==42);
    GridWorld grid(applied.at("world"));
    CHECK(grid.xNum==450);
    CHECK(grid.yNum==450);
    CHECK(grid.validCount==101250);
    auto invalid=site;invalid["vertices_m"]={{0,0},{20,0},{10,5},{20,20},{0,20}};
    CHECK_THROWS_AS(gf::searchPolygonSettings(original,invalid),std::invalid_argument);
}

TEST_CASE("Parallelogram center domain agrees with independent pilot count") {
    const json settings={{"boundary",{{0,0},{4500,0},{8000,2250},{3500,2250}}},
        {"spacing",10},{"cell-domain","closed-polygon-centers-v1"}};
    GridWorld grid(settings);
    CHECK(grid.xNum==800);
    CHECK(grid.yNum==225);
    CHECK(grid.validCount==101250);
    CHECK_FALSE(grid.valid[grid.getIndex(0,224)]);
    CHECK_FALSE(grid.valid[grid.getIndex(799,0)]);
    CHECK(grid.valid[grid.getIndex(400,100)]);
}
