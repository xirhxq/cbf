#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task16FormationGovernor.hpp"

namespace {

std::map<gf::NodeId,Eigen::Vector2d> ledger(
    const Eigen::Vector2d& a,const Eigen::Vector2d& b) {
    std::map<gf::NodeId,Eigen::Vector2d> value;
    const auto squads=gf::task13UnifiedCoverageSquads();
    const Eigen::Vector2d base{1500.0,-50.0};
    const auto first=gf::task16ForwardTargets(squads[0],base,a,std::nullopt);
    const auto second=gf::task16ForwardTargets(squads[1],base,b,std::nullopt);
    value.insert(first.begin(),first.end());
    value.insert(second.begin(),second.end());
    return value;
}

}  // namespace

TEST_CASE("Task 16 governor advances each squad by one common lambda") {
    const auto old=ledger({500.0,500.0},{2500.0,500.0});
    const auto nominal=ledger({500.0,2500.0},{2500.0,2500.0});
    const auto result=gf::task16AdvanceFormationGovernor(old,nominal,
        [](const auto&,const std::map<std::string,double>& lambda) {
            return lambda.at("A")<=0.5 && lambda.at("B")<=0.25;
        });
    REQUIRE(result.valid);
    CHECK(result.common_fraction.at("A")==doctest::Approx(0.5));
    CHECK(result.common_fraction.at("B")==doctest::Approx(0.25));
    CHECK(result.targets.at(7).x()==doctest::Approx(500.0));
    CHECK(result.targets.at(7).y()==doctest::Approx(1500.0));
    CHECK(result.targets.at(14).x()==doctest::Approx(2500.0));
    CHECK(result.targets.at(14).y()==doctest::Approx(1000.0));
    for (gf::NodeId id=1;id<=7;++id)
        CHECK((result.targets.at(id)-
            (old.at(id)+0.5*(nominal.at(id)-old.at(id)))).norm()<1e-12);
    for (gf::NodeId id=8;id<=14;++id)
        CHECK((result.targets.at(id)-
            (old.at(id)+0.25*(nominal.at(id)-old.at(id)))).norm()<1e-12);
}

TEST_CASE("Task 16 governor represents a safe zero-step stall explicitly") {
    const auto old=ledger({500.0,500.0},{2500.0,500.0});
    const auto nominal=ledger({500.0,2500.0},{2500.0,2500.0});
    const auto stalled=gf::task16AdvanceFormationGovernor(old,nominal,
        [](const auto&,const std::map<std::string,double>& lambda) {
            return lambda.at("A")==0.0 && lambda.at("B")==0.0;
        });
    REQUIRE(stalled.valid);
    CHECK(stalled.stalled_squads==2);
    CHECK(stalled.targets.at(7).y()==doctest::Approx(500.0));
    CHECK(stalled.targets.at(14).y()==doctest::Approx(500.0));

    const auto impossible=gf::task16AdvanceFormationGovernor(old,nominal,
        [](const auto&,const auto&) { return false; });
    CHECK_FALSE(impossible.valid);
    CHECK(impossible.reason=="task16_no_safe_homotopy_pair");
}

TEST_CASE("Task 16 governor rejects incomplete ledgers") {
    auto old=ledger({500.0,500.0},{2500.0,500.0});
    auto nominal=ledger({500.0,2500.0},{2500.0,2500.0});
    old.erase(14);
    const auto result=gf::task16AdvanceFormationGovernor(old,nominal,
        [](const auto&,const auto&) { return true; });
    CHECK_FALSE(result.valid);
    CHECK(result.feasibility_evaluations==0);
}

TEST_CASE("Task 16 governor applies a squad-common analytic rate cap") {
    CHECK(gf::task16AnalyticReferenceSpeedMps(30.0,4.0,0.8)==
        doctest::Approx(25.0));
    CHECK(gf::task16AnalyticReferenceSpeedMps(30.0,4.0,0.8,2.0)==
        doctest::Approx(20.0));
    CHECK(gf::task16AnalyticReferenceSpeedMps(30.0,4.0,0.8,4.0)==
        doctest::Approx(10.0));
    CHECK(gf::task16AnalyticTrackingEnvelopeM(
        10.0,0.8,0.4,30.0,4.0)==doctest::Approx(132.5));
    const auto old=ledger({500.0,500.0},{2500.0,500.0});
    auto nominal=old;
    for (int id=1;id<=7;++id)
        nominal.at(id)+=Eigen::Vector2d(100.0,0.0);
    for (int id=8;id<=14;++id)
        nominal.at(id)+=Eigen::Vector2d(0.0,50.0);
    const auto result=gf::task16AdvanceFormationGovernor(
        old,nominal,[](const auto&,const auto&) { return true; },2.5);
    REQUIRE(result.valid);
    CHECK(result.common_fraction.at("A")==doctest::Approx(0.025));
    CHECK(result.common_fraction.at("B")==doctest::Approx(0.05));
    for (int id=1;id<=14;++id)
        CHECK((result.targets.at(id)-old.at(id)).norm()<=2.5+1e-12);
}
