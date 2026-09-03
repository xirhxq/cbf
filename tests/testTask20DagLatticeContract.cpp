#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task20DagLatticeContract.hpp"
#include "grand_finale/Task20GridOracle.hpp"

TEST_CASE("Task 20 formal initial certified set reproduces the published mask") {
    const auto initial=gf::task20FormalInitialCoverage(
        gf::task10p11rFixedBaselineScenario(),300,300);
    CHECK(initial.certified_count==3736);
    CHECK(initial.truth_count==3920);
    CHECK(initial.certified_hash==5505169043007961181ULL);
    CHECK(initial.truth_hash==13357789335370783887ULL);
}

TEST_CASE("Task 20 dual ladder is an exact Task 18 lifting special case") {
    const auto contract=gf::task20DagLatticeContract(
        gf::Task20LatticeMode::DualLadder);
    CHECK(contract.valid);
    CHECK(contract.coverage_units.size()==2);
    CHECK(contract.reference_edges==gf::task10p11rFixedReferenceTopology());
    const auto fixed=gf::task10p11rFixedBaselineScenario().fixed_positions;
    const std::map<std::string,Eigen::Vector2d> fronts{
        {"A",{725.0,2125.0}},{"B",{2415.0,2735.0}}};
    const auto lifted=gf::task20LiftTargets(contract,fixed,fronts);
    REQUIRE(lifted.valid);
    const auto squads=gf::task13UnifiedCoverageSquads();
    const auto expected_a=gf::task15ForwardTargets(
        squads[0],fixed.at(101),fronts.at("A"));
    const auto expected_b=gf::task15ForwardTargets(
        squads[1],fixed.at(101),fronts.at("B"));
    for (const auto& [owner,target]:expected_a)
        CHECK((lifted.targets.at(owner)-target).norm()<1.0e-12);
    for (const auto& [owner,target]:expected_b)
        CHECK((lifted.targets.at(owner)-target).norm()<1.0e-12);
}

TEST_CASE("Task 20 research modes are valid and structurally non-isomorphic") {
    std::set<std::string> signatures;
    for (const auto mode:{gf::Task20LatticeMode::DualLadder,
                          gf::Task20LatticeMode::MergedStrip,
                          gf::Task20LatticeMode::SplitThreeFront,
                          gf::Task20LatticeMode::CrossBracedDiamond}) {
        const auto contract=gf::task20DagLatticeContract(mode);
        CAPTURE(contract.id);
        REQUIRE(contract.valid);
        CHECK(contract.reference_edges.size()==28);
        CHECK(contract.member_roles.size()==14);
        CHECK(contract.topological_order.size()==17);
        CHECK(signatures.insert(contract.structural_signature).second);
        std::map<gf::NodeId,std::size_t> indegree;
        for (const auto& edge:contract.reference_edges) ++indegree[edge.owner];
        for (gf::NodeId owner=1;owner<=14;++owner)
            CHECK(indegree[owner]==2);
    }
}

TEST_CASE("Task 20 lifting is deterministic and separates target selection from follower lifting") {
    const auto fixed=gf::task10p11rFixedBaselineScenario().fixed_positions;
    for (const auto mode:{gf::Task20LatticeMode::MergedStrip,
                          gf::Task20LatticeMode::SplitThreeFront,
                          gf::Task20LatticeMode::CrossBracedDiamond}) {
        const auto contract=gf::task20DagLatticeContract(mode);
        std::map<std::string,Eigen::Vector2d> fronts;
        for (const auto& unit:contract.coverage_units)
            fronts.emplace(unit.id,Eigen::Vector2d{
                300.0+350.0*static_cast<double>(fronts.size()),2400.0});
        const auto first=gf::task20LiftTargets(contract,fixed,fronts);
        const auto second=gf::task20LiftTargets(contract,fixed,fronts);
        REQUIRE(first.valid);
        REQUIRE(second.valid);
        CHECK(first.targets.size()==14);
        for (const auto& [owner,target]:first.targets) {
            CHECK(target.allFinite());
            CHECK((target-second.targets.at(owner)).norm()<1.0e-12);
        }
    }
}

TEST_CASE("Task 20 member-pose inverse reconstructs each affine lattice witness") {
    const auto fixed=gf::task10p11rFixedBaselineScenario().fixed_positions;
    for (const auto mode:{gf::Task20LatticeMode::DualLadder,
                          gf::Task20LatticeMode::MergedStrip,
                          gf::Task20LatticeMode::SplitThreeFront,
                          gf::Task20LatticeMode::CrossBracedDiamond}) {
        const auto contract=gf::task20DagLatticeContract(mode);
        for (const auto& [member,role]:contract.member_roles) {
            const gf::NodeId captured_member=member;
            const Eigen::Vector2d service_pose{875.0+member,1425.0-member};
            const auto inverse=gf::task20FrontForMemberPose(
                contract,fixed,member,service_pose);
            CAPTURE(contract.id);
            CAPTURE(captured_member);
            REQUIRE(inverse.valid);
            std::map<std::string,Eigen::Vector2d> fronts;
            for (const auto& unit:contract.coverage_units)
                fronts[unit.id]=unit.id==role.coverage_unit
                    ?inverse.front:Eigen::Vector2d{1500.0,1200.0};
            const auto lifted=gf::task20LiftTargets(contract,fixed,fronts);
            REQUIRE(lifted.valid);
            CHECK((lifted.targets.at(member)-service_pose).norm()<1.0e-9);
        }
    }
}

TEST_CASE("Task 20 oracle uses all member roles and distinguishes nominal reference compatibility") {
    const auto fixed=gf::task10p11rFixedBaselineScenario().fixed_positions;
    for (const auto mode:{gf::Task20LatticeMode::DualLadder,
                          gf::Task20LatticeMode::MergedStrip,
                          gf::Task20LatticeMode::SplitThreeFront,
                          gf::Task20LatticeMode::CrossBracedDiamond}) {
        const auto contract=gf::task20DagLatticeContract(mode);
        for (const Eigen::Vector2d cell:{Eigen::Vector2d{5.0,1555.0},
                                        Eigen::Vector2d{1505.0,1505.0},
                                        Eigen::Vector2d{2995.0,2995.0}}) {
            const auto witness=gf::task20FindServiceWitness(contract,fixed,cell);
            CAPTURE(contract.id);
            CAPTURE(cell.x());
            CAPTURE(cell.y());
            REQUIRE(witness.serviceable);
            CHECK(witness.responsible_member>=1);
            CHECK(witness.responsible_member<=14);
            CHECK(witness.targets.size()==14);
            CHECK(witness.geometry.minimum_target_separation_m>10.0);
            CHECK(witness.radial_certified_margin_m>=0.0);
            CHECK(witness.angular_certified_margin_rad>=0.0);
            CHECK(witness.nominal_reference_compatible==
                (witness.geometry.maximum_reference_edge_m<850.0));
        }
    }
}
