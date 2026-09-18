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

TEST_CASE("Task 20 dual ladder lifts each unit about its anchor-centroid frame") {
    // Adopted 2026-09-18: the DualLadder unit frames are the centroids of the
    // anchors the unit's reference edges cite ({100,101} and {101,102}), so a
    // unit's head role coincides with its front and every member is the
    // triangular lattice image of that front about the unit's own frame.
    const auto contract=gf::task20DagLatticeContract(
        gf::Task20LatticeMode::DualLadder);
    CHECK(contract.valid);
    CHECK(contract.coverage_units.size()==2);
    CHECK(contract.reference_edges==gf::task10p11rFixedReferenceTopology());
    CHECK(contract.coverage_units[0].base_anchors==std::vector<gf::NodeId>{100,101});
    CHECK(contract.coverage_units[1].base_anchors==std::vector<gf::NodeId>{101,102});
    const auto fixed=gf::task10p11rFixedBaselineScenario().fixed_positions;
    const std::map<std::string,Eigen::Vector2d> fronts{
        {"A",{725.0,2125.0}},{"B",{2415.0,2735.0}}};
    const auto lifted=gf::task20LiftTargets(contract,fixed,fronts);
    REQUIRE(lifted.valid);
    for (const auto& unit:contract.coverage_units) {
        Eigen::Vector2d origin=Eigen::Vector2d::Zero();
        for (const auto anchor:unit.base_anchors) origin+=fixed.at(anchor);
        origin/=static_cast<double>(unit.base_anchors.size());
        const auto& head=contract.member_roles.at(unit.leader);
        CHECK(head.axial_fraction==1.0);
        CHECK(head.triangular_fraction==0.0);
        CHECK((lifted.targets.at(unit.leader)-fronts.at(unit.id)).norm()<1.0e-12);
        const Eigen::Vector2d displacement=fronts.at(unit.id)-origin;
        for (const auto member:unit.members) {
            const auto& role=contract.member_roles.at(member);
            const double sign=role.triangular_fraction<0.0?-1.0:1.0;
            const Eigen::Vector2d triangular{
                0.5*displacement.x()-sign*std::sqrt(3.0)/2.0*displacement.y(),
                sign*std::sqrt(3.0)/2.0*displacement.x()+0.5*displacement.y()};
            const Eigen::Vector2d expected=origin+role.axial_fraction*displacement+
                std::abs(role.triangular_fraction)*triangular;
            CHECK((lifted.targets.at(member)-expected).norm()<1.0e-12);
        }
    }
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
