#include "grand_finale/Task10p11agFullDomainPredecessor.hpp"
#include "grand_finale/Task10p11zEvidence.hpp"

#include <iostream>

namespace {

using json=nlohmann::json;

json controlsJson(const std::map<gf::NodeId,Eigen::Vector2d>& controls) {
    json result=json::object();
    for (const auto& [owner,control]:controls)
        result[std::to_string(owner)]={control.x(),control.y()};
    return result;
}

gf::SimpleCoverageControlStep advanceNative(
    gf::Task10p11rFixedBaselineFixture& fixture,
    std::optional<std::string>& active_pair) {
    const auto runtime=fixture.adapter.runtimeSnapshot();
    if (!active_pair.has_value()) {
        const auto rows=gf::buildCanonicalHardRows(
            fixture.adapter.snapshotHardRowRequest(
                runtime.estimate,runtime.topology));
        const auto diagnostic=gf::diagnoseTask10p11tOnlineConflicts(
            rows,runtime.estimate.mobile_ids,runtime.runtime_s,
            runtime.mode,runtime.topology,
            fixture.adapter.config().acceleration_half_box);
        if (!diagnostic.valid) throw std::runtime_error(diagnostic.reason);
        if (diagnostic.infeasible) {
            active_pair=gf::task10p11tUniqueOnlinePair(diagnostic);
            if (!active_pair.has_value())
                throw std::runtime_error(
                    diagnostic.mobile_pair_base_ids.size()>1
                        ?"multiple_mobile_pair_conflict"
                        :"dynamic_pair_conflict_not_unique");
        }
    }
    return active_pair.has_value()
        ?fixture.controller.advanceWithDynamicPairResponsibility(*active_pair)
        :fixture.controller.advance();
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=5) {
        std::cerr<<"usage: GrandFinaleTask10p11agDerivePacked "
            "SPARSE INIT_MANIFEST TARGET_TIME OUTPUT_PACKED\n";
        return 2;
    }
    try {
        const auto sparse=gf::readTask10p11vJson(argv[1]);
        const auto manifest=gf::readTask10p11vJson(argv[2]);
        const auto initialization=gf::task10p11acInitializationFromManifest(
            manifest,"P3");
        auto scenario=gf::task10p11rFixedBaselineScenario();
        scenario.mobile_positions=initialization.positions;
        auto settings=gf::task10p11acSwarmSettings(
            scenario,initialization.velocities);
        auto fixture=gf::makeTask10p11rFixedBaselineFixture(
            std::move(scenario),std::move(settings),
            gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
        const auto initialized=fixture->adapter.initializeStageZero();
        if (!initialized.initialized)
            throw std::runtime_error("stage_zero_initialization_failed:"+
                                     initialized.reason);
        auto restored=gf::restoreTask10p11vSparseRestartCheckpoint(
            *fixture,sparse);
        std::optional<std::string> active_pair=restored.active_pair;
        const double target=std::stod(argv[3]);
        if (target<fixture->swarm.robots.front()->runtime-1.0e-9)
            throw std::runtime_error("target_precedes_sparse_checkpoint");
        std::size_t advanced=0;
        while (fixture->swarm.robots.front()->runtime<target-1.0e-9) {
            const auto step=advanceNative(*fixture,active_pair);
            if (!step.step.advanced)
                throw std::runtime_error("suffix_replay_failed:"+
                    (step.reason.empty()?step.step.reason:step.reason));
            ++advanced;
            if (!fixture->topologyFrozen())
                throw std::runtime_error("fixed_topology_identity_failed");
        }
        if (std::abs(fixture->swarm.robots.front()->runtime-target)>1.0e-9)
            throw std::runtime_error("target_not_on_discrete_grid");
        const auto boundary=gf::task10p11zCaptureBeforeOverride(*fixture);
        const auto prepared=gf::task10p11zPrepareNativeBaseline(
            *fixture,gf::GammaFeedbackSelectionMode::LeastIntervention,
            22.0,active_pair);
        if (!prepared.valid || !prepared.control.step.advanced ||
            prepared.control.step.applied_controls.size()!=14 ||
            prepared.nominal_controls.size()!=14)
            throw std::runtime_error("derived_distributed_proposal_unavailable:"+
                                     prepared.reason);
        auto snapshot=gf::makeTask10p11sSnapshot(
            boundary.runtime,boundary.request,prepared.nominal_controls,
            fixture->adapter.config());
        auto packed=gf::makeTask10p11vRestartCheckpoint(
            std::move(snapshot),boundary.restart_fields,
            "task10p11ag_derived_from_sparse");
        packed["task10p11ag"]={{"protocol",
                "task10p11ag-derived-packed-from-sparse-v1"},
            {"derived_from_sparse",true},{"source_sparse_path",argv[1]},
            {"source_sparse_cycle",restored.cycle},
            {"suffix_advanced_cycles",advanced},{"target_time_s",target},
            {"tau_mps2",22.0},{"fixed_topology",true},
            {"active_pair",active_pair.has_value()?json(*active_pair):json(nullptr)},
            {"distributed_applied_controls",controlsJson(
                prepared.control.step.applied_controls)},
            {"claim_boundary",{{"original_packed_event",false},
                {"deterministic_suffix_replay",true},
                {"recursive_feasibility_claimed",false}}}};
        // Compatibility with the frozen Gate-1 oracle binary.  The protocol
        // above remains authoritative and explicitly labels this state as
        // derived rather than an original Task10p11aa event.
        packed["task10p11aa"]={{"protocol",
                "task10p11ag-derived-packed-compatibility-v1"},
            {"decision",{{"applied_controls",controlsJson(
                prepared.control.step.applied_controls)}}}};
        gf::writeTask10p11vJson(argv[4],packed);
        std::cout<<json{{"valid",true},{"target_time_s",target},
            {"suffix_advanced_cycles",advanced},
            {"active_pair",active_pair.has_value()?json(*active_pair):json(nullptr)},
            {"output",argv[4]}}.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ag sparse derivation failed: "
                 <<error.what()<<'\n';
        return 4;
    }
}
