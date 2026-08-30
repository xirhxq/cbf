#include "grand_finale/Task10p11aiTerminalCertificate.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace {

using json=nlohmann::json;

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> makeP3(
    const json& manifest) {
    const auto initialization=gf::task10p11acInitializationFromManifest(
        manifest,"P3");
    auto scenario=gf::task10p11rFixedBaselineScenario();
    scenario.mobile_positions=initialization.positions;
    auto settings=gf::task10p11acSwarmSettings(
        scenario,initialization.velocities);
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        std::move(scenario),std::move(settings),
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
    if (!fixture->adapter.initializeStageZero().initialized)
        throw std::runtime_error("P3 stage-zero initialization failed");
    return fixture;
}

json chainResultJson(const gf::Task10p11aiChainResult& result,
                     const gf::Task10p11aiChainInput& input,
                     bool canonical_confirmed) {
    std::vector<double> incumbent;
    for (const auto& control:result.relative_controls)
        incumbent.insert(incumbent.end(),{control.x(),control.y()});
    return {{"built",result.built},
        {"fail_reason",result.fail_reason},
        {"sign_fixation",{{"layer_1",result.sign_fixation[1]},
            {"layer_2",result.sign_fixation[2]},
            {"layer_3",result.sign_fixation[3]}}},
        {"pair",{{"collision",input.pair.collision},
            {"mobile_pair",input.pair.mobile_pair},
            {"owner_a",input.pair_owner_a},
            {"owner_b",input.pair_owner_b}}},
        {"solved",result.solved},{"gurobi_status",result.gurobi_status},
        {"global_optimal",result.global_optimal},
        {"incumbent_mps2",gf::task10p11w_detail::number(
            result.incumbent_mps2)},
        {"global_bound_mps2",gf::task10p11w_detail::number(
            result.bound_mps2)},
        {"mip_gap",gf::task10p11w_detail::number(result.mip_gap)},
        {"solve_time_s",result.solve_time_s},
        {"incumbent_relative_controls",incumbent},
        {"independent_formula_mps2",gf::task10p11w_detail::number(
            result.independent_formula_mps2)},
        {"canonical_builder_mps2",gf::task10p11w_detail::number(
            result.canonical_builder_mps2)},
        {"canonical_builder_confirmed",canonical_confirmed},
        {"strict",result.strict&&canonical_confirmed}};
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=5&&argc!=6) {
        std::cerr<<"usage: GrandFinaleTask10p11aiCertificate PACKED_157P8 "
            "INIT_MANIFEST OUTPUT_JSON PROGRESS_DIRECTORY [TOTAL_BUDGET_S]\n";
        return 2;
    }
    const auto started=std::chrono::steady_clock::now();
    const auto elapsed=[&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
    };
    try {
        const auto protocol=gf::task10p11aiProtocol();
        const double budget=argc==6?std::stod(argv[5]):
            protocol.stage_a_total_budget_s;
        const auto packed=gf::readTask10p11vJson(argv[1]);
        const auto manifest=gf::readTask10p11vJson(argv[2]);
        const std::filesystem::path progress_directory=argv[4];
        std::filesystem::create_directories(progress_directory);
        json record={{"protocol",protocol.protocol},
            {"preregistration",protocol.preregistration},
            {"stage","A"},{"complete",false}};
        auto emit=[&](const std::string& name,const json& payload) {
            gf::writeTask10p11vJson(progress_directory/name,payload);
        };
        emit("00-protocol.json",record);
        auto fixture=makeP3(manifest);
        gf::restoreTask10p11vRestartState(*fixture,
            packed.at("restart_checkpoint"));
        if (!fixture->topologyFrozen())
            throw std::runtime_error("fixed topology identity failed");
        // Anchor: reproduce the frozen 10.11ah best plan terminal successor.
        gf::Task10p11ahComponentPlan frozen_plan;
        frozen_plan.owner2_u0=Eigen::Vector2d(-4,4);
        frozen_plan.owner9_u0=Eigen::Vector2d(4,-4);
        frozen_plan.owner2_u1=Eigen::Vector2d(-4,4);
        frozen_plan.owner9_u1=Eigen::Vector2d(4,-4);
        const auto anchor=gf::evaluateTask10p11ahTerminalRecoveryPlan(
            *fixture,std::nullopt,frozen_plan);
        const bool anchor_valid=anchor.valid&&
            anchor.terminal_native_successor_feasible==false&&
            std::abs(anchor.terminal_native_successor_residual_mps2-
                protocol.anchor_successor_residual_mps2)<=
                protocol.anchor_tolerance_mps2;
        record["anchor"]={{"evaluated",anchor.valid},
            {"terminal_native_successor_residual_mps2",
                gf::task10p11w_detail::number(
                    anchor.terminal_native_successor_residual_mps2)},
            {"expected_mps2",protocol.anchor_successor_residual_mps2},
            {"valid",anchor_valid}};
        emit("01-anchor.json",record["anchor"]);
        if (!anchor_valid)
            throw std::runtime_error(
                "anchor mismatch against frozen 10.11ah result");
        const auto limiting=gf::task10p11aiIdentifyLimitingRow(*fixture,
            task10p11ahPlanVector(frozen_plan));
        // First-class deliverable: R0-family containment table over the
        // frozen P constraint classes (prereg v1.2 condition 4).  The
        // independent verifier validates this table before any bound.
        record["containment_table"]={
            {"x0_full_rows_with_D(S0)_externals",{{"treatment",
                "relaxed_superset"},{"direction","superset"},{"argument",
                "only the family pair row is kept at x0 (exactly, from the "
                "frozen S0 state); every other canonical x0 row is dropped; "
                "the relative pair row is independent of external controls"}}},
            {"x1_same_frame_externals_D(S1(u0))",{{"treatment",
                "relaxed_superset"},{"direction","superset"},{"argument",
                "externals freed to the input box, which contains every "
                "same-frame canonical control because the canonical QP "
                "enforces the input box rows"}}},
            {"x2_full_rows",{{"treatment","relaxed_superset"},{"direction",
                "superset"},{"argument","only the family pair row is kept at "
                "x2; all other rows dropped"}}},
            {"terminal_owner_local_gamma",{{"treatment","dropped"},
                {"direction","superset"},{"argument","owner-local gamma gate "
                "removed"}}},
            {"terminal_signed_transfer_interval",{{"treatment","dropped"},
                {"direction","superset"},{"argument","signed-transfer gate "
                "removed"}}},
            {"terminal_native_D(S2)_current_rows",{{"treatment",
                "relaxed_superset"},{"direction","superset"},{"argument",
                "the native terminal command is replaced by a free u2 in the "
                "input box, which contains D(S2)"}}},
            {"terminal_native_successor_after_D(S2)",{{"treatment",
                "relaxed_superset"},{"direction","superset"},{"argument",
                "successor feasibility after the native command is replaced "
                "by exact support maximization over the free relative box at "
                "the successor state; per-layer normal sign fixation is "
                "checked at runtime and fails closed"}}},
            {"component_control_box",{{"treatment","kept_exact"},{"direction",
                "equal"},{"argument","the relative domain [-2b,2b] per beat "
                "equals the exact difference set of two controls in [-b,b]"}}},
            {"relative_domain_argument",{{"domain_covers_box_differences",
                true},{"argument","per-beat relative domain covers every "
                "control difference of owners with controls in the input box"}}}};
        record["limiting_row"]={{"row_id",limiting.row_id},
            {"pair_row",limiting.pair_row},
            {"coupled_mobile_pair",limiting.coupled_mobile_pair},
            {"collision",limiting.collision},{"reference",limiting.reference},
            {"first",limiting.first},{"second",limiting.second},
            {"successor_minimum_residual_mps2",
                gf::task10p11w_detail::number(
                    limiting.successor_minimum_residual_mps2)}};
        emit("02-limiting-row.json",record["limiting_row"]);
        // Boundary request and per-layer tubes.
        auto boundary_frame=gf::task10p11ah_optimizer_detail::nativeFrame(
            *fixture,std::nullopt);
        const auto& request0=boundary_frame.boundary.request;
        const auto snapshot=boundary_frame.snapshot;
        const double dt=snapshot.at("successor_parameters").at("dt_s").
            get<double>();
        const auto estimate0=gf::task10p11s_capture_detail::estimateFromJson(
            snapshot.at("estimator"));
        std::map<gf::NodeId,Eigen::Vector2d> zero;
        for (gf::NodeId owner:request0.mobile_ids)
            zero.emplace(owner,Eigen::Vector2d::Zero());
        std::array<gf::JointEstimateSnapshot,4> estimates{estimate0,{},{},{}};
        std::array<gf::CanonicalHardRowRequest,4> requests{request0,{},{},{}};
        for (std::size_t layer=1;layer<=3;++layer) {
            estimates[layer]=gf::task10p11aa_detail::predictEstimate(snapshot,
                estimates[layer-1],zero);
            requests[layer]=gf::task10p11x_detail::requestAtEstimate(snapshot,
                estimates[layer]);
        }
        auto make_chain_input=[&](gf::NodeId a,gf::NodeId b,bool collision,
                                  bool mobile_pair) {
            gf::Task10p11aiChainInput input;
            input.pair.collision=collision;
            input.pair.mobile_pair=mobile_pair;
            input.pair.spec=collision?request0.collision_spec:
                request0.reference_spec;
            input.pair_owner_a=a;
            input.pair_owner_b=b;
            const auto& state_a=request0.states.at(a);
            const auto& state_b=request0.states.at(b);
            input.rel_pos_0=Eigen::Vector2d(
                state_a.position.x-state_b.position.x,
                state_a.position.y-state_b.position.y);
            input.rel_vel_0=state_a.velocity-state_b.velocity;
            input.dt_s=dt;
            input.half_box_mps2=protocol.half_box_mps2;
            for (std::size_t layer=0;layer<=3;++layer) {
                const std::string key=collision?
                    std::to_string(std::min(a,b))+"--"+
                        std::to_string(std::max(a,b)):
                    std::to_string(b)+"->"+std::to_string(a);
                const auto& tubes=collision?
                    requests[layer].collision_snapshot_tubes:
                    requests[layer].reference_snapshot_tubes;
                const auto found=tubes.find(key);
                if (found==tubes.end())
                    throw std::runtime_error("chain tube absent:"+key);
                input.tube_position_radius_m[layer]=
                    found->second.position_radius_m;
                input.tube_velocity_radius_mps[layer]=
                    found->second.velocity_radius_mps;
            }
            return input;
        };
        std::vector<std::pair<std::string,gf::Task10p11aiChainInput>> chains;
        chains.emplace_back("collision:2--9",make_chain_input(2,9,true,true));
        std::string limiting_chain_name;
        if (limiting.pair_row&&!(limiting.collision&&limiting.first==2&&
                limiting.second==9)) {
            const bool collision=limiting.collision;
            gf::NodeId owner=limiting.owner;
            gf::NodeId peer=limiting.peer;
            if (collision) {
                const bool first_mobile=std::find(request0.mobile_ids.begin(),
                    request0.mobile_ids.end(),limiting.first)!=
                    request0.mobile_ids.end();
                const bool second_mobile=std::find(request0.mobile_ids.begin(),
                    request0.mobile_ids.end(),limiting.second)!=
                    request0.mobile_ids.end();
                owner=first_mobile?limiting.first:limiting.second;
                peer=first_mobile?limiting.second:limiting.first;
            }
            const bool mobile_pair=std::find(request0.mobile_ids.begin(),
                request0.mobile_ids.end(),peer)!=request0.mobile_ids.end();
            const std::string key=collision?
                std::to_string(std::min(owner,peer))+"--"+
                    std::to_string(std::max(owner,peer)):
                std::to_string(peer)+"->"+std::to_string(owner);
            limiting_chain_name=(collision?"collision:":"reference:")+key;
            chains.emplace_back(limiting_chain_name,
                make_chain_input(owner,peer,collision,mobile_pair));
        }
        const double remaining=budget-elapsed();
        if (remaining<=0.0) throw std::runtime_error("budget exhausted early");
        record["stage_a1_chains"]=json::array();
        bool any_strict=false;
        bool all_optimal_positive=true;
        for (auto& [name,input]:chains) {
            auto chain_input=input;
            chain_input.time_limit_s=std::max(1.0,remaining/
                static_cast<double>(chains.size()));
            const auto result=gf::solveTask10p11aiPairChainBound(chain_input);
            bool canonical_confirmed=false;
            if (result.strict&&result.solved) {
                const auto canonical=gf::task10p11aiCanonicalChainValue(
                    *fixture,chain_input,result.relative_controls);
                canonical_confirmed=canonical.valid&&
                    std::abs(canonical.value_mps2-
                        result.independent_formula_mps2)<=
                        protocol.independent_agreement_mps2;
            }
            any_strict=any_strict||(result.strict&&canonical_confirmed);
            if (!(result.global_optimal&&result.built&&
                    result.bound_mps2>=0.0))
                all_optimal_positive=false;
            record["stage_a1_chains"].push_back(
                chainResultJson(result,chain_input,canonical_confirmed));
            emit("10-chain-"+name+".json",record["stage_a1_chains"].back());
        }
        record["stage_a2_single_row_granularity"]={
            {"note","chains are independent single-pair programs; the "
                "per-chain records above are the single-row fallback "
                "granularity"},
            {"activated",!any_strict}};
        if (any_strict) {
            record["classification"]=
                gf::task10p11aiClassificationName(
                    gf::Task10p11aiClassification::StrictlyInfeasible);
        } else {
            const double left=budget-elapsed();
            if (left>1.0) {
                const auto audit=gf::runTask10p11aiIntervalAudit(snapshot,
                    protocol.half_box_mps2);
                json layers=json::array();
                for (std::size_t layer=0;layer<=3;++layer) {
                    const auto& item=audit.layers[layer];
                    layers.push_back({{"layer",layer},
                        {"valid",item.valid},{"fail_reason",item.fail_reason},
                        {"row_count",item.row_count},
                        {"minimum_sup_mps2",gf::task10p11w_detail::number(
                            item.minimum_sup_mps2)},
                        {"limiting_row_id",item.limiting_row_id}});
                }
                record["stage_a3_interval_audit"]={{"layers",layers}};
                emit("20-interval-audit.json",
                    record["stage_a3_interval_audit"]);
            } else {
                record["stage_a3_interval_audit"]={{"skipped",
                    "budget_exhausted"}};
            }
            record["classification"]=gf::task10p11aiClassificationName(
                gf::Task10p11aiClassification::Undetermined);
            record["undetermined_reason"]=all_optimal_positive?
                "family_programs_globally_optimal_with_nonnegative_bound":
                "chain_programs_did_not_close";
        }
        record["complete"]=true;
        record["wall_time_s"]=elapsed();
        record["budget_total_s"]=budget;
        record["terminality"]={{"last_task_for_157p8",true},
            {"no_followup_subtasks",true},
            {"earlier_prevention_requires_new_preregistration",true}};
        record["claim_boundary"]={
            {"certificate_only_from_global_optimal_negative_bound",true},
            {"interval_audit_never_upgrades_classification",true},
            {"component_expansion",false},{"horizon_expansion",false},
            {"dynamic_topology",false},{"recursive_feasibility_claimed",
            false},{"recursive_infeasibility_claimed",false}};
        gf::writeTask10p11vJson(argv[3],record);
        std::cout<<record.dump(2)<<'\n';
        const bool strict=record["classification"]=="strictly_infeasible";
        return strict?0:5;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ai certificate failed: "<<error.what()<<'\n';
        return 4;
    }
}
